#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/wdt.h>

#define HEATER_VERSION "1.9"
#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "This firmware requires ATmega328P at 16 MHz"
#endif

void earlySafetyInit() __attribute__((used, naked, section(".init3")));
void earlySafetyInit() {
  MCUSR = 0;
  wdt_disable();

  PORTB &= static_cast<uint8_t>(~(_BV(PB0) | _BV(PB1)));
  DDRB |= _BV(PB0) | _BV(PB1);
}

// Настройки
// После изменения параметров загрузить скетч заново. Дробные значения: 32.0f.

// Расширенный вывод АЦП и результатов проверки силовой цепи.
constexpr bool DETAILED_DIAGNOSTICS = false;

// Напряжение обмотки, В. Задаёт пороги A1 и расчётные пределы нагрузки.
constexpr float SUPPLY_V = 42.0f;
// Сопротивление холодной обмотки, Ом. Используется для расчёта мощности.
constexpr float HEATER_R_REF_OHM = 19.47f;
// Расчётные пределы силовой части, А и Вт. Проверяются при компиляции.
constexpr float DESIGN_CURRENT_LIMIT_A = 3.0f;
constexpr float DESIGN_POWER_LIMIT_W = 120.0f;

// Температура поддержания по NTC, °C.
constexpr float TARGET_C = 32.0f;
// До этой температуры задаётся MAX_PWM, выше начинается регулирование, °C.
constexpr float NORMAL_FULL_POWER_C = 0.0f;
// Температура выхода на предел WARM_PWM, °C.
constexpr float NORMAL_GENTLE_C = 30.0f;
// Аварийная температура, °C. Отключает оба ключа с фиксацией ошибки.
constexpr float EMERGENCY_C = 40.0f;
// Зона индикации поддержания: от TARGET_C минус это значение, °C.
constexpr float HOLD_DISPLAY_BAND_C = 1.0f;

// Максимальный PWM, 0..255. Значение 255 соответствует 100%.
constexpr uint8_t MAX_PWM = 255;
// Предел PWM при NORMAL_GENTLE_C и выше, 0..255.
constexpr uint8_t WARM_PWM = 100;
// Скорость нарастания PWM, единиц в секунду. Снижение выполняется сразу.
constexpr float PWM_RISE_PER_SECOND = 32.0f;
// Коэффициенты PI-регулятора: PWM/°C и PWM/(°C·с).
constexpr float KP = 35.0f;
constexpr float KI = 0.8f;

// Параметры NTC: сопротивление при 25 °C, Ом; R1, Ом; коэффициент B, K.
constexpr float NTC_R25 = 10000.0f;
constexpr float NTC_FIXED = 10000.0f;
constexpr float NTC_BETA = 3977.0f;
// Поправка показаний NTC, °C. Определяется сравнением с термометром.
constexpr float NTC_OFFSET_C = 0.0f;
// Напряжение 5V–GND Nano, В. Используется АЦП для оценки питания обмотки.
constexpr float ADC_REF_V = 5.0f;
// Сопротивления R6 и R7 делителя A1, Ом.
constexpr float A1_TOP_OHM = 220000.0f;
constexpr float A1_BOTTOM_OHM = 10000.0f;
// Пороги силового самотеста как доли номинального показания A1.
constexpr float A1_HIGH_RATIO = 0.35f;
constexpr float A1_LOW_RATIO = 0.08f;

constexpr float NOMINAL_CURRENT_A = SUPPLY_V / HEATER_R_REF_OHM;
constexpr float NOMINAL_POWER_W = SUPPLY_V * NOMINAL_CURRENT_A;
constexpr float A1_EXPECTED_HIGH_ADC =
    SUPPLY_V * A1_BOTTOM_OHM / (A1_TOP_OHM + A1_BOTTOM_OHM) / ADC_REF_V * 1023.0f;
constexpr uint16_t DRAIN_HIGH_MIN =
    static_cast<uint16_t>(A1_EXPECTED_HIGH_ADC * A1_HIGH_RATIO + 0.5f);
constexpr uint16_t DRAIN_LOW_MAX =
    static_cast<uint16_t>(A1_EXPECTED_HIGH_ADC * A1_LOW_RATIO + 0.5f);

constexpr uint8_t PIN_PWM = 9, PIN_ENABLE = 8, PIN_BUTTON = 2;
constexpr uint8_t PIN_NTC = A0, PIN_DRAIN = A1, PIN_GREEN = 5, PIN_RED = 6;
constexpr uint8_t PIN_BOARD_LED = LED_BUILTIN;
constexpr uint32_t CONTROL_MS = 250, DEBOUNCE_MS = 35;
constexpr uint32_t READY_PERIOD_MS = 2000, READY_PULSE_MS = 80;
constexpr uint16_t FAULT_ON_MS = 300, FAULT_OFF_MS = 300, FAULT_GAP_MS = 2000;
constexpr uint32_t TELEMETRY_MS = 1000, SUPPLY_STABLE_MS = 1000;
constexpr uint32_t VOLTAGE_MAX_AGE_MS = 2000;
constexpr uint32_t FAULT_RESET_MS = 5000;
constexpr uint16_t SETTLE_US = 600;

static_assert(SUPPLY_V > 0.0f && HEATER_R_REF_OHM > 0.0f, "Invalid heater electrical parameters");
static_assert(NOMINAL_CURRENT_A <= DESIGN_CURRENT_LIMIT_A &&
                  NOMINAL_POWER_W <= DESIGN_POWER_LIMIT_W,
              "Calculated cold load exceeds configured hardware limits");
static_assert(TARGET_C < EMERGENCY_C, "Invalid temperature limits");
static_assert(HOLD_DISPLAY_BAND_C > 0.0f, "Invalid display band");
static_assert(NORMAL_FULL_POWER_C < NORMAL_GENTLE_C && NORMAL_GENTLE_C < TARGET_C &&
                  WARM_PWM <= MAX_PWM,
              "Invalid normal heating curve");
static_assert(PWM_RISE_PER_SECOND > 0.0f, "Invalid PWM rise rate");
static_assert(A1_EXPECTED_HIGH_ADC < 1020.0f && DRAIN_LOW_MAX < DRAIN_HIGH_MIN,
              "Invalid A1 divider or thresholds");

enum class Mode : uint8_t { Off, Heating, Fault };
enum class Fault : uint8_t {
  None = 0,
  Overheat = 2,
  SupplyOrHeater = 3,
  Sensor = 4,
  Q1 = 5,
  Q2 = 6,
  Conduction = 7
};
Mode mode = Mode::Off;
Fault faultCode = Fault::None;
uint32_t faultLedStartedAt = 0;
bool faultRetryInProgress = false;
float temperatureC = NAN, integral = 0.0f;
uint32_t lastControlAt = 0, buttonChangedAt = 0;
uint32_t faultPressAt = 0;
bool buttonRaw = HIGH, buttonStable = HIGH, buttonArmed = false;
bool faultResetHeld = false;
uint8_t commandedPwm = 0;
uint16_t lastNtcAdc = 0, lastTestAdc[4] = {0, 0, 0, 0};
uint8_t lastTestSeen = 0;
uint32_t lastTelemetryAt = 0, supplyHighAt = 0;
bool supplyHighPending = false;
float measuredSupplyV = 0.0f;
uint32_t voltageMeasuredAt = 0;
bool voltageMeasured = false;

constexpr int EE_MAGIC_ADDR = 0, EE_STATE_ADDR = 1, EE_INVERSE_ADDR = 2;
constexpr uint8_t EE_MAGIC = 0xB1, EE_OFF = 0x3C, EE_ON = 0xC3;
void tripFault(Fault reason);

bool heatingRequested() {
  return mode == Mode::Heating;
}

void pwmWrite(uint8_t value) {
  commandedPwm = value;
  analogWrite(PIN_PWM, value);
}

void gatesOff() {

  digitalWrite(PIN_ENABLE, LOW);
  pwmWrite(0);
}

uint8_t loadState() {
  const uint8_t magic = EEPROM.read(EE_MAGIC_ADDR);
  const uint8_t state = EEPROM.read(EE_STATE_ADDR);
  const uint8_t inverse = EEPROM.read(EE_INVERSE_ADDR);

  const bool validState = state == EE_OFF || state == EE_ON;
  if (magic != EE_MAGIC || !validState || inverse != static_cast<uint8_t>(~state))
    return EE_OFF;
  return state;
}

void saveState(uint8_t state) {
  if (EEPROM.read(EE_MAGIC_ADDR) == EE_MAGIC && EEPROM.read(EE_STATE_ADDR) == state &&
      EEPROM.read(EE_INVERSE_ADDR) == static_cast<uint8_t>(~state))
    return;
  EEPROM.update(EE_MAGIC_ADDR, 0);
  EEPROM.update(EE_STATE_ADDR, state);
  EEPROM.update(EE_INVERSE_ADDR, static_cast<uint8_t>(~state));
  EEPROM.update(EE_MAGIC_ADDR, EE_MAGIC);
  eeprom_busy_wait();
}

void tripFault(Fault reason) {
  gatesOff();
  if (mode == Mode::Fault && (!faultRetryInProgress || faultCode == reason))
    return;
  mode = Mode::Fault;
  faultCode = reason;
  faultLedStartedAt = millis();
  integral = 0.0f;
  saveState(EE_OFF);
}

uint16_t readAdc(uint8_t pin, uint8_t samples) {
  analogRead(pin);
  uint16_t sum = 0;
  for (uint8_t i = 0; i < samples; ++i)
    sum += static_cast<uint16_t>(analogRead(pin));
  return static_cast<uint16_t>((sum + samples / 2U) / samples);
}

bool sampleTemperature() {
  const uint16_t adc = readAdc(PIN_NTC, 16);
  lastNtcAdc = adc;
  temperatureC = NAN;
  if (adc <= 2 || adc >= 1021) {
    tripFault(Fault::Sensor);
    return false;
  }
  const float resistance = NTC_FIXED * adc / (1023.0f - adc);
  const float logRatio = static_cast<float>(logf(resistance / NTC_R25));
  const float inverseK = 1.0f / 298.15f + logRatio / NTC_BETA;
  const float value = 1.0f / inverseK - 273.15f + NTC_OFFSET_C;
  if (!isfinite(value) || value < -45.0f || value > 125.0f) {
    tripFault(Fault::Sensor);
    return false;
  }
  temperatureC = value;
  if (value >= EMERGENCY_C) {
    tripFault(Fault::Overheat);
    return false;
  }
  return true;
}

bool drainIsHigh(uint8_t stage) {
  delayMicroseconds(SETTLE_US);
  const uint16_t adc = readAdc(PIN_DRAIN, 4);
  lastTestAdc[stage] = adc;
  lastTestSeen |= static_cast<uint8_t>(1U << stage);
  if (stage == 0) {

    voltageMeasured = adc >= DRAIN_HIGH_MIN && adc < 1021;
    if (voltageMeasured) {
      measuredSupplyV = adc * ADC_REF_V / 1023.0f * (A1_TOP_OHM + A1_BOTTOM_OHM) / A1_BOTTOM_OHM;
      voltageMeasuredAt = millis();
    }
  }
  return adc >= DRAIN_HIGH_MIN;
}

bool checkPowerStage(bool testConduction) {

  gatesOff();
  lastTestSeen = 0;
  if (!drainIsHigh(0)) {
    tripFault(Fault::SupplyOrHeater);
    return false;
  }
  digitalWrite(PIN_ENABLE, HIGH);
  const bool q1Ok = drainIsHigh(1);
  gatesOff();
  if (!q1Ok) {
    tripFault(Fault::Q1);
    return false;
  }
  delayMicroseconds(SETTLE_US);
  pwmWrite(255);
  const bool q2Ok = drainIsHigh(2);
  gatesOff();
  if (!q2Ok) {
    tripFault(Fault::Q2);
    return false;
  }
  if (testConduction) {
    digitalWrite(PIN_ENABLE, HIGH);
    pwmWrite(255);
    delayMicroseconds(SETTLE_US);
    const uint16_t adc = readAdc(PIN_DRAIN, 4);
    lastTestAdc[3] = adc;
    lastTestSeen |= 8;
    gatesOff();
    if (adc > DRAIN_LOW_MAX) {
      tripFault(Fault::Conduction);
      return false;
    }
  }
  return true;
}

void startHeating() {
  integral = 0.0f;
  mode = Mode::Heating;
}

void stopHeating() {
  gatesOff();
  mode = Mode::Off;
  integral = 0.0f;
}

uint8_t normalPwmLimit() {
  const float position = constrain(
      (temperatureC - NORMAL_FULL_POWER_C) / (NORMAL_GENTLE_C - NORMAL_FULL_POWER_C), 0.0f, 1.0f);
  return static_cast<uint8_t>(MAX_PWM - (MAX_PWM - WARM_PWM) * position + 0.5f);
}

uint8_t regulatedPwm(float dtSeconds) {

  const float error = TARGET_C - temperatureC;
  const float limit = normalPwmLimit();
  integral = min(integral, limit);
  const float proposed = constrain(integral + KI * error * dtSeconds, 0.0f, limit);
  const float output = KP * error + proposed;

  if (!((output > limit && error > 0.0f) || (output < 0.0f && error < 0.0f)))
    integral = proposed;
  const float limited = constrain(KP * error + integral, 0.0f, limit);
  return static_cast<uint8_t>(limited + 0.5f);
}

void controlStep(float dtSeconds) {
  const uint8_t previousPwm = commandedPwm;
  if (mode == Mode::Fault) {
    gatesOff();
    return;
  }
  if (!sampleTemperature())
    return;
  if (!checkPowerStage(false))
    return;
  if (!heatingRequested())
    return;
  uint8_t pwm;
  if (temperatureC <= NORMAL_FULL_POWER_C) {
    pwm = MAX_PWM;
    integral =
        constrain(previousPwm - KP * (TARGET_C - temperatureC), 0.0f, static_cast<float>(MAX_PWM));
  } else {
    pwm = regulatedPwm(dtSeconds);
  }
  const uint8_t riseLimit =
      static_cast<uint8_t>(constrain(PWM_RISE_PER_SECOND * dtSeconds, 1.0f, 255.0f));
  const uint16_t nextLimit = static_cast<uint16_t>(previousPwm) + riseLimit;
  if (pwm > nextLimit)
    pwm = static_cast<uint8_t>(nextLimit);
  if (pwm != 0) {
    digitalWrite(PIN_ENABLE, HIGH);
    pwmWrite(pwm);
  }
}

void onButtonPress() {
  if (mode == Mode::Fault)
    return;
  if (heatingRequested()) {
    stopHeating();
    saveState(EE_OFF);
    return;
  }
  gatesOff();
  if (!sampleTemperature())
    return;
  if (!checkPowerStage(true))
    return;
  saveState(EE_ON);
  startHeating();
}

void tryClearFault() {
  gatesOff();
  faultRetryInProgress = true;
  const bool passed = sampleTemperature() && checkPowerStage(true);
  faultRetryInProgress = false;
  if (!passed)
    return;
  saveState(EE_OFF);
  stopHeating();
  faultCode = Fault::None;
}

void serviceButton(uint32_t now) {
  const bool raw = digitalRead(PIN_BUTTON) != LOW;
  if (raw != buttonRaw) {
    buttonRaw = raw;
    buttonChangedAt = now;
  }
  if (static_cast<uint32_t>(now - buttonChangedAt) < DEBOUNCE_MS)
    return;
  if (buttonStable == buttonRaw) {
    if (buttonStable == HIGH)
      buttonArmed = true;
    if (faultResetHeld && static_cast<uint32_t>(now - faultPressAt) >= FAULT_RESET_MS) {
      faultResetHeld = false;
      tryClearFault();
    }
    return;
  }
  buttonStable = buttonRaw;
  if (buttonStable == HIGH) {
    buttonArmed = true;
    faultResetHeld = false;
  } else if (buttonArmed) {
    buttonArmed = false;
    faultResetHeld = mode == Mode::Fault;
    faultPressAt = now;
    if (!faultResetHeld)
      onButtonPress();
  }
}

void updateLed(uint32_t now) {
  digitalWrite(PIN_GREEN, mode == Mode::Off ? HIGH : LOW);
  bool boardLed = heatingRequested() || now % READY_PERIOD_MS < READY_PULSE_MS;
  if (mode == Mode::Fault) {
    const uint8_t count = static_cast<uint8_t>(faultCode);
    const uint32_t pulsePeriod = FAULT_ON_MS + FAULT_OFF_MS;
    const uint32_t active = static_cast<uint32_t>(count) * pulsePeriod - FAULT_OFF_MS;
    const uint32_t phase = static_cast<uint32_t>(now - faultLedStartedAt) % (active + FAULT_GAP_MS);
    boardLed = phase < active && phase % pulsePeriod < FAULT_ON_MS;
  }
  digitalWrite(PIN_RED, (mode == Mode::Fault ? boardLed : heatingRequested()) ? HIGH : LOW);
  digitalWrite(PIN_BOARD_LED, boardLed ? HIGH : LOW);
}

void serviceSupplyRecovery(uint32_t now) {

  if (mode != Mode::Fault || faultCode != Fault::SupplyOrHeater) {
    supplyHighPending = false;
    return;
  }
  gatesOff();
  if (readAdc(PIN_DRAIN, 4) < DRAIN_HIGH_MIN) {
    supplyHighPending = false;
    return;
  }
  if (!supplyHighPending) {
    supplyHighPending = true;
    supplyHighAt = now;
  }
  if (static_cast<uint32_t>(now - supplyHighAt) >= SUPPLY_STABLE_MS) {
    supplyHighPending = false;
    tryClearFault();
  }
}

void printTelemetry(uint32_t now) {
  if (static_cast<uint32_t>(now - lastTelemetryAt) < TELEMETRY_MS)
    return;
  lastTelemetryAt = now;
  Serial.print(F("Режим: "));
  switch (mode) {
  case Mode::Off:
    Serial.print(F("ВЫКЛЮЧЕН"));
    break;
  case Mode::Heating:
    Serial.print(temperatureC >= TARGET_C - HOLD_DISPLAY_BAND_C ? F("ПОДДЕРЖАНИЕ") : F("НАГРЕВ"));
    break;
  case Mode::Fault:
    Serial.print(F("АВАРИЯ"));
    break;
  }
  Serial.print(F(" | Температура: "));
  if (isfinite(temperatureC)) {
    Serial.print(temperatureC, 1);
    Serial.print(F(" °C"));
  } else
    Serial.print(F("нет достоверных данных"));
  const uint8_t percent =
      static_cast<uint8_t>((static_cast<uint16_t>(commandedPwm) * 100U + 127U) / 255U);
  Serial.print(F(" | Нагрев: "));
  Serial.print(percent);
  Serial.print('%');
  const bool voltageFresh =
      voltageMeasured && static_cast<uint32_t>(now - voltageMeasuredAt) <= VOLTAGE_MAX_AGE_MS;
  Serial.print(F(" | Питание: "));
  if (voltageFresh) {
    Serial.print('~');
    Serial.print(measuredSupplyV, 1);
    Serial.print(F(" В"));
  } else
    Serial.print(F("нет данных"));
  Serial.print(F(" | Мощность расч.: "));
  if (commandedPwm == 0)
    Serial.print(F("0.0 Вт"));
  else if (voltageFresh) {
    const float estimatedPowerW =
        measuredSupplyV * measuredSupplyV / HEATER_R_REF_OHM * commandedPwm / 255.0f;
    Serial.print('~');
    Serial.print(estimatedPowerW, 1);
    Serial.print(F(" Вт"));
  } else
    Serial.print(F("нет данных"));
  if (faultCode == Fault::None)
    Serial.print(F(" | Ошибок нет"));
  else {
    Serial.print(F(" | Ошибка "));
    Serial.print(static_cast<uint8_t>(faultCode));
    Serial.print(F(": "));
    switch (faultCode) {
    case Fault::Overheat:
      Serial.print(F("перегрев"));
      break;
    case Fault::SupplyOrHeater:
      Serial.print(F("нет силового питания или обрыв нагревателя"));
      break;
    case Fault::Sensor:
      Serial.print(F("неисправность датчика температуры"));
      break;
    case Fault::Q1:
      Serial.print(F("ошибка проверки отключения Q1"));
      break;
    case Fault::Q2:
      Serial.print(F("ошибка проверки отключения Q2"));
      break;
    case Fault::Conduction:
      Serial.print(F("ошибка проверки включения силовой цепи"));
      break;
    case Fault::None:
      break;
    }
  }
  if (DETAILED_DIAGNOSTICS) {
    Serial.print(F(" | ms="));
    Serial.print(now);
    Serial.print(F(" A0="));
    Serial.print(lastNtcAdc);
    Serial.print(F(" A1="));
    Serial.print(readAdc(PIN_DRAIN, 4));
    Serial.print(F(" PWM_raw="));
    Serial.print(commandedPwm);
    Serial.print(F(" Q2="));
    Serial.print(digitalRead(PIN_ENABLE));
    Serial.print(F(" D2="));
    Serial.print(digitalRead(PIN_BUTTON));
    Serial.print(F(" TEST_ADC[OFF/Q2/Q1/ON]="));
    for (uint8_t i = 0; i < 4; ++i) {
      if (i != 0)
        Serial.print('/');
      if (lastTestSeen & static_cast<uint8_t>(1U << i))
        Serial.print(lastTestAdc[i]);
      else
        Serial.print('-');
    }
  }
  Serial.println();
  Serial.println();
}

void setup() {
  digitalWrite(PIN_PWM, LOW);
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);
  pinMode(PIN_ENABLE, OUTPUT);
  gatesOff();
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  digitalWrite(PIN_GREEN, LOW);
  digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_BOARD_LED, LOW);
  pinMode(PIN_BOARD_LED, OUTPUT);
  analogReference(DEFAULT);
  Serial.begin(115200);
  wdt_enable(WDTO_1S);
  buttonRaw = digitalRead(PIN_BUTTON) != LOW;
  buttonStable = buttonRaw;
  buttonChangedAt = millis();
  const uint8_t saved = loadState();
  if (sampleTemperature() && checkPowerStage(true)) {
    if (saved == EE_ON)
      startHeating();
    else {
      stopHeating();
      saveState(EE_OFF);
    }
  }
  lastControlAt = millis();
  updateLed(lastControlAt);
  Serial.println(F("Модуль подогрева " HEATER_VERSION));
  Serial.println();
  wdt_reset();
}

void loop() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - lastControlAt;
  if (elapsed >= CONTROL_MS) {
    lastControlAt = now;
    if (mode == Mode::Fault) {

      sampleTemperature();
      serviceSupplyRecovery(millis());
    } else
      controlStep(min(elapsed, 1000UL) / 1000.0f);
  }
  serviceButton(millis());
  updateLed(millis());
  printTelemetry(millis());
  wdt_reset();
}
