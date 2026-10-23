/*
  Модуль подогрева 1.4. Arduino Nano V3 / ATmega328P, 5 В, 16 МГц.
  D9 -> R2 -> Q1 Gate; D8 -> R4 -> Q2 Gate. +42 V -> W1 -> Q1 -> Q2 -> GND.
  A1: R6 220k / R7 10k, C2 10nF. A0: +5 -> R1 10k -> NTC 10k B3977 -> GND,
  C1 100nF между A0 и GND. D2: кнопка на GND. D5/D6: зелёный/красный LED.
  Встроенный L (D13): короткая вспышка раз в 2 с -> OFF; светит -> запрос ON;
  серии вспышек с паузой -> FAULT. Внешний LED необязателен.
  Коды L: 1 - старая/неизвестная ошибка; 2 - перегрев/горячо для сброса; 3 - питание/обрыв;
  4 - NTC: обрыв, КЗ или диапазон; 5 - не пройден тест отключения Q1;
  6 - не пройден тест отключения Q2; 7 - цепь не проводит при обоих ON;
  8 - watchdog; 9 - brownout; 10 - повреждённая запись состояния EEPROM.
  Код указывает результат теста, а не доказывает поломку конкретного компонента.
  Ошибка и её код сохраняются при перезапуске; удержание D2 5 с повторяет тест.
  Неуспешная попытка показывает текущую найденную причину; успешная -> OFF.

  FAULT снимается удержанием кнопки D2 5 с и успешным самотестом -> OFF.
  Reset и отключение питания не снимают сохранённый FAULT.
  ON/OFF восстанавливается из EEPROM, включая неизвестный перезапуск.
  Тёплый руль сохраняет ON, но ждёт остывания и полного силового самотеста.
  Распознанные watchdog/brownout -> FAULT. Причина - из MCUSR;
  для проверенного Optiboot >=4.6 с передачей MCUSR в r2 есть опция ниже.
  Загрузчик должен корректно обрабатывать watchdog: кнопка не лечит boot-loop.
  Самотест не является измерителем тока или независимой термозащитой.
*/
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <avr/wdt.h>

#define HEATER_VERSION "1.4"
#ifndef HEATER_OPTIBOOT_RESET_CAUSE
// Оставить 0; ставить 1 только для проверенного Optiboot >=4.6, передающего причину сброса через r2.
#define HEATER_OPTIBOOT_RESET_CAUSE 0
#endif
#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "This firmware requires ATmega328P at 16 MHz"
#endif

uint8_t resetCause __attribute__((section(".noinit")));
#if HEATER_OPTIBOOT_RESET_CAUSE
uint8_t bootResetCause __attribute__((section(".noinit")));
void captureBootReset() __attribute__((used, naked, section(".init0")));
void captureBootReset() {
  __asm__ __volatile__("sts %0, r2" : "=m" (bootResetCause));
}
#endif
void earlySafetyInit() __attribute__((used, naked, section(".init3")));
void earlySafetyInit() {
  resetCause = MCUSR;
#if HEATER_OPTIBOOT_RESET_CAUSE
  if (resetCause == 0) resetCause = bootResetCause;
#endif
  MCUSR = 0;
  wdt_disable();
  // D8/PB0 и D9/PB1 в LOW до инициализации Arduino и EEPROM.
  PORTB &= static_cast<uint8_t>(~(_BV(PB0) | _BV(PB1)));
  DDRB |= _BV(PB0) | _BV(PB1);
}

// ================= НАСТРОЙКИ ПОЛЬЗОВАТЕЛЯ =================
// Для своей обмотки указать напряжение и холодное сопротивление, затем температуры BOOST/HOLD.
// Дроби писать через точку: 33.2f. После изменений заново скомпилировать и загрузить скетч.
// Электрические данные: расчёт I=U/R, P=U*U/R. Реальный ток и мощность модуль не измеряет.
constexpr float SUPPLY_V = 42.0f;             // В: фактическое питание обмотки; меняет расчёт I/P и пороги A1.
// SUPPLY_V не регулирует преобразователь: его выходное напряжение выставить вручную.
constexpr float HEATER_R_REF_OHM = 19.47f;     // Ом: измеренное холодное сопротивление; меняет расчёт I/P.
constexpr float HEATER_R_REF_C = 21.0f;        // °C: температура измерения сопротивления; только справочная запись.
constexpr float WIRE_DIAMETER_MM = 0.204f;     // мм: диаметр вольфрама; только справочная запись, на PWM не влияет.
constexpr float DESIGN_CURRENT_LIMIT_A = 3.0f; // А: допустимый расчётный ток аппаратуры, проверяется при компиляции.
constexpr float DESIGN_POWER_LIMIT_W = 120.0f;// Вт: допустимая расчётная мощность, проверяется при компиляции.
// Эти два предела не ограничивают реальный ток; задавать по возможностям собранной силовой цепи.

// Температуры относятся к месту установки NTC. После BOOST нагрев ждёт остывания до цели HOLD.
constexpr float BOOST_EXIT_C = 36.4f;         // °C: конец быстрого прогрева; для нашего NTC около 6,11 кОм.
constexpr float TARGET_C = 33.2f;             // °C: цель поддержания; выше значение - теплее, сейчас около 7,00 кОм.
constexpr float FORCE_OFF_C = 39.9f;          // °C: закрыть оба ключа, запретить и диагностические импульсы.
constexpr float SOFT_OFF_C = 44.0f;           // °C: включить дополнительную температурную блокировку.
constexpr float SOFT_RESET_C = 42.0f;         // °C: снять эту блокировку после остывания; рабочая цель отдельная.
constexpr float EMERGENCY_C = 49.0f;          // °C: записать FAULT; сброс кнопкой 5 с после успешного самотеста.
// Порядок: TARGET_C < BOOST_EXIT_C < FORCE_OFF_C < SOFT_OFF_C < EMERGENCY_C;
// SOFT_RESET_C < SOFT_OFF_C. Защитные температуры при повторении схемы оставить прежними.
constexpr uint32_t BOOST_MAX_MS = 300000;     // мс: предел времени BOOST; 300000 = 5 минут, затем HOLD.
constexpr uint8_t BOOST_PWM = 255;            // 0..255: мощность BOOST; 255 = 100%, 128 примерно 50%.
constexpr uint8_t HOLD_MAX_PWM = 180;         // 0..255: верхний предел мощности HOLD; 180 примерно 71%.
// PWM меняет среднюю мощность; ток во время включённого импульса определяется напряжением и обмоткой.
constexpr float KP = 35.0f;                  // PWM/°C: реакция на недогрев; больше - резче. Обычно не менять.
constexpr float KI = 0.8f;                   // PWM/(°C*с): устранение постоянного недогрева; обычно не менять.

// Менять следующие параметры при замене NTC, резисторов делителей или после калибровки.
constexpr float NTC_R25 = 10000.0f;           // Ом: сопротивление NTC при 25 °C, взять из его документации.
constexpr float NTC_FIXED = 10000.0f;         // Ом: установленный R1; используется для расчёта сопротивления NTC.
constexpr float NTC_BETA = 3977.0f;           // K: коэффициент B датчика; определяет пересчёт сопротивления в °C.
constexpr float NTC_OFFSET_C = 0.0f;          // °C: прибавка к показанию; +1.0f повышает его на 1 °C.
// Поправку NTC_OFFSET_C определять сравнением с термометром при установившейся температуре.
constexpr float ADC_REF_V = 5.0f;            // В: опорное напряжение АЦП, здесь питание Nano; влияет на пороги A1.
constexpr float A1_TOP_OHM = 220000.0f;       // Ом: установленный R6, верхнее плечо делителя самотеста.
constexpr float A1_BOTTOM_OHM = 10000.0f;     // Ом: установленный R7, нижнее плечо; оба номинала меняют пороги A1.
constexpr float A1_HIGH_RATIO = 0.35f;        // Доля расчётного A1: минимум для HIGH; обычно оставить 0.35f.
constexpr float A1_LOW_RATIO = 0.08f;         // Доля расчётного A1: максимум для LOW; обычно оставить 0.08f.
// Доли задают допуски самотеста, а не точность стабилизации питания. Пороги ниже вычисляются автоматически.
// ==========================================================

constexpr float NOMINAL_CURRENT_A = SUPPLY_V / HEATER_R_REF_OHM;
constexpr float NOMINAL_POWER_W = SUPPLY_V * NOMINAL_CURRENT_A;
constexpr float A1_EXPECTED_HIGH_ADC = SUPPLY_V * A1_BOTTOM_OHM /
    (A1_TOP_OHM + A1_BOTTOM_OHM) / ADC_REF_V * 1023.0f;
constexpr uint16_t DRAIN_HIGH_MIN =
    static_cast<uint16_t>(A1_EXPECTED_HIGH_ADC * A1_HIGH_RATIO + 0.5f);
constexpr uint16_t DRAIN_LOW_MAX =
    static_cast<uint16_t>(A1_EXPECTED_HIGH_ADC * A1_LOW_RATIO + 0.5f);

constexpr uint8_t PIN_PWM = 9, PIN_ENABLE = 8, PIN_BUTTON = 2;
constexpr uint8_t PIN_NTC = A0, PIN_DRAIN = A1, PIN_GREEN = 5, PIN_RED = 6;
constexpr uint8_t PIN_BOARD_LED = LED_BUILTIN; // Встроенный L на Nano V3, D13; оставить свободным от внешних цепей.
constexpr uint32_t CONTROL_MS = 250, DEBOUNCE_MS = 35, BLINK_MS = 125;
constexpr uint32_t READY_PERIOD_MS = 2000, READY_PULSE_MS = 80; // Редкая вспышка L: питание есть, режим OFF.
constexpr uint16_t FAULT_ON_MS = 120, FAULT_OFF_MS = 180, FAULT_GAP_MS = 1800; // Серии L: вспышка/интервал/пауза.
constexpr uint32_t FAULT_RESET_MS = 5000;
constexpr uint16_t SETTLE_US = 600;

static_assert(SUPPLY_V > 0.0f && HEATER_R_REF_OHM > 0.0f,
              "Invalid heater electrical parameters");
static_assert(NOMINAL_CURRENT_A <= DESIGN_CURRENT_LIMIT_A &&
              NOMINAL_POWER_W <= DESIGN_POWER_LIMIT_W,
              "Calculated cold load exceeds configured hardware limits");
static_assert(TARGET_C < BOOST_EXIT_C && BOOST_EXIT_C < FORCE_OFF_C &&
              FORCE_OFF_C < SOFT_OFF_C && SOFT_RESET_C < SOFT_OFF_C &&
              SOFT_OFF_C < EMERGENCY_C, "Invalid temperature limits");
static_assert(A1_EXPECTED_HIGH_ADC < 1020.0f &&
              DRAIN_LOW_MAX < DRAIN_HIGH_MIN, "Invalid A1 divider or thresholds");

enum class Mode : uint8_t { Off, Boost, Hold, Fault };
enum class Fault : uint8_t {
  None = 0, Stored = 1, Overheat = 2, SupplyOrHeater = 3, Sensor = 4,
  Q1 = 5, Q2 = 6, Conduction = 7, Watchdog = 8, Brownout = 9, Eeprom = 10
};
Mode mode = Mode::Off;
volatile Fault faultCode = Fault::None;  // Код серии L; хранится в EEPROM, обновляется при повторном тесте.
uint32_t faultLedStartedAt = 0;
bool faultRetryInProgress = false; // Только ручной повторный тест может заменить уже сохранённую причину.
bool softCutoff = false;
bool powerStageReady = false;  // Полный самотест пройден; на тёплом старте отложен.
float temperatureC = NAN, filteredC = NAN, integral = 0.0f;
uint32_t boostStartedAt = 0, lastControlAt = 0, buttonChangedAt = 0;
uint32_t faultPressAt = 0;
bool buttonRaw = HIGH, buttonStable = HIGH, buttonArmed = false;
bool faultResetHeld = false;

// Совместимое расширение: прежние байты 0..2; код и инверсия в 3..4. Запись с маркером последним.
constexpr int EE_MAGIC_ADDR = 0, EE_STATE_ADDR = 1, EE_INVERSE_ADDR = 2;
constexpr int EE_REASON_ADDR = 3, EE_REASON_INVERSE_ADDR = 4;
constexpr uint8_t EE_MAGIC = 0xB1, EE_OFF = 0x3C, EE_ON = 0xC3, EE_FAULT = 0x5A;
Fault loadFaultReason();
void tripFault(Fault reason);

bool heatingRequested() {
  return mode == Mode::Boost || mode == Mode::Hold;
}

void pwmWrite(uint8_t value) {
  analogWrite(PIN_PWM, value);
}

void gatesOff() {
  // Q2 закрывается первым: пробитый Q1 не мешает отключению нагрузки.
  digitalWrite(PIN_ENABLE, LOW);
  pwmWrite(0);
}

uint8_t loadState() {
  const uint8_t magic = EEPROM.read(EE_MAGIC_ADDR);
  const uint8_t state = EEPROM.read(EE_STATE_ADDR);
  const uint8_t inverse = EEPROM.read(EE_INVERSE_ADDR);
  // Чистая EEPROM и миграция старого формата всегда начинают с OFF.
  if ((magic == 0xFF && state == 0xFF && inverse == 0xFF) || magic == 0xA5)
    return EE_OFF;
  const bool validState = state == EE_OFF || state == EE_ON || state == EE_FAULT;
  if (magic != EE_MAGIC || !validState || inverse != static_cast<uint8_t>(~state))
    return 0;  // Повреждённая или оборванная запись; не восстанавливать ON.
  return state;
}

void saveState(uint8_t state) {
  const uint8_t reason = state == EE_FAULT ? static_cast<uint8_t>(faultCode) : 0;
  if (EEPROM.read(EE_MAGIC_ADDR) == EE_MAGIC &&
      EEPROM.read(EE_STATE_ADDR) == state &&
      EEPROM.read(EE_INVERSE_ADDR) == static_cast<uint8_t>(~state) &&
      (state != EE_FAULT || (EEPROM.read(EE_REASON_ADDR) == reason &&
       EEPROM.read(EE_REASON_INVERSE_ADDR) == static_cast<uint8_t>(~reason)))) return;
  EEPROM.update(EE_MAGIC_ADDR, 0);  // Сначала запретить чтение старой команды.
  EEPROM.update(EE_STATE_ADDR, state);
  EEPROM.update(EE_INVERSE_ADDR, static_cast<uint8_t>(~state));
  EEPROM.update(EE_REASON_ADDR, reason);
  EEPROM.update(EE_REASON_INVERSE_ADDR, static_cast<uint8_t>(~reason));
  EEPROM.update(EE_MAGIC_ADDR, EE_MAGIC);  // Маркер действительности пишется последним.
  eeprom_busy_wait();  // Команда считается сохранённой только после записи.
}

Fault loadFaultReason() {
  const uint8_t reason = EEPROM.read(EE_REASON_ADDR);
  if (reason < static_cast<uint8_t>(Fault::Stored) || reason > static_cast<uint8_t>(Fault::Eeprom) ||
      EEPROM.read(EE_REASON_INVERSE_ADDR) != static_cast<uint8_t>(~reason)) return Fault::Stored;
  return static_cast<Fault>(reason);
}

void tripFault(Fault reason) {
  gatesOff();
  if (mode == Mode::Fault && (!faultRetryInProgress || faultCode == reason)) return;
  mode = Mode::Fault;
  faultCode = reason;
  faultLedStartedAt = millis(); // Каждая новая причина начинает полную серию с первой вспышки.
  integral = 0.0f;
  softCutoff = false;
  powerStageReady = false;
  saveState(EE_FAULT);
}

uint16_t readAdc(uint8_t pin, uint8_t samples) {
  analogRead(pin);  // Первая конверсия после переключения мультиплексора отбрасывается.
  uint16_t sum = 0;
  for (uint8_t i = 0; i < samples; ++i) sum += static_cast<uint16_t>(analogRead(pin));
  return static_cast<uint16_t>((sum + samples / 2U) / samples);
}

bool sampleTemperature() {
  const uint16_t adc = readAdc(PIN_NTC, 16);
  if (adc <= 2 || adc >= 1021) { tripFault(Fault::Sensor); return false; }
  const float resistance = NTC_FIXED * adc / (1023.0f - adc);
  const float logRatio = static_cast<float>(logf(resistance / NTC_R25));
  const float inverseK = 1.0f / 298.15f + logRatio / NTC_BETA;
  const float value = 1.0f / inverseK - 273.15f + NTC_OFFSET_C;
  if (!isfinite(value) || value < -45.0f || value > 125.0f) {
    tripFault(Fault::Sensor); return false;
  }
  temperatureC = value;
  if (value >= EMERGENCY_C) { tripFault(Fault::Overheat); return false; }
  filteredC = isnan(filteredC) ? value : filteredC + 0.25f * (value - filteredC);
  if (value >= SOFT_OFF_C) softCutoff = true;
  else if (value <= SOFT_RESET_C) softCutoff = false;
  return true;
}

bool drainIsHigh() {
  delayMicroseconds(SETTLE_US);
  return readAdc(PIN_DRAIN, 4) >= DRAIN_HIGH_MIN;
}

bool checkPowerStage(bool testConduction) {
  // На входе и при любом выходе оба ключа закрыты; старый PWM не восстанавливается.
  gatesOff();
  if (!drainIsHigh()) { tripFault(Fault::SupplyOrHeater); return false; }
  digitalWrite(PIN_ENABLE, HIGH);  // Только Q2: исправный Q1 удерживает Drain высоким.
  const bool q1Ok = drainIsHigh();
  gatesOff();
  if (!q1Ok) { tripFault(Fault::Q1); return false; }
  delayMicroseconds(SETTLE_US);
  pwmWrite(255);  // Только Q1: исправный Q2 разрывает путь к GND.
  const bool q2Ok = drainIsHigh();
  gatesOff();
  if (!q2Ok) { tripFault(Fault::Q2); return false; }
  if (testConduction) {
    digitalWrite(PIN_ENABLE, HIGH);
    pwmWrite(255);  // Короткий импульс проверяет замыкание пути, но не измеряет ток.
    delayMicroseconds(SETTLE_US);
    const uint16_t adc = readAdc(PIN_DRAIN, 4);
    gatesOff();
    if (adc > DRAIN_LOW_MAX) { tripFault(Fault::Conduction); return false; }
    powerStageReady = true;
  }
  return true;
}

bool mustStopHeat() {
  // Текущая температура также ограничивает выход: фильтр не задерживает отключение.
  return softCutoff || temperatureC >= FORCE_OFF_C || filteredC >= FORCE_OFF_C;
}

void startHeating() {
  integral = 0.0f;
  boostStartedAt = millis();
  mode = temperatureC < BOOST_EXIT_C && filteredC < BOOST_EXIT_C ? Mode::Boost : Mode::Hold;
}

void stopHeating() {
  gatesOff();
  mode = Mode::Off;
  integral = 0.0f;
  // Гистерезис температурной блокировки не сбрасывается кнопкой.
}

uint8_t holdPwm(float dtSeconds) {
  // Использовать более горячее из текущего и фильтрованного значений: после
  // завершения BOOST не продолжать нагрев из-за запаздывания фильтра.
  const float controlC = max(temperatureC, filteredC);
  const float error = TARGET_C - controlC;
  const float proposed = constrain(integral + KI * error * dtSeconds, 0.0f,
                                   static_cast<float>(HOLD_MAX_PWM));
  const float output = KP * error + proposed;
  // Anti-windup: не накапливать интеграл дальше предела доступной мощности.
  if (!((output > HOLD_MAX_PWM && error > 0.0f) || (output < 0.0f && error < 0.0f)))
    integral = proposed;
  const float limited = constrain(KP * error + integral, 0.0f,
                                  static_cast<float>(HOLD_MAX_PWM));
  return static_cast<uint8_t>(limited + 0.5f);
}

void controlStep(uint32_t now, float dtSeconds) {
  if (mode == Mode::Fault) { gatesOff(); return; }
  if (!sampleTemperature()) return;
  if (mustStopHeat()) {
    gatesOff();  // До любых тестовых переключений; никаких импульсов выше порога.
    integral = 0.0f;
    if (mode == Mode::Boost) mode = Mode::Hold;
    return;
  }
  if (!checkPowerStage(!powerStageReady)) return;
  if (!heatingRequested()) return;
  if (mode == Mode::Boost && (temperatureC >= BOOST_EXIT_C ||
      filteredC >= BOOST_EXIT_C || static_cast<uint32_t>(now - boostStartedAt) >= BOOST_MAX_MS)) {
    mode = Mode::Hold;
    integral = 0.0f;
  }
  const uint8_t pwm = mode == Mode::Boost ? BOOST_PWM : holdPwm(dtSeconds);
  if (pwm != 0) { digitalWrite(PIN_ENABLE, HIGH); pwmWrite(pwm); }
}

void onButtonPress() {
  if (mode == Mode::Fault) return;
  if (heatingRequested()) { stopHeating(); saveState(EE_OFF); return; }
  gatesOff();
  if (!sampleTemperature()) return;  // Не использовать устаревший отсчёт.
  if (mustStopHeat()) powerStageReady = false;  // Принять ON без тока, проверить после остывания.
  else if (!checkPowerStage(true)) return;
  saveState(EE_ON);  // Запись завершена до разрешения рабочего нагрева.
  startHeating();
}

void tryClearFault() {
  gatesOff();
  filteredC = NAN;  // После ошибки оценивать свежую температуру, не старое значение фильтра.
  faultRetryInProgress = true;
  const bool measured = sampleTemperature();
  if (measured && mustStopHeat()) tripFault(Fault::Overheat); // Температура пока не разрешает сброс.
  const bool passed = measured && !mustStopHeat() && checkPowerStage(true);
  faultRetryInProgress = false;
  if (!passed) return; // Неисправность осталась: FAULT, с актуальным кодом неуспешного теста.
  saveState(EE_OFF);  // Сначала завершить запись; ни одного автоматического ON.
  stopHeating();
  faultCode = Fault::None;
}

void serviceButton(uint32_t now) {
  const bool raw = digitalRead(PIN_BUTTON) != LOW;
  if (raw != buttonRaw) { buttonRaw = raw; buttonChangedAt = now; }
  if (static_cast<uint32_t>(now - buttonChangedAt) < DEBOUNCE_MS) return;
  if (buttonStable == buttonRaw) {
    if (buttonStable == HIGH) buttonArmed = true;
    if (faultResetHeld && static_cast<uint32_t>(now - faultPressAt) >= FAULT_RESET_MS) {
      faultResetHeld = false;  // Одна попытка на удержание, даже если самотест не пройден.
      tryClearFault();
    }
    return;
  }
  buttonStable = buttonRaw;
  if (buttonStable == HIGH) { buttonArmed = true; faultResetHeld = false; }
  else if (buttonArmed) {
    buttonArmed = false;
    faultResetHeld = mode == Mode::Fault;  // Начать удержание заново уже после ошибки.
    faultPressAt = now;
    if (!faultResetHeld) onButtonPress();
  }
}

void updateLed(uint32_t now) {
  digitalWrite(PIN_GREEN, mode == Mode::Off ? HIGH : LOW);
  const bool red = mode == Mode::Fault ? (now / BLINK_MS) % 2U != 0 : heatingRequested();
  digitalWrite(PIN_RED, red ? HIGH : LOW);
  bool boardLed = heatingRequested() || now % READY_PERIOD_MS < READY_PULSE_MS;
  if (mode == Mode::Fault) {
    const uint8_t count = static_cast<uint8_t>(faultCode); // Fault 1..10 = число вспышек в серии.
    const uint32_t pulsePeriod = FAULT_ON_MS + FAULT_OFF_MS;
    const uint32_t active = static_cast<uint32_t>(count) * pulsePeriod;
    const uint32_t phase = static_cast<uint32_t>(now - faultLedStartedAt) % (active + FAULT_GAP_MS);
    boardLed = phase < active && phase % pulsePeriod < FAULT_ON_MS;
  }
  digitalWrite(PIN_BOARD_LED, boardLed ? HIGH : LOW); // Без delay: кнопка и защита продолжают обслуживаться.
}

void setup() {
  digitalWrite(PIN_PWM, LOW); pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW); pinMode(PIN_ENABLE, OUTPUT);
  gatesOff();
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_GREEN, OUTPUT); pinMode(PIN_RED, OUTPUT);
  digitalWrite(PIN_GREEN, LOW); digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_BOARD_LED, LOW); pinMode(PIN_BOARD_LED, OUTPUT);
  analogReference(DEFAULT);
  wdt_enable(WDTO_1S);  // Защищает и стартовые проверки, не только loop().
  buttonRaw = digitalRead(PIN_BUTTON) != LOW;
  buttonStable = buttonRaw;
  buttonChangedAt = millis();
  const uint8_t saved = loadState();
  const uint8_t known = _BV(PORF) | _BV(EXTRF) | _BV(BORF) | _BV(WDRF);
  const bool knownReset = resetCause != 0 && (resetCause & static_cast<uint8_t>(~known)) == 0;
  const bool powerOn = knownReset && (resetCause & _BV(PORF)) != 0 &&
                       (resetCause & (_BV(EXTRF) | _BV(WDRF))) == 0;
  if (saved == EE_FAULT) tripFault(loadFaultReason()); // Сохранённая причина важнее следующего сброса.
  else if (knownReset && (resetCause & _BV(WDRF)) != 0) tripFault(Fault::Watchdog);
  else if (knownReset && (resetCause & _BV(BORF)) != 0 && !powerOn) tripFault(Fault::Brownout);
  else if (saved == 0) tripFault(Fault::Eeprom);
  else if (sampleTemperature()) {
    // Тёплый старт не стирает ON; полный тест обязателен перед первым рабочим PWM.
    if (mustStopHeat() || checkPowerStage(true)) {
      if (saved == EE_ON) startHeating();
      else { stopHeating(); saveState(EE_OFF); }
    }
  }
  lastControlAt = millis();
  updateLed(lastControlAt);
  wdt_reset();
}

void loop() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - lastControlAt;  // Корректно при переполнении millis().
  if (elapsed >= CONTROL_MS) {
    lastControlAt = now;
    controlStep(now, min(elapsed, 1000UL) / 1000.0f);
  }
  serviceButton(millis());
  updateLed(millis());
  wdt_reset();  // Только после выполнения всего прохода; внутри ожиданий не кормить.
}
