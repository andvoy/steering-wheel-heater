const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const {Sim,F,OFF,HEATING,FAULT,saved,legacyFault,avr,WD_RESET,hexSha256} = require('./avr-model.cjs');
const results=[];
function test(name,fn){fn();results.push({name,status:'PASS'});console.log('PASS '+name);}
for(let code=1;code<=10;code++)test('legacy stored fault '+code+' migrates to OFF after healthy startup',()=>{
 const s=new Sim({eeprom:legacyFault(code)}).run(400);
 assert.equal(s.mode,OFF);assert.equal(s.fault,0);s.safe();assert.equal(s.eeprom.memory[1],0x3C);
});
test('startup retest reports present cause rather than stored cause',()=>{
 const s=new Sim({eeprom:legacyFault(F.Conduction),ntc:1023}).run(400);s.expectFault(F.Sensor);
});
test('USB before 42 V: fault 3 -> stable supply -> OFF, no working heat',()=>{
 const s=new Sim({supply:false,eeprom:saved(0xC3)}).run(1500);s.expectFault(F.Supply);
 s.supply=true;s.run(700);s.expectFault(F.Supply);
 s.run(800);assert.equal(s.mode,OFF);assert.equal(s.fault,0);s.safe();
 s.press();assert.equal(s.mode,HEATING);s.run(5000);assert.equal(s.pwm,126);
});
test('supply chatter restarts the continuous one-second recovery interval',()=>{
 const s=new Sim({supply:false}).run(500);s.supply=true;s.run(650);
 s.supply=false;s.run(300);s.supply=true;s.run(700);s.expectFault(F.Supply);
 s.run(800);assert.equal(s.mode,OFF);s.safe();
});
test('supply recovery detects shorted Q1 and stays faulted after repair',()=>{
 const s=new Sim({supply:false,q1Short:true}).run(400);s.expectFault(F.Supply);
 s.supply=true;s.run(1600);s.expectFault(F.Q1);
 s.q1Short=false;s.run(1600);s.expectFault(F.Q1);
});
test('healthy saved ON resumes normal heating above zero; button OFF persists',()=>{
 const s=new Sim({eeprom:saved(0xC3)}).run(5000);assert.equal(s.mode,HEATING);assert.equal(s.pwm,126);
 s.press();assert.equal(s.mode,OFF);s.safe();assert.equal(s.eeprom.memory[1],0x3C);
});
test('saved OFF and erased EEPROM do not start working heat',()=>{
 for(const eeprom of [saved(0x3C),undefined]){const s=new Sim({eeprom}).run(1200);assert.equal(s.mode,OFF);s.safe();}
});
test('above target HEATING stops heat, below target resumes',()=>{
 const s=new Sim({eeprom:saved(0xC3)}).run(400);s.temp=37;s.run(400);
 assert.equal(s.mode,HEATING);s.safe();s.temp=30;s.run(2800);assert.ok(s.pwm>0&&s.pwm<=100);
});
test('40 C emergency closes both keys and stays latched after cooling',()=>{
 const s=new Sim({temp:41,eeprom:saved(0xC3)}).run(400);s.expectFault(F.Overheat);assert.equal(s.loadPulses,0);
 s.temp=25;s.run(1500);s.expectFault(F.Overheat);assert.equal(s.loadPulses,0);
});
for(const [options,code] of [[{q1Short:true},F.Q1],[{q2Short:true},F.Q2],
 [{q1Open:true},F.Conduction],[{q2Open:true},F.Conduction],[{ntc:0},F.Sensor],
 [{ntc:1023},F.Sensor],[{heater:false},F.Supply]])test('fault detection '+JSON.stringify(options),()=>{
 new Sim(options).run(400).expectFault(code);
});
test('sensor repair needs manual hold or reset, not auto recovery',()=>{
 const s=new Sim({ntc:1023}).run(400);s.expectFault(F.Sensor);s.ntc=null;s.run(500);s.expectFault(F.Sensor);
 s.press().run(4700);s.expectFault(F.Sensor);s.run(400);assert.equal(s.mode,OFF);s.safe();
});
test('readable telemetry: one line, blank separator, percent, no raw diagnostics',()=>{
 const s=new Sim({eeprom:saved(0xC3)}).run(5300);
 assert.match(s.serialText,/Модуль подогрева 1.9\r\n\r\n/);
 assert.match(s.serialText,/Режим: НАГРЕВ \| Температура: 2[45]\.\d °C \| Нагрев: 49% \| Питание: ~42\.0 В \| Мощность расч\.: ~44\.9 Вт \| Ошибок нет\r\n\r\n/);
 assert.doesNotMatch(s.serialText,/A0=|A1=|PWM_raw=|TEST_ADC/);
 s.temp=37;s.run(1300);assert.match(s.serialText,/Режим: ПОДДЕРЖАНИЕ \| Температура: 3[67]\.\d °C \| Нагрев: 0% \| Питание: ~42\.0 В \| Мощность расч\.: 0\.0 Вт \| Ошибок нет\r\n\r\n/);
});
test('invalid NTC shown as nan, not old healthy temperature',()=>{
 const s=new Sim().run(400);s.ntc=1023;s.run(1000);s.expectFault(F.Sensor);
 assert.match(s.serialText,/Режим: АВАРИЯ \| Температура: нет достоверных данных \| Нагрев: 0% \| Питание: [^\r\n]+ \| Мощность расч\.: 0\.0 Вт \| Ошибка 4: неисправность датчика температуры\r\n\r\n/);
});
test('full-power region joins temperature regulation without a power discontinuity',()=>{
 const s=new Sim({temp:-10,eeprom:saved(0xC3)}).run(9000);let previous=s.pwm;
 assert.equal(previous,255);
 for(const t of [-2.8,-2.0,-1.2,-0.5]){
  s.temp=t;s.run(600);assert.equal(s.mode,HEATING);assert.equal(s.pwm,255);previous=s.pwm;
 }
 s.temp=0.2;s.run(600);assert.equal(s.mode,HEATING);assert.ok(s.pwm>=250&&s.pwm<=255);
});
test('fresh temperature resumes HEATING promptly after cooling',()=>{
 const s=new Sim({temp:35,eeprom:saved(0xC3)}).run(400);assert.equal(s.mode,HEATING);s.safe();
 s.temp=31.5;s.run(300);assert.ok(s.pwm>0);
});
for(const t of [-20,-5,1,20,31.8,35,41])test('trip restart at '+t+' C respects saved request and temperature',()=>{
 const on=new Sim({temp:t,eeprom:saved(0xC3)}).run(450);
 assert.equal(on.mode,t>=40?FAULT:HEATING);
 if(t>=32)on.safe();else assert.ok(on.pwm>0&&on.pwm<=16);
 const restarted=new Sim({temp:t,eeprom:on.eeprom.memory.slice()}).run(450);
 assert.equal(restarted.mode,t>=40?FAULT:HEATING);
 if(t>=32)restarted.safe();else assert.ok(restarted.pwm<=16);
 const off=new Sim({temp:t,eeprom:saved(0x3C)}).run(450);assert.equal(off.mode,t>=40?FAULT:OFF);off.safe();
});
test('cold OFF waits for button; OFF is retained for the next trip',()=>{
 const s=new Sim({temp:-15,eeprom:saved(0x3C)}).run(400);s.safe();s.press().run(400);
 assert.equal(s.mode,HEATING);s.release().press();assert.equal(s.mode,OFF);s.safe();
 const next=new Sim({temp:-15,eeprom:s.eeprom.memory.slice()}).run(400);assert.equal(next.mode,OFF);next.safe();
});
test('soft rise reaches normal cap, button OFF and emergency are not ramped down',()=>{
 const s=new Sim({temp:15,eeprom:saved(0xC3)}).run(400);assert.ok(s.pwm>0&&s.pwm<=16);
 s.run(6500);assert.ok(s.pwm>=177&&s.pwm<=179);s.press();s.safe();s.release().press().run(600);assert.ok(s.pwm>0);
 s.temp=50;s.run(300);s.expectFault(F.Overheat);
});
test('temperature alone selects output across a ten-minute uptime change',()=>{
 const s=new Sim({temp:-15,eeprom:saved(0xC3)}).run(400);assert.equal(s.mode,HEATING);
 s.jump(600100).run(400);assert.equal(s.mode,HEATING);assert.ok(s.pwm>0);
 s.temp=0.2;s.run(400);assert.equal(s.mode,HEATING);assert.ok(s.pwm>0);
 s.temp=-1;s.run(400);assert.equal(s.mode,HEATING);
});
test('hot restart faults; cooled manual recovery leaves OFF, then button starts HEATING',()=>{
 const s=new Sim({temp:41,eeprom:saved(0xC3)}).run(400);s.expectFault(F.Overheat);
 s.temp=31;s.run(500);s.expectFault(F.Overheat);s.press().run(5100);assert.equal(s.mode,OFF);s.safe();
 s.release().press().run(1000);assert.equal(s.mode,HEATING);assert.ok(s.pwm>0&&s.pwm<=100);
});
test('telemetry does not change remembered ON/OFF',()=>{
 const s=new Sim({temp:20,eeprom:saved(0xC3)}).run(400);const count=s.writes.length;
 s.run(1800);assert.equal(s.writes.length,count);assert.equal(s.eeprom.memory[1],0xC3);
});
for(const [temp,expected] of [[2,245],[10,203],[20,152],[29.3,104]])test('normal heating curve at '+temp+' C',()=>{
 const s=new Sim({temp,eeprom:saved(0xC3)}).run(9000);
 assert.equal(s.mode,HEATING);assert.ok(Math.abs(s.pwm-expected)<=1,`${temp}: PWM=${s.pwm}, expected ~${expected}`);
});
test('near target PI requests less than warm ceiling; at target no forced minimum',()=>{
 const near=new Sim({temp:31.8,eeprom:saved(0xC3)}).run(4000);assert.ok(near.pwm<30);
 const above=new Sim({temp:33,eeprom:saved(0xC3)}).run(4000);above.safe();
});
test('temperature jump lowers ceiling immediately without a downward ramp',()=>{
 const s=new Sim({temp:-10,eeprom:saved(0xC3)}).run(9000);assert.equal(s.pwm,255);
 s.temp=29.3;s.run(300);assert.equal(s.mode,HEATING);assert.ok(s.pwm<=105);
});
test('startup prints version and live status; EEPROM retains only ON/OFF',()=>{
 const on=new Sim({eeprom:saved(0xC3),reset:2}).run(1400);
 assert.match(on.serialText,/Модуль подогрева 1.9/);assert.equal(on.mode,HEATING);
 assert.doesNotMatch(on.serialText,/MCUSR|Причина сброса|Optiboot/);
 const stale=new Sim({eeprom:legacyFault(F.Supply)}).run(400);assert.equal(stale.mode,OFF);stale.safe();
});
test('voltage estimate uses OFF sample, never the low Drain during PWM',()=>{
 const s=new Sim({temp:-10,eeprom:saved(0xC3)}).run(10500);assert.equal(s.pwm,255);
 assert.match(s.serialText,/Нагрев: 100% \| Питание: ~42\.0 В \| Мощность расч\.: ~90\.8 Вт/);
});
test('voltage changes affect power quadratically using actual PWM',()=>{
 const s=new Sim({temp:-10,eeprom:saved(0xC3)});
 Object.defineProperty(s,'sense',{get(){return s.portB.pinState(1)!==avr.PinState.High && s.portB.pinState(0)!==avr.PinState.High?320:null;}});
 s.run(10500);assert.equal(s.pwm,255);
 assert.match(s.serialText,/Нагрев: 100% \| Питание: ~36\.0 В \| Мощность расч\.: ~66\.5 Вт/);
});
test('hot start has no fabricated supply estimate or new diagnostic heat pulse',()=>{
 const s=new Sim({temp:41,eeprom:saved(0xC3)}).run(1400);s.safe();assert.equal(s.loadPulses,0);
 assert.match(s.serialText,/Питание: нет данных \| Мощность расч\.: 0\.0 Вт/);
});
test('stale voltage is not presented as a current measurement',()=>{
 const s=new Sim({eeprom:saved(0xC3)}).run(500);s.temp=41;s.run(3000);s.safe();
 const latest=s.serialText.trim().split(/\r?\n/).filter(Boolean).pop();
 assert.match(latest,/Питание: нет данных \| Мощность расч\.: 0\.0 Вт/);
});
test('missing power or open heater does not masquerade as a supply measurement',()=>{
 for(const options of [{supply:false},{heater:false}]){
  const s=new Sim(options).run(1400);s.expectFault(F.Supply);
  assert.match(s.serialText,/Питание: нет данных \| Мощность расч\.: 0\.0 Вт/);
 }
});
test('both red LEDs: 300 ms ON, 300 ms OFF, exactly 2 s between series',()=>{
 const s=new Sim({heater:false}).run(400);s.expectFault(F.Supply);
 const origin=s.u32('faultLedStartedAt');
 // Allow UART output to finish before observing LEDs; stay clear of pulse edges.
 for(const [offset,on] of [[50,true],[250,true],[350,false],[550,false],
  [650,true],[950,false],[1250,true],[1450,true],[1550,false],[3450,false],[3550,true]]){
  s.u32('timer0_millis',origin+offset);s.u32('lastControlAt',origin+offset);s.run(40);
  assert.equal(s.boardLed,on,'L at '+offset);
  assert.equal(s.portD.pinState(6)===avr.PinState.High,on,'D6 at '+offset);
 }
});
test('reset flags leave normal self-test and saved ON/OFF behaviour unchanged',()=>{
 for(const reset of [0,1,2,4,8]){
  const s=new Sim({reset,eeprom:saved(0xC3)}).run(400);assert.equal(s.mode,HEATING);assert.equal(s.fault,0);
 }
});
test('running overheat stops at the next control sample and keeps both keys off',()=>{
 const s=new Sim({temp:20,eeprom:saved(0xC3)}).run(1500);assert.ok(s.pwm>0);
 s.temp=40.5;s.run(300);s.expectFault(F.Overheat);const pulses=s.loadPulses;
 s.run(500);s.expectFault(F.Overheat);assert.equal(s.loadPulses,pulses);
 s.temp=25;s.run(800);s.expectFault(F.Overheat);assert.equal(s.loadPulses,pulses);
});
test('hot manual reset stays locked and produces no conduction pulse',()=>{
 const s=new Sim({temp:41,eeprom:saved(0xC3)}).run(400);s.expectFault(F.Overheat);
 s.press().run(5100);s.expectFault(F.Overheat);assert.equal(s.loadPulses,0);
});
test('after an overheat fault, a healthy restart stays OFF and waits for button',()=>{
 const s=new Sim({temp:41,eeprom:saved(0xC3)}).run(400);s.expectFault(F.Overheat);
 const next=new Sim({temp:25,eeprom:s.eeprom.memory.slice()}).run(400);assert.equal(next.mode,OFF);next.safe();
 next.press().run(400);assert.equal(next.mode,HEATING);assert.ok(next.pwm>0);
});
test('fault handling writes ON/OFF bytes and leaves old reason bytes untouched',()=>{
 const memory=new Uint8Array(1024).fill(255);memory.set(saved(0xC3));memory[3]=0x12;memory[4]=0x34;
 const s=new Sim({temp:41,eeprom:memory}).run(400);s.expectFault(F.Overheat);
 assert.equal(s.eeprom.memory[3],0x12);assert.equal(s.eeprom.memory[4],0x34);
});
test('corrupt state starts OFF after a healthy self-test',()=>{
 const s=new Sim({eeprom:Uint8Array.from([0xB1,0xC3,0])}).run(400);assert.equal(s.mode,OFF);s.safe();
});
test('source keeps comments in settings and removes obsolete temperature/reset branches',()=>{
 const source=fs.readFileSync(path.join(__dirname,'../firmware/steering_wheel_heater_v1_9/steering_wheel_heater_v1_9.ino'),'utf8');
 const before=source.slice(0,source.indexOf('// Настройки'));
 const after=source.slice(source.indexOf('constexpr float NOMINAL_CURRENT_A'));
 assert.doesNotMatch(before+after,/\/\/|\/\*/);
 assert.doesNotMatch(source,/OPTIBOOT|resetCause|EE_REASON|loadFaultReason|FORCE_OFF_C|SOFT_OFF_C|SOFT_RESET_C|softCutoff|filteredC|powerStageReady|[Bb][Oo][Oo][Ss][Tt]|holdPwm|Mode::Hold/);
 assert.match(source,/EMERGENCY_C\s*=\s*40\.0f/);
});
test('button ON checks fresh overtemperature before a conduction test',()=>{
 const s=new Sim({temp:25}).run(400);const pulses=s.loadPulses;
 s.temp=40.5;s.press();s.expectFault(F.Overheat);assert.equal(s.loadPulses,pulses);
});
test('39.5 C remains below the emergency boundary with normal HEATING regulation',()=>{
 const s=new Sim({temp:39.5,eeprom:saved(0xC3)}).run(400);assert.equal(s.mode,HEATING);assert.equal(s.fault,0);s.safe();
});
test('display shows heating at 26.3 C and maintaining only near the target',()=>{
 const s=new Sim({temp:26.3,eeprom:saved(0xC3)}).run(1400);
 assert.match(s.serialText,/Режим: НАГРЕВ \| Температура: 26\.3/);
 assert.doesNotMatch(s.serialText,/Режим: ПОДДЕРЖАНИЕ/);
 s.temp=31.5;s.run(1300);assert.equal(s.mode,HEATING);
 assert.match(s.serialText,/Режим: ПОДДЕРЖАНИЕ \| Температура: 31\.5/);
 s.temp=30;s.run(1300);assert.equal(s.mode,HEATING);
 assert.match(s.serialText,/Режим: НАГРЕВ \| Температура: 30\.0/);
});
test('cold full output survives uptime changes and returns after cooling',()=>{
 const s=new Sim({temp:-5,eeprom:saved(0xC3)}).run(9000);assert.equal(s.mode,HEATING);assert.equal(s.pwm,255);
 s.jump(1800000).run(400);assert.equal(s.mode,HEATING);assert.equal(s.pwm,255);
 s.temp=20;s.run(400);assert.ok(s.pwm>=151&&s.pwm<=153);
 s.temp=-5;s.run(4000);assert.equal(s.pwm,255);assert.equal(s.mode,HEATING);
});
test('stuck ADC still trips watchdog',()=>{
 const s=new Sim({adcHang:true});assert.throws(()=>s.run(1500),e=>e===WD_RESET);
});
test('millis rollover: control and telemetry continue without unsafe output',()=>{
 const s=new Sim().run(400);s.u32('timer0_millis',0xffffff00);s.u32('lastControlAt',0xffffff00);
 s.run(1400);assert.equal(s.mode,OFF);s.safe();assert.match(s.serialText,/Режим: ВЫКЛЮЧЕН/);
});
console.log('TOTAL '+results.length+' PASS');
if(process.env.REPORT_PATH)fs.writeFileSync(process.env.REPORT_PATH,JSON.stringify({firmware:'1.9',hexSha256,scope:'compiled AVR binary; ideal switches and steady ADC model, not a bench test',tests:results},null,2));
