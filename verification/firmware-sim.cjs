// Executes the compiled AVR application, not a translated copy of its control logic.
// Model scope: ideal switches, steady ADC values, button, EEPROM, timers and watchdog.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {execFileSync} = require('node:child_process');
const {createHash} = require('node:crypto');
const avr = require('avr8js');

const build = path.resolve(process.argv[2] || 'tmp/firmware-1.3');
const name = 'steering_wheel_heater_v1_3.ino';
const avrBin = process.env.AVR_BIN;
assert.ok(avrBin, 'Set AVR_BIN to the bin directory of the AVR toolchain used for compilation.');
const symbols = {};
for (const line of execFileSync(path.join(avrBin,process.platform==='win32'?'avr-nm.exe':'avr-nm'),['-n','-C',path.join(build,name+'.elf')],{encoding:'utf8'}).split('\n')) {
  const m = line.match(/^([0-9a-f]+)\s+[bBdD]\s+(\S+)\s*$/);
  if (m) symbols[m[2]] = parseInt(m[1],16)-0x800000;
}
for (const key of ['mode','faultCode','timer0_millis','lastControlAt','boostStartedAt','integral','temperatureC','softCutoff','powerStageReady']) assert.ok(key in symbols,key);
const flash = new Uint8Array(32768);
for (const line of fs.readFileSync(path.join(build,name+'.hex'),'utf8').trim().split(/\r?\n/)) {
  const bytes = Buffer.from(line.slice(1),'hex');
  assert.equal(bytes.reduce((s,b)=>s+b,0)&255,0,'HEX checksum');
  if (bytes[3]===0) flash.set(bytes.subarray(4,4+bytes[0]),bytes.readUInt16BE(1));
}
const saved = state => Uint8Array.from([0xB1,state,state^255]);
const OFF=0, BOOST=1, HOLD=2, FAULT=3;
const F={Sensor:1,Overheat:2,Supply:3,Q1:4,Q2:5,Conduction:6,Watchdog:7,Brownout:8,Stored:9,Eeprom:10};
const SUPPLY_HIGH_ADC=374; // 42 V through 220k/10k divider at 5 V ADC reference.
const ntcAdc = resistance => Math.round(1023*resistance/(10000+resistance));
const WD_RESET = Symbol('watchdog reset');

class Sim {
  constructor(options={}) {
    Object.assign(this,{temp:25,supply:true,heater:true,q1Short:false,q2Short:false,q1Open:false,q2Open:false,sense:null,ntc:null,adcHang:false},options);
    this.cpu = new avr.CPU(new Uint16Array(flash.buffer.slice(0)),2048);
    this.cpu.data[0x54] = options.reset===undefined ? 1 : options.reset;
    if(options.bootFlags!==undefined)this.cpu.data[2]=options.bootFlags;
    this.clock = new avr.AVRClock(this.cpu,16000000);
    this.portB = new avr.AVRIOPort(this.cpu,avr.portBConfig);
    this.portD = new avr.AVRIOPort(this.cpu,avr.portDConfig);
    new avr.AVRIOPort(this.cpu,avr.portCConfig);
    new avr.AVRTimer(this.cpu,avr.timer0Config);
    new avr.AVRTimer(this.cpu,avr.timer1Config);
    new avr.AVRTimer(this.cpu,avr.timer2Config);
    this.portD.setPin(2,!(options.buttonLow));
    this.eeprom = new avr.EEPROMMemoryBackend(1024);
    if (options.eeprom) this.eeprom.memory.set(options.eeprom);
    this.writes=[];
    const write = this.eeprom.writeMemory.bind(this.eeprom);
    this.eeprom.writeMemory = (addr,value) => {write(addr,value);this.writes.push(this.eeprom.memory.slice());};
    new avr.AVREEPROM(this.cpu,this.eeprom);
    this.adc = new avr.AVRADC(this.cpu,avr.adcConfig);
    this.adc.onADCRead = input => {
      if (this.adcHang) return;
      let value;
      if (input.channel===0) {
        const r=10000*Math.exp(3977*(1/(273.15+this.temp)-1/298.15));
        value=this.ntc===null ? Math.round(1023*r/(10000+r)) : this.ntc;
      } else {
        const q1=this.q1Short || (!this.q1Open && this.portB.pinState(1)===avr.PinState.High);
        const q2=this.q2Short || (!this.q2Open && this.portB.pinState(0)===avr.PinState.High);
        value=this.sense===null ? (!this.supply || !this.heater || (q1&&q2) ? 0 : SUPPLY_HIGH_ADC) : this.sense;
      }
      this.cpu.addClockEvent(()=>this.adc.completeADCRead(value),this.adc.sampleCycles);
    };
    this.watchdog = new avr.AVRWatchdog(this.cpu,avr.watchdogConfig,this.clock);
    this.cpu.reset = () => { this.resetAt=this.cpu.cycles; throw WD_RESET; };
    this.loadPulses=0;
    this.portB.addListener(()=>{
      if (this.portB.pinState(0)===avr.PinState.High && this.portB.pinState(1)===avr.PinState.High) this.loadPulses++;
    });
  }
  run(ms) {
    const stop=this.cpu.cycles+ms*16000;
    while(this.cpu.cycles<stop) {avr.avrInstruction(this.cpu);this.cpu.tick();}
    return this;
  }
  byte(key) {return this.cpu.data[symbols[key]];}
  u32(key,value) {if(value===undefined)return this.cpu.dataView.getUint32(symbols[key],true);this.cpu.dataView.setUint32(symbols[key],value>>>0,true);}
  float(key) {return this.cpu.dataView.getFloat32(symbols[key],true);}
  get mode() {return this.byte('mode');}
  get fault() {return this.byte('faultCode');}
  get pwm() {return this.cpu.data[0x80]&0x80 ? this.cpu.data[0x88] : this.cpu.data[0x25]&2 ? 255 : 0;}
  get enabled() {return !!(this.cpu.data[0x25]&1);}
  press() {this.portD.setPin(2,false);return this.run(60);}
  release() {this.portD.setPin(2,true);return this.run(60);}
  jump(ms) {this.u32('timer0_millis',this.u32('timer0_millis')+ms);return this.run(80);}
  safe() {assert.equal(this.pwm,0);assert.equal(this.enabled,false);}
  expectFault(code) {assert.equal(this.mode,FAULT);assert.equal(this.fault,code);this.safe();assert.equal(this.eeprom.memory[1],0x5A);}
}
const results=[];
function test(label,fn){const start=Date.now();fn();results.push({test:label,status:'PASS'});console.log(`PASS ${label} (${Date.now()-start}ms)`);}

test('blank EEPROM: self-test, READY/OFF, green LED',()=>{
  const s=new Sim().run(100);assert.equal(s.mode,OFF);s.safe();assert.equal(s.portD.pinState(5),avr.PinState.High);assert.equal(s.eeprom.memory[1],0x3C);
});
test('stored ON restores cold BOOST after self-test',()=>{
  const s=new Sim({eeprom:saved(0xC3)}).run(330);assert.equal(s.mode,BOOST);assert.equal(s.pwm,255);assert.ok(s.enabled);
});
test('legacy EEPROM migrates to OFF',()=>{const s=new Sim({eeprom:Uint8Array.from([0xA5,1,0])}).run(100);assert.equal(s.mode,OFF);s.safe();});
test('external Reset restores saved ON only after self-test',()=>{const s=new Sim({reset:2,eeprom:saved(0xC3)}).run(330);assert.equal(s.mode,BOOST);assert.equal(s.byte('powerStageReady'),1);});
for (const reset of [0,0x80,0x81]) test(`unknown reset ${reset}: restores saved ON, button can switch OFF`,()=>{
  const s=new Sim({reset,eeprom:saved(0xC3)}).run(330);assert.equal(s.mode,BOOST);assert.equal(s.byte('powerStageReady'),1);s.press();assert.equal(s.mode,OFF);s.safe();
});
test('external reset plus POR flag preserves saved ON',()=>{const s=new Sim({reset:3,eeprom:saved(0xC3)}).run(330);assert.equal(s.mode,BOOST);});
if(process.env.OPTIBOOT_TEST==='1') {
  test('Optiboot r2 power-on flag is captured before C initialization',()=>{const s=new Sim({reset:0,bootFlags:1}).run(100);assert.equal(s.mode,OFF);s.safe();});
  test('Optiboot r2 watchdog flag is not mistaken for power-on',()=>new Sim({reset:0,bootFlags:8,eeprom:saved(0xC3)}).run(100).expectFault(F.Watchdog));
}
test('brownout does not clear FAULT',()=>new Sim({reset:4,eeprom:saved(0x5A)}).run(100).expectFault(F.Brownout));
test('watchdog reset is latched even with stored ON',()=>new Sim({reset:8,eeprom:saved(0xC3)}).run(100).expectFault(F.Watchdog));
test('stored FAULT survives external Reset',()=>new Sim({reset:2,eeprom:saved(0x5A)}).run(100).expectFault(F.Stored));
test('power-on does not clear stored FAULT or pulse heater',()=>{const s=new Sim({eeprom:saved(0x5A)}).run(100);s.expectFault(F.Stored);assert.equal(s.loadPulses,0);});
test('unknown reset does not clear stored FAULT',()=>new Sim({reset:0,eeprom:saved(0x5A)}).run(100).expectFault(F.Stored));
for(const [name,options,code] of [
  ['NTC open',{ntc:1023},F.Sensor],['NTC short',{ntc:0},F.Sensor],['NTC out of range',{temp:140},F.Sensor],
  ['overtemperature',{temp:50},F.Overheat],['42 V absent',{supply:false},F.Supply],['heater open',{heater:false},F.Supply],
  ['Q1 D-S short',{q1Short:true},F.Q1],['Q2 D-S short',{q2Short:true},F.Q2],
  ['Q1 cannot conduct',{q1Open:true},F.Conduction],['Q2 cannot conduct',{q2Open:true},F.Conduction],
  ['sense stuck low',{sense:0},F.Supply],['sense stuck high',{sense:SUPPLY_HIGH_ADC},F.Conduction]
]) test('startup detects '+name,()=>new Sim(options).run(100).expectFault(code));
test('35 ms debounce, one action while held, second press switches OFF',()=>{
  const s=new Sim().run(100);
  s.portD.setPin(2,false);s.run(10);s.portD.setPin(2,true);s.run(10);s.portD.setPin(2,false);s.run(20);assert.equal(s.mode,OFF);
  s.run(45);assert.equal(s.mode,BOOST);s.run(500);assert.equal(s.mode,BOOST);
  s.release().press();assert.equal(s.mode,OFF);s.safe();assert.equal(s.eeprom.memory[1],0x3C);
});
test('button held at startup requires release before ON',()=>{const s=new Sim({buttonLow:true}).run(500);assert.equal(s.mode,OFF);s.release().press();assert.equal(s.mode,BOOST);});
test('FAULT ignores short press and does not repeatedly write EEPROM',()=>{
  const s=new Sim({ntc:1023}).run(100);const n=s.writes.length;s.press().release().run(600);s.expectFault(F.Sensor);assert.equal(s.writes.length,n);
});
test('real 5-second hold: not early, self-test then OFF, release and new press required',()=>{
  const s=new Sim({reset:0,eeprom:saved(0x5A)}).run(100).press().run(4960);
  s.expectFault(F.Stored);assert.equal(s.loadPulses,0);
  s.run(80);assert.equal(s.mode,OFF);s.safe();assert.equal(s.fault,0);assert.equal(s.eeprom.memory[1],0x3C);
  assert.equal(s.portD.pinState(5),avr.PinState.High);assert.equal(s.portD.pinState(6),avr.PinState.Low);
  const pulses=s.loadPulses,writes=s.writes.length;s.run(1000);assert.equal(s.mode,OFF);assert.equal(s.loadPulses,pulses);assert.equal(s.writes.length,writes);
  s.release().press().run(300);assert.equal(s.mode,BOOST);
});
test('interrupted long press restarts timing; bounce under 35 ms is ignored',()=>{
  const s=new Sim({eeprom:saved(0x5A)}).run(100).press().jump(4000).release().press().jump(4000);s.expectFault(F.Stored);
  s.portD.setPin(2,true);s.run(10);s.portD.setPin(2,false);s.run(50);s.expectFault(F.Stored);
  s.jump(1100);assert.equal(s.mode,OFF);s.safe();
});
test('button held across startup cannot clear FAULT until released and held anew',()=>{
  const s=new Sim({eeprom:saved(0x5A),buttonLow:true}).run(100).jump(6000);s.expectFault(F.Stored);
  s.release().press().jump(5100);assert.equal(s.mode,OFF);s.safe();
});
test('button held before a new FAULT cannot automatically clear that FAULT',()=>{
  const s=new Sim().run(100).press().run(300);s.ntc=1023;s.run(300);s.expectFault(F.Sensor);
  s.ntc=null;s.jump(6000);s.expectFault(F.Sensor);s.release().press().jump(5100);assert.equal(s.mode,OFF);s.safe();
});
for(const [name,options] of [
  ['NTC open',{ntc:1023}],['NTC short',{ntc:0}],['NTC range',{temp:140}],['hot wheel',{temp:40}],['overheat',{temp:50}],
  ['no 42 V',{supply:false}],['heater open',{heater:false}],['Q1 short',{q1Short:true}],['Q2 short',{q2Short:true}],
  ['Q1 open',{q1Open:true}],['Q2 open',{q2Open:true}],['sense low',{sense:0}],['sense high',{sense:SUPPLY_HIGH_ADC}]
]) test(`long hold cannot clear persistent ${name}`,()=>{
  const s=new Sim({...options,eeprom:saved(0x5A)}).run(100);const writes=s.writes.length;
  s.press().jump(5100);s.expectFault(F.Stored);assert.equal(s.writes.length,writes);
  if(options.temp>=39.9||options.ntc!==undefined)assert.equal(s.loadPulses,0);
});
test('failed long hold is one-shot, repairing needs release and a new 5-second hold',()=>{
  const s=new Sim({q1Short:true,eeprom:saved(0x5A)}).run(100).press().jump(5100);s.expectFault(F.Stored);
  s.q1Short=false;s.jump(6000);s.expectFault(F.Stored);s.release().press().jump(5100);assert.equal(s.mode,OFF);s.safe();
});
test('5-second hold survives millis rollover',()=>{
  const s=new Sim({eeprom:saved(0x5A)}).run(100);s.u32('timer0_millis',0xfffffff0);s.u32('lastControlAt',0xfffffff0);s.u32('buttonChangedAt',0xfffffff0);
  s.press().jump(5100);assert.equal(s.mode,OFF);s.safe();
});
test('corrupt EEPROM can be repaired only by successful held-button self-test',()=>{
  const s=new Sim({eeprom:Uint8Array.from([0,0xC3,0x3C])}).run(100);s.expectFault(F.Eeprom);
  s.press().jump(5100);assert.equal(s.mode,OFF);assert.deepEqual(s.eeprom.memory.slice(0,3),saved(0x3C));s.safe();
});
test('reset after cooling uses a fresh measurement, not stale fault-time filtering',()=>{
  const s=new Sim({eeprom:saved(0x5A)}).run(100);
  s.cpu.dataView.setFloat32(symbols.filteredC,48,true);s.temp=39;
  s.press().jump(5100);assert.equal(s.mode,OFF);s.safe();
});
test('watchdog interrupts stuck ADC during manual FAULT reset',()=>{
  const s=new Sim({eeprom:saved(0x5A)}).run(100).press();s.adcHang=true;s.u32('timer0_millis',s.u32('timer0_millis')+5100);
  assert.throws(()=>s.run(1300),e=>e===WD_RESET);new Sim({reset:0,eeprom:s.eeprom.memory}).run(100).expectFault(F.Stored);
});
test('documented limitation: lost watchdog reason can restore saved ON after healthy self-test',()=>{
  const s=new Sim({eeprom:saved(0xC3)}).run(330);s.adcHang=true;assert.throws(()=>s.run(1500),e=>e===WD_RESET);
  const next=new Sim({reset:0,eeprom:s.eeprom.memory}).run(330);assert.equal(next.mode,BOOST);assert.equal(next.byte('powerStageReady'),1);
});
for(const [name,key,value,code] of [['Q1 short','q1Short',true,F.Q1],['Q2 short','q2Short',true,F.Q2],['heater open','heater',false,F.Supply],['NTC open','ntc',1023,F.Sensor],['overheat','temp',50,F.Overheat]])
  test('running detection within 300 ms: '+name,()=>{const s=new Sim({eeprom:saved(0xC3)}).run(330);s[key]=value;s.run(300);s.expectFault(code);});
test('39.9 C cutoff uses fresh reading and emits no diagnostic heat pulses',()=>{
  const s=new Sim({eeprom:saved(0xC3)}).run(330);s.temp=40;s.run(300);s.safe();assert.equal(s.mode,HOLD);
  const n=s.loadPulses;s.run(600);s.safe();assert.equal(s.loadPulses,n);
});
test('hot startup preserves ON with no load pulse; full self-test deferred',()=>{
  const s=new Sim({temp:41,eeprom:saved(0xC3)}).run(500);s.safe();assert.equal(s.mode,HOLD);assert.equal(s.loadPulses,0);
  assert.equal(s.byte('powerStageReady'),0);assert.equal(s.eeprom.memory[1],0xC3);assert.equal(s.writes.length,0);
});
test('ON pressed on hot wheel is remembered without energizing load',()=>{
  const s=new Sim({temp:41}).run(100).press();assert.equal(s.mode,HOLD);s.safe();assert.equal(s.loadPulses,0);assert.equal(s.eeprom.memory[1],0xC3);
});
for(const reset of [0,1,2,3,5,0x80]) for(const temp of [-20,25,45]) test(`summer OFF stays OFF after reset ${reset}, temperature ${temp}`,()=>{
  const s=new Sim({reset,temp,eeprom:saved(0x3C)}).run(600);assert.equal(s.mode,OFF);s.safe();assert.equal(s.eeprom.memory[1],0x3C);
  assert.equal(s.portD.pinState(5),avr.PinState.High);assert.equal(s.writes.length,0);
});
test('trip sequence ON -> power-off -> cold restart -> OFF button -> next trip OFF',()=>{
  const trip1=new Sim().run(100).press().release().run(300);assert.equal(trip1.mode,BOOST);
  const trip2=new Sim({reset:0,eeprom:trip1.eeprom.memory}).run(330);assert.equal(trip2.mode,BOOST);trip2.press().release();trip2.safe();
  const trip3=new Sim({reset:0,temp:-20,eeprom:trip2.eeprom.memory}).run(500);assert.equal(trip3.mode,OFF);trip3.safe();
});
test('repeated warm restarts keep ON, then cooling runs full test before HOLD power',()=>{
  let memory=saved(0xC3);
  for(let n=0;n<3;n++) {const s=new Sim({reset:0,temp:41,eeprom:memory}).run(300);assert.equal(s.mode,HOLD);s.safe();assert.equal(s.loadPulses,0);assert.equal(s.writes.length,0);memory=s.eeprom.memory;}
  const s=new Sim({reset:0,temp:41,eeprom:memory}).run(300);s.temp=30;s.run(2400);
  assert.equal(s.mode,HOLD);assert.equal(s.byte('powerStageReady'),1);assert.ok(s.pwm>0);assert.ok(s.enabled);assert.equal(s.eeprom.memory[1],0xC3);
});
for(const [name,options,code] of [
  ['Q1 open',{q1Open:true},F.Conduction],['Q2 open',{q2Open:true},F.Conduction],
  ['Q1 short',{q1Short:true},F.Q1],['Q2 short',{q2Short:true},F.Q2],['heater open',{heater:false},F.Supply]
]) test(`deferred warm-start self-test detects ${name} before normal heating`,()=>{
  const s=new Sim({...options,temp:41,eeprom:saved(0xC3)}).run(300);assert.equal(s.mode,HOLD);s.safe();assert.equal(s.loadPulses,0);
  s.temp=35;s.run(1800);s.expectFault(code);assert.equal(s.byte('powerStageReady'),0);
});
test('OFF button during warm ON is remembered across next restart',()=>{
  const s=new Sim({temp:41,eeprom:saved(0xC3)}).run(100).press();assert.equal(s.mode,OFF);s.safe();
  const next=new Sim({reset:0,eeprom:s.eeprom.memory}).run(500);assert.equal(next.mode,OFF);next.safe();
});
test('warm requested ON remains below emergency threshold; emergency still latches',()=>{
  const s=new Sim({temp:45,eeprom:saved(0xC3)}).run(300);assert.equal(s.mode,HOLD);s.safe();assert.equal(s.loadPulses,0);
  s.temp=50;s.run(300);s.expectFault(F.Overheat);
});
test('documented power-sequencing limitation: loss of 42 V with Nano alive latches FAULT',()=>{
  const s=new Sim({eeprom:saved(0xC3)}).run(330);s.supply=false;s.run(300);s.expectFault(F.Supply);
  new Sim({reset:0,eeprom:s.eeprom.memory}).run(100).expectFault(F.Stored);
});
test('36.4 C transition to HOLD and power clamp <=180',()=>{
  const s=new Sim({eeprom:saved(0xC3)}).run(330);s.temp=37;s.run(300);assert.equal(s.mode,HOLD);assert.ok(s.pwm<=180);
});
test('measured NTC points map 6.11 kOhm to 36.4 C and 7.00 kOhm to 33.2 C',()=>{
  const boostExit=new Sim({ntc:ntcAdc(6110)}).run(100);
  const holdTarget=new Sim({ntc:ntcAdc(7000)}).run(100);
  assert.ok(Math.abs(boostExit.float('temperatureC')-36.4)<0.25);
  assert.ok(Math.abs(holdTarget.float('temperatureC')-33.2)<0.25);
});
test('BOOST exits at 36.4 C, HOLD stays off above target and resumes below 33.2 C',()=>{
  const s=new Sim({eeprom:saved(0xC3)}).run(330);
  s.temp=36.5;s.run(300);assert.equal(s.mode,HOLD);assert.equal(s.pwm,0);
  s.temp=32.5;s.run(2400);assert.equal(s.mode,HOLD);assert.ok(s.pwm>0);assert.ok(s.enabled);
});
test('BOOST timeout and actual millis interval',()=>{
  const s=new Sim({eeprom:saved(0xC3)}).run(330);const jump=s.u32('timer0_millis')+300001;s.u32('timer0_millis',jump);s.run(10);assert.equal(s.mode,HOLD);assert.ok(s.float('integral')<=180);
});
test('millis rollover preserves scheduling and debounce',()=>{
  const s=new Sim().run(100);s.u32('timer0_millis',0xfffffff0);s.u32('lastControlAt',0xfffffff0);s.u32('buttonChangedAt',0xfffffff0);
  s.run(80).press().run(300);assert.equal(s.mode,BOOST);s.release().press();assert.equal(s.mode,OFF);
});
test('soft temperature latch is not reset by OFF button',()=>{
  const s=new Sim({eeprom:saved(0xC3)}).run(330);s.temp=45;s.run(300);assert.equal(s.byte('softCutoff'),1);s.press();assert.equal(s.mode,OFF);assert.equal(s.byte('softCutoff'),1);
  s.temp=42.5;s.run(300);assert.equal(s.byte('softCutoff'),1);s.temp=41.5;s.run(300);assert.equal(s.byte('softCutoff'),0);s.safe();
});
test('torn EEPROM transactions cannot restore a partial ON/FAULT',()=>{
  for(const bytes of [[0,0x3C,0xC3],[0,0xC3,0xC3],[0,0xC3,0x3C],[0xB1,0xC3,0xC3]]) new Sim({eeprom:Uint8Array.from(bytes)}).run(100).expectFault(F.Eeprom);
});
test('watchdog interrupts a stuck ADC during setup',()=>{
  const s=new Sim({adcHang:true});assert.throws(()=>s.run(1300),e=>e===WD_RESET);assert.ok(s.resetAt/16000<1100);
  new Sim({reset:8,eeprom:s.eeprom.memory}).run(100).expectFault(F.Watchdog);
});
test('watchdog interrupts a stuck running loop, next startup is FAULT',()=>{
  const s=new Sim({eeprom:saved(0xC3)}).run(330);const start=s.cpu.cycles;s.adcHang=true;
  assert.throws(()=>s.run(1500),e=>e===WD_RESET);assert.ok((s.resetAt-start)/16000<1350);
  new Sim({reset:8,eeprom:s.eeprom.memory}).run(100).expectFault(F.Watchdog);
});
console.log(`\n${results.length} tests passed. Model excludes analog transients, bootloader, physical temperature and real wiring.`);
if(process.env.REPORT_PATH)fs.writeFileSync(process.env.REPORT_PATH,JSON.stringify({firmware:'1.3',simulator:'avr8js 0.21.0',hexSha256:createHash('sha256').update(fs.readFileSync(path.join(build,name+'.hex'))).digest('hex'),tests:results},null,2)+'\n');
