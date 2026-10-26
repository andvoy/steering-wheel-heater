// AVR8js ATmega328P hardware model for compiled firmware tests.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {execFileSync} = require('node:child_process');
const {createHash} = require('node:crypto');
const avr = require('avr8js');

const build = path.resolve(process.argv[2] || 'build');
const name = 'steering_wheel_heater_v1_9.ino';
const avrBin = process.env.AVR_BIN;
assert.ok(avrBin, 'Set AVR_BIN to the bin directory of the AVR toolchain used for compilation.');
const symbols = {};
for (const line of execFileSync(path.join(avrBin,process.platform==='win32'?'avr-nm.exe':'avr-nm'),['-n','-C',path.join(build,name+'.elf')],{encoding:'utf8'}).split('\n')) {
  const m = line.match(/^([0-9a-f]+)\s+[bBdD]\s+(\S+)\s*$/);
  if (m) symbols[m[2]] = parseInt(m[1],16)-0x800000;
}
for (const key of ['mode','faultCode','timer0_millis','lastControlAt','integral','temperatureC']) assert.ok(key in symbols,key);
const flash = new Uint8Array(32768);
for (const line of fs.readFileSync(path.join(build,name+'.hex'),'utf8').trim().split(/\r?\n/)) {
  const bytes = Buffer.from(line.slice(1),'hex');
  assert.equal(bytes.reduce((s,b)=>s+b,0)&255,0,'HEX checksum');
  if (bytes[3]===0) flash.set(bytes.subarray(4,4+bytes[0]),bytes.readUInt16BE(1));
}
const saved = state => Uint8Array.from([0xB1,state,state^255]);
const OFF=0, HEATING=1, FAULT=2;
const F={Overheat:2,Supply:3,Sensor:4,Q1:5,Q2:6,Conduction:7};
const legacyFault = code => Uint8Array.from([0xB1,0x5A,0xA5,code,code^255]);
const SUPPLY_HIGH_ADC=374; // 42 V through 220k/10k divider at 5 V ADC reference.
const ntcAdc = resistance => Math.round(1023*resistance/(10000+resistance));
const WD_RESET = Symbol('watchdog reset');

class Sim {
  constructor(options={}) {
    Object.assign(this,{temp:25,supply:true,heater:true,q1Short:false,q2Short:false,q1Open:false,q2Open:false,sense:null,ntc:null,adcHang:false},options);
    this.cpu = new avr.CPU(new Uint16Array(flash.buffer.slice(0)),2048);
    this.cpu.data[0x54] = options.reset===undefined ? 1 : options.reset;
    if(options.bootFlags!==undefined)this.cpu.data[2]=options.bootFlags;
    this.serialBytes=[];
    this.serial=new avr.AVRUSART(this.cpu,avr.usart0Config,16000000);
    this.serial.onByteTransmit=byte=>this.serialBytes.push(byte);
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
  get serialText() {return Buffer.from(this.serialBytes).toString("utf8");}
  get mode() {return this.byte('mode');}
  get fault() {return this.byte('faultCode');}
  get pwm() {return this.cpu.data[0x80]&0x80 ? this.cpu.data[0x88] : this.cpu.data[0x25]&2 ? 255 : 0;}
  get enabled() {return !!(this.cpu.data[0x25]&1);}
  get boardLed() {return this.portB.pinState(5)===avr.PinState.High;}
  press() {this.portD.setPin(2,false);return this.run(60);}
  release() {this.portD.setPin(2,true);return this.run(60);}
  jump(ms) {this.u32('timer0_millis',this.u32('timer0_millis')+ms);return this.run(80);}
  safe() {assert.equal(this.pwm,0);assert.equal(this.enabled,false);}
  expectFault(code) {assert.equal(this.mode,FAULT);assert.equal(this.fault,code);this.safe();assert.equal(this.eeprom.memory[1],0x3C);}
}

const hexSha256=createHash("sha256").update(fs.readFileSync(path.join(build,name+".hex"))).digest("hex");
module.exports={Sim,F,OFF,HEATING,FAULT,saved,legacyFault,avr,WD_RESET,hexSha256};

