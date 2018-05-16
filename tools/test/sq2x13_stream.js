#!/usr/bin/env nodejs

require('use-strict');

// libraries
const fs = require('fs');
const SerialPort = require('serialport');

// arguments
if(process.argv.length < 3){
  console.log('Usage:', process.argv[1], 'file <device=/dev/ttyUSB0> <baud=230400>')
  process.exit(0);
}
let file = process.argv[2];
let path = process.argv[3] || '/dev/ttyUSB0';
let rate = parseInt(process.argv[4] || '230400');

let port = new SerialPort(path, { autoOpen: false, baudRate: rate });

let data = fs.readFileSync(file, 'utf-8');
let lines = data.split('\n');
// console.log(lines.length + ' lines of data');

// Open errors will be emitted as an error event
port.on('error', function(err) {
  console.log('Error: ', err);
});

let writeIdx = 0;
let sendData = function(){
  let line = lines[writeIdx];
  // console.log('line: ' + line);
  if(++writeIdx >= lines.length)
    writeIdx = 0; // wrap around so we have infinite amount of data
  let value = parseInt(line.split(' ')[1]);
  // console.log('value: ' + value);
  // reduce to 8 bits (assuming 10 bits)
  let val8b = value >> 2;
  // console.log('Sending ' + val8b);
  let arr = new Uint8Array(1);
  arr[0] = val8b;
  port.write(Buffer.from(arr));
};

let output = function(buffer){

  // unpack SQ2x13 from two bytes
  let bits = (buffer[0] << 8) | (buffer[1]);
  // console.log('bits: 0b' + bits.toString(2));
  let sign = bits >> 15;

  let int16 = bits & 0xFFFF;
  if(sign){
    int16 |= 0xFFFF0000;
  }
  let num = (1.0 / 8192.0) * int16;
  console.log(+Date.now() + ' ' + num);

  // send data
  sendData();
};

// Data flow
let headBuf = [0, 0];
let headIdx = 0;
let resetHead = function(){
  headBuf[0] = 0;
  headBuf[1] = 0;
  headIdx = 0;
};
let byteBuf = [0, 0];
let byteIdx = 0;
let readByte = function(data){
  // console.log('byte:', data.toString(2));
  if(headBuf[0] == 0xFF && headBuf[1] == 0x01){
    // console.log('data(' + byteIdx + '):', data.toString(2));
    byteBuf[byteIdx] = data;
    ++byteIdx;
    if(byteIdx >= 2){
      byteIdx = 0;
      output(byteBuf);
      resetHead();
    }
  } else if(data == 0xFF){
    headIdx = 1;
    headBuf[0] = 0xFF;
    byteIdx = 0;
  } else if(data == 0x01 && headIdx == 1){
    headBuf[headIdx] = data;
    headIdx = 0;
    byteIdx = 0;
  }
};

port.on('data', function(data) {
  for(let i = 0; i < data.length; ++i){
    readByte(data[i]);
  }
});

port.open(function(err){
  if(err)
    console.error('Could not open: ', err);
});

