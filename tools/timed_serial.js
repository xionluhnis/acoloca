#!/usr/bin/env nodejs

"use strict";

let path = process.argv[2] || '/dev/ttyUSB0';
let rate = parseInt(process.argv[3] || '115200');
let numB = parseInt(process.argv[4] || '1');
let msbFirst = process.argv[5] != 'LSB';

const fs = require('fs');
const SerialPort = require('serialport');
let port = new SerialPort(path, { autoOpen: false, baudRate: rate });

// Open errors will be emitted as an error event
port.on('error', function(err) {
  console.log('Error: ', err.message);
});

let output = function(buffer){
  let num = 0;
  // MSB first
  if(msbFirst){
    for(let i = buffer.length - 1, shift = 0; i >= 0; --i, shift += 8)
      num += buffer[i] << shift;
  } else {
    for(let i = 0, shift = 0; i < buffer.length; ++i, shift+= 8)
      num += buffer[i] << shift;
  }
  console.log(+Date.now() + ", " + num);
};

// Data flow
let byteBuf = new Array(numB);
let byteIdx = 0;
port.on('data', function(data) {
  for(let i = 0; i < data.length; ++i){
    byteBuf[byteIdx] = data[i];
    ++byteIdx;
    if(byteIdx >= numB){
      byteIdx = 0;
      output(byteBuf);
    }
  }
});

port.open(function(err){
  if(err)
    console.log('Could not open: ', err);
});
