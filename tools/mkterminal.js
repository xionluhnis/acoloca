//----------------------------------------- mkterminal
"use strict";
/*
small program for sending apa packets along a serial line
*/

// check that we have portname
if (process.argv.length < 3) {
    logAdvice()
    process.exit(-1)
}

if (process.argv[2] == '-h' || process.argv[2] == 'help') {
    logAdvice()
    process.exit(-1)
}

var SerialPort = require('serialport');

var ByteLengthParser = SerialPort.parsers.ByteLength;

var port = new SerialPort(process.argv[2], {
    baudRate: 115200,
    dataBits: 8,
    parity: 'none',
    flowControl: false,
});

//----------------------------------------- readline
/*
key input from terminal
*/

const readline = require('readline');

const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
});

rl.on('line', parseLineIn);


let history = [];
let ports = [1,2,3];
let direction = [1,1,1];
let block_speed   = 250000; // very fast: 1500000;
let block_acc     = 60000;
let block_acclen  = 2000;
let block_declen  = 190;

const PTR = 254;
const END = 255;
const TEST = 127;
const STEPS = 128;
const BLOCK = 129;

function parseLineIn(data) {

    /*
    packet like "packet port,port,etc,ptr,byte,byte,byte..."
    does like "len,int,int,int,254?,byte,byte,byte,etc"
    next do key:type:values
    apa_end_addr_delimiter 255
    apa_addr_pointer 254
    apa_addr_flood 253
    key_position_float 127
    key_speed_float 128
    key_position_steps 129
    key_speed_steps 130
    */
    if (data.includes('packet')) {
        var argstring = data.split(' ')[1]; // everything following 'packet '
        var args = argstring.split(',');
        var packet = new Array();
        var i = 0; // args
        var u = 0; // packet
        while (i < args.length) {
            switch (args[i]) {
                case 'ptr':
                    packet[u] = PTR;
                    i++;
                    u++;
                    break;
                case 'end':
                    packet[u] = END;
                    i++;
                    u++;
                    break;
                case 'test':
                    packet[u] = TEST;
                    i++;
                    u++;
                    break;
                case 'steps':
                    // steps, position-wise, in signed int32_t
                    packet[u] = STEPS;
                    var steps = parseInt(args[i + 1])
                    packet = packet.concat(pack32(steps))
                    var speed = parseInt(args[i + 2])
                    packet = packet.concat(pack32(speed))
                    i += 3;
                    u += 9;
                    break;
                case 'block':
                    packet[u] = BLOCK;
                    // a linked acceleration planned segment
                    // we'll be going betwee float-space for steps in the higher levels, so
                    var steps = parseInt(args[i+1])
                    packet = packet.concat(pack32(steps))
                    // but accel and speeds need to be in floats
                    var entry = parseInt(args[i+2])
                    packet = packet.concat(pack32(entry))
                    // pack 64 bit float to 32 bit?
                    var accel = parseInt(args[i+3])
                    packet = packet.concat(pack32(accel))

                    var accelLength = parseInt(args[i+4])
                    packet = packet.concat(pack32(accelLength))

                    var deccelLength = parseInt(args[i+5])
                    packet = packet.concat(pack32(deccelLength))

                    i += 6
                    u += 21
                    break;
                default:
                    packet[u] = parseInt(args[i]);
                    u++;
                    i++;
                    break;
            }
        }
        packet.unshift(packet.length + 1); // push the length into header
        data_out(packet);
        // now through bytes
    } else if(data.includes('undo') || data == 'u'){
      if(history.length == 0){
        console.error('Cannot undo nothing');
      } else {
        let last_packet = history[history.length-1];
        if(last_packet.length == 3){
          data_out(last_packet.map(x => -x));
        } else {
          // full reconstruction with inversion of first post-type argument
          let i = 0;
          for(; i < last_packet.length && last_packet[i] != 255; ++i);
          if(i+2 < last_packet.length){
            let inv_packet = last_packet.map(x => x);
            inv_packet[i+2] = -inv_packet[i+2];
            data_out(inv_packet);
          }
        }
      }

    } else if(data.includes('redo') || data == 'r'){
      if(history.length == 0){
        console.error('Cannot redo nothing');
      } else {
        data_out(history[history.length-1]);
      }

    } else if(data.includes('dir')){
        var argstring = data.split(' ')[1]; // everything following 'packet '
        var args = argstring.split(',');
        for(let i = 0; i < args.length && i < 3; ++i){
          direction[i] = parseInt(args[i]);
        }
        console.log('Set direction to ' + direction.join(','));

    } else if(data.includes('speed')){
        var speed = data.split(' ')[1];
        block_speed = parseInt(speed);
        console.log('Set speed to ' + block_speed);

    } else if(data.includes('acc')){
        block_acc = parseInt(data.split(' ')[1]);
        console.log('Set acceleration to ' + block_acc);

    } else if(data.includes('acclen')){
        block_acclen = parseInt(data.split(' ')[1]);
        console.log('Set acceleration length to ' + block_acclen);

    } else if(data.includes('declen')){
        block_declen = parseInt(data.split(' ')[1]);
        console.log('Set decceleration length to ' + block_declen);

    } else if(data.includes('move')){
      let argstring = data.split(' ')[1];
      let args = argstring.split(',');
      if(args.length == 1){
          let val = parseInt(args[0]);
          data_out([val, val, val]);
      } else if(args.length == 3){
          data_out(args.map(x => parseInt(x)));
      } else {
          console.error('move x [y z]');
      }

    } else if(data.includes('next') || data[0] == 'n'){
      let val = parseInt(data.split(' ')[1]);
      data_out([direction[0] * val, direction[1] * val, direction[2] * val]);
         
    } else if (data.includes('dummy packet')) {
        var packet = Buffer.from([12, 3, 1, 254, 4, 255, 1, 2, 3, 4, 5, 6]);
        data_out(packet);
    } else if (data.includes('help')) {
        logAdvice();
    } else {
        data_out(data);
    }
}

function pack32(val) {
    var pack = new Array();
    pack[0] = (val >> 24) & 255;
    pack[1] = (val >> 16) & 255;
    pack[2] = (val >> 8) & 255;
    pack[3] = val & 255;

    return pack;
}

function packFloat(val){
	var pack; // the array of bytes
	// as a view into a typed array
	var view = new DataView(new ArrayBuffer(4));
	view.setFloat32(0, val);

	pack = Array.apply(null, {length: 4}).map((_, i) => view.getUint8(i))
	console.log('packed', val, 'as', pack)

	return pack
}

function logAdvice() {
    console.log('use serialport-list to find ports')
    console.log("command line: node serialterminal portname");
    console.log('route,ptr,end,command,args')
    console.log('test: sends byte 128 to network test')
    console.log('steps: {steps uint32_t, speed steps/s float, dir uint8_t}')
    console.log('block: {steps float (also dir), cruise speed steps/s float, accelleration steps/s/s float, start speed float, end speed float')
}

//----------------------------------------- parsing
/*
does the business
*/

var parser = port.pipe(new ByteLengthParser({ length: 1 }));

parser.on('data', data_in);

function data_in(data) {
    console.log(data[0]);
}

function data_out(data) {
    history.push(data);
    if(data.length == 3){
      // multiplexing packets
      data.forEach(function(steps, idx){
        if(steps == 0)
          return;
        let p = [0, ports[idx], 254, 255, BLOCK];
        let tail = [steps, block_speed, block_acc, block_acclen, block_declen];
        for(let i = 0; i < tail.length; ++i)
          p = p.concat(pack32(tail[i]));
        p.unshift(p.length + 1);
        console.log('sent: ', p);
        port.write(p, err => console.log('port error on write', err.message));
      });
    } else {
      console.log('sent: ', data);
      port.write(data, function(err) {
        if (err) {
            return console.log('port error on write: ', err.message);
        }
      });
    }
}

port.on('open', function(err) {
    console.log('port open');
});

port.on('error', function(err) {
    console.log('port error: ', err.message);
});
