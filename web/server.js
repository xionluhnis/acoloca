#!/usr/bin/env nodejs

// Alexandre Kaspar <akaspar@mit.edu>

"use strict";

// libraries

const fs = require('fs');
const Readline = require('readline');
const ws = require('ws');
const https = require('https');
const finalhandler = require('finalhandler');
const serveStatic = require('serve-static');

// parameters
const port = 4443;

// instances
const serve = serveStatic('./');
const server = new https.createServer({
  cert: fs.readFileSync('../private/server.pem'),
  key: fs.readFileSync('../private/server.pem')
}, function(req, res){
  let done = finalhandler(req, res);
  serve(req, res, done);
});
const rl = Readline.createInterface({ input: process.stdin, output: process.stdout });
const wss = new ws.Server({ server });
let sockets = {};

wss.on('connection', function(socket){
  // register socket
  let addr = socket._socket.remoteAddress;
  console.log('Connection from ' + addr);
  sockets[addr] = socket;
  let ext = '';

  // message handler
  socket.on('message', function(data){
    if( data instanceof Buffer ){
      let fname = './data/' + addr + '_';
      let i = 0;
      while(fs.existsSync(fname + i + ext))
        ++i;
      console.log(addr, ' | buffer | ', fname + i + ext);
      fs.writeFileSync(fname + i + ext, data);
    } else if(data.includes('image')){
      let newExt = '.' + data.split('/')[1];
      if(ext != newExt){
        console.log(addr, ' | ', data);
        ext = newExt;
      }
    } else {
      console.log(addr, ' | ' + data);
    }
  });

  // close handler
  socket.on('close', function(){
    console.log('client', addr, ' closing');
    delete sockets[addr];
  });
});

let sendMessage = function(msg){
  for(let addr in sockets){
    let socket = sockets[addr];
    socket.send(msg);
  }
};

rl.on('line', function(line){
  if(line.includes('help')){
    console.log('image: acquire image from each client');
    console.log('echo: send echo');
  } else if(line.includes('image')){
    sendMessage('image');
  } else if(line.includes('echo')){
    sendMessage('echo');
  }
});

server.listen(port, 'krakowiak.csail.mit.edu');
