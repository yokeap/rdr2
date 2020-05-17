'use strict';

var express = require('express')
  , process = require('process')
  , path = require('path')
  , app = express()
  , http = require('http').Server(app)
  , io = require('socket.io')(http)
  , exec = require('child_process').exec
  , spawn = require('child_process').spawn
  // , imuSensor = require(__dirname + '/plugins/navigation-data')
  // , imuData = new imuSensor()
  , si = require('systeminformation')
  , SerialPort = require('serialport')
  //, events = require('events')
  , dataBuffer  = null  
  , initBuffer  = null
  , initFrame   = null 

  , ffmpeg_options  = '-threads 1 -f v4l2 -video_size 1280x720 -i /dev/video1 \
                      -c:v copy -f mp4 -g 1 -threads 2 -movflags empty_moov+default_base_moof+frag_keyframe -frag_size 8 \
                      -tune zerolatency -';

console.log(__dirname);


let motorPwm = {
  m1 : 0,
  m2 : 0,
  m3 : 0,
  m4 : 0
};

var DueSerialport = new SerialPort('/dev/ttyS1', {
  autoOpen: false,
  baudRate: 115200,
  highWaterMark: 65535,
  parser: new SerialPort.parsers.Readline('\r')
});

var MicroSerialport = new SerialPort('/dev/ttyS2', {
  autoOpen: false,
  baudRate: 115200,
  highWaterMark: 65535,
  parser: new SerialPort.parsers.Readline('\n')
});

// server config 
http.listen(9010, () => {
    console.log('server start at localhost:9010');
});

app.use(express.static(path.join(__dirname, 'public')));
app.use(express.static(path.join(__dirname, "./")));
app.use(express.static(path.join(__dirname, "./public/")));
app.use(express.static(path.join(__dirname, "./public/bower_components")));
app.use(express.static(path.join(__dirname, "./public/webcomponents")));
app.use(express.static(path.join(__dirname, "./public/js")));

/**********************************************************************************************
* ELP camera custom driver initializaton config, bitrate:500kbps with QP 1
* 
* *********************************************************************************************/
var camera_settings = exec('H264_UVC_TestAP /dev/video1 --xuset-br 2500000 --xuset-qp 13 --shrp-set 0', 
    (error, stdout, stderr) =>{
    console.log(`stdout: ${stdout}`);
    console.log(`stderr: ${stderr}`);
    if (error !== null) {
        console.log(`exec error: ${error}`);
    }
});

/**********************************************************************************************
* ffmpeg mp4 fragmentation capturing stdout process.
* 
* *********************************************************************************************/
var ffmpegProc;

function ffmpegProccess(){
  ffmpegProc = spawn( 'ffmpeg', ffmpeg_options.match( /\S+/g ));
  ffmpegProc.stdout.on('data', function(data){
  //init frame process, the init frame must be containing ftype(24bytes) on top of header stream.
    if( initFrame === null ){
      initBuffer = initBuffer == null ? data : Buffer.concat( [initBuffer,data] );
      //checking fytp header
      if( initBuffer.length < 25 ){
        // if( initBuffer.length < 848 ){
        // return if that is fytp header (24bytes)
        return;
      }
      initFrame = initBuffer;
      // ffmpegProc.emit('init_frame', initFrame);
      return;                       //return the buffer concatenation of fytp+moov+moof+mdat(not sure)  
    }
    /*********************************************************************************
    * 
    * note: not sure what is section meaning, now this is not used.
    *          assumming this involved the I-frame (GOP).
    * *******************************************************************************/
    if( data.length == 8192 ){
      dataBuffer = dataBuffer == null ? data : Buffer.concat([dataBuffer,data]);
      return;
      }

    //continues streaming  
    dataBuffer = dataBuffer == null ? data : Buffer.concat([dataBuffer,data]);
    ffmpegProc.emit('stream_start', dataBuffer);
    dataBuffer = null;
  });
  ffmpegProc.stderr.on('data', function(error){
      // console.log("FFMPEG: " + error.toString());
      //var timeStampInMs = window.performance && window.performance.now && window.performance.timing && window.performance.timing.navigationStart ? window.performance.now() + window.performance.timing.navigationStart : Date.now();
    
  });
}


/**********************************************************************************************
 * set cpu priority process for FFMPEG
* *********************************************************************************************/
var proc= spawn("renice", [-20, process.pid]);
  proc.on('exit', function (code) {
      if (code !== 0){
          console.log("Process "+ "cmd" +" exec failed with code - " +code);
      }
  });
  proc.stdout.on('data', function(data){
      console.log('stdout: ' + data);
  });
  proc.stderr.on('data', function(data){
      console.log('stderr: '+ data);
  });

/**********************************************************************************************
 * socket routine process
 * 
* *********************************************************************************************/
io.on('connection', function(socket) {
    console.log('A user has been connected');

    ffmpegProccess();

    socket.on('disconnect', () => {
      //stop();
        console.log('A user disconnected');
        // ffmpegProc.removeListener('stream.start', function(data){
        //   console.log('Streaming stop');
        // });
        ffmpegProc.removeListener('stream_start', emitSegment);
        ffmpegProc.kill('SIGINT');
    });

    socket.on('request_Init_Segment', function(fn){
        fn( new Buffer( initFrame, 'binary' ) );
        console.log("request init frame");
        ffmpegProc.on('stream_start', emitSegment);
        setInterval(sendRawMotionVal, 100);
    });

    socket.on('motion-data', function(data){
      motorPwm = data;
      // console.log(data);
    });

    
    //Occured from ffmpegProc.on
    function emitSegment(data) {
        socket.compress(false).volatile.emit('x-h264-video.data', data);

        // console.log(data.length);
        // socket.emit('x-h264-video.data', data);
    }
    
});

/**********************************************************************************************
 * Arduino Due motor control, serial port handler
* *********************************************************************************************/
DueSerialport.open(function (err) {
  if (err) {
    return console.log('Error opening port: ', err.message);
  }

  // Because there's no callback to write, write errors will be emitted on the port:
  // DueSerialport.write('main screen turn on');
  console.log("Arduino Due has been connected");
});

function sendRawMotionVal(callback) {

  let strTemp = String(':' + motorPwm.m2 + ',' + motorPwm.m1 + ',' + motorPwm.m4 + ',' + motorPwm.m3 + '\r');
  // console.log(strTemp);
  DueSerialport.write(strTemp , function (error) {
		if(error){console.log(error);}
	  else{
	    //console.log('Write Completed/n');
			// waits until all output data has been transmitted to the serial port.
		  // DueSerialport.drain(callback);
      DueSerialport.flush();
		}
  });
  motorPwm.m1 = 0;
  motorPwm.m2 = 0;
  motorPwm.m3 = 0;
  motorPwm.m4 = 0;
}