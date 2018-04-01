var gazebojs = require('gazebojs');
var gazebo = new gazebojs.Gazebo();


if (process.argv.length != 5)
{
  console.log('node publish.js [msg type] [topic name] [message]');
  console.log('ex:\nnode publish.js "gazebo.msgs.WorldControl"  "~/world_control" "{\\\"pause\\\": true}"\n');

  process.exit(-1);
}


var type  = process.argv[2];
var topic = process.argv[3];
var msgS   = process.argv[4];

console.log("type:  [" + type + "]");
console.log("topic: [" + topic+ "]");

var msg = JSON.parse(msgS);
console.log("msg:   [" + require('util').inspect(msg)+ "]" ) ;

gazebo.publish(type, topic , msg);
console.log('\npublished!');

setInterval(function (){
  console.log("bye");
  process.exit(0);
},3000);

