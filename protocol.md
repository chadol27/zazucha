protocol: `command:arg1,arg2,...<\n>`  
command: only 1 char long  
lowercase only

commands:
* m: set motor status
  * arg1: left direction
  * arg2: left speed
  * arg3: right direction
  * arg4: right speed
  * example: `m:f,255,f,255` `m:s,0,s,0`
* l: set led colors
  * arg1: r
  * arg2: g
  * arg3: b
  * example: `l:255,0,0`

motors:
* left: IN3, IN4, ENB
* right: IN1, IN2, ENA

directions:
* f: Forward
* s: Stop
* b: Backward

speed: 0-255

color:
* r: 0-255
* g: 0-255
* b: 0-255
