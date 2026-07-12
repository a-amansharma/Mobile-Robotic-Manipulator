#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

const char* WIFI_NAME = "Mobile-Robotic-Manipulator";
const char* WIFI_PASSWORD = "87654321";

WebServer server(80);

const uint8_t PCA_SDA_PIN = 21;
const uint8_t PCA_SCL_PIN = 22;

const uint8_t MPU_SDA_PIN = 18;
const uint8_t MPU_SCL_PIN = 19;

TwoWire MPU_WIRE = TwoWire(1);

const uint8_t LEFT_RPWM_PIN = 25;
const uint8_t LEFT_LPWM_PIN = 26;

const uint8_t RIGHT_RPWM_PIN = 27;
const uint8_t RIGHT_LPWM_PIN = 14;

const bool REVERSE_LEFT_SIDE = false;
const bool REVERSE_RIGHT_SIDE = false;

const uint32_t MOTOR_PWM_FREQUENCY = 5000;
const uint8_t MOTOR_PWM_RESOLUTION = 8;
const uint8_t LEFT_RPWM_CHANNEL = 0;
const uint8_t LEFT_LPWM_CHANNEL = 1;
const uint8_t RIGHT_RPWM_CHANNEL = 2;
const uint8_t RIGHT_LPWM_CHANNEL = 3;

const int MOTOR_SPEED_LIMIT = 255;
const int JOYSTICK_DEADZONE = 8;
const int MOTOR_RAMP_STEP = 5;
const unsigned long MOTOR_RAMP_INTERVAL_MS = 12;
int targetLeftMotor = 0;
int targetRightMotor = 0;
int currentLeftMotor = 0;
int currentRightMotor = 0;
unsigned long lastMotorRampTime = 0;

const unsigned long MOTOR_COMMAND_TIMEOUT_MS = 650;

unsigned long lastMotorCommandTime = 0;

const uint8_t BUZZER_PIN = 13;
const uint8_t HEADLIGHT_PIN = 23;

const bool BUZZER_ACTIVE_HIGH = true;
const bool HEADLIGHT_ACTIVE_HIGH = true;

bool buzzerOn = false;
bool headlightOn = false;

const unsigned long BUZZER_TIMEOUT_MS = 700;
unsigned long lastBuzzerRefreshTime = 0;
int previousClientCount = 0;
bool beepSequenceActive = false;
uint8_t beepPulsesRemaining = 0;
bool beepOutputState = false;
unsigned long beepPhaseTime = 0;
const unsigned long BEEP_ON_MS = 110;
const unsigned long BEEP_OFF_MS = 120;

const uint8_t SERVO_A_PIN = 32;
const uint8_t SERVO_B_PIN = 33;

const uint32_t SERVO_PWM_FREQUENCY = 50;
const uint8_t SERVO_PWM_RESOLUTION = 16;
const uint8_t SERVO_A_CHANNEL = 4;
const uint8_t SERVO_B_CHANNEL = 5;

const uint16_t DIRECT_SERVO_MIN_US = 500;
const uint16_t DIRECT_SERVO_MAX_US = 2500;

Adafruit_PWMServoDriver pca9685 =
  Adafruit_PWMServoDriver(0x40, Wire);

const uint8_t SERVO_C_CHANNEL = 4;
const uint8_t SERVO_D_CHANNEL = 6;
const uint8_t SERVO_E_CHANNEL = 8;
const uint8_t SERVO_F_CHANNEL = 10;

const uint16_t PCA_SERVO_MIN = 102;
const uint16_t PCA_SERVO_MAX = 512;

Adafruit_MPU6050 mpu6050;

bool mpuDetected = false;

bool tiltProtectionEnabled = true;

bool dangerousTilt = false;

float rollAngle = 0.0f;
float pitchAngle = 0.0f;

float rollOffset = 0.0f;
float pitchOffset = 0.0f;

float filteredRoll = 0.0f;
float filteredPitch = 0.0f;

const float TILT_STOP_ANGLE = 45.0f;
const float TILT_RESET_ANGLE = 40.0f;

const unsigned long MPU_INTERVAL_MS = 20;
unsigned long lastMPUReadTime = 0;

const unsigned long STATUS_INTERVAL_MS = 150;

int armAngleA = 0;
int armAngleB = 0;
int armAngleC = 94;
int armAngleD = 16;
int armAngleE = 180;
int armAngleF = 73;
int targetArmA = 0;
int targetArmB = 0;
int targetArmC = 94;
int targetArmD = 16;
int targetArmE = 180;
int targetArmF = 73;
const unsigned long SERVO_RAMP_INTERVAL_MS = 28;
unsigned long lastServoRampTime = 0;

const char WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,minimum-scale=1,user-scalable=no,viewport-fit=cover"><title>Mobile Robotic Manipulator</title><style>
:root{--bg:#07111f;--panel:#0d1c30;--panel2:#12263f;--text:#f7fbff;--muted:#9ab0c7;--blue:#108dff;--blue2:#004fbd;--red:#ef304b;--green:#28da91}*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent;-webkit-touch-callout:none}html,body{width:100%;height:100%;margin:0;overflow:hidden;overscroll-behavior:none;touch-action:none;background:linear-gradient(145deg,#030914,#0a1830 55%,#06101e);font-family:Arial,Helvetica,sans-serif;color:var(--text)}body{padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}.app{width:100%;height:100dvh;max-width:760px;margin:auto;padding:6px;display:grid;grid-template-rows:47% 53%;gap:6px}.panel{min-height:0;overflow:hidden;border-radius:22px;background:linear-gradient(150deg,#142b47,#091526 72%);box-shadow:0 14px 30px rgba(0,0,0,.52),inset 0 1px 0 rgba(255,255,255,.1),inset 0 -2px 0 rgba(0,0,0,.42)}.title{height:32px;display:flex;align-items:center;justify-content:center;gap:9px;font-size:15px;font-weight:900;letter-spacing:1.1px;text-transform:uppercase;text-shadow:0 2px 5px #000}.dot{width:9px;height:9px;border-radius:50%;background:var(--green);box-shadow:0 0 13px var(--green)}.drive-area{height:calc(100% - 32px);display:grid;grid-template-columns:31% 69%;gap:8px;padding:4px 9px 9px}.left-controls{display:flex;flex-direction:column;justify-content:space-between;gap:6px;min-width:0}.btn{border:0;color:#fff;font-weight:900;letter-spacing:.5px;background:linear-gradient(145deg,#38b8ff,#087fdc 53%,#004eaa);box-shadow:0 9px 15px rgba(0,0,0,.38),inset 0 3px 4px rgba(255,255,255,.38),inset 0 -5px 7px rgba(0,30,80,.42);text-shadow:0 2px 3px rgba(0,0,0,.45)}.btn:active,.btn.pressed{transform:translateY(2px);box-shadow:0 4px 7px rgba(0,0,0,.45),inset 0 4px 8px rgba(0,20,60,.5)}.btn.big{height:48px;border-radius:16px;font-size:11px}.btn.small{height:38px;width:80%;align-self:center;border-radius:14px;font-size:10px}.btn.off{background:linear-gradient(145deg,#354861,#1b2d43 55%,#0c1726);color:#c6d4e2}.btn.stop{background:linear-gradient(145deg,#ff6f82,#ef304b 55%,#a80f27)}.readouts{display:grid;grid-template-columns:1fr 1fr;gap:5px}.mini{padding:6px 2px;border-radius:12px;background:linear-gradient(145deg,#182f4b,#0a1728);box-shadow:0 5px 9px rgba(0,0,0,.35),inset 0 1px rgba(255,255,255,.08);text-align:center}.mini span{display:block;font-size:8px;color:var(--muted);font-weight:900;letter-spacing:.7px}.mini b{font-size:15px}.right-controls{min-width:0;display:grid;grid-template-rows:28px minmax(0,1fr) 46px;gap:5px;align-items:center}.controlbox{width:88%;justify-self:center;padding:4px 10px;border-radius:15px;background:linear-gradient(145deg,#182f4b,#0a1728);box-shadow:0 6px 12px rgba(0,0,0,.35),inset 0 1px rgba(255,255,255,.08)}.rhead{display:flex;justify-content:space-between;font-size:9px;font-weight:900;margin-bottom:1px}.speedbox .rhead{font-size:12px}.joywrap{min-height:0;display:flex;align-items:center;justify-content:center}.joy{position:relative;width:min(46vw,190px);height:min(46vw,190px);max-width:100%;max-height:100%;aspect-ratio:1;border-radius:50%;touch-action:none;background:radial-gradient(circle at 35% 25%,rgba(255,255,255,.30),rgba(67,103,142,.42) 28%,rgba(18,41,68,.75) 61%,rgba(2,10,21,.96) 100%),linear-gradient(145deg,#233f61,#071426);box-shadow:18px 18px 32px rgba(0,0,0,.55),-8px -8px 20px rgba(57,108,157,.13),inset 8px 8px 18px rgba(255,255,255,.14),inset -12px -12px 25px rgba(0,0,0,.48)}.joy:before{content:"";position:absolute;width:78%;height:78%;left:11%;top:11%;border-radius:50%;background:radial-gradient(circle at center,rgba(16,141,255,.08),rgba(9,23,40,.35) 62%,rgba(0,0,0,.42));box-shadow:inset 10px 10px 22px rgba(0,0,0,.34),inset -9px -9px 18px rgba(93,145,192,.10)}.axisH,.axisV{position:absolute;z-index:2;border-radius:20px;background:linear-gradient(145deg,#3d5870,#122338);box-shadow:inset 2px 2px 5px rgba(255,255,255,.11),inset -2px -2px 5px rgba(0,0,0,.45)}.axisH{width:52%;height:5px;left:24%;top:50%}.axisV{height:52%;width:5px;top:24%;left:50%}.stick{position:absolute;z-index:5;left:32%;top:32%;width:36%;height:36%;border-radius:50%;pointer-events:none;background:radial-gradient(circle at 30% 22%,#fff,#bfe6ff 20%,#268fff 48%,#064bbd 78%,#021c56 100%);box-shadow:0 20px 22px rgba(0,0,0,.48),inset 8px 8px 14px rgba(255,255,255,.55),inset -10px -12px 18px rgba(0,0,0,.42),0 0 24px rgba(0,120,255,.48)}.stick:before{content:"";position:absolute;width:48%;height:24%;border-radius:50%;left:18%;top:13%;background:rgba(255,255,255,.45);filter:blur(1px);transform:rotate(-20deg)}.joyText{position:absolute;z-index:6;left:15%;bottom:5%;width:70%;height:25%;pointer-events:none}.joyText svg{width:100%;height:100%;overflow:visible}.joyText text{font-size:9px;font-weight:900;letter-spacing:1.2px;fill:#dcecff;opacity:.9}.range{width:100%;height:22px;margin:0;appearance:none;-webkit-appearance:none;background:transparent;touch-action:none}.range::-webkit-slider-runnable-track{height:7px;border-radius:8px;background:#263d55;box-shadow:inset 0 2px 4px rgba(0,0,0,.55)}.range::-webkit-slider-thumb{appearance:none;-webkit-appearance:none;width:22px;height:22px;margin-top:-8px;border-radius:50%;background:radial-gradient(circle at 30% 22%,#fff,#8ed5ff 34%,#0077ff 75%,#003d99);box-shadow:0 6px 10px rgba(0,0,0,.45),inset 3px 3px 6px rgba(255,255,255,.52),inset -3px -3px 6px rgba(0,0,0,.28)}.trimbox{width:62%;padding:3px 9px}.trimbox .range{height:14px}.trimbox .range::-webkit-slider-thumb{width:16px;height:16px;margin-top:-5px}.arm{height:calc(100% - 32px);display:grid;grid-template-rows:repeat(6,minmax(0,1fr));gap:4px;padding:3px 9px 9px}.servo{min-height:0;display:grid;grid-template-columns:62px 1fr 42px;align-items:center;gap:7px;padding:2px 9px;border-radius:14px;background:linear-gradient(145deg,#172f4b,#091627);box-shadow:0 4px 9px rgba(0,0,0,.34),inset 0 1px rgba(255,255,255,.07)}.name{font-size:18px;font-weight:900}.name small{display:block;color:var(--muted);font-size:11px;margin-top:1px;font-weight:800}.angle{text-align:right;font-size:15px;font-weight:900}.danger{outline:2px solid #ff4059}@media(max-height:700px){.app{grid-template-rows:46% 54%}.title{height:29px;font-size:13px}.drive-area,.arm{height:calc(100% - 29px)}.btn.big{height:42px}.btn.small{height:33px}.right-controls{grid-template-rows:26px minmax(0,1fr) 41px}.joy{width:min(42vw,170px);height:min(42vw,170px)}.servo{padding:1px 8px}.name{font-size:16px}.name small{font-size:10px}}
</style></head><body><main class="app"><section class="panel"><div class="title"><span class="dot"></span>Mobile Robotic Manipulator</div><div class="drive-area"><div class="left-controls"><button id="lightButton" class="btn big off" onclick="toggleLight()">LIGHT OFF</button><button id="buzzerButton" class="btn small">HOLD HORN</button><button id="gyroButton" class="btn small" onclick="toggleGyro()">TILT ON</button><button class="btn big stop" onclick="emergencyStop()">STOP</button><div class="readouts"><div class="mini"><span>STEER</span><b id="xd">0</b></div><div class="mini"><span>DRIVE</span><b id="yd">0</b></div></div></div><div class="right-controls"><div class="controlbox trimbox"><div class="rhead"><span>TRIM L/R</span><span id="trimValue">0</span></div><input id="trim" class="range" type="range" min="-25" max="25" value="0"></div><div class="joywrap"><div id="joy" class="joy"><div class="axisH"></div><div class="axisV"></div><div id="stick" class="stick"></div><div class="joyText"><svg viewBox="0 0 210 58"><path id="uTextPath" d="M20 0 Q105 65 199 0" fill="none"/><text><textPath href="#uTextPath" startOffset="50%" text-anchor="middle">360° SMOOTH DRIVE</textPath></text></svg></div></div></div><div class="controlbox speedbox"><div class="rhead"><span>SPEED</span><span id="speedValue">55%</span></div><input id="speed" class="range" type="range" min="15" max="100" value="55"></div></div></div></section><section class="panel"><div class="title">Six-Axis Robotic Arm</div><div class="arm">
<div class="servo"><div class="name">A<small>Gripper</small></div><input class="range" type="range" min="0" max="180" value="0" data-servo="A"><div id="valueA" class="angle">0°</div></div>
<div class="servo"><div class="name">B<small>Roll</small></div><input class="range" type="range" min="0" max="180" value="0" data-servo="B"><div id="valueB" class="angle">0°</div></div>
<div class="servo"><div class="name">C<small>Pitch</small></div><input class="range" type="range" min="0" max="180" value="94" data-servo="C"><div id="valueC" class="angle">94°</div></div>
<div class="servo"><div class="name">D<small>Elbow</small></div><input class="range" type="range" min="0" max="180" value="16" data-servo="D"><div id="valueD" class="angle">16°</div></div>
<div class="servo"><div class="name">E<small>Shoulder</small></div><input class="range" type="range" min="0" max="180" value="180" data-servo="E"><div id="valueE" class="angle">180°</div></div>
<div class="servo"><div class="name">F<small>Base</small></div><input class="range" type="range" min="0" max="180" value="73" data-servo="F"><div id="valueF" class="angle">73°</div></div>
</div></section></main><script>
document.addEventListener('contextmenu',e=>e.preventDefault());document.addEventListener('selectstart',e=>e.preventDefault());document.addEventListener('gesturestart',e=>e.preventDefault());document.addEventListener('dblclick',e=>e.preventDefault());let lastTouch=0;document.addEventListener('touchend',e=>{const n=Date.now();if(n-lastTouch<320)e.preventDefault();lastTouch=n},{passive:false});
const joy=document.getElementById('joy'),stick=document.getElementById('stick'),xd=document.getElementById('xd'),yd=document.getElementById('yd'),speed=document.getElementById('speed'),trim=document.getElementById('trim');let active=false,pid=null,x=0,y=0,lastSend=0;const interval=42;speed.oninput=()=>speedValue.textContent=speed.value+'%';trim.oninput=()=>trimValue.textContent=trim.value;function sendDrive(force=false){const n=Date.now();if(!force&&n-lastSend<interval)return;lastSend=n;fetch(`/drive?x=${x}&y=${y}&speed=${speed.value}&trim=${trim.value}`,{cache:'no-store'}).catch(()=>{})}function move(cx,cy){const r=joy.getBoundingClientRect(),mx=r.left+r.width/2,my=r.top+r.height/2,rad=r.width*.31;let dx=cx-mx,dy=cy-my,d=Math.hypot(dx,dy);if(d>rad){dx*=rad/d;dy*=rad/d}stick.style.transform=`translate(${dx}px,${dy}px)`;x=Math.round(dx/rad*100);y=Math.round(-dy/rad*100);if(Math.abs(x)<5)x=0;if(Math.abs(y)<5)y=0;xd.textContent=x;yd.textContent=y;sendDrive()}function reset(){active=false;pid=null;x=0;y=0;stick.style.transform='translate(0,0)';xd.textContent='0';yd.textContent='0';sendDrive(true)}joy.onpointerdown=e=>{e.preventDefault();active=true;pid=e.pointerId;joy.setPointerCapture(pid);move(e.clientX,e.clientY)};joy.onpointermove=e=>{if(active&&e.pointerId===pid){e.preventDefault();move(e.clientX,e.clientY)}};joy.onpointerup=e=>{if(e.pointerId===pid)reset()};joy.onpointercancel=reset;joy.onlostpointercapture=reset;setInterval(()=>{if(active)sendDrive(true)},160);function emergencyStop(){reset();fetch('/stop',{cache:'no-store'}).catch(()=>{})}let gyroEnabled=true,headlightEnabled=false;function toggleGyro(){gyroEnabled=!gyroEnabled;fetch(`/tilt-enable?state=${gyroEnabled?1:0}`,{cache:'no-store'}).catch(()=>{});updateButtons()}function toggleLight(){headlightEnabled=!headlightEnabled;fetch(`/light?state=${headlightEnabled?1:0}`,{cache:'no-store'}).catch(()=>{});updateButtons()}function updateButtons(){gyroButton.textContent=gyroEnabled?'TILT ON':'TILT OFF';gyroButton.classList.toggle('off',!gyroEnabled);lightButton.textContent=headlightEnabled?'LIGHT ON':'LIGHT OFF';lightButton.classList.toggle('off',!headlightEnabled)}const horn=document.getElementById('buzzerButton');let held=false,ht=null;function hornOn(e){e&&e.preventDefault();if(held)return;held=true;horn.classList.add('pressed');fetch('/buzzer?state=1').catch(()=>{});ht=setInterval(()=>fetch('/buzzer?state=1').catch(()=>{}),300)}function hornOff(e){e&&e.preventDefault();held=false;horn.classList.remove('pressed');clearInterval(ht);fetch('/buzzer?state=0').catch(()=>{})}horn.onpointerdown=hornOn;horn.onpointerup=hornOff;horn.onpointercancel=hornOff;horn.onpointerleave=e=>{if(held)hornOff(e)};document.querySelectorAll('.servo input').forEach(sl=>{let t;sl.oninput=()=>{const id=sl.dataset.servo,a=Number(sl.value);document.getElementById('value'+id).textContent=a+'°';clearTimeout(t);t=setTimeout(()=>fetch(`/servo?id=${id}&angle=${a}`,{cache:'no-store'}).catch(()=>{}),18)};sl.onchange=()=>fetch(`/servo?id=${sl.dataset.servo}&angle=${sl.value}`,{cache:'no-store'}).catch(()=>{})});function status(){fetch('/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{gyroEnabled=!!d.tiltEnabled;headlightEnabled=!!d.light;updateButtons();gyroButton.classList.toggle('danger',!!d.danger)}).catch(()=>{})}setInterval(status,220);status();document.addEventListener('visibilitychange',()=>{if(document.hidden){emergencyStop();hornOff()}});window.addEventListener('pagehide',()=>{navigator.sendBeacon('/stop');navigator.sendBeacon('/buzzer?state=0')});
</script></body></html>
)rawliteral";


void writeLogicOutput(
  uint8_t pin,
  bool state,
  bool activeHigh
) {
  digitalWrite(
    pin,
    state == activeHigh ? HIGH : LOW
  );
}

void setBuzzer(bool state) {
  if(state){beepSequenceActive=false;beepPulsesRemaining=0;}
  buzzerOn = state;

  writeLogicOutput(
    BUZZER_PIN,
    buzzerOn,
    BUZZER_ACTIVE_HIGH
  );

  if (state) {
    lastBuzzerRefreshTime = millis();
  }
}

void startBeepSequence(uint8_t pulses){
  if(buzzerOn) return;
  beepSequenceActive=true;
  beepPulsesRemaining=pulses;
  beepOutputState=true;
  writeLogicOutput(BUZZER_PIN,true,BUZZER_ACTIVE_HIGH);
  beepPhaseTime=millis();
}

void updateBeepSequence(){
  if(!beepSequenceActive||buzzerOn)return;
  unsigned long now=millis();
  if(beepOutputState){
    if(now-beepPhaseTime>=BEEP_ON_MS){
      beepOutputState=false;
      writeLogicOutput(BUZZER_PIN,false,BUZZER_ACTIVE_HIGH);
      beepPhaseTime=now;
      if(beepPulsesRemaining>0)beepPulsesRemaining--;
      if(beepPulsesRemaining==0)beepSequenceActive=false;
    }
  }else if(now-beepPhaseTime>=BEEP_OFF_MS){
    beepOutputState=true;
    writeLogicOutput(BUZZER_PIN,true,BUZZER_ACTIVE_HIGH);
    beepPhaseTime=now;
  }
}

void monitorWiFiClients(){
  int clients=WiFi.softAPgetStationNum();
  if(clients>previousClientCount)startBeepSequence(1);
  else if(clients<previousClientCount)startBeepSequence(2);
  previousClientCount=clients;
}

void setHeadlight(bool state) {
  headlightOn = state;

  writeLogicOutput(
    HEADLIGHT_PIN,
    headlightOn,
    HEADLIGHT_ACTIVE_HIGH
  );
}

void writeMotorSide(
  int command,
  uint8_t rpwmChannel,
  uint8_t lpwmChannel,
  bool reverseDirection
) {
  command = constrain(command, -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);

  if (reverseDirection) {
    command = -command;
  }

  if (command > 0) {
    ledcWriteChannel(rpwmChannel, command);
    ledcWriteChannel(lpwmChannel, 0);
  }
  else if (command < 0) {
    ledcWriteChannel(rpwmChannel, 0);
    ledcWriteChannel(lpwmChannel, -command);
  }
  else {
    ledcWriteChannel(rpwmChannel, 0);
    ledcWriteChannel(lpwmChannel, 0);
  }
}

void outputMotors(int leftCommand,int rightCommand){
  if(tiltProtectionEnabled&&dangerousTilt){leftCommand=0;rightCommand=0;}
  writeMotorSide(leftCommand,LEFT_RPWM_CHANNEL,LEFT_LPWM_CHANNEL,REVERSE_LEFT_SIDE);
  writeMotorSide(rightCommand,RIGHT_RPWM_CHANNEL,RIGHT_LPWM_CHANNEL,REVERSE_RIGHT_SIDE);
}

void driveMotors(int leftCommand,int rightCommand){
  targetLeftMotor=constrain(leftCommand,-MOTOR_SPEED_LIMIT,MOTOR_SPEED_LIMIT);
  targetRightMotor=constrain(rightCommand,-MOTOR_SPEED_LIMIT,MOTOR_SPEED_LIMIT);
}

void stopMotors(){
  targetLeftMotor=0;
  targetRightMotor=0;
}

void hardStopMotors(){
  targetLeftMotor=targetRightMotor=currentLeftMotor=currentRightMotor=0;
  outputMotors(0,0);
}

int rampValue(int currentValue,int targetValue,int step){
  if(currentValue<targetValue)return min(currentValue+step,targetValue);
  if(currentValue>targetValue)return max(currentValue-step,targetValue);
  return currentValue;
}

void updateMotorRamp(){
  unsigned long now=millis();
  if(now-lastMotorRampTime<MOTOR_RAMP_INTERVAL_MS)return;
  lastMotorRampTime=now;
  if(tiltProtectionEnabled&&dangerousTilt){targetLeftMotor=0;targetRightMotor=0;}
  currentLeftMotor=rampValue(currentLeftMotor,targetLeftMotor,MOTOR_RAMP_STEP);
  currentRightMotor=rampValue(currentRightMotor,targetRightMotor,MOTOR_RAMP_STEP);
  outputMotors(currentLeftMotor,currentRightMotor);
}

int applyJoystickDeadZone(int value) {
  if (abs(value) <= JOYSTICK_DEADZONE) {
    return 0;
  }

  return value;
}

int percentageToPWM(int percentage) {
  return map(
    constrain(abs(percentage), 0, 100),
    0,
    100,
    0,
    MOTOR_SPEED_LIMIT
  );
}

void writeDirectServo(
  uint8_t servoChannel,
  int angle
) {
  angle = constrain(angle, 0, 180);

  uint32_t pulseWidthUs = map(
    angle,
    0,
    180,
    DIRECT_SERVO_MIN_US,
    DIRECT_SERVO_MAX_US
  );

  uint32_t duty =
    ((uint64_t)pulseWidthUs * 65535ULL) /
    20000ULL;

  ledcWriteChannel(servoChannel, duty);
}

void writePCA9685Servo(
  uint8_t channel,
  int angle
) {
  angle = constrain(angle, 0, 180);

  uint16_t pulse = map(
    angle,
    0,
    180,
    PCA_SERVO_MIN,
    PCA_SERVO_MAX
  );

  pca9685.setPWM(
    channel,
    0,
    pulse
  );
}

void writeArmNow(char servoID,int angle){
  angle=constrain(angle,0,180);
  switch(servoID){
    case 'A': armAngleA=angle; writeDirectServo(SERVO_A_CHANNEL,angle); break;
    case 'B': armAngleB=angle; writeDirectServo(SERVO_B_CHANNEL,angle); break;
    case 'C': armAngleC=angle; writePCA9685Servo(SERVO_C_CHANNEL,angle); break;
    case 'D': armAngleD=angle; writePCA9685Servo(SERVO_D_CHANNEL,angle); break;
    case 'E': armAngleE=angle; writePCA9685Servo(SERVO_E_CHANNEL,angle); break;
    case 'F': armAngleF=angle; writePCA9685Servo(SERVO_F_CHANNEL,180-angle); break;
  }
}

void setArmServo(char servoID,int angle){
  angle=constrain(angle,0,180);
  switch(servoID){
    case 'A': targetArmA=angle; writeArmNow('A',angle); break;
    case 'B': targetArmB=angle; writeArmNow('B',angle); break;
    case 'C': targetArmC=angle; break;
    case 'D': targetArmD=angle; break;
    case 'E': targetArmE=angle; break;
    case 'F': targetArmF=angle; break;
  }
}

void updateServoRamp(){
  unsigned long now=millis();
  if(now-lastServoRampTime<SERVO_RAMP_INTERVAL_MS)return;
  lastServoRampTime=now;
  int c=rampValue(armAngleC,targetArmC,1),d=rampValue(armAngleD,targetArmD,1),e=rampValue(armAngleE,targetArmE,1),f=rampValue(armAngleF,targetArmF,1);
  if(c!=armAngleC)writeArmNow('C',c);
  if(d!=armAngleD)writeArmNow('D',d);
  if(e!=armAngleE)writeArmNow('E',e);
  if(f!=armAngleF)writeArmNow('F',f);
}

void calculateRawTilt(
  float ax,
  float ay,
  float az,
  float& rawRoll,
  float& rawPitch
) {
  rawRoll =
    atan2f(
      ay,
      sqrtf(ax * ax + az * az)
    ) * 180.0f / PI;

  rawPitch =
    atan2f(
      -ax,
      sqrtf(ay * ay + az * az)
    ) * 180.0f / PI;
}

void calibrateMPU() {
  if (!mpuDetected) {
    return;
  }

  Serial.println(
    "Keep the UGV flat and still."
  );

  Serial.println(
    "Calibrating MPU6050..."
  );

  const int samples = 120;

  float rollTotal = 0.0f;
  float pitchTotal = 0.0f;

  for (int i = 0; i < samples; i++) {

    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu6050.getEvent(
      &acceleration,
      &gyro,
      &temperature
    );

    float rawRoll;
    float rawPitch;

    calculateRawTilt(
      acceleration.acceleration.x,
      acceleration.acceleration.y,
      acceleration.acceleration.z,
      rawRoll,
      rawPitch
    );

    rollTotal += rawRoll;
    pitchTotal += rawPitch;

    delay(10);
  }

  rollOffset = rollTotal / samples;
  pitchOffset = pitchTotal / samples;

  filteredRoll = 0.0f;
  filteredPitch = 0.0f;

  Serial.print("Roll offset: ");
  Serial.println(rollOffset, 2);

  Serial.print("Pitch offset: ");
  Serial.println(pitchOffset, 2);

  Serial.println(
    "MPU6050 calibration complete."
  );
}

void updateMPU6050() {
  if (!mpuDetected) {
    return;
  }

  unsigned long now = millis();

  if (
    now - lastMPUReadTime <
    MPU_INTERVAL_MS
  ) {
    return;
  }

  lastMPUReadTime = now;

  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu6050.getEvent(
    &acceleration,
    &gyro,
    &temperature
  );

  float rawRoll;
  float rawPitch;

  calculateRawTilt(
    acceleration.acceleration.x,
    acceleration.acceleration.y,
    acceleration.acceleration.z,
    rawRoll,
    rawPitch
  );

  rawRoll -= rollOffset;
  rawPitch -= pitchOffset;

  const float filterAlpha = 0.22f;

  filteredRoll =
    filteredRoll +
    filterAlpha * (rawRoll - filteredRoll);

  filteredPitch =
    filteredPitch +
    filterAlpha * (rawPitch - filteredPitch);

  rollAngle = filteredRoll;
  pitchAngle = filteredPitch;

  if (!tiltProtectionEnabled) {
    dangerousTilt = false;
    return;
  }

  bool thresholdExceeded =
    abs(rollAngle) >= TILT_STOP_ANGLE ||
    abs(pitchAngle) >= TILT_STOP_ANGLE;

  bool safelyRecovered =
    abs(rollAngle) <= TILT_RESET_ANGLE &&
    abs(pitchAngle) <= TILT_RESET_ANGLE;

  if (thresholdExceeded) {

    if (!dangerousTilt) {
      Serial.println(
        "DANGEROUS TILT: MOTORS STOPPED"
      );
    }

    dangerousTilt = true;
    hardStopMotors();
  }
  else if (
    dangerousTilt &&
    safelyRecovered
  ) {
    dangerousTilt = false;

    Serial.println(
      "Tilt returned to safe range."
    );
  }
}

void retryMPUDetection(){
  static unsigned long lastRetry=0;
  if(mpuDetected||millis()-lastRetry<3000)return;
  lastRetry=millis();
  mpuDetected=mpu6050.begin(0x68,&MPU_WIRE);
  if(!mpuDetected)mpuDetected=mpu6050.begin(0x69,&MPU_WIRE);
  if(mpuDetected){
    mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);
    tiltProtectionEnabled=true;
    calibrateMPU();
  }
}

void handleMainPage() {
  server.send_P(
    200,
    "text/html",
    WEBPAGE
  );
}

void handleDrive(){
  if(!server.hasArg("x")||!server.hasArg("y")){stopMotors();server.send(400,"text/plain","Missing joystick values");return;}
  if(tiltProtectionEnabled&&dangerousTilt){hardStopMotors();server.send(423,"text/plain","Drive locked by tilt protection");return;}
  int rawX=constrain(server.arg("x").toInt(),-100,100);
  int rawY=constrain(server.arg("y").toInt(),-100,100);
  int steering=-rawX;
  int throttle=-rawY;
  if(rawY<0) steering=-steering;
  int speedPercent=server.hasArg("speed")?constrain(server.arg("speed").toInt(),15,100):55;
  int trimValue=server.hasArg("trim")?constrain(server.arg("trim").toInt(),-25,25):0;
  steering=applyJoystickDeadZone(steering);
  throttle=applyJoystickDeadZone(throttle);
  if(throttle!=0) steering=constrain(steering+trimValue,-100,100);
  int leftPercentage=throttle+steering;
  int rightPercentage=throttle-steering;
  int largestMagnitude=max(abs(leftPercentage),abs(rightPercentage));
  if(largestMagnitude>100){leftPercentage=leftPercentage*100/largestMagnitude;rightPercentage=rightPercentage*100/largestMagnitude;}
  leftPercentage=leftPercentage*speedPercent/100;
  rightPercentage=rightPercentage*speedPercent/100;
  int leftPWM=leftPercentage==0?0:percentageToPWM(leftPercentage)*(leftPercentage<0?-1:1);
  int rightPWM=rightPercentage==0?0:percentageToPWM(rightPercentage)*(rightPercentage<0?-1:1);
  driveMotors(leftPWM,rightPWM);
  lastMotorCommandTime=millis();
  server.send(200,"text/plain","OK");
}

void handleServo() {
  if (
    !server.hasArg("id") ||
    !server.hasArg("angle")
  ) {
    server.send(
      400,
      "text/plain",
      "Missing servo data"
    );

    return;
  }

  String idArgument = server.arg("id");

  if (idArgument.length() != 1) {
    server.send(
      400,
      "text/plain",
      "Invalid servo ID"
    );

    return;
  }

  char servoID =
    toupper(idArgument.charAt(0));

  if (
    servoID < 'A' ||
    servoID > 'F'
  ) {
    server.send(
      400,
      "text/plain",
      "Servo must be A-F"
    );

    return;
  }

  int angle = constrain(
    server.arg("angle").toInt(),
    0,
    180
  );

  setArmServo(
    servoID,
    angle
  );

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

void handleTiltEnable() {
  if (!server.hasArg("state")) {
    server.send(
      400,
      "text/plain",
      "Missing state"
    );

    return;
  }

  tiltProtectionEnabled =
    server.arg("state").toInt() == 1;

  if (!tiltProtectionEnabled) {
    dangerousTilt = false;
  }
  else {
    bool currentlyUnsafe =
      abs(rollAngle) >= TILT_STOP_ANGLE ||
      abs(pitchAngle) >= TILT_STOP_ANGLE;

    if (currentlyUnsafe) {
      dangerousTilt = true;
      stopMotors();
    }
  }

  server.send(
    200,
    "text/plain",
    tiltProtectionEnabled
      ? "TILT PROTECTION ON"
      : "TILT PROTECTION OFF"
  );
}

void handleLight() {
  if (!server.hasArg("state")) {
    server.send(
      400,
      "text/plain",
      "Missing state"
    );

    return;
  }

  setHeadlight(
    server.arg("state").toInt() == 1
  );

  server.send(
    200,
    "text/plain",
    headlightOn
      ? "LIGHT ON"
      : "LIGHT OFF"
  );
}

void handleBuzzer() {
  if (!server.hasArg("state")) {
    setBuzzer(false);

    server.send(
      400,
      "text/plain",
      "Missing state"
    );

    return;
  }

  bool requestedState =
    server.arg("state").toInt() == 1;

  setBuzzer(requestedState);

  server.send(
    200,
    "text/plain",
    buzzerOn
      ? "BUZZER ON"
      : "BUZZER OFF"
  );
}

void handleStop() {
  hardStopMotors();
  setBuzzer(false);

  server.send(
    200,
    "text/plain",
    "STOPPED"
  );
}

void handleStatus() {
  String json = "{";

  json += "\"mpu\":";
  json += mpuDetected ? "true" : "false";

  json += ",\"roll\":";
  json += String(rollAngle, 1);

  json += ",\"pitch\":";
  json += String(pitchAngle, 1);

  json += ",\"tiltEnabled\":";
  json += tiltProtectionEnabled
    ? "true"
    : "false";

  json += ",\"danger\":";
  json += dangerousTilt
    ? "true"
    : "false";

  json += ",\"light\":";
  json += headlightOn
    ? "true"
    : "false";

  json += ",\"buzzer\":";
  json += buzzerOn
    ? "true"
    : "false";

  json += ",\"A\":";
  json += armAngleA;

  json += ",\"B\":";
  json += armAngleB;

  json += ",\"C\":";
  json += armAngleC;

  json += ",\"D\":";
  json += armAngleD;

  json += ",\"E\":";
  json += armAngleE;

  json += ",\"F\":";
  json += armAngleF;

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}

void handleNotFound() {
  server.send(
    404,
    "text/plain",
    "Page not found"
  );
}

void setup() {
  Serial.begin(115200);

  delay(300);

  Serial.println();
  Serial.println(
    "===================================="
  );
  Serial.println(
    "Mobile Robotic Manipulator"
  );
  Serial.println(
    "===================================="
  );

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(HEADLIGHT_PIN, OUTPUT);

  setBuzzer(false);
  setHeadlight(false);

  bool motor1 = ledcAttachChannel(
    LEFT_RPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION,
    LEFT_RPWM_CHANNEL
  );

  bool motor2 = ledcAttachChannel(
    LEFT_LPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION,
    LEFT_LPWM_CHANNEL
  );

  bool motor3 = ledcAttachChannel(
    RIGHT_RPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION,
    RIGHT_RPWM_CHANNEL
  );

  bool motor4 = ledcAttachChannel(
    RIGHT_LPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION,
    RIGHT_LPWM_CHANNEL
  );

  if (
    !motor1 ||
    !motor2 ||
    !motor3 ||
    !motor4
  ) {
    Serial.println(
      "Motor PWM initialization failed."
    );
  }

  hardStopMotors();

  bool directServoA = ledcAttachChannel(
    SERVO_A_PIN,
    SERVO_PWM_FREQUENCY,
    SERVO_PWM_RESOLUTION,
    SERVO_A_CHANNEL
  );

  bool directServoB = ledcAttachChannel(
    SERVO_B_PIN,
    SERVO_PWM_FREQUENCY,
    SERVO_PWM_RESOLUTION,
    SERVO_B_CHANNEL
  );

  if (
    !directServoA ||
    !directServoB
  ) {
    Serial.println(
      "Direct servo PWM initialization failed."
    );
  }

  Wire.begin(
    PCA_SDA_PIN,
    PCA_SCL_PIN,
    400000
  );

  pca9685.begin();
  pca9685.setOscillatorFrequency(27000000);
  pca9685.setPWMFreq(50);

  delay(10);

  MPU_WIRE.begin(
    MPU_SDA_PIN,
    MPU_SCL_PIN,
    100000
  );

  mpuDetected = mpu6050.begin(0x68, &MPU_WIRE);
  if (!mpuDetected) mpuDetected = mpu6050.begin(0x69, &MPU_WIRE);

  if (mpuDetected) {

    mpu6050.setAccelerometerRange(
      MPU6050_RANGE_8_G
    );

    mpu6050.setGyroRange(
      MPU6050_RANGE_500_DEG
    );

    mpu6050.setFilterBandwidth(
      MPU6050_BAND_21_HZ
    );

    Serial.println(
      "MPU6050 detected."
    );

    calibrateMPU();
  }
  else {
    tiltProtectionEnabled = false;

    Serial.println(
      "MPU6050 not detected."
    );

    Serial.println(
      "Tilt protection disabled."
    );
  }

  writeArmNow('A',0);
  writeArmNow('B',0);
  writeArmNow('C',94);
  writeArmNow('D',16);
  writeArmNow('E',180);
  writeArmNow('F',73);

  Serial.println(
    "Servo positions: "
    "A=0 B=0 C=94 D=16 E=180 F=73"
  );

  WiFi.mode(WIFI_AP);

  bool accessPointStarted = WiFi.softAP(
    WIFI_NAME,
    WIFI_PASSWORD
  );

  if (accessPointStarted) {
    Serial.println(
      "Wi-Fi access point started."
    );
  }
  else {
    Serial.println(
      "Wi-Fi access point failed."
    );
  }

  Serial.print("Wi-Fi: ");
  Serial.println(WIFI_NAME);

  Serial.print("Password: ");
  Serial.println(WIFI_PASSWORD);

  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());

  server.on(
    "/",
    HTTP_GET,
    handleMainPage
  );

  server.on(
    "/drive",
    HTTP_GET,
    handleDrive
  );

  server.on(
    "/servo",
    HTTP_GET,
    handleServo
  );

  server.on(
    "/tilt-enable",
    HTTP_GET,
    handleTiltEnable
  );

  server.on(
    "/light",
    HTTP_GET,
    handleLight
  );

  server.on(
    "/buzzer",
    HTTP_ANY,
    handleBuzzer
  );

  server.on(
    "/stop",
    HTTP_ANY,
    handleStop
  );

  server.on(
    "/status",
    HTTP_GET,
    handleStatus
  );

  server.onNotFound(handleNotFound);

  server.begin();

  lastMotorCommandTime = millis();

  Serial.println(
    "Web server started."
  );

  Serial.println(
    "System ready."
  );
}

void loop() {
  server.handleClient();

  updateMPU6050();
  retryMPUDetection();
  monitorWiFiClients();
  updateBeepSequence();
  updateMotorRamp();
  updateServoRamp();

  unsigned long now = millis();

  if (
    now - lastMotorCommandTime >
    MOTOR_COMMAND_TIMEOUT_MS
  ) {
    stopMotors();
  }

  if (
    buzzerOn &&
    now - lastBuzzerRefreshTime >
    BUZZER_TIMEOUT_MS
  ) {
    setBuzzer(false);
  }
}