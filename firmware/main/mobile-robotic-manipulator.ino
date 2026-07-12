#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
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

const uint8_t MPU_ADDRESS_PRIMARY = 0x68;
const uint8_t MPU_ADDRESS_SECONDARY = 0x69;
uint8_t mpuAddress = MPU_ADDRESS_PRIMARY;

const uint8_t MPU_PWR_MGMT_1 = 0x6B;
const uint8_t MPU_CONFIG = 0x1A;
const uint8_t MPU_GYRO_CONFIG = 0x1B;
const uint8_t MPU_ACCEL_CONFIG = 0x1C;
const uint8_t MPU_DATA_START = 0x3B;
const uint8_t MPU_WHO_AM_I = 0x75;

bool mpuDetected = false;
bool tiltProtectionEnabled = true;
bool dangerousTilt = false;

float rollAngle = 0.0f;
float pitchAngle = 0.0f;
float yawAngle = 0.0f;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

const float TILT_STOP_ANGLE = 45.0f;
const float TILT_RESET_ANGLE = 40.0f;
const float COMPLEMENTARY_GYRO_WEIGHT = 0.96f;

const unsigned long MPU_INTERVAL_MS = 10;
unsigned long lastMPUReadTime = 0;

const unsigned long STATUS_INTERVAL_MS = 150;

int armAngleA = 0;
int armAngleB = 0;
int armAngleC = 8;
int armAngleD = 8;
int armAngleE = 165;
int armAngleF = 73;
int targetArmA = 0;
int targetArmB = 0;
int targetArmC = 8;
int targetArmD = 8;
int targetArmE = 165;
int targetArmF = 73;
const unsigned long SERVO_RAMP_INTERVAL_MS = 22;
unsigned long lastServoRampTime = 0;

const char WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,minimum-scale=1,user-scalable=no,viewport-fit=cover"><title>Mobile Robotic Manipulator</title><style>
:root{--blue:#20aef2;--blue2:#087fc7;--ink:#17324a;--muted:#71869a;--glass:rgba(255,255,255,.82)}*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}html{width:100%;min-height:100%;overscroll-behavior:none}body{width:100%;min-height:100%;margin:0;overflow-x:hidden;overscroll-behavior:none;font-family:Montserrat,Arial,sans-serif;color:var(--ink);background:radial-gradient(circle at 50% -10%,#fff 0,#effaff 44%,#dceff8 100%);padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}button,input,.joy{touch-action:none}.app{width:100%;max-width:980px;margin:auto;padding:8px;display:grid;grid-template-rows:auto auto;gap:9px}.panel{min-height:0;border-radius:28px;border:1px solid rgba(255,255,255,.98);background:linear-gradient(145deg,rgba(255,255,255,.98),rgba(221,242,251,.9));box-shadow:0 16px 34px rgba(70,126,157,.22),inset 0 1px 0 #fff;overflow:hidden}.main-title{height:55px;display:flex;align-items:center;justify-content:center;gap:10px;text-align:center;font-size:18px;font-weight:950;letter-spacing:.8px;text-transform:uppercase}.dot{width:10px;height:10px;border-radius:50%;background:#35d58c;box-shadow:0 0 13px #35d58c}.drive{display:grid;grid-template-columns:minmax(0,1fr) 175px;gap:10px;padding:2px 12px 14px}.left-controls{min-width:0;display:grid;grid-template-rows:auto auto auto;align-content:center;justify-items:center;gap:8px}.trim-box{width:min(76%,280px);padding:5px 12px!important}.trim-box .rangehead{margin-bottom:-2px}.trim-box input[type=range]{height:24px}.trim-box input[type=range]::-webkit-slider-runnable-track{height:6px}.trim-box input[type=range]::-webkit-slider-thumb{width:17px;height:17px;margin-top:-6px}.joy-wrap{width:100%;display:grid;place-items:center}.joy-label{margin-bottom:-3px;font-size:12px;font-weight:900;letter-spacing:2px;color:#657787;text-transform:uppercase}.joy{position:relative;width:min(68vw,330px);height:min(68vw,330px);aspect-ratio:1;border-radius:50%;background:radial-gradient(circle at 44% 34%,#fff 0,#fbfeff 26%,#e8f4f9 52%,#d4e5ed 100%);border:22px solid rgba(255,255,255,.94);box-shadow:0 22px 38px rgba(69,119,147,.25),0 0 0 18px rgba(232,240,245,.92),0 0 0 31px rgba(255,255,255,.88),inset 0 0 0 4px #d2e4ec,inset 16px 17px 25px rgba(255,255,255,.95),inset -18px -20px 25px rgba(84,127,151,.19)}.joy:before,.joy:after{content:"";position:absolute;left:13%;right:13%;top:50%;height:10px;transform:translateY(-50%);border-radius:12px;background:linear-gradient(180deg,#c4d0d8,#e5edf2);box-shadow:inset 0 2px 4px rgba(66,91,108,.22)}.joy:after{top:13%;bottom:13%;left:50%;right:auto;width:10px;height:auto;transform:translateX(-50%)}.stick{position:absolute;z-index:2;left:29%;top:29%;width:42%;height:42%;border-radius:50%;pointer-events:none;background:radial-gradient(circle at 29% 20%,#fff 0,#d8f4ff 10%,#7bd7ff 25%,#24b4f4 49%,#098ed4 73%,#075f9f 100%);border:3px solid rgba(255,255,255,.9);box-shadow:0 18px 26px rgba(18,102,158,.38),0 0 22px rgba(49,185,245,.34),inset 10px 11px 12px rgba(255,255,255,.4),inset -11px -13px 15px rgba(0,61,113,.28);transition:transform .04s linear}.speed-box{width:min(95%,420px);padding:11px 18px!important;border-radius:34px!important}.speed-box .rangehead{font-size:17px;margin-bottom:2px}.speed-box input[type=range]{height:35px}.side{display:flex;flex-direction:column;gap:7px;min-height:0}.card,.control{border-radius:19px;background:linear-gradient(145deg,#fff,#e6f4fa);border:1px solid #fff;box-shadow:0 8px 15px rgba(66,121,151,.15),inset 0 1px 0 #fff;padding:9px}.minirow{display:grid;grid-template-columns:1fr 1fr;gap:7px}.label{font-size:8px;color:var(--muted);font-weight:850;letter-spacing:.8px;text-transform:uppercase}.value{font-size:17px;font-weight:950;margin-top:3px}.button{min-height:43px;border:1px solid rgba(255,255,255,.98);border-radius:16px;color:#fff;font-size:10px;font-weight:950;letter-spacing:.4px;background:linear-gradient(145deg,#82dcff,#149ee6);box-shadow:0 8px 14px rgba(25,144,205,.25),inset 0 2px 2px rgba(255,255,255,.55),inset 0 -3px 4px rgba(0,77,137,.2)}.button:active,.button.pressed{transform:translateY(2px)}.button.off{background:linear-gradient(145deg,#f5fbfd,#cbdde5);color:#536c7c}.button.stop{background:linear-gradient(145deg,#ff929c,#e7394a)}.twobtn{display:grid;grid-template-columns:1fr 1fr;gap:7px}.rangehead{display:flex;justify-content:space-between;align-items:center;font-size:10px;font-weight:950;margin-bottom:2px}.tilt.danger{background:#ffe4e7}.arm-title{height:45px;display:flex;align-items:center;justify-content:center;font-size:15px;font-weight:950;letter-spacing:1.7px;text-transform:uppercase}.arm{display:grid;grid-template-rows:repeat(6,68px);gap:7px;padding:2px 12px 14px}.servo{display:grid;grid-template-columns:78px 1fr 52px;align-items:center;gap:9px;padding:5px 12px;border-radius:19px;background:linear-gradient(145deg,#fff,#e8f6fc);border:1px solid #fff;box-shadow:0 6px 11px rgba(67,123,153,.14),inset 0 1px 0 #fff}.name{font-size:16px;font-weight:950}.name small{display:block;color:var(--muted);font-size:8px;margin-top:2px}.angle{text-align:right;font-size:15px;font-weight:950}input[type=range]{width:100%;height:32px;margin:0;appearance:none;-webkit-appearance:none;background:transparent}input[type=range]::-webkit-slider-runnable-track{height:9px;border-radius:9px;background:linear-gradient(180deg,#cbd9e0,#eef6f9);box-shadow:inset 0 2px 4px rgba(61,94,114,.23)}input[type=range]::-webkit-slider-thumb{appearance:none;-webkit-appearance:none;width:27px;height:27px;margin-top:-9px;border-radius:50%;background:radial-gradient(circle at 29% 22%,#fff,#c6eeff 25%,#42c2fb 60%,#0b84cb);border:3px solid #fff;box-shadow:0 6px 10px rgba(16,121,181,.3),inset 3px 3px 4px rgba(255,255,255,.72)}
@media(orientation:landscape){body{overflow:hidden}.app{max-width:none;height:100svh;grid-template-columns:48% 52%;grid-template-rows:100%;gap:8px}.main-title{height:38px;font-size:14px}.drive{height:calc(100% - 38px);grid-template-columns:minmax(0,1fr) 160px;padding:2px 9px 9px}.left-controls{gap:4px}.joy-label{font-size:9px}.joy{width:min(27vw,235px);height:min(27vw,235px);border-width:15px;box-shadow:0 15px 25px rgba(69,119,147,.25),0 0 0 10px rgba(232,240,245,.92),0 0 0 18px rgba(255,255,255,.88),inset 0 0 0 3px #d2e4ec}.trim-box{width:78%;padding:2px 8px!important}.speed-box{width:96%;padding:5px 11px!important}.speed-box .rangehead{font-size:11px}.side{gap:4px}.card,.control{padding:5px;border-radius:13px}.button{min-height:31px;border-radius:11px;font-size:8px}.value{font-size:13px}.arm-title{height:34px;font-size:12px}.arm{height:calc(100% - 34px);grid-template-rows:repeat(6,minmax(0,1fr));gap:4px;padding:2px 9px 8px}.servo{min-height:0;grid-template-columns:73px 1fr 44px;padding:2px 9px;border-radius:13px}.name{font-size:12px}.angle{font-size:12px}input[type=range]{height:26px}}
</style></head><body><main class="app"><section class="panel"><div class="main-title"><span class="dot"></span>Mobile Robotic Manipulator</div><div class="drive"><div class="left-controls"><div class="control trim-box"><div class="rangehead"><span>Trim L/R</span><span id="trimValue">0</span></div><input id="trim" type="range" min="-25" max="25" value="0"></div><div class="joy-wrap"><div class="joy-label">360° Smooth Drive</div><div id="joy" class="joy"><div id="stick" class="stick"></div></div></div><div class="control speed-box"><div class="rangehead"><span>Speed</span><span id="speedValue">55%</span></div><input id="speed" type="range" min="15" max="100" value="55"></div></div><div class="side"><div class="minirow"><div class="card"><div class="label">Steer</div><div id="xd" class="value">0</div></div><div class="card"><div class="label">Drive</div><div id="yd" class="value">0</div></div></div><div id="tiltCard" class="card tilt"><div class="label">Roll / Pitch</div><div class="value"><span id="roll">0.0</span>° / <span id="pitch">0.0</span>°</div><div id="tiltStatus" class="label">CHECKING MPU</div></div><button id="gyroButton" class="button" onclick="toggleGyro()">TILT ON</button><button id="lightButton" class="button off" onclick="toggleLight()">LIGHT OFF</button><button id="buzzerButton" class="button">HOLD HORN</button><button class="button stop" onclick="emergencyStop()">EMERGENCY STOP</button></div></div></section><section class="panel"><div class="arm-title">Six-Axis Robotic Arm</div><div class="arm">
<div class="servo"><div class="name">A<small>Gripper</small></div><input type="range" min="0" max="180" value="0" data-servo="A"><div id="valueA" class="angle">0°</div></div>
<div class="servo"><div class="name">B<small>Roll</small></div><input type="range" min="0" max="180" value="0" data-servo="B"><div id="valueB" class="angle">0°</div></div>
<div class="servo"><div class="name">C<small>Pitch</small></div><input type="range" min="0" max="180" value="8" data-servo="C"><div id="valueC" class="angle">8°</div></div>
<div class="servo"><div class="name">D<small>Elbow</small></div><input type="range" min="0" max="180" value="8" data-servo="D"><div id="valueD" class="angle">8°</div></div>
<div class="servo"><div class="name">E<small>Shoulder</small></div><input type="range" min="0" max="180" value="165" data-servo="E"><div id="valueE" class="angle">165°</div></div>
<div class="servo"><div class="name">F<small>Base</small></div><input type="range" min="0" max="180" value="73" data-servo="F"><div id="valueF" class="angle">73°</div></div>
</div></section></main><script>
const joy=document.getElementById('joy'),stick=document.getElementById('stick'),xd=document.getElementById('xd'),yd=document.getElementById('yd'),speed=document.getElementById('speed'),trim=document.getElementById('trim');let active=false,pid=null,x=0,y=0,lastSend=0;const interval=40;speed.oninput=()=>speedValue.textContent=speed.value+'%';trim.oninput=()=>trimValue.textContent=trim.value;function sendDrive(force=false){let n=Date.now();if(!force&&n-lastSend<interval)return;lastSend=n;fetch(`/drive?x=${x}&y=${y}&speed=${speed.value}&trim=${trim.value}`,{cache:'no-store'}).catch(()=>{})}function move(cx,cy){let r=joy.getBoundingClientRect(),mx=r.left+r.width/2,my=r.top+r.height/2,rad=r.width*.29,dx=cx-mx,dy=cy-my,d=Math.hypot(dx,dy);if(d>rad){dx*=rad/d;dy*=rad/d}stick.style.transform=`translate(${dx}px,${dy}px)`;x=Math.round(dx/rad*100);y=Math.round(-dy/rad*100);if(Math.abs(x)<5)x=0;if(Math.abs(y)<5)y=0;xd.textContent=x;yd.textContent=y;sendDrive()}function reset(){active=false;pid=null;x=0;y=0;stick.style.transform='translate(0,0)';xd.textContent='0';yd.textContent='0';sendDrive(true)}joy.onpointerdown=e=>{e.preventDefault();active=true;pid=e.pointerId;joy.setPointerCapture(pid);move(e.clientX,e.clientY)};joy.onpointermove=e=>{if(active&&e.pointerId===pid){e.preventDefault();move(e.clientX,e.clientY)}};joy.onpointerup=e=>{if(e.pointerId===pid)reset()};joy.onpointercancel=reset;joy.onlostpointercapture=reset;setInterval(()=>{if(active)sendDrive(true)},160);function emergencyStop(){reset();fetch('/stop',{cache:'no-store'}).catch(()=>{})}let gyroEnabled=true,headlightEnabled=false;function toggleGyro(){gyroEnabled=!gyroEnabled;fetch(`/tilt-enable?state=${gyroEnabled?1:0}`,{cache:'no-store'}).catch(()=>{});updateButtons()}function toggleLight(){headlightEnabled=!headlightEnabled;fetch(`/light?state=${headlightEnabled?1:0}`,{cache:'no-store'}).catch(()=>{});updateButtons()}function updateButtons(){gyroButton.textContent=gyroEnabled?'TILT ON':'TILT OFF';gyroButton.classList.toggle('off',!gyroEnabled);lightButton.textContent=headlightEnabled?'LIGHT ON':'LIGHT OFF';lightButton.classList.toggle('off',!headlightEnabled)}const horn=document.getElementById('buzzerButton');let held=false,ht=null;function hornOn(e){e&&e.preventDefault();if(held)return;held=true;horn.classList.add('pressed');fetch('/buzzer?state=1').catch(()=>{});ht=setInterval(()=>fetch('/buzzer?state=1').catch(()=>{}),300)}function hornOff(e){e&&e.preventDefault();held=false;horn.classList.remove('pressed');clearInterval(ht);fetch('/buzzer?state=0').catch(()=>{})}horn.onpointerdown=hornOn;horn.onpointerup=hornOff;horn.onpointercancel=hornOff;horn.onpointerleave=e=>{if(held)hornOff(e)};document.querySelectorAll('.servo input').forEach(sl=>{let t;sl.oninput=()=>{let id=sl.dataset.servo,a=Number(sl.value);document.getElementById('value'+id).textContent=a+'°';clearTimeout(t);t=setTimeout(()=>fetch(`/servo?id=${id}&angle=${a}`,{cache:'no-store'}).catch(()=>{}),25)};sl.onchange=()=>fetch(`/servo?id=${sl.dataset.servo}&angle=${sl.value}`,{cache:'no-store'}).catch(()=>{})});function status(){fetch('/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{roll.textContent=Number(d.roll).toFixed(1);pitch.textContent=Number(d.pitch).toFixed(1);gyroEnabled=!!d.tiltEnabled;headlightEnabled=!!d.light;updateButtons();tiltCard.classList.toggle('danger',!!d.danger);tiltStatus.textContent=!d.mpu?'MPU NOT FOUND':!gyroEnabled?'PROTECTION OFF':d.danger?'MOTORS LOCKED':'SAFE'}).catch(()=>{})}setInterval(status,180);status();document.addEventListener('visibilitychange',()=>{if(document.hidden){emergencyStop();hornOff()}});window.addEventListener('pagehide',()=>{navigator.sendBeacon('/stop');navigator.sendBeacon('/buzzer?state=0')});let lastTouch=0;document.addEventListener('touchend',e=>{let n=Date.now();if(n-lastTouch<320)e.preventDefault();lastTouch=n},{passive:false});document.addEventListener('gesturestart',e=>e.preventDefault());document.addEventListener('dblclick',e=>e.preventDefault());
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
    case 'A': targetArmA=angle; break;
    case 'B': targetArmB=angle; break;
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
  int a=rampValue(armAngleA,targetArmA,1),b=rampValue(armAngleB,targetArmB,1),c=rampValue(armAngleC,targetArmC,1),d=rampValue(armAngleD,targetArmD,1),e=rampValue(armAngleE,targetArmE,1),f=rampValue(armAngleF,targetArmF,1);
  if(a!=armAngleA)writeArmNow('A',a);
  if(b!=armAngleB)writeArmNow('B',b);
  if(c!=armAngleC)writeArmNow('C',c);
  if(d!=armAngleD)writeArmNow('D',d);
  if(e!=armAngleE)writeArmNow('E',e);
  if(f!=armAngleF)writeArmNow('F',f);
}

bool writeMPURegister(uint8_t reg, uint8_t value) {
  MPU_WIRE.beginTransmission(mpuAddress);
  MPU_WIRE.write(reg);
  MPU_WIRE.write(value);
  return MPU_WIRE.endTransmission() == 0;
}

bool readMPURegisters(uint8_t startRegister, uint8_t* buffer, size_t length) {
  MPU_WIRE.beginTransmission(mpuAddress);
  MPU_WIRE.write(startRegister);
  if (MPU_WIRE.endTransmission(false) != 0) return false;

  size_t received = MPU_WIRE.requestFrom((int)mpuAddress, (int)length, (int)true);
  if (received != length) return false;

  for (size_t i = 0; i < length; i++) buffer[i] = MPU_WIRE.read();
  return true;
}

bool readMPURegister(uint8_t reg, uint8_t& value) {
  return readMPURegisters(reg, &value, 1);
}

bool detectMPUAtAddress(uint8_t address) {
  mpuAddress = address;
  uint8_t whoAmI = 0;
  if (!readMPURegister(MPU_WHO_AM_I, whoAmI)) return false;
  return whoAmI == 0x68 || whoAmI == 0x69;
}

bool initializeMPU6050() {
  if (!detectMPUAtAddress(MPU_ADDRESS_PRIMARY) && !detectMPUAtAddress(MPU_ADDRESS_SECONDARY)) return false;

  if (!writeMPURegister(MPU_PWR_MGMT_1, 0x00)) return false;
  delay(100);
  writeMPURegister(MPU_CONFIG, 0x03);
  writeMPURegister(MPU_GYRO_CONFIG, 0x08);
  writeMPURegister(MPU_ACCEL_CONFIG, 0x00);
  delay(50);
  return true;
}

bool readMPURaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t data[14];
  if (!readMPURegisters(MPU_DATA_START, data, sizeof(data))) return false;

  ax = (int16_t)((data[0] << 8) | data[1]);
  ay = (int16_t)((data[2] << 8) | data[3]);
  az = (int16_t)((data[4] << 8) | data[5]);
  gx = (int16_t)((data[8] << 8) | data[9]);
  gy = (int16_t)((data[10] << 8) | data[11]);
  gz = (int16_t)((data[12] << 8) | data[13]);
  return true;
}

void calibrateMPU() {
  if (!mpuDetected) return;

  Serial.println("Keep the Mobile Robotic Manipulator flat and completely still.");
  Serial.println("Calibrating MPU6050 gyroscope...");

  const int samples = 600;
  double totalX = 0.0, totalY = 0.0, totalZ = 0.0;
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPURaw(ax, ay, az, gx, gy, gz)) {
      totalX += gx / 65.5f;
      totalY += gy / 65.5f;
      totalZ += gz / 65.5f;
      validSamples++;
    }
    delay(4);
  }

  if (validSamples == 0) {
    mpuDetected = false;
    tiltProtectionEnabled = false;
    Serial.println("MPU6050 calibration failed: no valid readings.");
    return;
  }

  gyroOffsetX = totalX / validSamples;
  gyroOffsetY = totalY / validSamples;
  gyroOffsetZ = totalZ / validSamples;

  int16_t ax, ay, az, gx, gy, gz;
  if (readMPURaw(ax, ay, az, gx, gy, gz)) {
    rollAngle = atan2f((float)ay, (float)az) * 180.0f / PI;
    pitchAngle = atan2f(-(float)ax, sqrtf((float)ay * ay + (float)az * az)) * 180.0f / PI;
  } else {
    rollAngle = 0.0f;
    pitchAngle = 0.0f;
  }
  yawAngle = 0.0f;
  lastMPUReadTime = micros();

  Serial.print("Gyro offsets X/Y/Z: ");
  Serial.print(gyroOffsetX, 3); Serial.print(" / ");
  Serial.print(gyroOffsetY, 3); Serial.print(" / ");
  Serial.println(gyroOffsetZ, 3);
  Serial.println("MPU6050 calibration complete.");
}

void updateMPU6050() {
  if (!mpuDetected) return;

  unsigned long nowMicros = micros();
  if (nowMicros - lastMPUReadTime < MPU_INTERVAL_MS * 1000UL) return;

  float dt = (nowMicros - lastMPUReadTime) / 1000000.0f;
  lastMPUReadTime = nowMicros;
  if (dt <= 0.0f || dt > 0.25f) dt = MPU_INTERVAL_MS / 1000.0f;

  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  if (!readMPURaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) return;

  float ax = axRaw / 16384.0f;
  float ay = ayRaw / 16384.0f;
  float az = azRaw / 16384.0f;
  float gx = gxRaw / 65.5f - gyroOffsetX;
  float gy = gyRaw / 65.5f - gyroOffsetY;
  float gz = gzRaw / 65.5f - gyroOffsetZ;

  float accelRoll = atan2f(ay, az) * 180.0f / PI;
  float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  rollAngle = COMPLEMENTARY_GYRO_WEIGHT * (rollAngle + gx * dt) + (1.0f - COMPLEMENTARY_GYRO_WEIGHT) * accelRoll;
  pitchAngle = COMPLEMENTARY_GYRO_WEIGHT * (pitchAngle + gy * dt) + (1.0f - COMPLEMENTARY_GYRO_WEIGHT) * accelPitch;
  yawAngle += gz * dt;

  if (!tiltProtectionEnabled) {
    dangerousTilt = false;
    return;
  }

  bool thresholdExceeded = fabsf(rollAngle) >= TILT_STOP_ANGLE || fabsf(pitchAngle) >= TILT_STOP_ANGLE;
  bool safelyRecovered = fabsf(rollAngle) <= TILT_RESET_ANGLE && fabsf(pitchAngle) <= TILT_RESET_ANGLE;

  if (thresholdExceeded) {
    if (!dangerousTilt) Serial.println("DANGEROUS TILT: MOTORS STOPPED");
    dangerousTilt = true;
    stopMotors();
  } else if (dangerousTilt && safelyRecovered) {
    dangerousTilt = false;
    Serial.println("Tilt returned to safe range.");
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
  if(tiltProtectionEnabled&&dangerousTilt){stopMotors();server.send(423,"text/plain","Drive locked by tilt protection");return;}
  int steering=-constrain(server.arg("x").toInt(),-100,100);
  int throttle=-constrain(server.arg("y").toInt(),-100,100);
  int speedPercent=server.hasArg("speed")?constrain(server.arg("speed").toInt(),15,100):55;
  int trimValue=server.hasArg("trim")?constrain(server.arg("trim").toInt(),-25,25):0;
  steering=applyJoystickDeadZone(steering);
  throttle=applyJoystickDeadZone(throttle);
  int leftPercentage=throttle+steering;
  int rightPercentage=throttle-steering;
  int largestMagnitude=max(abs(leftPercentage),abs(rightPercentage));
  if(largestMagnitude>100){leftPercentage=leftPercentage*100/largestMagnitude;rightPercentage=rightPercentage*100/largestMagnitude;}
  leftPercentage=leftPercentage*speedPercent/100;
  rightPercentage=rightPercentage*speedPercent/100;
  if(trimValue>0)leftPercentage=leftPercentage*(100-trimValue)/100;
  if(trimValue<0)rightPercentage=rightPercentage*(100+trimValue)/100;
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
    400000
  );

  mpuDetected = initializeMPU6050();

  if (mpuDetected) {
    Serial.print("MPU6050 detected at address 0x");
    Serial.println(mpuAddress, HEX);
    calibrateMPU();
  }
  else {
    tiltProtectionEnabled = false;
    Serial.println("MPU6050 not detected at 0x68 or 0x69.");
    Serial.println("Tilt protection disabled.");
  }

  writeArmNow('A',0);
  writeArmNow('B',0);
  writeArmNow('C',8);
  writeArmNow('D',8);
  writeArmNow('E',165);
  writeArmNow('F',73);

  Serial.println(
    "Servo positions: "
    "A=0 B=0 C=8 D=8 E=165 F=73"
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