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
int armAngleC = 8;
int armAngleD = 8;
int armAngleE = 67;
int armAngleF = 105;
int targetArmA = 0;
int targetArmB = 0;
int targetArmC = 8;
int targetArmD = 8;
int targetArmE = 67;
int targetArmF = 105;
const unsigned long SERVO_RAMP_INTERVAL_MS = 12;
unsigned long lastServoRampTime = 0;

const char WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,minimum-scale=1,user-scalable=no,viewport-fit=cover"><title>Mobile Robotic Manipulator</title><style>
:root{--sky:#39bfff;--sky2:#0f8ed7;--ink:#17324a;--muted:#71869a;--line:#d7e5ef;--glass:rgba(255,255,255,.76)}*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent;touch-action:manipulation}html,body{width:100%;height:100%;margin:0;overflow:hidden;overscroll-behavior:none;font-family:Montserrat,Arial,sans-serif;color:var(--ink);background:radial-gradient(circle at 15% 0,#fff 0,#eefaff 40%,#dff2fb 100%)}body{padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}.app{height:100dvh;width:100%;max-width:900px;margin:auto;padding:8px;display:grid;grid-template-rows:46% 54%;gap:8px}.panel{min-height:0;border-radius:24px;border:1px solid rgba(255,255,255,.95);background:linear-gradient(145deg,rgba(255,255,255,.96),rgba(224,244,253,.84));box-shadow:0 14px 32px rgba(75,132,163,.22),inset 0 1px 0 #fff;overflow:hidden}.title{height:34px;display:flex;align-items:center;justify-content:center;gap:8px;font-size:12px;font-weight:900;letter-spacing:1.2px;text-transform:uppercase}.dot{width:8px;height:8px;border-radius:50%;background:#35d58c;box-shadow:0 0 10px #35d58c}.drive{height:calc(100% - 34px);display:grid;grid-template-columns:minmax(0,1fr) 145px;gap:10px;padding:2px 12px 10px}.joy-zone{min-height:0;display:grid;place-items:center}.joy{position:relative;width:min(49vw,225px);height:min(49vw,225px);max-height:100%;aspect-ratio:1;border-radius:50%;touch-action:none;background:radial-gradient(circle at 42% 34%,#fff 0,#f7fdff 28%,#d9edf7 60%,#bdd7e4 100%);border:12px solid rgba(255,255,255,.9);box-shadow:0 18px 30px rgba(65,119,150,.27),inset 0 0 0 3px #d5e7f0,inset 14px 16px 22px rgba(255,255,255,.9),inset -16px -18px 24px rgba(94,139,164,.22)}.joy:before,.joy:after{content:"";position:absolute;left:14%;right:14%;top:50%;height:7px;border-radius:9px;background:#c7d4dc;box-shadow:inset 0 2px 3px rgba(70,100,120,.2)}.joy:after{top:14%;bottom:14%;left:50%;width:7px;height:auto}.stick{position:absolute;z-index:2;left:31%;top:31%;width:38%;height:38%;border-radius:50%;pointer-events:none;background:radial-gradient(circle at 29% 20%,#fff 0,#b8ecff 13%,#55caff 35%,#159ee8 67%,#0872b8 100%);border:2px solid rgba(255,255,255,.8);box-shadow:0 15px 22px rgba(26,121,179,.38),0 0 18px rgba(57,191,255,.32),inset 8px 9px 10px rgba(255,255,255,.38),inset -9px -11px 12px rgba(0,72,130,.26)}.side{display:flex;flex-direction:column;justify-content:center;gap:6px}.card,.control{border-radius:15px;background:linear-gradient(145deg,#fff,#e7f5fb);border:1px solid #fff;box-shadow:0 7px 13px rgba(66,121,151,.16),inset 0 1px 0 #fff;padding:7px}.minirow{display:grid;grid-template-columns:1fr 1fr;gap:6px}.label{font-size:7px;color:var(--muted);font-weight:800;letter-spacing:.7px;text-transform:uppercase}.value{font-size:14px;font-weight:900;margin-top:2px}.button{min-height:36px;border:1px solid rgba(255,255,255,.95);border-radius:12px;color:#fff;font-size:9px;font-weight:900;background:linear-gradient(145deg,#78d9ff,#119ee7);box-shadow:0 7px 12px rgba(25,144,205,.25),inset 0 2px 2px rgba(255,255,255,.55),inset 0 -3px 4px rgba(0,77,137,.2)}.button:active,.button.pressed{transform:translateY(2px)}.button.off{background:linear-gradient(145deg,#eaf4f8,#c8dbe4);color:#526b7b}.button.stop{background:linear-gradient(145deg,#ff8c96,#e6384a)}.twobtn{display:grid;grid-template-columns:1fr 1fr;gap:6px}.rangebox{padding:7px 9px}.rangehead{display:flex;justify-content:space-between;align-items:center;font-size:9px;font-weight:900;margin-bottom:2px}.arm{height:calc(100% - 34px);display:grid;grid-template-rows:repeat(6,minmax(0,1fr));gap:5px;padding:2px 10px 10px}.servo{min-height:0;display:grid;grid-template-columns:70px 1fr 42px;align-items:center;gap:8px;padding:3px 10px;border-radius:14px;background:linear-gradient(145deg,#fff,#e8f6fc);border:1px solid #fff;box-shadow:0 5px 10px rgba(67,123,153,.14),inset 0 1px 0 #fff}.name{font-size:12px;font-weight:900}.name small{display:block;color:var(--muted);font-size:7px;margin-top:1px}.angle{text-align:right;font-size:12px;font-weight:900}input[type=range]{width:100%;height:30px;margin:0;appearance:none;-webkit-appearance:none;background:transparent;touch-action:none}input[type=range]::-webkit-slider-runnable-track{height:8px;border-radius:8px;background:linear-gradient(180deg,#d5e3eb,#eef7fa);box-shadow:inset 0 2px 4px rgba(61,94,114,.22)}input[type=range]::-webkit-slider-thumb{appearance:none;-webkit-appearance:none;width:23px;height:23px;margin-top:-8px;border-radius:50%;background:radial-gradient(circle at 30% 24%,#fff,#bcecff 27%,#38bfff 64%,#0c85cb);border:2px solid #fff;box-shadow:0 5px 9px rgba(16,121,181,.28),inset 2px 2px 3px rgba(255,255,255,.7)}.tilt.danger{background:#ffe4e7}.trim input[type=range]::-webkit-slider-thumb{width:18px;height:18px;margin-top:-5px}@media(orientation:landscape){.app{max-width:none;grid-template-columns:43% 57%;grid-template-rows:100%;gap:8px}.drive{grid-template-columns:minmax(0,1fr) 138px;padding:2px 9px 9px}.joy{width:min(28vw,245px);height:min(28vw,245px)}.arm{gap:5px}.servo{grid-template-columns:76px 1fr 42px}}@media(max-height:540px) and (orientation:landscape){.title{height:28px}.drive,.arm{height:calc(100% - 28px)}.button{min-height:30px}.card,.control{padding:5px}.rangebox{padding:4px 7px}.servo{padding:2px 8px}.joy{width:min(27vw,205px);height:min(27vw,205px)}}
</style></head><body><main class="app"><section class="panel"><div class="title"><span class="dot"></span>UGV Control</div><div class="drive"><div class="joy-zone"><div id="joy" class="joy"><div id="stick" class="stick"></div></div></div><div class="side"><div class="minirow"><div class="card"><div class="label">Steer</div><div id="xd" class="value">0</div></div><div class="card"><div class="label">Drive</div><div id="yd" class="value">0</div></div></div><div class="control rangebox"><div class="rangehead"><span>Speed</span><span id="speedValue">55%</span></div><input id="speed" type="range" min="15" max="100" value="55"></div><div class="control rangebox trim"><div class="rangehead"><span>Trim L/R</span><span id="trimValue">0</span></div><input id="trim" type="range" min="-25" max="25" value="0"></div><div id="tiltCard" class="card tilt"><div class="label">Roll / Pitch</div><div class="value"><span id="roll">0.0</span>° / <span id="pitch">0.0</span>°</div><div id="tiltStatus" class="label">SAFE</div></div><div class="twobtn"><button id="gyroButton" class="button" onclick="toggleGyro()">TILT ON</button><button id="lightButton" class="button off" onclick="toggleLight()">LIGHT OFF</button></div><div class="twobtn"><button id="buzzerButton" class="button">HOLD HORN</button><button class="button stop" onclick="emergencyStop()">STOP</button></div></div></div></section><section class="panel"><div class="title">Six-Axis Robotic Arm</div><div class="arm">
<div class="servo"><div class="name">A<small>Gripper</small></div><input type="range" min="0" max="180" value="0" data-servo="A"><div id="valueA" class="angle">0°</div></div>
<div class="servo"><div class="name">B<small>Roll</small></div><input type="range" min="0" max="180" value="0" data-servo="B"><div id="valueB" class="angle">0°</div></div>
<div class="servo"><div class="name">C<small>Pitch</small></div><input type="range" min="0" max="180" value="8" data-servo="C"><div id="valueC" class="angle">8°</div></div>
<div class="servo"><div class="name">D<small>Elbow</small></div><input type="range" min="0" max="180" value="8" data-servo="D"><div id="valueD" class="angle">8°</div></div>
<div class="servo"><div class="name">E<small>Shoulder</small></div><input type="range" min="0" max="180" value="67" data-servo="E"><div id="valueE" class="angle">67°</div></div>
<div class="servo"><div class="name">F<small>Base</small></div><input type="range" min="0" max="180" value="105" data-servo="F"><div id="valueF" class="angle">105°</div></div>
</div></section></main><script>
const joy=document.getElementById('joy'),stick=document.getElementById('stick'),xd=document.getElementById('xd'),yd=document.getElementById('yd'),speed=document.getElementById('speed'),trim=document.getElementById('trim');let active=false,pid=null,x=0,y=0,lastSend=0;const interval=40;speed.oninput=()=>speedValue.textContent=speed.value+'%';trim.oninput=()=>trimValue.textContent=trim.value;function sendDrive(force=false){let n=Date.now();if(!force&&n-lastSend<interval)return;lastSend=n;fetch(`/drive?x=${x}&y=${y}&speed=${speed.value}&trim=${trim.value}`,{cache:'no-store'}).catch(()=>{})}function move(cx,cy){let r=joy.getBoundingClientRect(),mx=r.left+r.width/2,my=r.top+r.height/2,rad=r.width*.31,dx=cx-mx,dy=cy-my,d=Math.hypot(dx,dy);if(d>rad){dx*=rad/d;dy*=rad/d}stick.style.transform=`translate(${dx}px,${dy}px)`;x=Math.round(dx/rad*100);y=Math.round(-dy/rad*100);if(Math.abs(x)<5)x=0;if(Math.abs(y)<5)y=0;xd.textContent=x;yd.textContent=y;sendDrive()}function reset(){active=false;pid=null;x=0;y=0;stick.style.transform='translate(0,0)';xd.textContent='0';yd.textContent='0';sendDrive(true)}joy.onpointerdown=e=>{e.preventDefault();active=true;pid=e.pointerId;joy.setPointerCapture(pid);move(e.clientX,e.clientY)};joy.onpointermove=e=>{if(active&&e.pointerId===pid){e.preventDefault();move(e.clientX,e.clientY)}};joy.onpointerup=e=>{if(e.pointerId===pid)reset()};joy.onpointercancel=reset;joy.onlostpointercapture=reset;setInterval(()=>{if(active)sendDrive(true)},160);function emergencyStop(){reset();fetch('/stop',{cache:'no-store'}).catch(()=>{})}let gyroEnabled=true,headlightEnabled=false;function toggleGyro(){gyroEnabled=!gyroEnabled;fetch(`/tilt-enable?state=${gyroEnabled?1:0}`,{cache:'no-store'}).catch(()=>{});updateButtons()}function toggleLight(){headlightEnabled=!headlightEnabled;fetch(`/light?state=${headlightEnabled?1:0}`,{cache:'no-store'}).catch(()=>{});updateButtons()}function updateButtons(){gyroButton.textContent=gyroEnabled?'TILT ON':'TILT OFF';gyroButton.classList.toggle('off',!gyroEnabled);lightButton.textContent=headlightEnabled?'LIGHT ON':'LIGHT OFF';lightButton.classList.toggle('off',!headlightEnabled)}const horn=document.getElementById('buzzerButton');let held=false,ht=null;function hornOn(e){e&&e.preventDefault();if(held)return;held=true;horn.classList.add('pressed');fetch('/buzzer?state=1').catch(()=>{});ht=setInterval(()=>fetch('/buzzer?state=1').catch(()=>{}),300)}function hornOff(e){e&&e.preventDefault();held=false;horn.classList.remove('pressed');clearInterval(ht);fetch('/buzzer?state=0').catch(()=>{})}horn.onpointerdown=hornOn;horn.onpointerup=hornOff;horn.onpointercancel=hornOff;horn.onpointerleave=e=>{if(held)hornOff(e)};document.querySelectorAll('.servo input').forEach(sl=>{let t;sl.oninput=()=>{let id=sl.dataset.servo,a=Number(sl.value);document.getElementById('value'+id).textContent=a+'°';clearTimeout(t);t=setTimeout(()=>fetch(`/servo?id=${id}&angle=${a}`,{cache:'no-store'}).catch(()=>{}),18)};sl.onchange=()=>fetch(`/servo?id=${sl.dataset.servo}&angle=${sl.value}`,{cache:'no-store'}).catch(()=>{})});function status(){fetch('/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{roll.textContent=Number(d.roll).toFixed(1);pitch.textContent=Number(d.pitch).toFixed(1);gyroEnabled=!!d.tiltEnabled;headlightEnabled=!!d.light;updateButtons();tiltCard.classList.toggle('danger',!!d.danger);tiltStatus.textContent=!d.mpu?'MPU NOT FOUND':!gyroEnabled?'PROTECTION OFF':d.danger?'MOTORS LOCKED':'SAFE'}).catch(()=>{})}setInterval(status,180);status();document.addEventListener('visibilitychange',()=>{if(document.hidden){emergencyStop();hornOff()}});window.addEventListener('pagehide',()=>{navigator.sendBeacon('/stop');navigator.sendBeacon('/buzzer?state=0')});let lastTouch=0;document.addEventListener('touchend',e=>{let n=Date.now();if(n-lastTouch<320)e.preventDefault();lastTouch=n},{passive:false});document.addEventListener('gesturestart',e=>e.preventDefault());
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
    stopMotors();
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

  mpuDetected = mpu6050.begin(
    0x68,
    &MPU_WIRE
  );

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
  writeArmNow('C',8);
  writeArmNow('D',8);
  writeArmNow('E',67);
  writeArmNow('F',105);

  Serial.println(
    "Servo positions: "
    "A=0 B=0 C=8 D=8 E=67 F=105"
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