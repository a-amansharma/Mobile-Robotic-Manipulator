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
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,minimum-scale=1,user-scalable=no,viewport-fit=cover"><title>Mobile Robotic Manipulator</title><style>
:root{--ink:#17324a;--muted:#7890a2;--blue1:#8be4ff;--blue2:#27b7f5;--blue3:#087fc6}*{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}html,body{width:100%;height:100%;margin:0;overflow:hidden;overscroll-behavior:none;touch-action:none;font-family:Montserrat,Arial,sans-serif;color:var(--ink);background:linear-gradient(145deg,#fff,#eefaff 55%,#dff4fd)}body{padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}.app{width:100%;height:100dvh;max-width:760px;margin:auto;padding:6px;display:grid;grid-template-rows:47% 53%;gap:6px}.panel{min-height:0;overflow:hidden;border-radius:24px;background:linear-gradient(145deg,rgba(255,255,255,.98),rgba(231,248,255,.94));box-shadow:0 12px 28px rgba(72,132,161,.18),inset 7px 7px 16px rgba(255,255,255,.92),inset -8px -8px 18px rgba(116,173,202,.12)}.title{height:32px;display:flex;align-items:center;justify-content:center;gap:8px;font-size:12px;font-weight:900;letter-spacing:1.05px;text-transform:uppercase}.dot{width:8px;height:8px;border-radius:50%;background:#35d58c;box-shadow:0 0 10px #35d58c}.drive-area{height:calc(100% - 32px);display:grid;grid-template-columns:31% 69%;gap:7px;padding:4px 8px 8px}.left-controls{min-width:0;display:flex;flex-direction:column;align-items:stretch;justify-content:space-between;gap:6px;padding:2px 0}.btn{border:0;color:#fff;font-weight:900;letter-spacing:.45px;background:linear-gradient(145deg,var(--blue1),var(--blue2) 52%,var(--blue3));box-shadow:0 9px 16px rgba(23,139,199,.25),inset 0 3px 4px rgba(255,255,255,.72),inset 0 -5px 7px rgba(0,80,145,.2);text-shadow:0 1px rgba(0,64,110,.25)}.btn:active,.btn.pressed{transform:translateY(2px);box-shadow:0 4px 8px rgba(23,139,199,.2),inset 0 4px 7px rgba(0,76,130,.18)}.btn.big{height:50px;border-radius:18px;font-size:9px}.btn.small{height:39px;width:78%;align-self:center;border-radius:15px;font-size:8px}.btn.off{background:linear-gradient(145deg,#f9fdff,#d7e8f0 55%,#bfd5e0);color:#597180;text-shadow:none}.btn.stop{background:linear-gradient(145deg,#ff9ba5,#f15f70 52%,#dc3046)}.readouts{display:grid;grid-template-columns:1fr 1fr;gap:5px}.mini{padding:5px 2px;border-radius:13px;background:linear-gradient(145deg,#fff,#e7f4fa);box-shadow:0 5px 10px rgba(68,124,152,.12),inset 2px 2px 4px #fff,inset -3px -3px 5px rgba(111,164,191,.1);text-align:center}.mini span{display:block;font-size:6px;color:var(--muted);font-weight:900;letter-spacing:.6px}.mini b{font-size:12px}.right-controls{min-width:0;display:grid;grid-template-rows:32px minmax(0,1fr) 48px;gap:5px;align-items:center}.controlbox{width:92%;justify-self:center;padding:5px 11px;border-radius:17px;background:linear-gradient(145deg,#fff,#e8f6fb);box-shadow:0 7px 14px rgba(64,120,149,.14),inset 3px 3px 6px #fff,inset -4px -4px 8px rgba(104,159,188,.1)}.rhead{display:flex;justify-content:space-between;font-size:8px;font-weight:900;margin-bottom:1px}.speedbox .rhead{font-size:10px}.joywrap{min-height:0;display:flex;align-items:center;justify-content:center}.joy{position:relative;width:min(47vw,194px);height:min(47vw,194px);max-width:100%;max-height:100%;aspect-ratio:1;border-radius:50%;touch-action:none;background:radial-gradient(circle at 40% 28%,#fff 0,#f9fdff 25%,#e9f5fa 52%,#cadfe9 100%);box-shadow:0 18px 28px rgba(62,125,157,.24),inset 12px 14px 22px rgba(255,255,255,.96),inset -15px -17px 24px rgba(77,130,158,.2)}.joy:before,.joy:after{content:"";position:absolute;left:17%;right:17%;top:50%;height:7px;border-radius:10px;background:linear-gradient(#d5e2e8,#b9ccd5,#edf5f8);box-shadow:inset 0 2px 4px rgba(57,88,106,.18)}.joy:after{top:17%;bottom:17%;left:50%;width:7px;height:auto}.stick{position:absolute;z-index:2;left:31%;top:31%;width:38%;height:38%;border-radius:50%;pointer-events:none;background:radial-gradient(circle at 27% 17%,#fff 0,#d8f5ff 11%,#79dcff 30%,#26b6f5 58%,#0679bf 100%);box-shadow:0 16px 23px rgba(15,116,177,.36),0 0 0 4px rgba(255,255,255,.88),inset 8px 9px 11px rgba(255,255,255,.55),inset -10px -12px 13px rgba(0,70,128,.25)}input[type=range]{width:100%;height:24px;margin:0;appearance:none;-webkit-appearance:none;background:transparent;touch-action:none}input[type=range]::-webkit-slider-runnable-track{height:7px;border-radius:8px;background:linear-gradient(180deg,#c8d9e1,#edf5f8);box-shadow:inset 0 2px 4px rgba(61,94,114,.2)}input[type=range]::-webkit-slider-thumb{appearance:none;-webkit-appearance:none;width:21px;height:21px;margin-top:-7px;border-radius:50%;background:radial-gradient(circle at 30% 22%,#fff,#caf1ff 28%,#42c4ff 62%,#0b84ca);box-shadow:0 6px 10px rgba(16,121,181,.28),0 0 0 3px rgba(255,255,255,.9),inset 2px 2px 4px rgba(255,255,255,.75)}.trimbox{width:72%;padding:3px 10px}.trimbox input[type=range]{height:15px}.trimbox input[type=range]::-webkit-slider-thumb{width:16px;height:16px;margin-top:-5px}.arm{height:calc(100% - 32px);display:grid;grid-template-rows:repeat(6,minmax(0,1fr));gap:4px;padding:3px 9px 9px}.servo{min-height:0;display:grid;grid-template-columns:58px 1fr 37px;align-items:center;gap:7px;padding:2px 9px;border-radius:15px;background:linear-gradient(145deg,#fff,#e9f7fc);box-shadow:0 4px 9px rgba(67,123,153,.11),inset 3px 3px 6px #fff,inset -4px -4px 7px rgba(111,164,191,.08)}.name{font-size:15px;font-weight:900}.name small{display:block;color:var(--muted);font-size:9px;margin-top:2px;font-weight:800}.angle{text-align:right;font-size:12px;font-weight:900}.danger{outline:2px solid #ff5969}@media(max-height:700px){.app{grid-template-rows:46% 54%}.title{height:29px}.drive-area,.arm{height:calc(100% - 29px)}.btn.big{height:43px}.btn.small{height:34px}.right-controls{grid-template-rows:30px minmax(0,1fr) 42px}.joy{width:min(43vw,174px);height:min(43vw,174px)}.servo{padding:1px 8px}}
</style></head><body><main class="app"><section class="panel"><div class="title"><span class="dot"></span>Mobile Robotic Manipulator</div><div class="drive-area"><div class="left-controls"><button id="lightButton" class="btn big off" onclick="toggleLight()">LIGHT OFF</button><button id="buzzerButton" class="btn small">HOLD HORN</button><button id="gyroButton" class="btn small" onclick="toggleGyro()">TILT ON</button><button class="btn big stop" onclick="emergencyStop()">STOP</button><div class="readouts"><div class="mini"><span>STEER</span><b id="xd">0</b></div><div class="mini"><span>DRIVE</span><b id="yd">0</b></div></div></div><div class="right-controls"><div class="controlbox trimbox"><div class="rhead"><span>Trim L/R</span><span id="trimValue">0</span></div><input id="trim" type="range" min="-25" max="25" value="0"></div><div class="joywrap"><div id="joy" class="joy"><div id="stick" class="stick"></div></div></div><div class="controlbox speedbox"><div class="rhead"><span>Speed</span><span id="speedValue">55%</span></div><input id="speed" type="range" min="15" max="100" value="55"></div></div></div></section><section class="panel"><div class="title">Six-Axis Robotic Arm</div><div class="arm">
<div class="servo"><div class="name">A<small>Gripper</small></div><input type="range" min="0" max="180" value="0" data-servo="A"><div id="valueA" class="angle">0°</div></div>
<div class="servo"><div class="name">B<small>Roll</small></div><input type="range" min="0" max="180" value="0" data-servo="B"><div id="valueB" class="angle">0°</div></div>
<div class="servo"><div class="name">C<small>Pitch</small></div><input type="range" min="0" max="180" value="94" data-servo="C"><div id="valueC" class="angle">94°</div></div>
<div class="servo"><div class="name">D<small>Elbow</small></div><input type="range" min="0" max="180" value="16" data-servo="D"><div id="valueD" class="angle">16°</div></div>
<div class="servo"><div class="name">E<small>Shoulder</small></div><input type="range" min="0" max="180" value="180" data-servo="E"><div id="valueE" class="angle">180°</div></div>
<div class="servo"><div class="name">F<small>Base</small></div><input type="range" min="0" max="180" value="73" data-servo="F"><div id="valueF" class="angle">73°</div></div>
</div></section></main><script>
const joy=document.getElementById('joy'),stick=document.getElementById('stick'),xd=document.getElementById('xd'),yd=document.getElementById('yd'),speed=document.getElementById('speed'),trim=document.getElementById('trim');let active=false,pid=null,x=0,y=0,lastSend=0;const interval=40;speed.oninput=()=>speedValue.textContent=speed.value+'%';trim.oninput=()=>trimValue.textContent=trim.value;function sendDrive(force=false){let n=Date.now();if(!force&&n-lastSend<interval)return;lastSend=n;fetch(`/drive?x=${x}&y=${y}&speed=${speed.value}&trim=${trim.value}`,{cache:'no-store'}).catch(()=>{})}function move(cx,cy){let r=joy.getBoundingClientRect(),mx=r.left+r.width/2,my=r.top+r.height/2,rad=r.width*.31,dx=cx-mx,dy=cy-my,d=Math.hypot(dx,dy);if(d>rad){dx*=rad/d;dy*=rad/d}stick.style.transform=`translate(${dx}px,${dy}px)`;x=Math.round(dx/rad*100);y=Math.round(-dy/rad*100);if(Math.abs(x)<5)x=0;if(Math.abs(y)<5)y=0;xd.textContent=x;yd.textContent=y;sendDrive()}function reset(){active=false;pid=null;x=0;y=0;stick.style.transform='translate(0,0)';xd.textContent='0';yd.textContent='0';sendDrive(true)}joy.onpointerdown=e=>{e.preventDefault();active=true;pid=e.pointerId;joy.setPointerCapture(pid);move(e.clientX,e.clientY)};joy.onpointermove=e=>{if(active&&e.pointerId===pid){e.preventDefault();move(e.clientX,e.clientY)}};joy.onpointerup=e=>{if(e.pointerId===pid)reset()};joy.onpointercancel=reset;joy.onlostpointercapture=reset;setInterval(()=>{if(active)sendDrive(true)},160);function emergencyStop(){reset();fetch('/stop',{cache:'no-store'}).catch(()=>{})}let gyroEnabled=true,headlightEnabled=false;function toggleGyro(){gyroEnabled=!gyroEnabled;fetch(`/tilt-enable?state=${gyroEnabled?1:0}`,{cache:'no-store'}).catch(()=>{});updateButtons()}function toggleLight(){headlightEnabled=!headlightEnabled;fetch(`/light?state=${headlightEnabled?1:0}`,{cache:'no-store'}).catch(()=>{});updateButtons()}function updateButtons(){gyroButton.textContent=gyroEnabled?'TILT ON':'TILT OFF';gyroButton.classList.toggle('off',!gyroEnabled);lightButton.textContent=headlightEnabled?'LIGHT ON':'LIGHT OFF';lightButton.classList.toggle('off',!headlightEnabled)}const horn=document.getElementById('buzzerButton');let held=false,ht=null;function hornOn(e){e&&e.preventDefault();if(held)return;held=true;horn.classList.add('pressed');fetch('/buzzer?state=1').catch(()=>{});ht=setInterval(()=>fetch('/buzzer?state=1').catch(()=>{}),300)}function hornOff(e){e&&e.preventDefault();held=false;horn.classList.remove('pressed');clearInterval(ht);fetch('/buzzer?state=0').catch(()=>{})}horn.onpointerdown=hornOn;horn.onpointerup=hornOff;horn.onpointercancel=hornOff;horn.onpointerleave=e=>{if(held)hornOff(e)};document.querySelectorAll('.servo input').forEach(sl=>{let t;sl.oninput=()=>{let id=sl.dataset.servo,a=Number(sl.value);document.getElementById('value'+id).textContent=a+'°';clearTimeout(t);t=setTimeout(()=>fetch(`/servo?id=${id}&angle=${a}`,{cache:'no-store'}).catch(()=>{}),18)};sl.onchange=()=>fetch(`/servo?id=${sl.dataset.servo}&angle=${sl.value}`,{cache:'no-store'}).catch(()=>{})});function status(){fetch('/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{gyroEnabled=!!d.tiltEnabled;headlightEnabled=!!d.light;updateButtons();gyroButton.classList.toggle('danger',!!d.danger)}).catch(()=>{})}setInterval(status,220);status();document.addEventListener('visibilitychange',()=>{if(document.hidden){emergencyStop();hornOff()}});window.addEventListener('pagehide',()=>{navigator.sendBeacon('/stop');navigator.sendBeacon('/buzzer?state=0')});let lastTouch=0;document.addEventListener('touchend',e=>{let n=Date.now();if(n-lastTouch<320)e.preventDefault();lastTouch=n},{passive:false});document.addEventListener('gesturestart',e=>e.preventDefault());document.addEventListener('dblclick',e=>e.preventDefault());
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