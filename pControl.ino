#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 // [3097] LED핀 9번으로 설정
#define PIN_SERVO 10 // [3097] 서보핀 9번으로 설정
#define PIN_IR A0 // [3097] 적외선핀 A0번으로 설정

// Framework setting
#define _DIST_TARGET 255 //[0711] 목표값
#define _DIST_MIN 100 //[0711] 측정 최솟값
#define _DIST_MAX 430 //[0711] 측정 최댓값

// Distance sensor
#define _DIST_ALPHA 0.5 // [3095] ema필터의 alpha 값을 설정

// Servo range
#define _DUTY_MIN 1200  // [1615] 서보 제어 펄스 폭: 최고 각도
#define _DUTY_NEU 1400  // [1615] 서보 제어 펄스 폭: 수평
#define _DUTY_MAX 1550  // [1615] 서보 제어 펄스 폭: 최저 각도

// Servo speed control
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 50

// Event periods
#define _INTERVAL_DIST 10   // [3074] 거리측정주기 (ms)
#define _INTERVAL_SERVO 10  // [3078] 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // [3078] Serial제어주기 (ms)

// PID parameters
#define _KP 0.6 //[0711] 비례이득

//////////////////////
// global variables //
//////////////////////

float dist_min, dist_max, dist_cali;

// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; // location to send the ball
float dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist;
bool event_servo, event_serial;

// Servo speed control
float duty_chg_per_interval; 
float duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

const float coE[] = {-0.0000563, 0.0339311, -4.6805541, 278.3446514};

void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 0);
 myservo.attach(PIN_SERVO); 


// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX; 
  duty_curr = _DUTY_NEU;

// move servo to neutral position
 myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); 
}
  
void loop() {
/////////////////////
// Event generator // 
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
      event_dist = false;
  // get a distance reading from the distance sensor
  
   dist_cali = ir_distance_filtered();

  // PID control logic
    error_curr = _DIST_TARGET - dist_cali; // [3073] 현재 읽어들인 데이터와 기준 값의 차이
    pterm = _KP *error_curr; // [3073] p게인 값인 kp와 error 값의 곱
    control = pterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
  
    last_sampling_time_dist = millis();
  }
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
  }
//     update servo position
      myservo.writeMicroseconds(duty_curr);
      last_sampling_time_servo = millis();

  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_cali);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
//    Serial.print(duty_target);
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    last_sampling_time_serial = millis();

  }
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered_ema(void) {
  static float val = _DIST_TARGET;
  static float dist_ema = 0;
  float raw = ir_distance();
  if (raw >= _DIST_MIN && raw <= _DIST_MAX)
      val = raw;
  dist_ema = (1.0 - _DIST_ALPHA) * val + _DIST_ALPHA * dist_ema;
  return dist_ema;
}


float ir_distance_filtered(void) {
  float x = ir_distance_filtered_ema();
  dist_cali = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  return dist_cali;

}
