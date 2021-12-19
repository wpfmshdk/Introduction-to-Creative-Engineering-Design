#include <Servo.h>

//////
// 설정값
//////L

#define PIN_SERVO 10
#define PIN_IR A0

#define _DUTY_MIN (_DUTY_NEU - 270) // 임의로 300정도 빼줌.
#define _DUTY_NEU 1514 // 프레임워크에서 공이 안움직이는 구간. // 1400
#define _DUTY_MAX (_DUTY_NEU + 270) // 임의로 300 정도 더해줌.

#define _EMA_ALPHA 0.1 // EMA 필터 계산값

#define _INTERVAL_DIST 5 // 거리 측정 주기
#define _INTERVAL_SERVO 5 // 서보 조정 주기
#define _INTERVAL_SERIAL 100 // 시리얼 출력 주기

#define _DIST_TARGET 255  // 공 목표위치
#define _DIST_MIN 100 // 공 최소 위치 
#define _DIST_MAX 410 // 공 최대 위치

#define _SERVO_SPEED 340        // servo 속도 설정

#define _KP 0.7  // P Control 민감도 0.8
#define _KD 350  // D Control 민감도 300
#define _KI 0.04 // I Control 민감도 0.1

#define _ITERM_MAX 55.0 // I 인자 제한치 50

//////
// 전역변수
//////

Servo myservo; // 서보 클래스

float dist_raw; // 적외선 센서가 측정한 탁구공 거리값을 저장
float dist_cali; // 센서가 측정한 값을 선형보간법으로 보정한 값을 저장
float ema_res; // ema 필터링된 결과값을 저장

float dist_min; // float 형태의 공 최소 위치
float dist_max; // float 형태의 공 최대 위치

int duty_chg_per_interval; // 이벤트 주기마다 펄스주기값을 얼마나 변경할지 알려줌.
int duty_target;  // 목표 펄스주기값
int duty_curr;    // 현재 펄스주기값

bool event_dist; // 거리 측정 관련 이벤트
bool event_servo; // 서보 조정 관련 이벤트
bool event_serial; // 시리얼 출력 관련 이벤트

unsigned long last_sampling_time_dist; // 거리 측정 마지막 시간
unsigned long last_sampling_time_servo; // 서보 조정 마지막 시간
unsigned long last_sampling_time_serial; // 시리얼 출력 마지막 시간

float error_curr;
float error_prev;
float control;
float pterm; 
float dterm;
float iterm;

float dist_target;

void setup() 
{
  // 서보 접속
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  // 전역변수 초기화
  dist_raw = 0.0;
  dist_cali = 0.0;
  ema_res = 0.0;

  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;

  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * (float)(_SERVO_SPEED / 180.0) * (float)(_INTERVAL_SERVO / 1000.0);
  duty_target = 0;
  duty_curr = _DUTY_NEU;

  dist_target = _DIST_TARGET;

  event_dist = false;
  event_servo = false;
  event_serial = false;

  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

    
  error_curr = 0.0;
  error_prev = 0.0;
  control = 0.0;
  pterm = 0.0; 
  dterm = 0.0;
  iterm = 0.0;


  // 보드레이트 설정
  Serial.begin(57600);
}

float ir_distance()
{
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val; // mm 단위로 적외선 센서가 측정한 거리 반환
}

float ir_filter(float x)
{
  float val = 0.0;
  if(x >= 66.0 && x <= 77.0) {val = 50.0 / 11.0 * x - 200;} // 100 ~ 150
  else if(x > 77.0 && x <= 130.0){val = 50.0 / 53.0 * x + 78;}// 150 ~ 200
  else if(x > 130.0 && x <= 177.0){val = 50.0 / 47.0 * x + 63;}// 200 ~ 250
  else if(x > 177.0 && x <= 224.0){val = 50.0 / 47.0 * x + 63;} // 250 ~ 300
  else if(x > 224.0 && x <= 264.0){val = 50.0 / 40.0 * x + 20;} // 300 ~ 350
  else if(x > 264.0 && x <= 354.0){val = 50.0 / 90.0 * x + 203;} // 350 ~ 400
  else if(x > 354.0) {val = 0.7 *x + 153;} // 400 이상

  return val; // mm 단위로 보정된 센서 측정값을 반환
}

// void loop(){}

void loop()
{

  // 이벤트 생성기
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;
  
  // 거리 측정 및 측정값 가공
  if(event_dist)
  {
    event_dist = false;
    dist_raw = ir_distance();
    float dist_cali = ir_filter(dist_raw);
    ema_res = (_EMA_ALPHA * dist_cali) + ((1-_EMA_ALPHA) * ema_res); 

    error_curr = _DIST_TARGET - ema_res;
    if(error_curr < 0.0)
    {
      pterm = (_KP + 0.2) * error_curr;
      dterm = (_KD - 50) * (error_curr - error_prev); 
      iterm += (_KI + 0.05) * error_curr;
    }
    else
    {
      pterm = _KP * error_curr;
      dterm = _KD * (error_curr - error_prev); 
      iterm += _KI * error_curr;
    }
     
    

    control = pterm + dterm + iterm;
    duty_target = _DUTY_NEU + control;

    if(abs(iterm) > _ITERM_MAX) iterm = 0; // 적분항 해소.

    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; // 상한성
    if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; // 하한선

    error_prev = error_curr;

    last_sampling_time_dist = millis(); // 과거 오차 정보 저장
  }

   if(event_servo)
   {
      event_servo = false;

      if(duty_target > duty_curr)
        duty_curr += (int)(duty_chg_per_interval * 1.5);
      else if(duty_target < duty_curr)
        duty_curr -= duty_chg_per_interval;

      myservo.writeMicroseconds(duty_curr);

      last_sampling_time_servo = millis();
   }

  if(event_serial)
  {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(ema_res); // -> Serial.print(dist_ema)
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    
    last_sampling_time_serial = millis();
  }
}
