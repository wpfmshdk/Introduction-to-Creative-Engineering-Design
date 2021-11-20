#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 944 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2033 // servo full counterclockwise position (180 degree)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
int a, b; // unit: mm
Servo myservo;

void setup() {
// initialize GPIO pins

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

  
  a = 95;
  b = 390;

// initialize last sampling time
  last_sampling_time = 0;

  delay(1000);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  
  if(dist_cali > 255)
  {
    myservo.writeMicroseconds(_DUTY_NEU - 100);
  }
  else if(dist_cali < 255)
  {
    myservo.writeMicroseconds(_DUTY_NEU + 220);
  }

  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);

   
// update last sampling time
  last_sampling_time += INTERVAL;
}
