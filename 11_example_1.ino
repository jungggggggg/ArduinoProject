#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

#define _EMA_ALPHA 0.3    // EMA weight of new sample (range: 0 to 1)

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
#define _DUTY_MIN 1000   // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500   // servo neutral position (90 degree)
#define _DUTY_MAX 2000   // servo full counterclockwise position (180 degree)

// global variables
float dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;      // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);  // set servo to neutral (90 degree)

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;

  // wait until next sampling time
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  // 측정 거리 읽기
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance
  
  // 범위 필터 (18cm ~ 36cm로 제한)
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;           // Cut higher than maximum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;           // cut lower than minimum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // 범위 안에 들어오면 LED ON
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, 0);       // LED ON
  }

  // Apply EMA filter (지수이동평균 필터 적용)
  dist_ema = (_EMA_ALPHA * dist_raw) + ((1 - _EMA_ALPHA) * dist_prev);

  // 서보 모터 각도 제어
  float servo_angle;

  if (dist_ema <= _DIST_MIN) {
    servo_angle = _DUTY_MIN;  // 0도
  } else if (dist_ema >= _DIST_MAX) {
    servo_angle = _DUTY_MAX;  // 180도
  } else {
    // 거리에 비례하여 서보 각도 계산 (18cm에서 36cm 사이에서 0도 ~ 180도)
    float proportion = (dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN);
    servo_angle = _DUTY_MIN + (proportion * (_DUTY_MAX - _DUTY_MIN));
  }

  myservo.writeMicroseconds(servo_angle);  // 서보 모터에 각도 전달

  // 시리얼 모니터 출력
  Serial.print("Raw Distance: "); Serial.print(dist_raw);
  Serial.print(" | EMA Distance: "); Serial.print(dist_ema);
  Serial.print(" | Servo Angle: "); Serial.println(servo_angle);

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// 초음파 센서로 거리 측정
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // 단위: mm
}
