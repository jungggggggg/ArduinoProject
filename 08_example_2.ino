#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // 샘플링 주기 25ms
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float distance;

  // wait until next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // LED 밝기 제어: 200mm에서 최대, 100mm 및 300mm에서 최소, 150mm 및 250mm에서 50% 밝기
  int brightness = calculateLEDBrightness(distance);

  analogWrite(PIN_LED, brightness);  // LED 밝기 설정

  // output the distance to the serial port
  Serial.print("Distance: ");  Serial.print(distance);
  Serial.print(" mm, LED Brightness: "); Serial.println(brightness);
  
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// 거리 값에 따라 LED 밝기 계산 (0 ~ 255)
int calculateLEDBrightness(float distance) {
  if (distance < _DIST_MIN || distance > _DIST_MAX) {
    return 0;  // 범위를 벗어나면 LED 끔
  }

  // 150mm와 250mm에서 50% 밝기 (127), 200mm에서 최대 밝기 (255)
  if (distance == 200.0) {
    return 255;  // 최대 밝기
  } else if (distance == 150.0 || distance == 250.0) {
    return 127;  // 50% 밝기
  } else {
    // 선형적으로 밝기 조정
    if (distance < 200.0) {
      return map(distance, 100, 200, 0, 255);  // 100mm에서 200mm까지 밝기 증가
    } else {
      return map(distance, 200, 300, 255, 0);  // 200mm에서 300mm까지 밝기 감소
    }
  }
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
