#include <Servo.h>

// 핀 정의
#define PIN_POTENTIOMETER A3   // 가변 저항을 A3 핀에 연결
#define PIN_SERVO         10   // 서보를 디지털 핀 10에 연결
#define PIN_IR_SENSOR     A0    // 적외선 거리 센서를 A0 핀에 연결

// 서보 PWM 신호 범위 설정 (서보에 따라 다를 수 있음)
#define _DUTY_MIN 1000  // 서보의 최대 시계 방향 위치 (0도)
#define _DUTY_NEU 1500  // 서보의 중립 위치 (90도)
#define _DUTY_MAX 2000  // 서보의 최대 반시계 방향 위치 (180도)

#define LOOP_INTERVAL 50   // 루프 간격 (밀리초)

// 서보 객체 생성
Servo myservo;
unsigned long last_loop_time = 0;   // 이전 루프 시간

void setup()
{
  // 서보 초기화 및 중립 위치 설정
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);  // 서보를 중립 위치로 초기화
  
  // 시리얼 통신 시작
  Serial.begin(57600);
}

void loop()
{
  unsigned long time_curr = millis();
  int a_pot, duty, a_ir;
  float distance;
  
  // 다음 이벤트 시간까지 대기
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  // 가변 저항 값 읽기
  a_pot = analogRead(PIN_POTENTIOMETER);  // 0 ~ 1023

  // 서보 제어: 가변 저항 값을 서보 PWM 신호로 매핑
  duty = map(a_pot, 0, 1023, _DUTY_MIN, _DUTY_MAX);
  myservo.writeMicroseconds(duty);

  // 적외선 센서 값 읽기
  a_ir = analogRead(PIN_IR_SENSOR);  // 0 ~ 1023

  // 거리 계산: 선형 모델 사용
  // distance_raw = (6762.0 / (a_value - 9) - 4.0) * 10.0
  if (a_ir > 9) {  // 분모가 0이 되지 않도록 9보다 클 때만 계산
    distance = (6762.0 / (a_ir - 9) - 4.0) * 10.0;  // mm 단위
  }
  else {
    distance = -1;  // 유효하지 않은 거리
  }

  // 0~6cm (60mm) 범위 무시
  if (distance > 60) {
    // 유효 거리
  }
  else {
    distance = -1;  // 무시
  }

  // 시리얼 출력
  Serial.print("Pot ADC: ");
  Serial.print(a_pot);
  Serial.print(" => Duty: ");
  Serial.print(duty);
  
  Serial.print(" | IR ADC: ");
  Serial.print(a_ir);
  Serial.print(" => Distance: ");
  if (distance > 60)
    Serial.print(distance);
  else
    Serial.print("Error or <6cm");
  Serial.println(" mm");
  
  // 필요 시, 거리 값에 따라 추가 동작 구현 가능
}
