// ========================================
// Arduino Mega 2560 - Moebius Board
// 3 Wheel Omni Robot with Encoders + PID + Odometry
// ROS2 Serial Communication
// Custom Inverse Kinematics
// ========================================

#include <Arduino.h>

// ===== 로봇 파라미터 =====
const float R_WHEEL = 0.04;  // 휠 반지름 (m)
const float R_BASE = 0.10;   // 베이스 반지름 (m)

// ===== Moebius 보드 모터 핀 설정 =====
// Motor 1 (W1 - 210도, 후방)
#define MOTOR1_PWM 12
#define MOTOR1_DIR1 35
#define MOTOR1_DIR2 34

// Motor 2 (W2 - 330도, 우측전방)
#define MOTOR2_PWM 8
#define MOTOR2_DIR1 37
#define MOTOR2_DIR2 36

// Motor 3 (W3 - 90도, 좌측전방)
#define MOTOR3_PWM 6
#define MOTOR3_DIR1 42
#define MOTOR3_DIR2 43

// ===== Encoders =====
#define ENCODER1_A 18  // INT3 (Motor1 - LeftFront)
#define ENCODER1_B 31
#define ENCODER2_A 19  // INT2 (Motor2 - RightFront)
#define ENCODER2_B 38
#define ENCODER3_A 3   // INT5 (Motor3 - Back)
#define ENCODER3_B 49

// ===== 엔코더 카운트 =====
volatile long encoder1_count = 0;
volatile long encoder2_count = 0;
volatile long encoder3_count = 0;

// ===== 엔코더 설정 =====
const float ENCODER1_PPR = 618.0;   
const float ENCODER2_PPR = 327.0;
const float ENCODER3_PPR = 330.0;

// ===== 통신 설정 =====
const unsigned long SERIAL_BAUD = 115200;
const unsigned long TIMEOUT_MS = 500;  // 0.5초 타임아웃
unsigned long lastCommandTime = 0;

// ===== 현재 휠 속도 (rad/s) - 목표값 =====
float wheel1_speed = 0.0;
float wheel2_speed = 0.0;
float wheel3_speed = 0.0;

// ===== 최대 RPM 및 PWM 설정 =====
const float MAX_RPM = 100.0;
const float MAX_RAD_S = (MAX_RPM * 2 * PI) / 60.0;
const int PWM_MAX = 255;

// ========== PID 제어 파라미터 ==========
const float Kp = 20.0;
const float Ki = 5.0;
const float Kd = 0.5;

struct PIDController {
  float target_speed = 0;
  float measured_speed = 0;
  float error = 0;
  float prev_error = 0;
  float integral = 0;
  float derivative = 0;
  int pwm_output = 0;
};

PIDController pid1, pid2, pid3;
bool pid_enabled = false;

// ========== 순기구학용 변수 ==========
long prev_encoder1 = 0;
long prev_encoder2 = 0;
long prev_encoder3 = 0;
unsigned long prev_time = 0;

float measured_vx = 0.0;
float measured_vy = 0.0;
float measured_wz = 0.0;

// ===== 함수 선언 =====
void setupMotors();
void setupEncoders();
void parseSerialCommand();
void emergencyStop();
void computeWheelSpeeds(float vx, float vy, float wz, float &w1, float &w2, float &w3);
void computeRobotVelocity(float w1, float w2, float w3, float &vx, float &vy, float &wz);
void updatePIDControl();
void sendEncoderFeedback();
void sendOdometry();
int computePID(PIDController &pid, float dt);
void resetPID(PIDController &pid);
void setMotorPWM(int motorNum, int pwm);

// ===== 인터럽트 서비스 루틴 =====
void encoder1_ISR() {
  if (digitalRead(ENCODER1_B) == HIGH) encoder1_count++;
  else encoder1_count--;
}

void encoder2_ISR() {
  // Motor2 엔코더 방향 반대로 수정 (FIX)
  if (digitalRead(ENCODER2_B) == HIGH) encoder2_count--;
  else encoder2_count++;
}

void encoder3_ISR() {
  if (digitalRead(ENCODER3_B) == HIGH) encoder3_count++;
  else encoder3_count--;
}

// ========================================
// Setup
// ========================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  setupMotors();
  setupEncoders();
  
  // PID 초기화
  resetPID(pid1);
  resetPID(pid2);
  resetPID(pid3);
  prev_time = millis();
  
  // 시작 메시지
  Serial.println("=====================================");
  Serial.println("Moebius Board - Omni Robot Ready");
  Serial.println("Custom Inverse Kinematics Enabled");
  Serial.println("Motor2 Encoder: Direction Fixed");
  Serial.println("Waiting for ROS2 commands...");
  Serial.println("=====================================");
  
  lastCommandTime = millis();
}

// ========================================
// Loop
// ========================================
void loop() {
  // Serial 명령 수신
  if (Serial.available() > 0) {
    parseSerialCommand();
    lastCommandTime = millis();
  }
  
  // PID 제어 업데이트 (항상 실행되지만, pid_enabled가 false면 정지)
  updatePIDControl();
  
  // 타임아웃 체크: PID가 켜져 있으면 0.5초 후 정지
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    if (pid_enabled) {
      emergencyStop();
      Serial.println("WARN: Timeout - Emergency Stop (PID Active)");
    }
  }
  
  delay(10);  // 100Hz
}

// ========================================
// 모터 초기화
// ========================================
void setupMotors() {
  pinMode(MOTOR1_PWM, OUTPUT); pinMode(MOTOR1_DIR1, OUTPUT); pinMode(MOTOR1_DIR2, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT); pinMode(MOTOR2_DIR1, OUTPUT); pinMode(MOTOR2_DIR2, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT); pinMode(MOTOR3_DIR1, OUTPUT); pinMode(MOTOR3_DIR2, OUTPUT);
  emergencyStop();
}

// ========================================
// 엔코더 초기화
// ========================================
void setupEncoders() {
  pinMode(ENCODER1_A, INPUT_PULLUP); pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP); pinMode(ENCODER2_B, INPUT_PULLUP);
  pinMode(ENCODER3_A, INPUT_PULLUP); pinMode(ENCODER3_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A), encoder3_ISR, RISING);
}

// ========================================
// Serial 명령 파싱
// ========================================
void parseSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  if (command.startsWith("VEL ")) {
    command = command.substring(4);
    
    float vx = 0, vy = 0, wz = 0;
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      vx = command.substring(0, firstSpace).toFloat();
      vy = command.substring(firstSpace + 1, secondSpace).toFloat();
      wz = command.substring(secondSpace + 1).toFloat();
      
      // 커스텀 역기구학 계산
      computeWheelSpeeds(vx, vy, wz, wheel1_speed, wheel2_speed, wheel3_speed);
      
      // 정지 명령인지 확인
      if (vx == 0 && vy == 0 && wz == 0) {
        pid_enabled = false;
        emergencyStop();
        Serial.println("OK: Stop command received");
      } else {
        // 이동 명령
        pid1.target_speed = wheel1_speed;
        pid2.target_speed = wheel2_speed;
        pid3.target_speed = wheel3_speed;
        
        // I-term 리셋 (부드러운 시작)
        pid1.integral = 0; pid2.integral = 0; pid3.integral = 0;
        
        pid_enabled = true;
        
        Serial.print("OK: vx=");
        Serial.print(vx, 3);
        Serial.print(" vy=");
        Serial.print(vy, 3);
        Serial.print(" wz=");
        Serial.print(wz, 3);
        Serial.print(" | w1=");
        Serial.print(wheel1_speed, 3);
        Serial.print(" w2=");
        Serial.print(wheel2_speed, 3);
        Serial.print(" w3=");
        Serial.println(wheel3_speed, 3);
      }
    }
  }
  else if (command == "STOP") {
    emergencyStop();
    Serial.println("OK: Emergency Stop");
  }
  else if (command == "PING") {
    Serial.println("PONG");
  }
  else if (command == "ENC") {
    sendEncoderFeedback();
  }
  else if (command == "ODOM") {
    sendOdometry();
  }
  else if (command == "RESET_ENC") {
    encoder1_count = 0; encoder2_count = 0; encoder3_count = 0;
    prev_encoder1 = 0; prev_encoder2 = 0; prev_encoder3 = 0;
    Serial.println("OK: Encoders Reset");
  }
  else {
    Serial.println("ERR: Unknown command");
  }
}

// ========================================
// 역기구학: 커스텀 버전
// ========================================
void computeWheelSpeeds(float vx, float vy, float wz, float &w1, float &w2, float &w3) {
  w1 = 0.0;
  w2 = 0.0;
  w3 = 0.0;
  
  const float LINEAR_SCALE = 1.0 / R_WHEEL;
  const float ANGULAR_SCALE = R_BASE / R_WHEEL;
  const float THRESHOLD = 0.01;
  
  // ========================================
  // 1. 순수 회전 (wz만 있음)
  // ========================================
  if (abs(vx) < THRESHOLD && abs(vy) < THRESHOLD && abs(wz) > THRESHOLD) {
    w1 = wz * ANGULAR_SCALE;
    w2 = wz * ANGULAR_SCALE;
    w3 = -wz * ANGULAR_SCALE;  // Motor3 반대!
    Serial.print("[Rotation] ");
  }
  
  // ========================================
  // 2. 순수 전진/후진 (vx만 있음)
  // ========================================
  else if (abs(vx) > THRESHOLD && abs(vy) < THRESHOLD && abs(wz) < THRESHOLD) {
    w1 = -0.866 * vx * LINEAR_SCALE;
    w2 = 0.866 * vx * LINEAR_SCALE;
    w3 = 0.0;  // Motor3 정지!
    Serial.print("[Forward/Back] ");
  }
  
  // ========================================
  // 3. 대각선: 전진+왼쪽 (Q키)
  // ========================================
  else if (vx > THRESHOLD && vy > THRESHOLD && abs(wz) < THRESHOLD) {
    float speed = sqrt(vx*vx + vy*vy);
    w1 = 0.0;  // Motor1 정지!
    w2 = speed * LINEAR_SCALE;
    w3 = speed * LINEAR_SCALE;  // Motor3 반대!
    Serial.print("[Diagonal Q] ");
  }
  
  // ========================================
  // 4. 대각선: 전진+오른쪽 (E키)
  // ========================================
  else if (vx > THRESHOLD && vy < -THRESHOLD && abs(wz) < THRESHOLD) {
    float speed = sqrt(vx*vx + vy*vy);
    w1 = -speed * LINEAR_SCALE;
    w2 = 0.0;  // Motor2 정지!
    w3 = -speed * LINEAR_SCALE;  // Motor3 반대!
    Serial.print("[Diagonal E] ");
  }
  
  // ========================================
  // 5. 대각선: 후진+왼쪽 (A키)
  // ========================================
  else if (vx < -THRESHOLD && vy > THRESHOLD && abs(wz) < THRESHOLD) {
    float speed = sqrt(vx*vx + vy*vy);
    w1 = speed * LINEAR_SCALE;
    w2 = 0.0;  // Motor2 정지!
    w3 = speed * LINEAR_SCALE;  // Motor3 반대!
    Serial.print("[Diagonal A] ");
  }
  
  // ========================================
  // 6. 대각선: 후진+오른쪽 (D키)
  // ========================================
  else if (vx < -THRESHOLD && vy < -THRESHOLD && abs(wz) < THRESHOLD) {
    float speed = sqrt(vx*vx + vy*vy);
    w1 = 0.0;  // Motor1 정지!
    w2 = -speed * LINEAR_SCALE;
    w3 = -speed * LINEAR_SCALE;  // Motor3 반대!
    Serial.print("[Diagonal D] ");
  }
  
  // ========================================
  // 7. 순수 횡이동 (vy만 있음)
  // ========================================
  else if (abs(vx) < THRESHOLD && abs(vy) > THRESHOLD && abs(wz) < THRESHOLD) {
    w1 = 0.5 * vy * LINEAR_SCALE;
    w2 = 0.5 * vy * LINEAR_SCALE;
    w3 = -1.0 * vy * LINEAR_SCALE;  // Motor3 반대!
    Serial.print("[Left/Right] ");
  }
  
  // ========================================
  // 8. 복합 동작 (여러 값이 섞임)
  // ========================================
  else {
    w1 = (-0.866f * vx + 0.5f * vy + R_BASE * wz) / R_WHEEL;
    w2 = (0.866f * vx + 0.5f * vy + R_BASE * wz) / R_WHEEL;
    w3 = (0.0f * vx - 1.0f * vy - R_BASE * wz) / R_WHEEL;  // wz 반대!
    Serial.print("[Complex] ");
  }
  
  // 최대값 제한
  float maxW = max(max(abs(w1), abs(w2)), abs(w3));
  if (maxW > MAX_RAD_S) {
    float scale = MAX_RAD_S / maxW;
    w1 *= scale;
    w2 *= scale;
    w3 *= scale;
  }
}

// ========================================
// 순기구학: 휠 속도 → 로봇 속도
// ========================================
void computeRobotVelocity(float w1, float w2, float w3, float &vx, float &vy, float &wz) {
  const float theta1 = 11.0 * PI / 6.0;  // 330도 → W1
  const float theta2 = 7.0 * PI / 6.0;   // 210도 → W2
  const float theta3 = PI / 2.0;         // 90도  → W3

  vx = R_WHEEL * ( -sin(theta1)*w1 - sin(theta2)*w2 - sin(theta3)*w3 ) / 3.0;
  vy = R_WHEEL * (  cos(theta1)*w1 + cos(theta2)*w2 + cos(theta3)*w3 ) / 3.0;
  wz = R_WHEEL * (w1 + w2 + w3) / (3.0 * R_BASE);
}

// ========================================
// PID 제어 업데이트 (100Hz)
// ========================================
void updatePIDControl() {
  if (!pid_enabled) {
    // PID 꺼져 있으면 모터 정지 보장
    setMotorPWM(1, 0);
    setMotorPWM(2, 0);
    setMotorPWM(3, 0);
    return;
  }
  
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;
  
  if (dt >= 0.01) {  // 100Hz
    long delta1 = encoder1_count - prev_encoder1;
    long delta2 = encoder2_count - prev_encoder2;
    long delta3 = encoder3_count - prev_encoder3;
    
    pid1.measured_speed = (delta1 * 2.0 * PI) / (ENCODER1_PPR * dt);
    pid2.measured_speed = (delta2 * 2.0 * PI) / (ENCODER2_PPR * dt);
    pid3.measured_speed = (delta3 * 2.0 * PI) / (ENCODER3_PPR * dt);
    
    pid1.pwm_output = computePID(pid1, dt);
    pid2.pwm_output = computePID(pid2, dt);
    pid3.pwm_output = computePID(pid3, dt);
    
    setMotorPWM(1, pid1.pwm_output);
    setMotorPWM(2, pid2.pwm_output);
    setMotorPWM(3, pid3.pwm_output);
    
    computeRobotVelocity(pid1.measured_speed, pid2.measured_speed, pid3.measured_speed,
                        measured_vx, measured_vy, measured_wz);
    
    prev_encoder1 = encoder1_count;
    prev_encoder2 = encoder2_count;
    prev_encoder3 = encoder3_count;
    prev_time = current_time;
  }
}

// ========================================
// PID 계산
// ========================================
int computePID(PIDController &pid, float dt) {
  pid.error = pid.target_speed - pid.measured_speed;
  pid.integral += pid.error * dt;
  pid.integral = constrain(pid.integral, -10.0, 10.0);
  pid.derivative = (pid.error - pid.prev_error) / dt;
  
  float output = Kp * pid.error + Ki * pid.integral + Kd * pid.derivative;
  int pwm = constrain((int)output, -PWM_MAX, PWM_MAX);
  
  pid.prev_error = pid.error;
  return pwm;
}

// ========================================
// PID 리셋
// ========================================
void resetPID(PIDController &pid) {
  pid.target_speed = 0;
  pid.measured_speed = 0;
  pid.error = 0;
  pid.prev_error = 0;
  pid.integral = 0;
  pid.derivative = 0;
  pid.pwm_output = 0;
}

// ========================================
// PID용 PWM 제어
// ========================================
void setMotorPWM(int motorNum, int pwm) {
  int pwmPin, dir1Pin, dir2Pin;
  switch(motorNum) {
    case 1: pwmPin = MOTOR1_PWM; dir1Pin = MOTOR1_DIR1; dir2Pin = MOTOR1_DIR2; break;
    case 2: pwmPin = MOTOR2_PWM; dir1Pin = MOTOR2_DIR1; dir2Pin = MOTOR2_DIR2; break;
    case 3: pwmPin = MOTOR3_PWM; dir1Pin = MOTOR3_DIR1; dir2Pin = MOTOR3_DIR2; break;
    default: return;
  }
  
  bool forward = (pwm >= 0);
  int pwm_abs = abs(pwm);
  if (pwm_abs < 10) pwm_abs = 0;  // 데드존
  
  digitalWrite(dir1Pin, forward ? HIGH : LOW);
  digitalWrite(dir2Pin, forward ? LOW : HIGH);
  analogWrite(pwmPin, pwm_abs);
}

// ========================================
// 긴급 정지
// ========================================
void emergencyStop() {
  wheel1_speed = 0; wheel2_speed = 0; wheel3_speed = 0;
  pid_enabled = false;
  resetPID(pid1); resetPID(pid2); resetPID(pid3);
  
  analogWrite(MOTOR1_PWM, 0); analogWrite(MOTOR2_PWM, 0); analogWrite(MOTOR3_PWM, 0);
  digitalWrite(MOTOR1_DIR1, LOW); digitalWrite(MOTOR1_DIR2, LOW);
  digitalWrite(MOTOR2_DIR1, LOW); digitalWrite(MOTOR2_DIR2, LOW);
  digitalWrite(MOTOR3_DIR1, LOW); digitalWrite(MOTOR3_DIR2, LOW);
}

// ========================================
// 엔코더 피드백 전송
// ========================================
void sendEncoderFeedback() {
  Serial.print("ENC: e1=");
  Serial.print(encoder1_count);
  Serial.print(" e2=");
  Serial.print(encoder2_count);
  Serial.print(" e3=");
  Serial.println(encoder3_count);
}

// ========================================
// 오도메트리 전송
// ========================================
void sendOdometry() {
  Serial.print("ODOM: vx=");
  Serial.print(measured_vx, 4);
  Serial.print(" vy=");
  Serial.print(measured_vy, 4);
  Serial.print(" wz=");
  Serial.print(measured_wz, 4);
  Serial.print(" | w1=");
  Serial.print(pid1.measured_speed, 3);
  Serial.print(" w2=");
  Serial.print(pid2.measured_speed, 3);
  Serial.print(" w3=");
  Serial.println(pid3.measured_speed, 3);
}