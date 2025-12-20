#include <SCServo.h>
#include <AdaptiveGripper2.h>

SMS_STS sms_sts;

// ==================================================================================
// [좌표 설정] Teaching으로 얻은 값을 확인해서 넣으세요!
// ==================================================================================
struct RobotPose { int joints[5]; };

RobotPose POSE_HOME     = { {2287, 1358, 2351, 2908, 2199} };
RobotPose POSE_SCAN     = { {2743, 1298, 2708, 2724, 2245} };
RobotPose POSE_PRE_PICK = { {2817, 1676, 1931, 3151, 2244} };
RobotPose POSE_PICK     = { {2747, 1807, 2262, 2841, 2244} };
RobotPose POSE_PLACE    = { {3100, 1996, 2004, 2856, 2244} };

const int DEFAULT_OPEN_VAL = 1480;   
const int DEFAULT_CLOSE_VAL = 2372;  
int DEFAULT_LOAD_LIMIT = 500;        
int MOVE_SPEED = 1000;    
int MOVE_ACC = 30;

#define S_RXD 16
#define S_TXD 17
#define GRIPPER_ID       1
#define BTN_START_PIN    32
#define BTN_STOP_PIN     33
#define BTN_TORQ_ON_PIN  25
#define BTN_TORQ_OFF_PIN 26

#define ID_BASE 6
#define ID_SHOULDER 5
#define ID_ELBOW 4
#define ID_WRIST 3
#define ID_ROT 2

AdaptiveGripper2 gripper(sms_sts, GRIPPER_ID, BTN_START_PIN, BTN_STOP_PIN, BTN_TORQ_ON_PIN, BTN_TORQ_OFF_PIN);

void torqueAll(bool on) {
  gripper.torqueOn(on);
  for (int i = 2; i <= 6; i++) sms_sts.EnableTorque(i, on ? 1 : 0);
}

void moveToPose(RobotPose p, int time_ms) {
  sms_sts.WritePosEx(ID_BASE,     p.joints[0], MOVE_SPEED, MOVE_ACC);
  sms_sts.WritePosEx(ID_SHOULDER, p.joints[1], MOVE_SPEED, MOVE_ACC);
  sms_sts.WritePosEx(ID_ELBOW,    p.joints[2], MOVE_SPEED, MOVE_ACC);
  sms_sts.WritePosEx(ID_WRIST,    p.joints[3], MOVE_SPEED, MOVE_ACC);
  sms_sts.WritePosEx(ID_ROT,      p.joints[4], MOVE_SPEED, MOVE_ACC);
  delay(time_ms);
}

// [핵심 수정] 성공 여부를 반환하는 스마트 닫기 함수
// true: 물체 잡음 (중간에 부하 걸림)
// false: 허공 잡음 (끝까지 닫힘)
bool runSmoothAdaptiveClose() {
  gripper.torqueOn(true);
  sms_sts.WritePosEx(GRIPPER_ID, DEFAULT_CLOSE_VAL, 1000, 30);
  delay(400); 
  
  bool objectDetected = false;
  unsigned long startT = millis();
  
  while(millis() - startT < 3000) {
      int load = sms_sts.ReadLoad(GRIPPER_ID);
      int pos = sms_sts.ReadPos(GRIPPER_ID);
      if (load == -1 || pos == -1) { delay(5); continue; }
      
      // 1. 물체 감지됨!
      if (abs(load) > DEFAULT_LOAD_LIMIT) {
          sms_sts.WritePosEx(GRIPPER_ID, pos, 0, 0); 
          Serial.printf("Grip: Object Detected (Load %d)\n", load);
          objectDetected = true;
          break;
      }
      // 2. 끝까지 닫힘 (물체 없음)
      if (abs(pos - DEFAULT_CLOSE_VAL) < 30) {
          objectDetected = false;
          break;
      }
      delay(5);
  }
  return objectDetected;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sms_sts.pSerial = &Serial1;
  gripper.begin();
  gripper.setOpenClose(DEFAULT_OPEN_VAL, DEFAULT_CLOSE_VAL); 
  gripper.setLoadThreshold(DEFAULT_LOAD_LIMIT);    
  torqueAll(true);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // ==================================================================
    // [시나리오 1] 집기 (재시도 로직 포함)
    // ==================================================================
    if (input == "SEQ:PICK") {
        Serial.println("STATUS:BUSY_PICK");

        // 0. [재시도 대비] 그리퍼 확실히 열기
        sms_sts.WritePosEx(GRIPPER_ID, DEFAULT_OPEN_VAL, 1000, 30);
        
        // 1. 홈 -> 스캔 이동
        moveToPose(POSE_SCAN, 2000);
        
        // 2. 스캔 대기 (5초)
        delay(5000); 

        // 3. 집기 준비 -> 집기
        moveToPose(POSE_PRE_PICK, 2000);
        moveToPose(POSE_PICK, 1500);
        
        // 4. [핵심] 잡기 시도 및 결과 확인
        bool isGripped = runSmoothAdaptiveClose(); 
        delay(500);

        // 5. 들어 올리기
        moveToPose(POSE_PRE_PICK, 1500);

        // 6. 결과 처리
        if (isGripped) {
            // 성공: 물체 든 채로 홈 복귀
            moveToPose(POSE_HOME, 2000);
            Serial.println("DONE:PICK"); // 성공 신호 전송
        } else {
            // 실패: 그리퍼 다시 열고 홈 복귀 (재시도 준비)
            sms_sts.WritePosEx(GRIPPER_ID, DEFAULT_OPEN_VAL, 1000, 30);
            delay(1000);
            moveToPose(POSE_HOME, 2000);
            Serial.println("FAIL:PICK"); // [NEW] 실패 신호 전송
        }
    }

    // ==================================================================
    // [시나리오 2] 놓기
    // ==================================================================
    else if (input == "SEQ:RELEASE") {
        Serial.println("STATUS:BUSY_RELEASE");
        moveToPose(POSE_PLACE, 2000);
        sms_sts.WritePosEx(GRIPPER_ID, DEFAULT_OPEN_VAL, 1000, 30);
        delay(1000);
        moveToPose(POSE_HOME, 2000);
        Serial.println("DONE:RELEASE");
    }

    // === 기타 ===
    else if (input == "TORQUE:OFF") torqueAll(false);
    else if (input == "TORQUE:ON") torqueAll(true);
    else if (input == "G:OPEN") sms_sts.WritePosEx(GRIPPER_ID, DEFAULT_OPEN_VAL, 1000, 30);
    else if (input == "G:CLOSE") runSmoothAdaptiveClose();
  }
  gripper.update();
}
