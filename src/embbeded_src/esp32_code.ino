#include <SCServo.h>
#include <AdaptiveGripper2.h>

SMS_STS sms_sts;

// ==================================================================================
// [좌표 설정] Teaching으로 얻은 값을 확인해서 넣으세요!
// ==================================================================================
struct RobotPose { int joints[5]; };

RobotPose POSE_HOME     = { {2287, 1358, 2351, 2908, 2199} };
RobotPose POSE_SCAN     = { { 3373, 1756, 2068, 2821, 2206 } };
RobotPose POSE_PRE_PICK = { { 3366, 1411, 2576, 2426, 2197 } };
RobotPose POSE_PICK     = { { 3366, 1411, 2576, 2426, 2197 } };
RobotPose POSE_PLACE    = { { 1330, 746, 2554, 3156, 2143 } };

const int DEFAULT_OPEN_VAL = 1480;   
const int DEFAULT_CLOSE_VAL = 2372;  

// [설정] 부하 한계값 230
int DEFAULT_LOAD_LIMIT = 130;        
int MOVE_SPEED = 500;    
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

// [성공 메시지 포함] 스마트 닫기 함수
bool runSmoothAdaptiveClose() {
  Serial.println(">>> [GRIPPER] Adaptive Close Start"); 
  gripper.torqueOn(true);
  
  sms_sts.WritePosEx(GRIPPER_ID, DEFAULT_CLOSE_VAL, 500, 30);
  delay(500); 
  
  bool objectDetected = false;
  unsigned long startT = millis();
  
  while(millis() - startT < 3000) {
      int load = sms_sts.ReadLoad(GRIPPER_ID);
      int pos = sms_sts.ReadPos(GRIPPER_ID);
      
      if (load == -1 || pos == -1) { delay(5); continue; }
      
      Serial.printf("DEBUG: Load=%d (Limit=%d) / Pos=%d\n", load, DEFAULT_LOAD_LIMIT, pos);
      
      // 1. 물체 감지됨! (성공)
      if (abs(load) > DEFAULT_LOAD_LIMIT) {
          sms_sts.WritePosEx(GRIPPER_ID, pos, 0, 0); 
          Serial.printf(">>> [GRIPPER] ADAPTIVE SUCCESS! Object Detected (Load: %d)\n", load);
          objectDetected = true;
          break;
      }
      
      // 2. 끝까지 닫힘 (물체 없음)
      if (abs(pos - DEFAULT_CLOSE_VAL) < 30) {
          Serial.printf(">>> [GRIPPER] FAILED (Empty Hand). Pos: %d\n", pos);
          objectDetected = false;
          break;
      }
      delay(20); 
  }
  return objectDetected;
}

// =========================================================================
// SETUP 함수 (필수)
// =========================================================================
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

// =========================================================================
// LOOP 함수 (필수)
// =========================================================================
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // ==================================================================
    // [시나리오 1] 집기 (재시도 로직 포함)
    // ==================================================================
    if (input == "SEQ:PICK") {
        Serial.println("STATUS:BUSY_PICK");

        sms_sts.WritePosEx(GRIPPER_ID, DEFAULT_OPEN_VAL, 1000, 30);
        
        moveToPose(POSE_SCAN, 2000);
        delay(5000); 

        moveToPose(POSE_PRE_PICK, 2000);
        moveToPose(POSE_PICK, 1500);
        
        bool isGripped = runSmoothAdaptiveClose(); 
        delay(500);

        moveToPose(POSE_PRE_PICK, 1500);

        if (isGripped) {
            moveToPose(POSE_HOME, 2000);
            Serial.println("DONE:PICK"); 
        } else {
            sms_sts.WritePosEx(GRIPPER_ID, DEFAULT_OPEN_VAL, 1000, 30);
            delay(1000);
            moveToPose(POSE_HOME, 2000);
            Serial.println("FAIL:PICK"); 
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

    // ==================================================================
    // [추가된 기능] 현재 좌표 읽기 (GET_STAT)
    // ==================================================================
    else if (input == "GET_STAT") {
        int p1 = sms_sts.ReadPos(ID_BASE);
        int p2 = sms_sts.ReadPos(ID_SHOULDER);
        int p3 = sms_sts.ReadPos(ID_ELBOW);
        int p4 = sms_sts.ReadPos(ID_WRIST);
        int p5 = sms_sts.ReadPos(ID_ROT);
        
        // Python으로 데이터 전송 (POS:베이스:숄더:엘보:손목:회전)
        Serial.printf("POS:%d:%d:%d:%d:%d\n", p1, p2, p3, p4, p5);
    }

    // === 기타 수동 제어 ===
    else if (input == "TORQUE:OFF") torqueAll(false);
    else if (input == "TORQUE:ON") torqueAll(true);
    else if (input == "G:OPEN") sms_sts.WritePosEx(GRIPPER_ID, DEFAULT_OPEN_VAL, 1000, 30);
    else if (input == "G:CLOSE") runSmoothAdaptiveClose();
  }
  gripper.update();
}