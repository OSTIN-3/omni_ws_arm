// AdaptiveGripper2.cpp
#include "AdaptiveGripper2.h"

// === 공통 튜닝 상수들 ===
static const int STEP_DELTA         = 40;
static const int CMD_SPEED          = 1500;
static const int CMD_ACC            = 50;
static const int FEEDBACK_PERIOD_MS = 30;
static const int STEP_PERIOD_MS     = 50;
static const int DEBOUNCE_MS        = 30;

// 생성자
AdaptiveGripper2::AdaptiveGripper2(SMS_STS &driver,
                                   uint8_t servoId,
                                   uint8_t pinStart,
                                   uint8_t pinStop,
                                   uint8_t pinTon,
                                   uint8_t pinToff)
  : drv_(driver),
    id_(servoId),
    bStart_{pinStart, true, true, 0},
    bStop_{pinStop,  true, true, 0},
    bTon_{pinTon,    true, true, 0},
    bToff_{pinToff,  true, true, 0},
    state_(State::Idle),
    targetPos_(0),
    lastLoad_(0),
    lastCurrent_(0),
    t_last_feedback_(0),
    t_last_step_(0),
    // 기본값: 햄이 측정한 값 기준
    openPos_(1500),      // 그리퍼 완전 오픈 시 피텍 값
    closePos_(2351),     // 그리퍼 완전 클로즈 시 피텍 값
    loadThreshold_(285)  // 물체 감지 기준 (나중에 setLoadThreshold로 튜닝)
{
}

void AdaptiveGripper2::setOpenClose(int openPos, int closePos) {
  openPos_  = openPos;
  closePos_ = closePos;
}

void AdaptiveGripper2::setLoadThreshold(int threshold) {
  loadThreshold_ = threshold;
}

void AdaptiveGripper2::begin() {
  // 버튼 입력 풀업
  pinMode(bStart_.pin, INPUT_PULLUP);
  pinMode(bStop_.pin,  INPUT_PULLUP);
  pinMode(bTon_.pin,   INPUT_PULLUP);
  pinMode(bToff_.pin,  INPUT_PULLUP);

  torqueOn(true);

  // 시작은 오픈 위치로
  targetPos_ = openPos_;
  writePos(targetPos_);

  Serial.println("=== AdaptiveGripper2 begin ===");
}

void AdaptiveGripper2::update() {
  unsigned long now = millis();

  handleButtons();

  // 피드백 주기
  if (now - t_last_feedback_ >= FEEDBACK_PERIOD_MS) {
    t_last_feedback_ = now;
    if (drv_.FeedBack(id_) != -1) {
      lastLoad_    = drv_.ReadLoad(-1);
      lastCurrent_ = drv_.ReadCurrent(-1);
    }
  }

  // 상태머신 스텝
  if (now - t_last_step_ >= STEP_PERIOD_MS) {
    t_last_step_ = now;
    stepFSM();
  }
}

void AdaptiveGripper2::handleSerial(char c) {
  if (c == 's') {       // 닫기 시작
    torqueOn(true);
    startClose();
  }
  else if (c == 'x') {  // 홀드
    holdHere();
  }
  else if (c == 't') {  // 토크 ON
    torqueOn(true);
  }
  else if (c == 'r') {  // 토크 OFF
    torqueOn(false);
    state_ = State::Idle;
  }
  else if (c == 'o') {  // 열기 시작
    startOpen();
  }
}

void AdaptiveGripper2::startOpen() {
  if (state_ != State::Opening) {
    state_ = State::Opening;
    t_last_step_ = millis();
    Serial.println("[GRIPPER2] Opening...");
  }
}

void AdaptiveGripper2::startClose() {
  if (state_ != State::Closing) {
    state_ = State::Closing;
    t_last_step_ = millis();
    Serial.println("[GRIPPER2] Closing...");
  }
}

void AdaptiveGripper2::holdHere() {
  state_ = State::Hold;
  Serial.println(">>> [GRIPPER2] HOLD.");
}

void AdaptiveGripper2::torqueOn(bool on) {
  drv_.EnableTorque(id_, on ? 1 : 0);
  Serial.println(on ? "[GRIPPER2 Torque] ON" : "[GRIPPER2 Torque] OFF");
}

int  AdaptiveGripper2::lastLoad()    const { return lastLoad_; }
int  AdaptiveGripper2::lastCurrent() const { return lastCurrent_; }
int  AdaptiveGripper2::targetPos()   const { return targetPos_; }
AdaptiveGripper2::State AdaptiveGripper2::state() const { return state_; }

// ---- private helpers ----

bool AdaptiveGripper2::readButton(Btn &b) {
  bool r = digitalRead(b.pin);
  if (r != b.lastRead) {
    b.lastRead = r;
    b.lastT = millis();
  }
  if (millis() - b.lastT >= DEBOUNCE_MS) {
    if (b.lastStable != r) {
      b.lastStable = r;
      if (r == LOW) return true;
    }
  }
  return false;
}

void AdaptiveGripper2::handleButtons() {
  if (readButton(bStart_)) {
    // Start 버튼: 닫기
    if (state_ == State::Idle || state_ == State::Opening || state_ == State::Hold) {
      torqueOn(true);
      startClose();
    }
  }
  if (readButton(bStop_)) {
    // Stop 버튼: 현재 위치에서 홀드
    holdHere();
  }
  if (readButton(bTon_)) {
    torqueOn(true);
  }
  if (readButton(bToff_)) {
    torqueOn(false);
    state_ = State::Idle;
  }
}

void AdaptiveGripper2::writePos(int pos) {
  pos = constrain(pos, 0, 4095);
  Serial.printf("[GRIPPER2 WRITE] pos=%d\n", pos);
  drv_.WritePosEx(id_, pos, CMD_SPEED, CMD_ACC);
}

// 여기서는 햄 상황에 맞게:
//   openPos_ < closePos_ (예: 1500 < 2351)
//   → "열기" = 값 감소, "닫기" = 값 증가
void AdaptiveGripper2::stepFSM() {
  switch (state_) {
    case State::Idle:
      break;

    case State::Opening:
      // 열기: close(큰 값) -> open(작은 값) 방향으로 감소
      if (targetPos_ > openPos_) {
        targetPos_ = max(targetPos_ - STEP_DELTA, openPos_);
        writePos(targetPos_);
      } else {
        state_ = State::Idle;
        Serial.println("[GRIPPER2 STATE] Opened -> Idle");
      }
      break;
    case State::Closing:
      // 닫기: open(작은 값) -> close(큰 값) 방향으로 증가
      if (targetPos_ < closePos_) {
        targetPos_ = min(targetPos_ + STEP_DELTA, closePos_);
        writePos(targetPos_);
      }

      // 🔧 부하 조건: 물체를 잡았다고 판단
      {
        int loadAbs = abs(lastLoad_);
        if (loadAbs >= loadThreshold_) {
          Serial.printf("[GRIPPER2] Object detected - |load|=%d (>= %d)\n",
                        loadAbs, loadThreshold_);
          holdHere();
        }
      }

      // 완전 닫힘 위치까지 도달했을 때도 홀드
      if (targetPos_ >= closePos_) {
        Serial.println("[GRIPPER2] Reached CLOSE -> HOLD");
        holdHere();
      }
      break;
  }
}

