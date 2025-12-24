// AdaptiveGripper.cpp
#include "AdaptiveGripper.h"

// === 튜닝 상수들 ===
static const int CLOSE_LIMIT        = 1000;
static const int OPEN_POSITION      = 4095;
static const int STEP_DELTA         = 40;
static const int CMD_SPEED          = 1500;
static const int CMD_ACC            = 50;
static const int LOAD_THRESHOLD     = 285;
static const int FEEDBACK_PERIOD_MS = 30;
static const int STEP_PERIOD_MS     = 50;
static const int DEBOUNCE_MS        = 30;

// 생성자
AdaptiveGripper::AdaptiveGripper(SMS_STS &driver,
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
    bToff_{pinToff,  true, true, 0}
{
  state_           = State::Idle;
  targetPos_       = OPEN_POSITION;
  lastLoad_        = 0;
  lastCurrent_     = 0;
  t_last_feedback_ = 0;
  t_last_step_     = 0;
}

void AdaptiveGripper::begin() {
  // 버튼 입력 풀업
  pinMode(bStart_.pin, INPUT_PULLUP);
  pinMode(bStop_.pin,  INPUT_PULLUP);
  pinMode(bTon_.pin,   INPUT_PULLUP);
  pinMode(bToff_.pin,  INPUT_PULLUP);

  torqueOn(true);
  targetPos_ = OPEN_POSITION;
  writePos(targetPos_);

  Serial.println("=== AdaptiveGripper begin ===");
}

void AdaptiveGripper::update() {
  unsigned long now = millis();

  handleButtons();

  if (now - t_last_feedback_ >= FEEDBACK_PERIOD_MS) {
    t_last_feedback_ = now;
    if (drv_.FeedBack(id_) != -1) {
      lastLoad_    = drv_.ReadLoad(-1);
      lastCurrent_ = drv_.ReadCurrent(-1);
    }
  }

  if (now - t_last_step_ >= STEP_PERIOD_MS) {
    t_last_step_ = now;
    stepFSM();
  }
}

void AdaptiveGripper::handleSerial(char c) {
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

void AdaptiveGripper::startOpen() {
  if (state_ != State::Opening) {
    state_ = State::Opening;
    t_last_step_ = millis();
    Serial.println("[GRIPPER] Opening...");
  }
}

void AdaptiveGripper::startClose() {
  if (state_ != State::Closing) {
    state_ = State::Closing;
    t_last_step_ = millis();
    Serial.println("[GRIPPER] Closing...");
  }
}

void AdaptiveGripper::holdHere() {
  state_ = State::Hold;
  Serial.println(">>> [GRIPPER] HOLD.");
}

void AdaptiveGripper::torqueOn(bool on) {
  drv_.EnableTorque(id_, on ? 1 : 0);
  Serial.println(on ? "[Torque] ON" : "[Torque] OFF");
}

int  AdaptiveGripper::lastLoad()    const { return lastLoad_; }
int  AdaptiveGripper::lastCurrent() const { return lastCurrent_; }
int  AdaptiveGripper::targetPos()   const { return targetPos_; }
AdaptiveGripper::State AdaptiveGripper::state() const { return state_; }

// ---- private helpers ----

bool AdaptiveGripper::readButton(Btn &b) {
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

void AdaptiveGripper::handleButtons() {
  if (readButton(bStart_)) {
    if (state_ == State::Idle || state_ == State::Opening || state_ == State::Hold) {
      torqueOn(true);
      startClose();
    }
  }
  if (readButton(bStop_)) {
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

void AdaptiveGripper::writePos(int pos) {
  pos = constrain(pos, 0, 4095);
  drv_.WritePosEx(id_, pos, CMD_SPEED, CMD_ACC);
}

void AdaptiveGripper::stepFSM() {
  switch (state_) {
    case State::Idle:
      break;

    case State::Opening:
      if (targetPos_ < OPEN_POSITION) {
        targetPos_ = min(targetPos_ + STEP_DELTA, OPEN_POSITION);
        writePos(targetPos_);
      } else {
        state_ = State::Idle;
        Serial.println("[STATE] Opened -> Idle");
      }
      break;

    case State::Closing:
      if (targetPos_ > CLOSE_LIMIT) {
        targetPos_ = max(targetPos_ - STEP_DELTA, CLOSE_LIMIT);
        writePos(targetPos_);
      }

      if (lastLoad_ >= LOAD_THRESHOLD) {
        Serial.printf("[GRIPPER] Object detected - load=%d (>= %d)\n",
                      lastLoad_, LOAD_THRESHOLD);
        holdHere();
      }

      if (targetPos_ <= CLOSE_LIMIT) {
        Serial.println("[GRIPPER] Reached CLOSE_LIMIT -> HOLD");
        holdHere();
      }
      break;

    case State::Hold:
      break;
  }
}

