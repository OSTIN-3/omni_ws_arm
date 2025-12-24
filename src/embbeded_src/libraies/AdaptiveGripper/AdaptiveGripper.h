// AdaptiveGripper.h
#pragma once
#include <Arduino.h>
#include <SCServo.h>

class AdaptiveGripper {
public:
  enum class State : uint8_t { Idle, Opening, Closing, Hold };

  struct Btn {
    uint8_t pin;
    bool    lastStable;
    bool    lastRead;
    unsigned long lastT;
  };

  AdaptiveGripper(SMS_STS &driver,
                  uint8_t servoId,
                  uint8_t pinStart,
                  uint8_t pinStop,
                  uint8_t pinTon,
                  uint8_t pinToff);

  // 스케치에서 호출할 API
  void begin();                 // 핀 설정, 토크 ON, 초기 위치
  void update();                // loop()에서 매 프레임 호출
  void handleSerial(char c);   // 시리얼 입력 한 글자 처리

  void startOpen();
  void startClose();
  void holdHere();
  void torqueOn(bool on);

  int   lastLoad()    const;
  int   lastCurrent() const;
  int   targetPos()   const;
  State state()       const;

private:
  SMS_STS  &drv_;
  uint8_t   id_;
  State     state_;
  int       targetPos_;
  int       lastLoad_;
  int       lastCurrent_;
  unsigned long t_last_feedback_;
  unsigned long t_last_step_;

  Btn bStart_, bStop_, bTon_, bToff_;

  bool readButton(Btn &b);
  void handleButtons();
  void writePos(int pos);
  void stepFSM();
};

