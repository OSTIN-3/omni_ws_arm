#pragma once

#include <Arduino.h>
#include <SMS_STS.h>

class AdaptiveGripper2 {
public:
  enum class State : uint8_t {
    Idle = 0,
    Opening,
    Closing,
    Hold
  };

  struct Btn {
    uint8_t       pin;
    bool          lastRead;
    bool          lastStable;
    unsigned long lastT;
  };

  // 생성자
  AdaptiveGripper2(SMS_STS &driver,
                   uint8_t servoId,
                   uint8_t pinStart,
                   uint8_t pinStop,
                   uint8_t pinTon,
                   uint8_t pinToff);

  // 초기화 (버튼 핀 설정 + 오픈 위치로 이동)
  void begin();

  // 주기 업데이트 (loop()에서 계속 호출)
  void update();

  // 시리얼 명령 처리 (s/o/x/t/r)
  void handleSerial(char c);

  // 직접 제어용
  void startOpen();
  void startClose();
  void holdHere();
  void torqueOn(bool on);

  // 상태 조회
  int   lastLoad()    const;
  int   lastCurrent() const;
  int   targetPos()   const;
  State state()       const;

  // === 튜닝 파라미터 ===
  // 오픈/클로즈 위치 (Feetech 내부 값) 설정
  // 예: setOpenClose(1500, 2351);
  void setOpenClose(int openPos, int closePos);

  // LOAD_THRESHOLD (물체 감지 기준) 설정
  // 예: setLoadThreshold(220);
  void setLoadThreshold(int threshold);

private:
  SMS_STS  &drv_;
  uint8_t   id_;
  Btn       bStart_;
  Btn       bStop_;
  Btn       bTon_;
  Btn       bToff_;

  State     state_;
  int       targetPos_;
  int       lastLoad_;
  int       lastCurrent_;
  unsigned long t_last_feedback_;
  unsigned long t_last_step_;

  // 서보 위치 튜닝 값
  int openPos_;        // 완전 오픈 위치 (작은 값, 예: 1500)
  int closePos_;       // 완전 클로즈 위치 (큰 값, 예: 2351)

  // 부하 임계값
  int loadThreshold_;  // 예: 285, 그리퍼마다 다르게 튜닝

  // 내부 헬퍼
  bool readButton(Btn &b);
  void handleButtons();
  void writePos(int pos);
  void stepFSM();
};

