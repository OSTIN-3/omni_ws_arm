/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - 3개 모터 제어 및 32비트 누적 엔코더 + 키보드 제어
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// MPU9250 센서 데이터 구조체
typedef struct {
    float Ax;      // X축 가속도 (g)
    float Ay;      // Y축 가속도 (g)
    float Gz;      // Z축 각속도 (deg/s)
    float Yaw;     // Yaw 각도 (deg)

} IMU_t;

// 속도 계산용 구조체 추가
typedef struct {
    int32_t prev_count;        // 이전 엔코더 값
    float rpm;                 // 속도 (RPM)
    float rpm_filtered;        // 필터링된 속도 (RPM)
    uint32_t last_time;        // 마지막 계산 시간
} Motor_Speed_t;

// PID 제어 구조체
typedef struct {
    float Kp;              // 비례 게인
    float Ki;              // 적분 게인
    float Kd;              // 미분 게인

    float target_rpm;      // 목표 속도 (RPM)
    float error;           // 현재 오차
    float error_sum;       // 오차 적분 (누적)
    float error_prev;      // 이전 오차 (미분용)

    float output;          // PID 출력 (PWM)

    float output_min;      // PWM 최소값 (안티 와인드업)
    float output_max;      // PWM 최대값

    uint32_t last_time;    // 마지막 계산 시간
} PID_Controller_t;

// ⭐ [오도메트리] 구조체 추가!
typedef struct {
    float x;              // X 위치 (m)
    float y;              // Y 위치 (m)
    float theta;          // 방향 (rad, -π ~ +π)
    float vx;             // X 속도 (m/s, 글로벌)
    float vy;             // Y 속도 (m/s, 글로벌)
    float omega;          // 각속도 (rad/s)
    uint32_t last_time;   // 마지막 업데이트 시간 (ms)
} Odometry_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- [모터 방향 핀 정의] ---
#define M1_DIR_PORT GPIOE
#define M1_DIR_PIN GPIO_PIN_0

#define M2_DIR_PORT GPIOE
#define M2_DIR_PIN GPIO_PIN_1

#define M3_DIR_PORT GPIOE
#define M3_DIR_PIN GPIO_PIN_2

#define ENCODER_PPR 875.0

#define ODOM_CORRECTION 1.5625f   // ⭐ 오도메트리 보정 계수

// [오도메트리] 물리 파라미터 (실측값!)
#define WHEEL_RADIUS 0.04f        // 바퀴 반지름 (m) = 4cm
#define ROBOT_RADIUS 0.24f        // 로봇 중심-바퀴 거리 (m) = 24cm

// 모터 각도 (라디안) - 실측 확인
#define M1_ANGLE (150.0f * M_PI / 180.0f)   // 10시 = 150° = 2.618 rad
#define M2_ANGLE (30.0f * M_PI / 180.0f)    // 2시  = 30°  = 0.524 rad
#define M3_ANGLE (270.0f * M_PI / 180.0f)   // 6시  = 270° = 4.712 rad

// --- [속도 설정] ---
#define SPEED_MIN 100
#define SPEED_MAX 1000
#define SPEED_STEP 100

// --- [카운터 오버플로우 값 정의] ---
// 16비트 타이머 (TIM8)의 최대 카운트 + 1 (2^16)
#define COUNTER_MAX_16BIT (65536LL)
// 32비트 타이머 (TIM5, TIM2)의 최대 카운트 + 1 (2^32)
#define COUNTER_MAX_32BIT (4294967296LL)

// --- [MPU9250 설정] ---
#define MPU9250_ADDR    0xD0
#define WHO_AM_I_REG    0x75
#define PWR_MGMT_1_REG  0x6B
#define SMPLRT_DIV_REG  0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43

#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY  131.0f

// 로우패스 필터 계수 추가
#define LPF_ALPHA 0.3f  // 0~1, 작을수록 더 부드러움

#define RESET_ALL_PIDS() \
    PID_Reset(&pid_motor1); \
    PID_Reset(&pid_motor2); \
    PID_Reset(&pid_motor3); \
    PID_Reset(&pid_yaw); \
    imu.Yaw = 0.0f; \
    target_yaw = 0.0f;
float gyro_z_bias = 0.0f;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- [엔코더 32비트 누적 카운트 변수] ---
// 이 변수들은 ISR(인터럽트)에서 오버플로우/언더플로우 감지를 통해 누적됩니다.
volatile int64_t Encoder1_Count = 0; // 모터1 누적 (TIM5)
volatile int64_t Encoder2_Count = 0; // 모터2 누적 (TIM2)
volatile int64_t Encoder3_Count = 0; // 모터3 누적 (TIM8)

// --- [키보드 제어 변수] ---
uint16_t current_speed = 500; // 초기 속도 (중간값)
uint8_t rx_data = 0; // UART 수신 버퍼

// --- [IMU 변수] ---
IMU_t imu;
uint32_t last_imu_update_time = 0;
const uint16_t i2c_timeout = 100;

// 속도 계산 변수 추가
Motor_Speed_t motor1_speed = {0, 0.0f, 0.0f, 0};
Motor_Speed_t motor2_speed = {0, 0.0f, 0.0f, 0};
Motor_Speed_t motor3_speed = {0, 0.0f, 0.0f, 0};

// PID 제어 변수
PID_Controller_t pid_motor1;
PID_Controller_t pid_motor2;
PID_Controller_t pid_motor3;

// Yaw PID 제어 변수 추가!
PID_Controller_t pid_yaw;

// PID 제어 활성화 플래그
uint8_t pid_enabled = 0;  // 0: 수동 제어, 1: PID 제어

// PID 목표 속도 변수 추가!
float target_speed = 50.0f;  // 기본 목표 속도 (RPM)

// Yaw 제어 변수 추가!
float target_yaw = 0.0f;  // 목표 Yaw 각도 (직진: 0도)
uint8_t yaw_control_enabled = 1;  // Yaw 제어 활성화 (기본: ON)

// 전진 모드 플래그
uint8_t is_forward_mode = 0;  // 0: OFF, 1: 전진 중

// [오도메트리] 변수 추가!
Odometry_t odom = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0};
// UART 수신 버퍼
#define RX_BUFFER_SIZE 64
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

char uart_buffer[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
uint8_t MPU9250_Init(I2C_HandleTypeDef *I2Cx);
void MPU9250_Read_Sensors(I2C_HandleTypeDef *I2Cx, IMU_t *DataStruct);
void Send_Encoder_Data(void);
void Send_IMU_Data(void);
void Calculate_Motor_Speed(Motor_Speed_t *motor, int32_t current_count);
void Update_All_Motor_Speeds(void);
void Send_Speed_Data(void);
void Process_Command(uint8_t cmd);

// PID 함수 프로토타입
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd);
float PID_Compute(PID_Controller_t *pid, float current_rpm);
void PID_Reset(PID_Controller_t *pid);
void Apply_PID_Output(void);
void Set_Target_RPM(float rpm1, float rpm2, float rpm3);

// Yaw PID 함수 프로토타입 추가!
float PID_Compute_Yaw(PID_Controller_t *pid, float current_yaw);

// 모터 제어 함수 프로토타입
void M1_Set_Direction(uint8_t direction);
void M1_Set_Speed(uint16_t speed);
void M2_Set_Direction(uint8_t direction);
void M2_Set_Speed(uint16_t speed);
void M3_Set_Direction(uint8_t direction);
void M3_Set_Speed(uint16_t speed);

// 오도메트리 함수 프로토타입 추가!
void Forward_Kinematics(float v1, float v2, float v3,
                        float *vx, float *vy, float *omega);
void Update_Odometry(void);
void Send_Odometry_Data(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float yaw_correction = 0.0f;  // Yaw 보정값 추가!


int64_t Get_Safe_Encoder_Count(TIM_HandleTypeDef *htim, volatile int64_t *accum_count) {
    int64_t total;

    __disable_irq(); // 1. 인터럽트 비활성화 (멈춰!)

    // 2. 후다닥 계산 (누적값 + 현재 타이머값)
    total = *accum_count + (int64_t)__HAL_TIM_GET_COUNTER(htim);

    __enable_irq();  // 3. 인터럽트 재활성화 (계속해)

    return total;
}


// ============================================================================
// [Inverse Kinematics] 로봇 목표 속도(vx, vy, omega) -> 모터 RPM 변환
// ============================================================================
void Set_Velocity_From_CmdVel(float target_vx, float target_vy, float target_omega)
{
    // 1. 로봇의 기하학적 정보 (m 단위)
    // WHEEL_RADIUS (0.04m), ROBOT_RADIUS (0.24m) - 기존 define 사용

    // 2. 각 모터의 선속도(m/s) 계산
    // 공식: v_i = vx * cos(theta) + vy * sin(theta) + R * omega
    // (주의: 기존 코드의 좌표계 정의에 따라 부호가 달라질 수 있습니다. 표준 공식 기준입니다.)

    // Motor 1 (150도)
    float v1 = target_vx * cosf(M1_ANGLE) + target_vy * sinf(M1_ANGLE) + ROBOT_RADIUS * target_omega;

    // Motor 2 (30도)
    // 기존 코드에서 M2는 물리적으로 반대 방향이거나 배선이 반대일 수 있으므로
    // Update_Odometry에서 보정했던 것처럼 부호를 신경 써야 합니다.
    // 일단 표준 공식으로 계산합니다.
    float v2 = target_vx * cosf(M2_ANGLE) + target_vy * sinf(M2_ANGLE) + ROBOT_RADIUS * target_omega;

    // Motor 3 (270도)
    float v3 = target_vx * cosf(M3_ANGLE) + target_vy * sinf(M3_ANGLE) + ROBOT_RADIUS * target_omega;

    // 3. m/s -> RPM 변환
    // RPM = (v * 60) / (2 * PI * r)
    float rpm1 = (v1 * 60.0f) / (2.0f * M_PI * WHEEL_RADIUS);
    float rpm2 = (v2 * 60.0f) / (2.0f * M_PI * WHEEL_RADIUS);
    float rpm3 = (v3 * 60.0f) / (2.0f * M_PI * WHEEL_RADIUS);
    rpm1 = -rpm1;
    rpm2 = -rpm2;
    rpm3 = -rpm3;
    // 4. PID 목표값 설정 (PID가 활성화되어 있어야 함)
    // M2의 경우 오도메트리에서 부호를 뒤집었으므로, 여기서도 방향을 맞춰야 할 수 있습니다.
    // 테스트 후 반대로 돈다면 부호를 반전시키세요 (-rpm2).
    Set_Target_RPM(rpm1, rpm2, rpm3);

    // 5. PID 및 전진 모드 플래그 설정
    pid_enabled = 1; // 강제 PID 활성화

    // Yaw 제어(직진 보정)는 회전 명령(omega)이 없을 때만 적용
    if (fabs(target_omega) < 0.01f && (fabs(target_vx) > 0 || fabs(target_vy) > 0)) {
        if (!is_forward_mode) { // 새로 직진 시작할 때만 갱신
            target_yaw = imu.Yaw;
            pid_yaw.target_rpm = target_yaw;
            is_forward_mode = 1;
        }
    } else {
        is_forward_mode = 0; // 회전 중이거나 정지 시 Yaw 보정 끄기
    }
}

// ============================================================================
// [Yaw PID 계산 함수] - 각도 제어용 (음수 출력 허용)
// ============================================================================
/**
 * @brief Yaw 각도 제어용 PID 계산
 * @param pid: PID 제어기 포인터
 * @param current_yaw: 현재 Yaw 각도 (deg)
 * @return PID 출력 (-200 ~ +200, 회전 보정값)
 */
float PID_Compute_Yaw(PID_Controller_t *pid, float current_yaw)
{
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid->last_time) / 1000.0f;  // ms → s

    // dt가 너무 작으면 계산 생략
    if (dt < 0.01f) return pid->output;  // 10ms 미만 무시

    // ✅ Yaw는 절대값 안 쓰고 그대로 사용!
    // 오차 계산 (각도 차이)
    pid->error = pid->target_rpm - current_yaw;  // target_rpm을 target_yaw처럼 사용

    // ✅ 각도 오차 정규화 (-180 ~ +180)
    // 예: 오차가 +200도 → -160도로 변환 (짧은 경로)
    if (pid->error > 180.0f) pid->error -= 360.0f;
    if (pid->error < -180.0f) pid->error += 360.0f;

    // P항: 비례
    float P_term = pid->Kp * pid->error;

    // I항: 적분
    pid->error_sum += pid->error * dt;

    // 안티 와인드업 (적분 제한)
    float max_integral = 50.0f;  // Yaw는 적분 제한을 작게
    if (pid->error_sum > max_integral) pid->error_sum = max_integral;
    if (pid->error_sum < -max_integral) pid->error_sum = -max_integral;

    float I_term = pid->Ki * pid->error_sum;

    // D항: 미분
    float error_delta = pid->error - pid->error_prev;
    float D_term = pid->Kd * (error_delta / dt);

    // PID 출력 계산
    pid->output = P_term + I_term + D_term;

    // ✅ 출력 제한 (-200 ~ +200, 음수 허용!)
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < pid->output_min) pid->output = pid->output_min;

    // 다음 계산을 위해 저장
    pid->error_prev = pid->error;
    pid->last_time = current_time;

    return pid->output;
}

// ⭐ [오도메트리] Forward Kinematics
// ============================================================================
/**
 * @brief 옴니휠 Forward Kinematics (3륜, 120도 배치)
 * @param v1, v2, v3: 각 모터의 선속도 (m/s)
 * @param vx, vy: 로봇 좌표계 속도 출력 (m/s)
 * @param omega: 회전 속도 출력 (rad/s)
 *
 * 운동학 행렬:
 * [Vx]   [sin(150°)  sin(30°)   sin(270°)]   [v1]
 * [Vy] = [cos(150°)  cos(30°)   cos(270°)] × [v2]
 * [ω ]   [1/R        1/R        1/R      ]   [v3]
 */
void Forward_Kinematics(float v1, float v2, float v3,
                        float *vx, float *vy, float *omega)
{
    // Vx 계산
    // sin(150°) = 0.5, sin(30°) = 0.5, sin(270°) = -1.0
    *vx = (1.0f/3.0f) * (
        cosf(M1_ANGLE) * v1 +    // 0.5 × v1
		cosf(M2_ANGLE) * v2 +    // 0.5 × v2
		cosf(M3_ANGLE) * v3      // -1.0 × v3
    );

    // Vy 계산
    // cos(150°) = -0.866, cos(30°) = 0.866, cos(270°) = 0.0
    *vy = -1.0f * (1.0f/3.0f) * (
        sinf(M1_ANGLE) * v1 +    // -0.866 × v1
		sinf(M2_ANGLE) * v2 +    // 0.866 × v2
		sinf(M3_ANGLE) * v3      // 0.0 × v3
    );

    // ω (각속도) 계산
    // ω = (v1 + v2 + v3) / (3 × R)
    *omega = (1.0f/(3.0f * ROBOT_RADIUS)) * (v1 + v2 + v3);
}

// ============================================================================
// [오도메트리] UART 전송
// ============================================================================
/**
 * @brief 오도메트리 데이터를 UART로 전송
 *
 * 프로토콜:
 * "Odom - x:+0.123 | y:-0.456 | th:+1.234 | vx:+0.100 | vy:-0.050 | w:+0.020 | t:12345\r\n"
 */
//void Send_Odometry_Data(void)
//{
//    // 부동소수점 → 정수 변환 (소수점 3자리)
//    int32_t x_int = (int32_t)(odom.x * 1000.0f);
//    int32_t y_int = (int32_t)(odom.y * 1000.0f);
//    int32_t theta_int = (int32_t)(odom.theta * 1000.0f);
//    int32_t vx_int = (int32_t)(odom.vx * 1000.0f);
//    int32_t vy_int = (int32_t)(odom.vy * 1000.0f);
//    int32_t omega_int = (int32_t)(odom.omega * 1000.0f);
//
//    sprintf(uart_buffer,
//            "Odom - x:%+d.%03d | y:%+d.%03d | th:%+d.%03d | vx:%+d.%03d | vy:%+d.%03d | w:%+d.%03d | t:%lu\r\n",
//            x_int/1000, abs(x_int%1000),
//            y_int/1000, abs(y_int%1000),
//            theta_int/1000, abs(theta_int%1000),
//            vx_int/1000, abs(vx_int%1000),
//            vy_int/1000, abs(vy_int%1000),
//            omega_int/1000, abs(omega_int%1000),
//            HAL_GetTick());
//
//    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
//}
void Send_Odometry_Data(void)
{
    // [수정] 복잡한 정수 변환 없이 float을 바로 출력
    // %+.3f : 부호(+)를 항상 표시하고, 소수점 3자리(.3)까지 실수(f)로 출력

    sprintf(uart_buffer,
            "Odom - x:%+.3f | y:%+.3f | th:%+.3f | vx:%+.3f | vy:%+.3f | w:%+.3f | t:%lu\r\n",
            odom.x,
            odom.y,
            odom.theta,
            odom.vx,
            odom.vy,
            odom.omega,
            HAL_GetTick());

    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}
// ============================================================================
// [인터럽트 콜백] 엔코더 오버플로우/언더플로우
// ============================================================================
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//
//    // 모터 1 엔코더 (TIM5, 32비트)
//    if (htim->Instance == TIM5)
//    {
//        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
//            Encoder1_Count -= COUNTER_MAX_32BIT;
//        else
//            Encoder1_Count += COUNTER_MAX_32BIT;
//    }
//    // 모터 2 엔코더 (TIM2, 32비트)
//    else if (htim->Instance == TIM2)
//    {
//        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
//            Encoder2_Count -= COUNTER_MAX_32BIT;
//        else
//            Encoder2_Count += COUNTER_MAX_32BIT;
//    }
//    // 모터 3 엔코더 (TIM8, 16비트)
//    else if (htim->Instance == TIM8)
//    {
//        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
//            Encoder3_Count -= COUNTER_MAX_16BIT;
//        else
//            Encoder3_Count += COUNTER_MAX_16BIT;
//    }
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 모터 1 (32비트 타이머 TIM5)
    if (htim->Instance == TIM5)
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
            Encoder1_Count -= 4294967296LL; // 32비트 크기만큼 뺌
        else
            Encoder1_Count += 4294967296LL; // 32비트 크기만큼 더함
    }
    // 모터 3 (16비트 타이머 TIM8)
    else if (htim->Instance == TIM8)
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
            Encoder3_Count -= 65536LL;      // ⭐ 16비트 크기만큼 뺌
        else
            Encoder3_Count += 65536LL;      // ⭐ 16비트 크기만큼 더함
    }
}

// ============================================================================
// [MPU9250 초기화]
// ============================================================================
uint8_t MPU9250_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check;
    uint8_t Data;

    // ============================================================
    // [1] MPU9250 초기화
    // ============================================================

    // WHO_AM_I 확인 (MPU9250)
    HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check != 0x71) {
        return 1;  // MPU9250 감지 실패
    }

    // Reset
    Data = 0x80;
    HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
    HAL_Delay(100);

    // Clock Source 설정
    Data = 0x01;
    HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

    // 샘플링 속도 설정 (125Hz)
    Data = 0x07;
    HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

    // 가속도 설정 (±2g)
    Data = 0x00;
    HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

    // 자이로 설정 (±250 deg/s)
    Data = 0x00;
    HAL_I2C_Mem_Write(I2Cx, MPU9250_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);

    return 0;  // 성공
}

// ============================================================================
// [MPU9250 센서 읽기]
// ============================================================================
void MPU9250_Read_Sensors(I2C_HandleTypeDef *I2Cx, IMU_t *DataStruct) {
    uint8_t Rec_Data[6];
    int16_t raw_ax, raw_ay, raw_gz;

    // 1. 가속도 읽기 (생략 - 기존 코드 유지)
    HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
    raw_ax = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    raw_ay = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);

    DataStruct->Ax = raw_ax / ACCEL_SENSITIVITY;
    DataStruct->Ay = raw_ay / ACCEL_SENSITIVITY;

    // 2. 자이로 읽기
    HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
    raw_gz = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // 3. 자이로 값 변환 및 바이어스 빼기
    DataStruct->Gz = raw_gz / GYRO_SENSITIVITY;
    DataStruct->Gz -= gyro_z_bias;

    // ============================================================
    // ⭐ [추가] 데드존(Deadzone) 적용
    // 회전 속도가 너무 작으면(예: 0.5도/초 미만) 노이즈로 간주하고 0으로 만듦
    // ============================================================
    if (fabs(DataStruct->Gz) < 1.8f) { // 임계값 튜닝 가능 (0.1 ~ 1.0)
        DataStruct->Gz = 0.0f;
    }

    // 4. 자이로 적분으로 Yaw 계산
    uint32_t current_time = HAL_GetTick();

    if (last_imu_update_time > 0) {
        float dt = (current_time - last_imu_update_time) / 1000.0f;

        // 자이로 적분 (각속도 * 시간 = 각도)
        DataStruct->Yaw += DataStruct->Gz * dt;

        // 범위 제한 (-180 ~ +180)
        if (DataStruct->Yaw > 180.0f) {
            DataStruct->Yaw -= 360.0f;
        } else if (DataStruct->Yaw < -180.0f) {
            DataStruct->Yaw += 360.0f;
        }
    }

    last_imu_update_time = current_time;
}

void Calibrate_Gyro(I2C_HandleTypeDef *I2Cx) {
    float sum = 0.0f;
    IMU_t temp_imu;
    int sample_count = 200; // 200번 측정

    sprintf(uart_buffer, "Calibrating Gyro... Don't Move!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

    for(int i=0; i<sample_count; i++) {
        MPU9250_Read_Sensors(I2Cx, &temp_imu);
        sum += temp_imu.Gz; // 오차를 누적
        HAL_Delay(5);
    }

    gyro_z_bias = sum / sample_count; // 평균 오차 계산

    sprintf(uart_buffer, "Calibration Done! Bias: %.3f\r\n", gyro_z_bias);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}

//==============================================================================
//[앤코더 코드 시작
//==============================================================================


//// ============================================================================
//// [엔코더 값 UART 전송 함수]
//// ============================================================================
//void Send_Encoder_Data(void) {
//    // 현재 누적 엔코더 값 계산
//    int64_t M1_Total = Encoder1_Count + (int64_t)__HAL_TIM_GET_COUNTER(&htim5);
//    int64_t M2_Total = Encoder2_Count + (int64_t)__HAL_TIM_GET_COUNTER(&htim2);
//    int64_t M3_Total = Encoder3_Count + (int64_t)__HAL_TIM_GET_COUNTER(&htim8);
//
//    //  올바른 프로토콜: "Encoders - M1:%ld | M2:%ld | M3:%ld\r\n"
//    sprintf(uart_buffer, "Encoders - M1:%.0f | M2:%.0f | M3:%.0f\r\n",
//                (float)M1_Total, (float)M2_Total, (float)M3_Total);
//    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
//}
void Send_Encoder_Data(void) {
    // 1. 안전하게 값 읽기 (Get_Safe_Encoder_Count 사용 유지)
    int64_t M1_Total = Get_Safe_Encoder_Count(&htim5, &Encoder1_Count);
    int64_t M2_Total = Get_Safe_Encoder_Count(&htim2, &Encoder2_Count);
    int64_t M3_Total = Get_Safe_Encoder_Count(&htim8, &Encoder3_Count);

    // 2. [수정] 출력할 때 (int32_t)로 강제 변환 후 실수로 변경
    // 이렇게 하면 42억이라는 숫자가 -4000 같은 음수로 예쁘게 바뀝니다.
    // (M3는 이미 잘 나오고 있지만 통일성을 위해 같이 해줘도 됩니다.)

    sprintf(uart_buffer, "Encoders - M1:%.0f | M2:%.0f | M3:%.0f\r\n",
            (float)(int32_t)M1_Total,
            (float)(int32_t)M2_Total,
            (float)M3_Total); // M3는 64비트 로직이 잘 돌고 있으니 그대로 둬도 됨

    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}
// ============================================================================
// [IMU 데이터 전송]
// ============================================================================
void Send_IMU_Data(void) {
    // 소수점 2자리로 변환 (부동소수점 연산 최소화)
    int yaw_int = (int)(imu.Yaw * 100);
    int gz_int = (int)(imu.Gz * 100);
    int ax_int = (int)(imu.Ax * 100);
    int ay_int = (int)(imu.Ay * 100);


    sprintf(uart_buffer, "IMU - Yaw:%+d.%02d | Gz:%+d.%02d | Ax:%+d.%02d | Ay:%+d.%02d\r\n",
            yaw_int/100, abs(yaw_int%100),
            gz_int/100, abs(gz_int%100),
            ax_int/100, abs(ax_int%100),
            ay_int/100, abs(ay_int%100));
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}

// ============================================================================
// [속도 계산 함수]
// ============================================================================
/**
 * @brief 엔코더 카운트 → RPM 계산 + 로우패스 필터
 * @param motor: 모터 속도 구조체 포인터
 * @param current_count: 현재 누적 엔코더 카운트
 */
void Calculate_Motor_Speed(Motor_Speed_t *motor, int32_t current_count)
{
    uint32_t current_time = HAL_GetTick();

    // 첫 실행 시 초기화
    if (motor->last_time == 0) {
        motor->prev_count = current_count;
        motor->last_time = current_time;
        motor->rpm = 0.0f;
        motor->rpm_filtered = 0.0f;
        return;
    }

    // 시간 간격 (ms → s)
    float dt = (current_time - motor->last_time) / 1000.0f;

    // dt가 너무 작으면 계산 생략 (노이즈 방지)
    if (dt < 0.01f) return;  // 10ms 미만 무시

    // 엔코더 변화량 (펄스)
    int32_t delta_count = current_count - motor->prev_count;

    // RPM 계산
    // 1회전 = ENCODER_PPR * 4 펄스 (4배 체배)
    // RPM = (delta_count / (PPR * 4)) * (60 / dt)
    motor->rpm = (delta_count / (ENCODER_PPR * 4.0f)) * (60.0f / dt);

    // 로우패스 필터 적용
    // filtered = alpha * new + (1 - alpha) * prev_filtered
    motor->rpm_filtered = LPF_ALPHA * motor->rpm + (1.0f - LPF_ALPHA) * motor->rpm_filtered;

    // 다음 계산을 위해 저장
    motor->prev_count = current_count;
    motor->last_time = current_time;
}

// ============================================================================
// [PID 초기화 함수]
// ============================================================================
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->target_rpm = 0.0f;
    pid->error = 0.0f;
    pid->error_sum = 0.0f;
    pid->error_prev = 0.0f;
    pid->output = 0.0f;

    pid->output_min = 0.0f;      // PWM 최소값
    pid->output_max = 1000.0f;   // PWM 최대값

    pid->last_time = HAL_GetTick();
}

// ============================================================================
// [PID 계산 함수]
// ============================================================================
float PID_Compute(PID_Controller_t *pid, float current_rpm)
{
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid->last_time) / 1000.0f;  // ms → s

    // dt가 너무 작으면 계산 생략
    if (dt < 0.01f) return pid->output;  // 10ms 미만 무시

    // 절대값 기반 오차 계산
    float target_abs = fabs(pid->target_rpm);   // 지역 변수!
    float current_abs = fabs(current_rpm);

    // 오차 계산 (절대값 기준)
    pid->error = target_abs - current_abs;  // ✅ 지역 변수 사용!

    // P 항: 비례
    float P_term = pid->Kp * pid->error;

    // I 항: 적분
    pid->error_sum += pid->error * dt;

    // 안티 와인드업 (적분 포화 방지)
    float max_integral = 500.0f;  // 적분 제한
    if (pid->error_sum > max_integral) pid->error_sum = max_integral;
    if (pid->error_sum < -max_integral) pid->error_sum = -max_integral;

    float I_term = pid->Ki * pid->error_sum;

    // D 항: 미분
    float error_delta = pid->error - pid->error_prev;
    float D_term = pid->Kd * (error_delta / dt);

    // PID 출력 계산
    pid->output = P_term + I_term + D_term;

    // 출력 제한 (PWM 범위: 0 ~ 1000)
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    if (pid->output < pid->output_min) pid->output = pid->output_min;

    // 다음 계산을 위해 저장
    pid->error_prev = pid->error;
    pid->last_time = current_time;

    return pid->output;
}

// ============================================================================
// [PID 리셋 함수]
// ============================================================================
void PID_Reset(PID_Controller_t *pid)
{
    pid->error = 0.0f;
    pid->error_sum = 0.0f;
    pid->error_prev = 0.0f;
    pid->output = 0.0f;
    pid->last_time = HAL_GetTick();
}

/**
 * @brief 모든 모터 속도 계산
 */
void Update_All_Motor_Speeds(void)
{
    // [수정] 직접 더하지 않고 안전한 함수(Get_Safe_Encoder_Count)를 통해 읽어옵니다.
    // 이렇게 하면 읽는 도중 인터럽트가 터져서 값이 2배로 뛰는 현상이 사라집니다.

    int64_t M1_Total = Get_Safe_Encoder_Count(&htim5, &Encoder1_Count);
    int64_t M2_Total = Get_Safe_Encoder_Count(&htim2, &Encoder2_Count);
    int64_t M3_Total = Get_Safe_Encoder_Count(&htim8, &Encoder3_Count);

    // Calculate_Motor_Speed 함수는 (int32_t) 입력을 받으므로 캐스팅 유지
    // (속도 계산은 변화량(Delta)만 보므로 32비트로 잘라도 문제없음)
    Calculate_Motor_Speed(&motor1_speed, (int32_t)M1_Total);
    Calculate_Motor_Speed(&motor2_speed, (int32_t)M2_Total);
    Calculate_Motor_Speed(&motor3_speed, (int32_t)M3_Total);
}

/**
 * @brief 속도 데이터 UART 전송
 */
void Send_Speed_Data(void)
{
    // 소수점 2자리 정수 변환
    int rpm1_int = (int)(motor1_speed.rpm_filtered * 100);
    int rpm2_int = (int)(motor2_speed.rpm_filtered * 100);
    int rpm3_int = (int)(motor3_speed.rpm_filtered * 100);

    sprintf(uart_buffer, "Speed - M1:%+d.%02d | M2:%+d.%02d | M3:%+d.%02d\r\n",
            rpm1_int/100, abs(rpm1_int%100),
            rpm2_int/100, abs(rpm2_int%100),
            rpm3_int/100, abs(rpm3_int%100));
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}

// ============================================================================
//[모터 제어 함수 수정] - PID 출력 적용
// ============================================================================
void Apply_PID_Output(void)
{
    if (!pid_enabled) return;  // PID 비활성화 시 무시

    // ✅ 1단계: 기본 속도 PID 계산
    float pwm1 = PID_Compute(&pid_motor1, motor1_speed.rpm_filtered);
    float pwm2 = PID_Compute(&pid_motor2, motor2_speed.rpm_filtered);
    float pwm3 = PID_Compute(&pid_motor3, motor3_speed.rpm_filtered);

    // ✅ 2단계: Yaw 보정 (전진 중일 때만!)
    if (yaw_control_enabled && is_forward_mode) {
        yaw_correction = PID_Compute_Yaw(&pid_yaw, imu.Yaw);

        // 속도 기반 보정 강도 조절
        float m1_abs = fabs(pid_motor1.target_rpm);
        float m2_abs = fabs(pid_motor2.target_rpm);
        float max_speed = fmax(m1_abs, m2_abs);

        // 10 RPM 이상일 때만 보정
        if (max_speed > 10.0f) {
            // 속도에 비례하여 보정 (30 RPM에서 최대)
            float speed_factor = fmin(max_speed / 30.0f, 1.0f);
            yaw_correction *= speed_factor * 2.0f;  // 2배 강도

            // 부호 계산
            float sign1 = (pid_motor1.target_rpm >= 0) ? 1.0f : -1.0f;
            float sign2 = (pid_motor2.target_rpm >= 0) ? 1.0f : -1.0f;

            // 보정 적용 (전진 시: M1, M2만 사용)
            pwm1 = pwm1 - yaw_correction;
            pwm2 = pwm2 + yaw_correction;
        }
    }

    // 3단계: PWM 제한 (0 ~ 1000)
    if (pwm1 > 1000.0f) pwm1 = 1000.0f;
    if (pwm1 < 0.0f) pwm1 = 0.0f;

    if (pwm2 > 1000.0f) pwm2 = 1000.0f;
    if (pwm2 < 0.0f) pwm2 = 0.0f;

    if (pwm3 > 1000.0f) pwm3 = 1000.0f;
    if (pwm3 < 0.0f) pwm3 = 0.0f;

    // 4단계: 모터 방향 및 속도 설정
    if (pid_motor1.target_rpm >= 0) {
        M1_Set_Direction(1);
    } else {
        M1_Set_Direction(0);
    }
    M1_Set_Speed((uint16_t)pwm1);

    if (pid_motor2.target_rpm >= 0) {
        M2_Set_Direction(1);
    } else {
        M2_Set_Direction(0);
    }
    M2_Set_Speed((uint16_t)pwm2);

    if (pid_motor3.target_rpm >= 0) {
        M3_Set_Direction(1);
    } else {
        M3_Set_Direction(0);
    }
    M3_Set_Speed((uint16_t)pwm3);
}

// ============================================================================
//[PID 목표 속도 설정 함수]
// ============================================================================
void Set_Target_RPM(float rpm1, float rpm2, float rpm3)
{
    pid_motor1.target_rpm = rpm1;
    pid_motor2.target_rpm = rpm2;
    pid_motor3.target_rpm = rpm3;
}

// ============================================================================
// ⭐ [오도메트리] 업데이트 (Runge-Kutta 2차) - 완벽한 부호 보정
// ============================================================================
/**
 * @brief 오도메트리 계산 및 업데이트
 *
 * 핵심 수정사항:
 * - 모터2의 회전 방향이 다른 모터들과 반대
 * - 모터2의 각속도만 부호 반전 적용
 *
 * 동작별 검증:
 * Q(좌상): M2(-) M3(+) → M2 반전 → M2(+) M3(+) → 좌상 이동 ✅
 * W(전진): M1(+) M2(-) → M2 반전 → M1(+) M2(+) → 전진 ✅
 * E(우상): M1(+) M3(-) → M1(+) M3(-) → 우상 이동 ✅
 * A(좌하): M1(-) M3(+) → M1(-) M3(+) → 좌하 이동 ✅
 * S(후진): M1(-) M2(+) → M2 반전 → M1(-) M2(-) → 후진 ✅
 * D(우하): M2(+) M3(-) → M2 반전 → M2(-) M3(-) → 우하 이동 ✅
 * R(반시계): M1(-) M2(-) M3(-) → M2 반전 → M1(-) M2(+) M3(-) → 반시계 ✅
 * T(시계): M1(+) M2(+) M3(+) → M2 반전 → M1(+) M2(-) M3(+) → 시계 ✅
 */
void Update_Odometry(void)
{
    uint32_t current_time = HAL_GetTick();

    // 첫 실행 시 초기화
    if (odom.last_time == 0) {
        odom.last_time = current_time;
        return;
    }

    // dt 계산 (ms → s)
    float dt = (current_time - odom.last_time) / 1000.0f;

    // dt가 너무 작으면 무시 (노이즈 방지)
    if (dt < 0.001f) return;  // 1ms 미만 무시

    // ========================================
    // 1단계: RPM → 각속도 (rad/s)
    // ========================================

    // RPM → rad/s 변환
    // 엔코더 값을 그대로 사용 (부호 포함)
    float omega1 = motor1_speed.rpm_filtered * 2.0f * M_PI / 60.0f;  // rad/s
    float omega2 = motor2_speed.rpm_filtered * 2.0f * M_PI / 60.0f;
    float omega3 = motor3_speed.rpm_filtered * 2.0f * M_PI / 60.0f;

    // ========================================
    // ⭐ 2단계: 모터2 부호 보정
    // ========================================

    // 물리적 배치 차이로 인해 모터2만 회전 방향이 반대
    //
    // 예시 1 - 전진(W):
    //   엔코더: M1=+50, M2=-50, M3=0
    //   보정 후: M1=+50, M2=+50, M3=0 ✅ 둘 다 앞으로 구름
    //
    // 예시 2 - 후진(S):
    //   엔코더: M1=-50, M2=+50, M3=0
    //   보정 후: M1=-50, M2=-50, M3=0 ✅ 둘 다 뒤로 구름
    //
    // 예시 3 - 시계방향 회전(T):
    //   엔코더: M1=+50, M2=+50, M3=+50
    //   보정 후: M1=+50, M2=-50, M3=+50 ✅ 시계방향 회전

    float omega1_corrected = omega1;        // 모터1: 그대로
    float omega2_corrected = omega2;       // ⭐ 모터2: 부호 반전!
    float omega3_corrected = omega3;        // 모터3: 그대로

    // 각속도 → 선속도 (m/s)
    // v = ω × r
    float v1 = -omega1_corrected * WHEEL_RADIUS;  // m/s
    float v2 = -omega2_corrected * WHEEL_RADIUS;
    float v3 = -omega3_corrected * WHEEL_RADIUS;

    // ========================================
    // 3단계: Forward Kinematics (로봇 좌표계)
    // ========================================

    float vx_robot, vy_robot, omega_robot;
    Forward_Kinematics(v1, v2, v3, &vx_robot, &vy_robot, &omega_robot);

    // ========================================
    // 4단계: Runge-Kutta 2차 (중간점 방법)
    // ========================================

    // 중간 시점의 각도 계산
    // θ_mid = θ + (ω × dt / 2)
    float theta_mid = odom.theta + (omega_robot * dt / 2.0f);

    // 중간 시점의 회전 행렬로 글로벌 좌표 변환
    // [Vx_global]   [cos(θ_mid)  -sin(θ_mid)] [Vx_robot]
    // [Vy_global] = [sin(θ_mid)   cos(θ_mid)] [Vy_robot]
    float vx_global = vx_robot * cosf(theta_mid) - vy_robot * sinf(theta_mid);
    float vy_global = vx_robot * sinf(theta_mid) + vy_robot * cosf(theta_mid);

    // ========================================
    // 5단계: 위치 적분
    // ========================================

    odom.x += vx_global * dt * ODOM_CORRECTION;
    odom.y += vy_global * dt * ODOM_CORRECTION;
    odom.theta += omega_robot * dt;

    // ========================================
    // 6단계: 각도 정규화 (-π ~ +π)
    // ========================================

    while (odom.theta > M_PI) odom.theta -= 2.0f * M_PI;
    while (odom.theta < -M_PI) odom.theta += 2.0f * M_PI;

    // ========================================
    // 7단계: 속도 저장 (글로벌 좌표계)
    // ========================================

    odom.vx = vx_global;
    odom.vy = vy_global;
    odom.omega = omega_robot;

    odom.last_time = current_time;
}

// ============================================================================
// [UART 수신 인터럽트 콜백]
// ============================================================================
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART2)
//    {
//        Process_Command(rx_data);
//        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
//    }
//}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // 1. 줄바꿈(\n) 문자를 만날 때까지 버퍼에 저장
        if (rx_data != '\n' && rx_index < RX_BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = rx_data;
        }
        else {
            // 2. 줄바꿈을 만나면 문자열 종료 처리 및 파싱
            rx_buffer[rx_index] = '\0';
            rx_index = 0; // 인덱스 초기화

            // 3. 프로토콜 파싱
            // 포맷: "V,vx,vy,omega" (예: "V,0.25,0.0,-0.5")
            if (rx_buffer[0] == 'V') {
                float vx, vy, omega;
                // 콤마(,)로 구분하여 실수형 데이터 추출
                char *token = strtok((char*)rx_buffer, ","); // 'V'
                token = strtok(NULL, ","); vx = strtof(token, NULL);
                token = strtok(NULL, ","); vy = strtof(token, NULL);
                token = strtok(NULL, ","); omega = strtof(token, NULL);

                // 역기구학 함수 호출
                Set_Velocity_From_CmdVel(vx, vy, omega);
            }
            // 기존 키보드 제어 (길이가 1인 경우)
            else if (strlen((char*)rx_buffer) == 1) {
                Process_Command(rx_buffer[0]);
            }
        }

        // 다시 인터럽트 활성화
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

// ============================================================================
// [1] 모터1 제어 함수
// ============================================================================
/**
 * @brief 모터1의 방향을 설정합니다.
 * @param direction: 0 (정회전), 1 (역회전)
 */
void M1_Set_Direction(uint8_t direction) {
    if (direction == 1) {
        HAL_GPIO_WritePin(M1_DIR_PORT, M1_DIR_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(M1_DIR_PORT, M1_DIR_PIN, GPIO_PIN_SET);
    }
}

/**
 * @brief 모터1의 속도(PWM)를 설정합니다.
 * @param speed: 0 ~ 1000
 */
void M1_Set_Speed(uint16_t speed) {
    if (speed > 1000) speed = 1000;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
}

// ============================================================================
// [2] 모터2 제어 함수
// ============================================================================
/**
 * @brief 모터2의 방향을 설정합니다.
 * @param direction: 0 (정회전), 1 (역회전)
 */
void M2_Set_Direction(uint8_t direction) {
    if (direction == 1) {
        HAL_GPIO_WritePin(M2_DIR_PORT, M2_DIR_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(M2_DIR_PORT, M2_DIR_PIN, GPIO_PIN_SET);
    }
}

/**
 * @brief 모터2의 속도(PWM)를 설정합니다.
 * @param speed: 0 ~ 1000
 */
void M2_Set_Speed(uint16_t speed) {
    if (speed > 1000) speed = 1000;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}

// ============================================================================
// [3] 모터3 제어 함수
// ============================================================================
/**
 * @brief 모터3의 방향을 설정합니다.
 * @param direction: 0 (정회전), 1 (역회전)
 */
void M3_Set_Direction(uint8_t direction) {
    if (direction == 1) {
        HAL_GPIO_WritePin(M3_DIR_PORT, M3_DIR_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(M3_DIR_PORT, M3_DIR_PIN, GPIO_PIN_SET);
    }
}

/**
 * @brief 모터3의 속도(PWM)를 설정합니다.
 * @param speed: 0 ~ 1000
 */
void M3_Set_Speed(uint16_t speed) {
    if (speed > 1000) speed = 1000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}

// ============================================================================
// 옴니휠 이동 함수 (120도 배치: 10시-M1, 2시-M2, 6시-M3)
// ============================================================================

// 모든 모터 정지
void Stop_All(void) {
    M1_Set_Speed(0);
    M2_Set_Speed(0);
    M3_Set_Speed(0);
}

// 대각선 왼쪽 위 (Q) - 모터2, 3
void Move_DiagonalLeftUp(uint16_t speed) {
    M1_Set_Speed(0);
    M2_Set_Direction(0);
    M2_Set_Speed(speed);
    M3_Set_Direction(1);
    M3_Set_Speed(speed);
}

// 대각선 오른쪽 위 (E) - 모터1, 3
void Move_DiagonalRightUp(uint16_t speed) {
    M1_Set_Direction(1);
    M1_Set_Speed(speed);
    M2_Set_Speed(0);
    M3_Set_Direction(0);
    M3_Set_Speed(speed);
}

// 대각선 왼쪽 아래 (A) - 모터1, 3
void Move_DiagonalLeftDown(uint16_t speed) {
    M1_Set_Direction(0);
    M1_Set_Speed(speed);
    M2_Set_Speed(0);
    M3_Set_Direction(1);
    M3_Set_Speed(speed);
}

// 대각선 오른쪽 아래 (D) - 모터2, 3
void Move_DiagonalRightDown(uint16_t speed) {
    M1_Set_Speed(0);
    M2_Set_Direction(1);
    M2_Set_Speed(speed);
    M3_Set_Direction(0);
    M3_Set_Speed(speed);
}

// 전진 (W) - 모터1, 2
void Move_Forward(uint16_t speed) {
    M1_Set_Direction(1);
    M1_Set_Speed(speed);
    M2_Set_Direction(0);
    M2_Set_Speed(speed);
    M3_Set_Speed(0);
}

// 후진 (S) - 모터1, 2
void Move_Backward(uint16_t speed) {
    M1_Set_Direction(0);
    M1_Set_Speed(speed);
    M2_Set_Direction(1);
    M2_Set_Speed(speed);
    M3_Set_Speed(0);
}

// 제자리 시계방향 회전 (T) - 모든 모터 같은 방향
void Rotate_Clockwise(uint16_t speed) {
    M1_Set_Direction(1);
    M1_Set_Speed(speed);
    M2_Set_Direction(1);
    M2_Set_Speed(speed);
    M3_Set_Direction(1);
    M3_Set_Speed(speed);
}

// 제자리 반시계방향 회전 (R) - 모든 모터 반대 방향
void Rotate_CounterClockwise(uint16_t speed) {
    M1_Set_Direction(0);
    M1_Set_Speed(speed);
    M2_Set_Direction(0);
    M2_Set_Speed(speed);
    M3_Set_Direction(0);
    M3_Set_Speed(speed);
}

// 속도 증가 (F)
void Increase_Speed(void) {
    if (current_speed < SPEED_MAX) {
        current_speed += SPEED_STEP;
        if (current_speed > SPEED_MAX) current_speed = SPEED_MAX;
    }
    sprintf(uart_buffer, "Speed: %d\r\n", current_speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}

// 속도 감소 (G)
void Decrease_Speed(void) {
    if (current_speed > SPEED_MIN + SPEED_STEP) {
        current_speed -= SPEED_STEP;
    } else {
        current_speed = SPEED_MIN;
    }
    sprintf(uart_buffer, "Speed: %d\r\n", current_speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}

// ============================================================================
// 키보드 명령 처리
// ============================================================================
void Process_Command(uint8_t cmd) {
    switch(cmd) {
        case 'q':
        case 'Q':
            sprintf(uart_buffer, "Diagonal Left Up\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
            // PID 리셋 추가!
                        if (pid_enabled) {
                            RESET_ALL_PIDS();
                        }

                        if (!pid_enabled) {
                            Move_DiagonalLeftUp(current_speed);
                            is_forward_mode = 0;
                        } else {
                            Set_Target_RPM(0.0f, -target_speed, target_speed);
                            if (yaw_control_enabled) {
                               target_yaw = imu.Yaw;
                               pid_yaw.target_rpm = target_yaw;
                               is_forward_mode = 1;
                            } else {
                               is_forward_mode = 0;
                            }
            }
            break;

        case 'e':
        case 'E':
            sprintf(uart_buffer, "Diagonal Right Up\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
            // ✅ PID 리셋 추가!
                        if (pid_enabled) {
                            RESET_ALL_PIDS();
                        }

                        if (!pid_enabled) {
                            Move_DiagonalRightUp(current_speed);
                            is_forward_mode = 0;
                        } else {
                            Set_Target_RPM(target_speed, 0.0f, -target_speed);
                            if (yaw_control_enabled) {
                                target_yaw = imu.Yaw;
                                pid_yaw.target_rpm = target_yaw;
                                is_forward_mode = 1;
                            } else {
                                is_forward_mode = 0;
                            }
            }
            break;

        case 'a':
        case 'A':
            sprintf(uart_buffer, "Diagonal Left Down\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
            // ✅ PID 리셋 추가!
                        if (pid_enabled) {
                            RESET_ALL_PIDS();
                        }

                        if (!pid_enabled) {
                            Move_DiagonalLeftDown(current_speed);
                            is_forward_mode = 0;
                        } else {
                            Set_Target_RPM(-target_speed, 0.0f, target_speed);
                            if (yaw_control_enabled) {
                               target_yaw = imu.Yaw;
                               pid_yaw.target_rpm = target_yaw;
                               is_forward_mode = 1;
                            } else {
                               is_forward_mode = 0;
                            }
            }
            break;

        case 'd':
        case 'D':
            sprintf(uart_buffer, "Diagonal Right Down\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
            // ✅ PID 리셋 추가!
                        if (pid_enabled) {
                            RESET_ALL_PIDS();
                        }

                        if (!pid_enabled) {
                           Move_DiagonalRightDown(current_speed);
                           is_forward_mode = 0;
                        } else {
                           Set_Target_RPM(0.0f, target_speed, -target_speed);
                           if (yaw_control_enabled) {
                              target_yaw = imu.Yaw;
                              pid_yaw.target_rpm = target_yaw;
                              is_forward_mode = 1;
                           } else {
                              is_forward_mode = 0;
                           }
            }
            break;

        case 'w':
        case 'W':
            sprintf(uart_buffer, "Forward\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
            // ✅ PID 리셋 추가!
                        if (pid_enabled) {
                            RESET_ALL_PIDS();
                        }

                        if (!pid_enabled) {
                            Move_Forward(current_speed);
                        } else {
                            Set_Target_RPM(target_speed, -target_speed, 0.0f);
                            if (yaw_control_enabled) {
                               target_yaw = imu.Yaw;
                               pid_yaw.target_rpm = target_yaw;
                               is_forward_mode = 1;
                            }else {
                               is_forward_mode = 0;
                            }
            }
            break;

        case 's':
        case 'S':
            sprintf(uart_buffer, "Backward\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
            // ✅ PID 리셋 추가!
                        if (pid_enabled) {
                            RESET_ALL_PIDS();
                        }

                        if (!pid_enabled) {
                           Move_Backward(current_speed);
                           is_forward_mode = 0;
                        } else {
                           Set_Target_RPM(-target_speed, target_speed, 0.0f);
                           if (yaw_control_enabled) {
                              target_yaw = imu.Yaw;
                               pid_yaw.target_rpm = target_yaw;
                               is_forward_mode = 1;
                            } else {
                               is_forward_mode = 0;
                            }
            }
            break;

        case 't':
        case 'T':
            sprintf(uart_buffer, "Rotate Clockwise\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
            is_forward_mode = 0;  // ✅ 전진 아님
            if (!pid_enabled) {
                Rotate_Clockwise(current_speed);
            } else {
                 Set_Target_RPM(target_speed, target_speed, target_speed);
            }
            break;

        case 'r':
        case 'R':
            sprintf(uart_buffer, "Rotate Counter-Clockwise\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
            is_forward_mode = 0;  // ✅ 전진 아님
            if (!pid_enabled) {
               Rotate_CounterClockwise(current_speed);
            } else {
                Set_Target_RPM(-target_speed, -target_speed, -target_speed);
            }
            break;

        case 'f':
        case 'F':
        	if (!pid_enabled) {
        		Increase_Speed();
        	} else {
        	    // PID 모드: 목표 RPM 증가
        	    target_speed += 10.0f;  // 10 RPM씩 증가
        	    if (target_speed > 100.0f) target_speed = 100.0f;  // 최대 100 RPM
        	    sprintf(uart_buffer, "Target Speed: %.0f RPM\r\n", target_speed);
        	    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
        	}
            break;

        case 'g':
        case 'G':
        	if (!pid_enabled) {
        	    // 수동 모드: PWM 감소
        	    Decrease_Speed();
        	} else {
        	     // PID 모드: 목표 RPM 감소
        	     target_speed -= 10.0f;  // 10 RPM씩 감소
        	      if (target_speed < 10.0f) target_speed = 10.0f;  // 최소 10 RPM
        	      sprintf(uart_buffer, "Target Speed: %.0f RPM\r\n", target_speed);
        	      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
        	}
            break;

        case 'x':
        case 'X':
        case ' ':  // 스페이스바로도 정지
            sprintf(uart_buffer, "STOP\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
            if (!pid_enabled) {
                Stop_All();
            } else {
                 Set_Target_RPM(0.0f, 0.0f, 0.0f);

                 // ⭐️ 해결책: 정지 시 모든 PID 적분 오차 리셋 ⭐️
                 PID_Reset(&pid_motor1);
                 PID_Reset(&pid_motor2);
                 PID_Reset(&pid_motor3);

                 // 정지 시 Yaw 리셋 (드리프트 방지)
                 if (yaw_control_enabled) {
                     imu.Yaw = 0.0f;
                     target_yaw = 0.0f;
                     PID_Reset(&pid_yaw);
                 }
                 is_forward_mode = 0; // 정지 시 Yaw 보정 차단 (이전 수정 반영)
            }
            break;

        // PID 제어 활성화/비활성화
        case 'p':
        case 'P':
            pid_enabled = !pid_enabled;
            if (pid_enabled) {
                            // ----------------------------------------------------
                            // ✅ PID 활성화 시, Yaw 제어 동시 활성화 및 초기화
                            // ----------------------------------------------------
                            yaw_control_enabled = 1;  // Yaw 제어 활성화!

                            sprintf(uart_buffer, "PID Control ENABLED, Yaw Control ENABLED\r\n");

                            // PID 초기화
                            PID_Reset(&pid_motor1);
                            PID_Reset(&pid_motor2);
                            PID_Reset(&pid_motor3);

                            // Yaw 자동 리셋
                            imu.Yaw = 0.0f;           // Yaw 각도 리셋
                            target_yaw = 0.0f;
                            pid_yaw.target_rpm = 0.0f;
                            PID_Reset(&pid_yaw);

                            // 초기 목표 속도 0
                            Set_Target_RPM(0.0f, 0.0f, 0.0f);
            } else {
                            // ----------------------------------------------------
                            // ✅ PID 비활성화 시, Yaw 제어 동시 비활성화
                            // ----------------------------------------------------
                            yaw_control_enabled = 0;  // Yaw 제어 비활성화!

                            sprintf(uart_buffer, "PID Control DISABLED, Yaw Control DISABLED\r\n");
                            Stop_All();
                            is_forward_mode = 0;
             }
             HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
             break;


        default:
        	break;

    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // -------------------------------------------------------
  // [초기화] 주변장치 시작 및 초기 설정
  // -------------------------------------------------------

  // 1. 엔코더 타이머 시작 (인터럽트 활성화 포함)
  // 엔코더 인터페이스 시작 (카운트 시작)
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);  // 엔코더1
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // 엔코더2
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);  // 엔코더3

  // 업데이트 인터럽트 시작 (오버플로우/언더플로우 감지용)
  HAL_TIM_Base_Start_IT(&htim5); // 엔코더1
  HAL_TIM_Base_Start_IT(&htim2); // 엔코더2
  HAL_TIM_Base_Start_IT(&htim8); // 엔코더3

  // 2. PWM 타이머 시작
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // PWM1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PWM2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PWM3

  // 3. 초기 상태: 모터 정지
  Stop_All();

  // 4. UART 인터럽트 수신 시작
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  // 5. MPU9250 초기화
  sprintf(uart_buffer, "\r\n=== Omni-Wheel Robot + IMU ===\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

  if (MPU9250_Init(&hi2c1) == 0) {
      sprintf(uart_buffer, "MPU9250 Init Success!\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  } else {
      sprintf(uart_buffer, "MPU9250 Init Failed! IMU disabled.\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  }

  sprintf(uart_buffer, "System Ready!\r\n\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

  // IMU 초기화
  imu.Yaw = 0.0f;
  last_imu_update_time = HAL_GetTick();
  Calibrate_Gyro(&hi2c1);
  last_imu_update_time = HAL_GetTick();

  // 6. PID 제어 초기화
  // 초기 게인 설정 (튜닝 필요!)
  PID_Init(&pid_motor1, 12.0f, 0.5f, 0.1f);  // Kp=10, Ki=0.5, Kd=0.1
  PID_Init(&pid_motor2, 12.0f, 0.5f, 0.1f);
  PID_Init(&pid_motor3, 12.0f, 0.5f, 0.1f);

  // Yaw PID 초기화 (새로 추가!)
  PID_Init(&pid_yaw, 0.5f, 0.02f, 0.03f);  // Kp 감소, Ki 감소, Kd 대폭 감소
  pid_yaw.output_min = -40.0f;  // ±100 (PWM 대비 10%)
  pid_yaw.output_max = 40.0f;

  pid_yaw.target_rpm = 100.0f;  // 직진 목표 (0도)

  sprintf(uart_buffer, "PID Controller Initialized\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

  // 7. 시작 메시지 전송
  sprintf(uart_buffer, "\r\n=== Omni-Wheel Robot Control ===\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  sprintf(uart_buffer, "W/S: Forward/Backward\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  sprintf(uart_buffer, "Q/E/A/D: Diagonal movements\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  sprintf(uart_buffer, "R/T: Rotate CCW/CW\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  sprintf(uart_buffer, "F/G: Speed Up/Down\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  sprintf(uart_buffer, "X/Space: Stop\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
  sprintf(uart_buffer, "Current Speed: %d\r\n\r\n", current_speed);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

  // 오도메트리 초기화
  odom.x = 0.0f;
  odom.y = 0.0f;
  odom.theta = 0.0f;
  odom.vx = 0.0f;
  odom.vy = 0.0f;
  odom.omega = 0.0f;
  odom.last_time = 0;

  sprintf(uart_buffer, "Odometry Initialized (r=%.2fm, R=%.2fm)\r\n",
          WHEEL_RADIUS, ROBOT_RADIUS);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // ========================================
	  // 현재 시간 (모든 타이머에서 공통 사용)
	  // ========================================
	  uint32_t current_time = HAL_GetTick();

	  // ========================================
	  // 20ms 주기 (50Hz) - 속도 계산 + 오도메트리 + PID 제어
	  // ========================================
	  static uint32_t last_speed_calc = 0;
	  if (current_time - last_speed_calc >= 20)
	  {
	      last_speed_calc = current_time;

	      Update_All_Motor_Speeds();  // 엔코더 → RPM 계산
	      Update_Odometry();           // ⭐ 오도메트리 업데이트 (새로 추가!)
	      Apply_PID_Output();          // PID 제어
	  }

	  // ========================================
	  // 50ms 주기 (20Hz) - 엔코더 & 오도메트리 전송
	  // ========================================
	  static uint32_t last_encoder_send = 0;
	  if (current_time - last_encoder_send >= 50)
	  {
	      last_encoder_send = current_time;

	      Send_Encoder_Data();      // 엔코더 값
	      Send_Odometry_Data();     // ⭐ 오도메트리 전송 (새로 추가!)
	  }

	  // ========================================
	  // 100ms 주기 (10Hz) - IMU & 속도 전송
	  // ========================================
	  static uint32_t last_imu_send = 0;
	  if (current_time - last_imu_send >= 100)
	  {
	      last_imu_send = current_time;

	      // 1. IMU 센서 읽기
	      MPU9250_Read_Sensors(&hi2c1, &imu);

	      // 2. 데이터 전송
	      Send_IMU_Data();
	      Send_Speed_Data();        // 속도 전송 (기존 위치에서 이동)
	  }

	  // UART 인터럽트가 명령을 처리하므로 메인 루프는 대기 상태
	  HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0xFFFF;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M3_DIR_Pin|CS_I2C_SPI_Pin|M1_DIR_Pin|M2_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD5_Pin|LD6_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M3_DIR_Pin CS_I2C_SPI_Pin M1_DIR_Pin M2_DIR_Pin */
  GPIO_InitStruct.Pin = M3_DIR_Pin|CS_I2C_SPI_Pin|M1_DIR_Pin|M2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD5_Pin LD6_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD5_Pin|LD6_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

