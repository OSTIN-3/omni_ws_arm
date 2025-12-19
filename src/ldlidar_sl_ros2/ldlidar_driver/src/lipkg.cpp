/**
 * @file lipkg.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 * (Modified: Intuitive Configuration for Omni-Wheel Robot)
 */
#include "lipkg.h"
#include <math.h>
#include <iostream>
#include <algorithm>

namespace ldlidar {

static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8};

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len) {
  uint8_t crc = 0;
  while (data_len--) {
    crc = CrcTable[(crc ^ *data) & 0xff];
    data++;
  }
  return crc;
}

LiPkg::LiPkg()
  : lidar_measure_freq_(2300),
    typenumber_(LDType::NO_VER),
    lidarstatus_(LidarStatus::NORMAL),
    lidarerrorcode_(LIDAR_NO_ERROR),
    is_frame_ready_(false),
    is_noise_filter_(false),
    timestamp_(0),
    speed_(0),
    get_timestamp_(nullptr),
    is_poweron_comm_normal_(false),
    poweron_datapkg_count_(0),
    last_pkg_timestamp_(0) {
}

LiPkg::~LiPkg() {}

void LiPkg::SetProductType(LDType typenumber) {
  typenumber_ = typenumber;
  switch (typenumber) {
    case LDType::LD_14:
    case LDType::LD_14P_2300HZ:
      lidar_measure_freq_ = 2300;
      break;
    case LDType::LD_14P_4000HZ:
      lidar_measure_freq_ = 4000;
      break;
    default :
      lidar_measure_freq_ = 2300;
      break;
  }
}

void LiPkg::SetNoiseFilter(bool is_enable) { is_noise_filter_ = is_enable; }

void LiPkg::RegisterTimestampGetFunctional(std::function<uint64_t(void)> timestamp_handle) {
  get_timestamp_ = timestamp_handle;
}

bool LiPkg::AnalysisOne(uint8_t byte) {
  static enum { HEADER, VER_LEN, DATA } state = HEADER;
  static uint16_t count = 0;
  static uint8_t tmp[128] = {0};
  static uint16_t pkg_count = sizeof(LiDARFrameTypeDef);

  switch (state) {
    case HEADER:
      if (byte == PKG_HEADER) {
        tmp[count++] = byte;
        state = VER_LEN;
      }
      break;
    case VER_LEN:
      if (byte == PKG_VER_LEN) {
        tmp[count++] = byte;
        state = DATA;
      } else {
        state = HEADER; count = 0;
        return false;
      }
      break;
    case DATA:
      tmp[count++] = byte;
      if (count >= pkg_count) {
        memcpy((uint8_t *)&datapkg_, tmp, pkg_count);
        uint8_t crc = CalCRC8((uint8_t *)&datapkg_, pkg_count - 1);
        state = HEADER; count = 0;
        if (crc == datapkg_.crc8) return true;
        else return false;
      }
      break;
  }
  return false;  
}

bool LiPkg::Parse(const uint8_t *data, long len) {
  for (int i = 0; i < len; i++) {
    if (AnalysisOne(data[i])) {
      poweron_datapkg_count_++;
      if (poweron_datapkg_count_ >= 2) {
        poweron_datapkg_count_ = 0;
        is_poweron_comm_normal_ = true;
      }
      speed_ = datapkg_.speed;
      timestamp_ = datapkg_.timestamp;
      double diff = (datapkg_.end_angle / 100 - datapkg_.start_angle / 100 + 360) % 360;
      if (diff <= ((double)datapkg_.speed * POINT_PER_PACK / lidar_measure_freq_ * 1.5)) {
        if (0 == last_pkg_timestamp_) {
          last_pkg_timestamp_ = get_timestamp_();
        } else {
          uint64_t current_pack_stamp = get_timestamp_();
          double pack_stamp_point_step = static_cast<double>(current_pack_stamp - last_pkg_timestamp_) / static_cast<double>(POINT_PER_PACK - 1);
          uint32_t angle_diff = ((uint32_t)datapkg_.end_angle + 36000 - (uint32_t)datapkg_.start_angle) % 36000;
          float step = angle_diff / (POINT_PER_PACK - 1) / 100.0;
          float start = (double)datapkg_.start_angle / 100.0;
          for (int i = 0; i < POINT_PER_PACK; i++) {
            float angle = start + i * step;
            if (angle >= 360.0) angle -= 360.0;
            uint64_t st = static_cast<uint64_t>(last_pkg_timestamp_ + (pack_stamp_point_step * i));
            frame_tmp_.push_back(PointData(angle, datapkg_.point[i].distance, datapkg_.point[i].intensity, st));
          }
          last_pkg_timestamp_ = current_pack_stamp;
        }
      }
    }
  }
  return true;
}

// =========================================================================================
// ⭐⭐ 핵심 수정 함수: 사용자 설정에 따라 데이터 조립 ⭐⭐
// =========================================================================================
bool LiPkg::AssemblePacket() {
  float last_angle = 0;
  Points2D tmp, data;
  int count = 0;

  if (speed_ <= 0) {
    frame_tmp_.clear();
    return false;
  }

  for (auto n : frame_tmp_) {
    if ((n.angle < 20.0) && (last_angle > 340.0)) {
      if ((count * GetSpeed()) > (lidar_measure_freq_ * 1.4)) {
        frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
        return false;
      }
      data.insert(data.begin(), frame_tmp_.begin(), frame_tmp_.begin() + count);
      SlTransform trans(typenumber_);
      data = trans.Transform(data);
      
      // =======================================================================
      // ⭐⭐ [사용자 설정 영역] 이 변수들만 수정하세요! ⭐⭐
      // =======================================================================
      
      // 1. 라이다 설치 회전 보정 (뒤집혀 있으면 180.0, 아니면 0.0)
      float install_offset = 180.0f; 

      // 2. 거리 제한 (단위: mm)
      // 너무 가까워서 사라지면 dist_min을 줄이세요 (예: 100.0)
      float dist_min = 150.0f;   // 15cm 이내 무시
      float dist_max = 12000.0f; // 12m 이상 무시

      // 3. "삭제할" 각도 범위 (뒤쪽을 자르기 위해 설정)
      // 90도 ~ 270도 사이(뒤쪽 반원)를 잘라내면 -> 정면 180도만 남습니다.
      float cut_angle_start = 90.0f;
      float cut_angle_end   = 270.0f;

      // =======================================================================

      Points2D filtered_tmp; // 최종 데이터를 담을 벡터

      for (auto point : data) {
        
        // [A] 거리 필터링 (가까운 노이즈 제거)
        if (point.distance < dist_min || point.distance > dist_max) {
            continue; // 유효 거리 밖이면 버림
        }

        // [B] 설치 각도 보정 (회전)
        point.angle += install_offset;
        if (point.angle >= 360.0f) {
            point.angle -= 360.0f;
        }

        // [C] 각도 자르기 (원하는 영역만 남기기)
        // 설정한 범위(Start ~ End) 사이에 들어오면 버립니다.
        if (point.angle > cut_angle_start && point.angle < cut_angle_end) {
            continue; // 뒤쪽 영역이므로 버림
        }

        // 모든 조건을 통과한 포인트만 저장
        filtered_tmp.push_back(point);
      }
      
      tmp = filtered_tmp;

      // 데이터 정렬 및 전송 준비
      std::sort(tmp.begin(), tmp.end(), [](PointData a, PointData b) { return a.stamp < b.stamp; });
      if (tmp.size() > 0) {
        SetLaserScanData(tmp);
        SetFrameReady();
        frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
        return true;
      }
    }
    count++;
    if ((count * GetSpeed()) > (lidar_measure_freq_ * 2)) {
      frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
      return false;
    }
    last_angle = n.angle;
  }
  return false;
}

void LiPkg::CommReadCallBack(const char *byte, size_t len) {
  if (this->Parse((uint8_t *)byte, len)) {
    this->AssemblePacket();
  }
}

bool LiPkg::GetLaserScanData(Points2D& out) {
  if (IsFrameReady()) {
    ResetFrameReady();
    out = GetLaserScanData();
    return true;
  }
  return false;
}

double LiPkg::GetSpeed(void) { return (speed_ / 360.0); }
LidarStatus LiPkg::GetLidarStatus(void) { return lidarstatus_; }
uint8_t LiPkg::GetLidarErrorCode(void) { return lidarerrorcode_; }
bool LiPkg::GetLidarPowerOnCommStatus(void) {
  if (is_poweron_comm_normal_) { is_poweron_comm_normal_ = false; return true; }
  return false;
}
void LiPkg::SetLidarStatus(LidarStatus status) { lidarstatus_ = status; }
void LiPkg::SetLidarErrorCode(uint8_t errorcode) { lidarerrorcode_ = errorcode; }
bool LiPkg::IsFrameReady(void) { std::lock_guard<std::mutex> lg(mutex_lock1_); return is_frame_ready_; }
void LiPkg::ResetFrameReady(void) { std::lock_guard<std::mutex> lg(mutex_lock1_); is_frame_ready_ = false; }
void LiPkg::SetFrameReady(void) { std::lock_guard<std::mutex> lg(mutex_lock1_); is_frame_ready_ = true; }
void LiPkg::SetLaserScanData(Points2D& src) { std::lock_guard<std::mutex> lg(mutex_lock2_); lidar_frame_data_ = src; }
Points2D LiPkg::GetLaserScanData(void) { std::lock_guard<std::mutex> lg(mutex_lock2_); return lidar_frame_data_; }

} // namespace ldlidar