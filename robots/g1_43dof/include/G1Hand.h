#pragma once
#include "Types.h"

struct G1Hand {
  static std::unique_ptr<HandCmd_t> left_handcmd;
  static std::unique_ptr<HandCmd_t> right_handcmd;
  static std::shared_ptr<LeftHandState_t> left_handstate;
  static std::shared_ptr<RightHandState_t> right_handstate;

  static void init();
  static void lock();
  static void unlock();
  static void publish();
  static void set_gains(const std::vector<float> &left_kp,
                        const std::vector<float> &left_kd,
                        const std::vector<float> &right_kp,
                        const std::vector<float> &right_kd);
  static uint8_t dex3_mode(int motor_id, uint8_t status = 0x01,
                           uint8_t timeout = 0x00) {
    uint8_t mode = 0;
    mode |= (motor_id & 0x0F);
    mode |= (status & 0x07) << 4;
    mode |= (timeout & 0x01) << 7;
    return mode;
  }
  static void release();
};
