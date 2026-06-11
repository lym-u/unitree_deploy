#include "G1Hand.h"
#include "param.h"
#include <stdexcept>

std::unique_ptr<HandCmd_t> G1Hand::left_handcmd = nullptr;
std::unique_ptr<HandCmd_t> G1Hand::right_handcmd = nullptr;
std::shared_ptr<LeftHandState_t> G1Hand::left_handstate = nullptr;
std::shared_ptr<RightHandState_t> G1Hand::right_handstate = nullptr;

void G1Hand::init() {
  left_handcmd = std::make_unique<HandCmd_t>("rt/dex3/left/cmd");
  right_handcmd = std::make_unique<HandCmd_t>("rt/dex3/right/cmd");

  left_handstate = std::make_shared<LeftHandState_t>("rt/dex3/left/state");
  right_handstate = std::make_shared<RightHandState_t>("rt/dex3/right/state");

  left_handcmd->msg_.motor_cmd().resize(7);
  right_handcmd->msg_.motor_cmd().resize(7);

  // 给每个 motor 设置 mode/q/dq/tau/kp/kd
  for (int i = 0; i < 7; ++i) {
    auto &l = left_handcmd->msg_.motor_cmd()[i];
    auto &r = right_handcmd->msg_.motor_cmd()[i];

    l.mode() = r.mode() = dex3_mode(i);
    l.q() = r.q() = 0.0f;
    l.dq() = r.dq() = 0.0f;
    l.tau() = r.tau() = 0.0f;

    // 默认值; State_Mimic::enter() 将会通过 G1Hand::set_gains() 重写.
    l.kp() = r.kp() = 1.0f;
    l.kd() = r.kd() = 0.1f;
  }
}

void G1Hand::lock() {
  left_handcmd->lock();
  right_handcmd->lock();
}

void G1Hand::unlock() {
  left_handcmd->unlock();
  right_handcmd->unlock();
}

void G1Hand::publish() {
  if (param::dry_run) {
    unlock();
  } else {
    left_handcmd->unlockAndPublish();
    right_handcmd->unlockAndPublish();
  }
}

void G1Hand::set_gains(const std::vector<float> &left_kp,
                       const std::vector<float> &left_kd,
                       const std::vector<float> &right_kp,
                       const std::vector<float> &right_kd) {
  if (left_kp.size() != 7 || left_kd.size() != 7 || right_kp.size() != 7 ||
      right_kd.size() != 7) {
    throw std::runtime_error("Hand kp/kd size must be 7.");
  }
  lock();
  for (int i = 0; i < 7; ++i) {
    auto &l = left_handcmd->msg_.motor_cmd()[i];
    auto &r = right_handcmd->msg_.motor_cmd()[i];

    l.kp() = left_kp[i];
    l.kd() = left_kd[i];

    r.kp() = right_kp[i];
    r.kd() = right_kd[i];
  }
  unlock();
}

void G1Hand::release() {
  lock();
  for (int i = 0; i < 7; ++i) {
    auto &l = left_handcmd->msg_.motor_cmd()[i];
    auto &r = right_handcmd->msg_.motor_cmd()[i];

    l.mode() = r.mode() = dex3_mode(i, 0x01, 0x01);
    l.q() = r.q() = 0.0f;
    l.dq() = r.dq() = 0.0f;
    l.tau() = r.tau() = 0.0f;
    l.kp() = r.kp() = 0.0f;
    l.kd() = r.kd() = 0.0f;
  }
  publish();
}
