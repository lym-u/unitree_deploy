#pragma once

#include "G1Hand.h"
#include "unitree_articulation.h"
#include <mutex>
#include <stdexcept>

namespace unitree {

template <typename LowStatePtr>
class G1DexArticulation : public BaseArticulation<LowStatePtr> {
public:
  using Base = BaseArticulation<LowStatePtr>;

  explicit G1DexArticulation(LowStatePtr lowstate) : Base(lowstate) {}

  void update() override {
    if (this->data.joint_ids_map.size() != 43) {
      Base::update();
      return;
    }

    if (!G1Hand::left_handstate || !G1Hand::right_handstate) {
      throw std::runtime_error("G1 hand state is not initialized.");
    }

    std::scoped_lock lock(this->lowstate->mutex_,
                          G1Hand::left_handstate->mutex_,
                          G1Hand::right_handstate->mutex_);

    for (int i = 0; i < 3; ++i) {
      this->data.root_ang_vel_b[i] =
          this->lowstate->msg_.imu_state().gyroscope()[i];
    }

    this->data.root_quat_w =
        Eigen::Quaternionf(this->lowstate->msg_.imu_state().quaternion()[0],
                           this->lowstate->msg_.imu_state().quaternion()[1],
                           this->lowstate->msg_.imu_state().quaternion()[2],
                           this->lowstate->msg_.imu_state().quaternion()[3]);

    this->data.projected_gravity_b =
        this->data.root_quat_w.conjugate() * this->data.GRAVITY_VEC_W;

    // 手状态长度保护
    if (G1Hand::left_handstate->msg_.motor_state().size() < 7 ||
        G1Hand::right_handstate->msg_.motor_state().size() < 7) {
      return;
    }

    // 设置关节位置和速度，0-21：身体，22-28：左手，29-35：身体，36-42：右手
    const auto &ids = this->data.joint_ids_map;
    for (int i = 0; i < 43; ++i) {
      int id = ids[i];
      if (i < 22) {
        this->data.joint_pos[i] = this->lowstate->msg_.motor_state()[id].q();
        this->data.joint_vel[i] = this->lowstate->msg_.motor_state()[id].dq();
      } else if (i < 29) {
        this->data.joint_pos[i] =
            G1Hand::left_handstate->msg_.motor_state()[id].q();
        this->data.joint_vel[i] =
            G1Hand::left_handstate->msg_.motor_state()[id].dq();
      } else if (i < 36) {
        this->data.joint_pos[i] = this->lowstate->msg_.motor_state()[id].q();
        this->data.joint_vel[i] = this->lowstate->msg_.motor_state()[id].dq();
      } else {
        this->data.joint_pos[i] =
            G1Hand::right_handstate->msg_.motor_state()[id].q();
        this->data.joint_vel[i] =
            G1Hand::right_handstate->msg_.motor_state()[id].dq();
      }
    }
  }
};

} // namespace unitree