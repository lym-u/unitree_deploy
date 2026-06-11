#pragma once

#include "unitree/dds_wrapper/robots/g1/g1.h"
#include "unitree/dds_wrapper/robots/go2/go2.h"

using LowCmd_t = unitree::robot::g1::publisher::LowCmd;
using LowState_t = unitree::robot::g1::subscription::LowState;

using HandCmd_t =
    unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::HandCmd_>;
using LeftHandState_t = unitree::robot::g1::subscription::Dex3LeftHandState;
using RightHandState_t = unitree::robot::g1::subscription::Dex3RightHandState;
