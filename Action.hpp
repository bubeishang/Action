#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: IMU interface module
constructor_args: []
template_args: []
required_hardware:
  - imu
  - scl
  - sda
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <mutex>

#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

class Action : public LibXR::Application {
 public:
  Action(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
         RMMotor* motor,
         //LibXR::Thread::Priority priority = LibXR::Thread::Priority::MEDIUM,
         //float control_freq_hz = 1000.0f,
         LibXR::PID<float>::Param pos_pid_param = {},
         LibXR::PID<float>::Param vel_pid_param = {},
         float max_vel = 20.0f,
          float max_current = 3.0f)
      : motor_(motor),
        max_vel_(max_vel),
        max_current_(max_current),
        pos_pid_(pos_pid_param),
        vel_pid_(vel_pid_param)
         {
    UNUSED(hw);
    UNUSED(app);
    thread_.Create(this, ThreadFunc, "MotorCtrl",2048, LibXR::Thread::Priority::MEDIUM);
  }

  Motor::Feedback GetFeedback() { return cached_feedback_; }

  void Enable() { motor_->Enable(); }
  void Disable() { motor_->Disable(); }
  void Relax() { motor_->Relax(); }
  void OnMonitor() override {}

 private:
  static void ThreadFunc(Action* self) { self->ControlLoop(); }

  void ControlLoop() {

    last = LibXR::Timebase::GetMilliseconds();

    while (true) {
      motor_->Update();

      cached_feedback_ = motor_->GetFeedback();

      now = LibXR::Timebase::GetMilliseconds();
      float dt = (now - last) / 1000.0f;
      if (dt <= 0.001f) dt = 0.001f;
      last = now;

      float target_pos = 3.14f;

      float target_vel =
          pos_pid_.Calculate(target_pos, cached_feedback_.abs_angle, dt);
      target_vel = std::clamp(target_vel, -max_vel_, max_vel_);

      float current_cmd =
          vel_pid_.Calculate(target_vel, cached_feedback_.omega, dt);
      current_cmd = std::clamp(current_cmd, -max_current_, max_current_);

      Motor::MotorCmd cmd;
      cmd.mode = Motor::MODE_CURRENT;
      cmd.velocity =current_cmd;
      motor_->Control(cmd);

      LibXR::Thread::SleepUntil(last, 2);
    }
  }

  Motor* motor_;
  LibXR::MillisecondTimestamp control_period_ms_;
  float max_vel_;
  float max_current_;

  LibXR::PID<float> pos_pid_;
  LibXR::PID<float> vel_pid_;

  std::atomic<float> target_position_{0.0f};
  Motor::Feedback cached_feedback_;

  LibXR::Thread thread_;

  LibXR::MillisecondTimestamp now;
  LibXR::MillisecondTimestamp last;
};
