#pragma once
#include <string.h>

#include "Motor.hpp"
#include "app_framework.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"


#define MY_CLAMP(x, min, max) \
  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

class Action : public LibXR::Application {
 public:
  Action(LibXR::HardwareContainer& hw, LibXR::Terminal<32, 32, 5, 5>& app,
         Motor* motor_ptr, LibXR::PID<float>::Param speed_pid_param,
         LibXR::PID<float>::Param position_pid_param)
      : motor(motor_ptr),
        pid_speed(speed_pid_param),
        pid_position(position_pid_param) {
    if (motor) {
      thread_.Create(this, ThreadFunc, "ActionThread", 2048, LibXR::Thread::Priority::MEDIUM);
    }
  }

  static void ThreadFunc(Action* arg) {
    if (!arg) return;
    Action* action = static_cast<Action*>(arg);
    while (true) {
      action->ControlLoop(0.002f);
      LibXR::Thread::Sleep(2);
    }
  }

  void ControlLoop(float dt) {
    if (!motor) return;
    feedback = motor->GetFeedback();

    float target_pos = 45.0f;
    float pos_error = target_pos - feedback.abs_angle;
    while (pos_error > 180.0f) pos_error -= 360.0f;
    while (pos_error < -180.0f) pos_error += 360.0f;

    float target_vel = pid_position.Calculate(pos_error, 0, dt);
    target_vel = MY_CLAMP(target_vel, -20.0f, 20.0f);

    float target_current = pid_speed.Calculate(target_vel, feedback.omega, dt);
    target_current = MY_CLAMP(target_current, -20000.0f, 20000.0f);

    Motor::MotorCmd cmd;
    cmd.mode = Motor::MODE_CURRENT;
    cmd.current = target_current;
    motor->Control(cmd);
  }

  void OnMonitor() override {}

 private:
  LibXR::Thread thread_;
  Motor* motor;
  Motor::Feedback feedback;
  LibXR::PID<float> pid_speed;
  LibXR::PID<float> pid_position;
};
