#include "M3508_Motor.h"
#include <cmath>

#define PI 3.1415926f
#define G 9.81f
#define MASS 0.5f
#define ARMFORCE_LENGTH 0.05524f
// Define the motor instance
M3508_Motor Motor(19.2f);

// Constructor


M3508_Motor::M3508_Motor(float ratio)
    : ratio_(ratio), angle_(0.0f), delta_angle_(0.0f),
      ecd_angle_(0.0f), last_ecd_angle_(0.0f), delta_ecd_angle_(0.0f),
      rotate_speed_(0.0f), current_(0.0f), temp_(0.0f),
      spid_(10.0f, 0.5f, 0.1f, 50.0f, 100.0f),
      ppid_(5.0f, 0.3f, 0.05f, 30.0f, 100.0f),
      target_angle_(0.0f), fdb_angle_(0.0f),
      target_speed_(0.0f), fdb_speed_(0.0f), feedforward_speed_(0.0f),
      feedforward_intensity_(0.0f), output_intensity_(0.0f),
      control_method_(POSITION_SPEED) {}


float M3508_Motor::linearMapping(int in, int in_min, int in_max, float out_min, float out_max)
{
    return (float)(in - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


// CAN message callback
void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    ecd_angle_ = (rx_data[0] << 8) | rx_data[1];
    ecd_angle_ = linearMapping(ecd_angle_, 0, 8191,0.0f, 360.0f);

    rotate_speed_ = (rx_data[2] << 8) | rx_data[3];
    rotate_speed_ = linearMapping(rotate_speed_, -32768, 32767, -10000.0f, 10000.0f);

    current_ = (rx_data[4] << 8) | rx_data[5];
    current_ = linearMapping(current_, -32768, 32767, -20.0f, 20.0f);

    temp_ = rx_data[6];

    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle_ > 180.0f) {
        delta_ecd_angle_ -= 360.0f;
    } else if (delta_ecd_angle_ < -180.0f) {
        delta_ecd_angle_ += 360.0f;
    }
    
    delta_angle_ = delta_ecd_angle_ / ratio_;
    angle_ += delta_angle_;
    last_ecd_angle_ = ecd_angle_;
}

void M3508_Motor::SetIntensity(float intensity)
{
    control_method_ = TORQUE;
    feedforward_intensity_ = intensity;
}

void M3508_Motor::SetSpeed(float target_speed, float feedforward_intensity)
{
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}

void M3508_Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity)
{
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
}

void M3508_Motor::handle()
{
    switch (control_method_)
    {
    case TORQUE:
        feedforward_intensity_ = FeedforwardIntensityCalc(angle_);
        output_intensity_ = feedforward_intensity_;
        break;

    case SPEED:
        feedforward_intensity_ = FeedforwardIntensityCalc(target_angle_);
        output_intensity_ = spid_.calc(target_speed_, rotate_speed_) + feedforward_intensity_;
        break;

    case POSITION_SPEED:
        feedforward_intensity_ = FeedforwardIntensityCalc(target_angle_);
        target_speed_ = ppid_.calc(target_angle_, angle_) + feedforward_speed_;
        output_intensity_ = spid_.calc(target_speed_, rotate_speed_) + feedforward_intensity_;
        break;
    }
}

float M3508_Motor::FeedforwardIntensityCalc(float current_angle)
{
    float current_angle_rad = current_angle * PI / 180.0f;
    float load_gravity_torque = MASS * G * ARMFORCE_LENGTH * cosf(current_angle_rad);
    const float kt = 0.3f;

    float motor_gravity_torque = load_gravity_torque / ratio_;
    // float friction_torque = (current_ > 0) ? 0.1f : ((current_ < 0) ? -0.1f : 0.0f);
    float total_gravity_intensity = motor_gravity_torque;

    return total_gravity_intensity / kt;
}

// C-compatible wrapper function
extern "C" void M3508_Motor_RxCallback(const uint8_t rx_data[8]) {
    Motor.canRxMsgCallback(rx_data);
}