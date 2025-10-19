#include "M3508_Motor.h"
#include <cmath>

#define PI 3.1415926f
#define G 9.81f
#define MASS 0.5f
#define ARMFORCE_LENGTH 0.05524f
#define MAX_INTESITY 16384.0f
#define MAX_CURRENT 20.0f
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
    // 1. 原始数据提取
    int16_t raw_ecd_value = (rx_data[0] << 8) | rx_data[1];
    int16_t raw_rotate_speed = (rx_data[2] << 8) | rx_data[3];
    int16_t raw_current = (rx_data[4] << 8) | rx_data[5];
    temp_ = rx_data[6];

    rotate_speed_ = linearMapping(raw_rotate_speed, -10000, 10000, -10000.0f, 10000.0f); // 电机转速 (RPM)
    current_ = linearMapping(raw_current, -16384, 16384, -20.0f, 20.0f); // 实际电流 (A)

    ecd_value_ = raw_ecd_value;

    int16_t delta_ecd_value = ecd_value_ - last_ecd_value_;

    const int16_t HALF_ECD = 4096; // 8192 / 2
    const int16_t MAX_ECD = 8192; // 编码器总刻度

    if (delta_ecd_value > HALF_ECD) {
        delta_ecd_value -= MAX_ECD;
    } else if (delta_ecd_value < -HALF_ECD) {
        delta_ecd_value += MAX_ECD;
    }

    const float DEG_PER_ECD = 360.0f / MAX_ECD;
    delta_angle_ = (float)delta_ecd_value * DEG_PER_ECD / ratio_;

    angle_ += delta_angle_;

    last_ecd_value_ = ecd_value_;
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
    const float INTENSITY_PER_AMP = MAX_INTESITY / MAX_CURRENT;
    float current_angle_rad = current_angle * PI / 180.0f;
    float load_gravity_torque = MASS * G * ARMFORCE_LENGTH * sinf(current_angle_rad);
    const float kt_motor = 0.3f;

    float motor_gravity_torque = load_gravity_torque / ratio_;
    float motor_current = motor_gravity_torque / kt_motor;
    float total_gravity_intensity = motor_current * INTENSITY_PER_AMP;

    return total_gravity_intensity;
}

// C-compatible wrapper function
extern "C" void M3508_Motor_RxCallback(const uint8_t rx_data[8])
{
    Motor.canRxMsgCallback(rx_data);
}

extern "C" void M3508_Motor_SetTorqueMode(void)
{
    Motor.control_method_ = M3508_Motor::TORQUE;
}

extern "C" void M3508_Motor_Handle(void)
{
    Motor.handle();
}

extern "C" int16_t M3508_Motor_GetOutputIntensity(void) {
    return (int16_t)Motor.output_intensity_;
}