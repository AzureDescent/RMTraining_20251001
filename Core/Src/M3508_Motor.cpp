#include "M3508_Motor.h"
#include <math.h>

// Define the motor instance
M3508_Motor Motor(19.2f);

// Constructor
M3508_Motor::M3508_Motor(float ratio)
    : ratio_(ratio), angle_(0.0f), delta_angle_(0.0f),
      ecd_angle_(0.0f), last_ecd_angle_(0.0f), delta_ecd_angle_(0.0f),
      rotate_speed_(0.0f), current_(0.0f), temp_(0.0f) {
}

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
// C-compatible wrapper function
extern "C" void M3508_Motor_RxCallback(const uint8_t rx_data[8]) {
    Motor.canRxMsgCallback(rx_data);
}