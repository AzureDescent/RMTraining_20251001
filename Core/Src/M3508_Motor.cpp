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

// CAN message callback
void M3508_Motor::canRxMsgCallback(const uint8_t rx_data[8]) {

}

// C-compatible wrapper function
extern "C" void M3508_Motor_RxCallback(const uint8_t rx_data[8]) {
    Motor.canRxMsgCallback(rx_data);
}