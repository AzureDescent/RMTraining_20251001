#ifndef M3508_MOTOR_H
#define M3508_MOTOR_H

#ifdef __cplusplus

#include <stdint.h>
#include "pid.h"

extern float target_angle;

class M3508_Motor
{
private:
    const float ratio_; // 减速比
    float angle_; // 角度
    float delta_angle_; // 角度变化量
    float ecd_angle_; // 编码器角度
    float last_ecd_angle_; // 上一次编码器角度
    float delta_ecd_angle_; // 编码器角度变化量
    float rotate_speed_; // 转速
    float current_; // 电流
    float temp_; // 温度

    int16_t ecd_value_;
    int16_t last_ecd_value_;

    bool first_data_received_ = false;

public:
    // 构造函数
    explicit M3508_Motor(float ratio = 19.2f);
    float linearMapping(int in, int in_min, int in_max, float out_min, float out_max);

    // CAN接收消息回调函数
    void canRxMsgCallback(const uint8_t rx_data[8]);

    PID spid_, ppid_;
    float target_angle_, fdb_angle_;
    float target_speed_, fdb_speed_, feedforward_speed_;
    float feedforward_intensity_, output_intensity_;

    enum
    {
        TORQUE,
        SPEED,
        POSITION_SPEED,
    } control_method_;

    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);

    void handle();

    float FeedforwardIntensityCalc(float current_angle);

    void resetAngle(void);

    // void initEcd(int16_t ecd_value);
};

// Declare the motor instance for C++ files
extern M3508_Motor Motor;

#else
// For C files, declare an opaque pointer
// typedef struct M3508_Motor M3508_Motor;
#endif

// C-compatible function declarations
#ifdef __cplusplus
extern "C" {
#endif

void m3508_motor_rx_callback(const uint8_t rx_data[8]);

void M3508_Motor_SetPositionSpeedMode(void);
void M3508_Motor_Handle(void);
int16_t M3508_Motor_GetOutputIntensity(void);

void M3508_Motor_ResetAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* M3508_MOTOR_H */