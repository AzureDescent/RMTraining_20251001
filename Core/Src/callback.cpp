#include "M3508_Motor.h"
#include "can.h"

extern float target_angle;
extern uint32_t can_tx_mailbox;
extern uint8_t stop_flag;

extern "C" {
    extern CAN_RxHeaderTypeDef rx_header;
    extern CAN_TxHeaderTypeDef tx_header;
    extern uint8_t rx_data[8];
    extern uint8_t tx_data[8];
}

extern M3508_Motor Motor;
extern uint32_t can_tx_mailbox;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId == 0x201) {  // Adjust ID as needed
            Motor.canRxMsgCallback(rx_data);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6)
    {
        int16_t intensity_to_send=0;

        if (stop_flag == 0)
        {
            // switch (Motor.control_method_)
            // {
            // case Motor.TORQUE:
            //     M3508_Motor_SetTorqueMode();
            //     break;
            // case Motor.SPEED:
            //     M3508_Motor_SetSpeedMode();
            //     Motor.SetSpeed(target_angle, 0.0f);
            //     break;
            // case Motor.POSITION_SPEED:
            //     M3508_Motor_SetPositionSpeedMode();
            //     Motor.SetPosition(target_angle, 0.0f, 0.0f);
            //     break;
            // default:
            //     M3508_Motor_SetPositionSpeedMode();
            //     Motor.SetPosition(target_angle, 0.0f, 0.0f);
            //     break;
            // }
            Motor.SetPosition(target_angle, 0.0f, 0.0f);
            M3508_Motor_Handle();

            intensity_to_send = M3508_Motor_GetOutputIntensity();
        }
        else
        {
            M3508_Motor_Stop();
        }

        tx_data[0] = (uint8_t)(intensity_to_send>>8);
        tx_data[1] = (uint8_t)(intensity_to_send&0xFF);
        for (int i = 2; i < 8; i++)
        {
            tx_data[i] = 0;
        }

        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mailbox);
    }
}