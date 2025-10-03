#include "M3508_Motor.h"
#include "can.h"

extern "C" {
    extern CAN_RxHeaderTypeDef rx_header;
    extern CAN_TxHeaderTypeDef tx_header;
    extern uint8_t rx_data[8];
    extern uint8_t tx_data[8];
}
uint32_t can_tx_mail_box;

extern M3508_Motor Motor;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    if (rx_header.StdId == 0x201) {  // Adjust ID as needed
        Motor.canRxMsgCallback(rx_data);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box);
    }
}


// #include "main.h"
// #include "gpio.h"
// #include "tim.h"
// #include "usart.h"
// #include "can.h"
// #include "M3508_Motor.h"
//
// extern CAN_RxHeaderTypeDef rx_header;
// extern CAN_TxHeaderTypeDef tx_header;
// extern uint8_t rx_msg[8];
// extern uint8_t tx_msg[8];
// extern M3508_Motor Motor;
//
// // void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// // {
// //     if (hcan->Instance == CAN1)
// //     {
// //         HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
// //         if (rx_header.StdId == 0x201)
// //         {
// //             Motor.canRxMsgCallback(rx_data);
// //         }
// //     }
// // }
// // void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// // {
// //     if (htim->Instance == htim6.Instance)
// //     {
// //         HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box);
// //     }
// // }
// // class M3508_Motor{
// // private:
// //     const float ratio_ = 19.2f;
// //
// //     float angle_ = 0.f;
// //     float delta_angle_ = 0.f;
// //     float ecd_angle_ = 0.f;
// //     float last_ecd_angle_ = 0.f;
// //     float delta_ecd_angle_ = 0.f;
// //
// //     float rotate_speed_ = 0.f;
// //     float current_ = 0.f;
// //     float temp_ = 0.f;
// //
// // public:
// //     explicit M3508_Motor(float ratio):ratio_(ratio) {};
// //     void canRxMsgCallback(const uint8_t rx_data[8]);
// // };