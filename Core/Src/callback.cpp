#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

extern uint8_t rx_msg[4];
extern uint8_t tx_msg[];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BUTTON_Pin)
    {
        uint32_t arr_value = __HAL_TIM_GetAutoreload(&htim1) + 1;
        uint32_t brightness = (__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_2) + 100) % arr_value;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, brightness);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart8)
    {
        HAL_UART_Receive_IT(&huart8, rx_msg, 3);
        HAL_UART_Transmit(&huart8, tx_msg, 10, 10000);
    }
}