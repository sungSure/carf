#include "stm32f1xx_hal.h"
#include <string.h>
#include "main.h"
#include "wheel.h"
#include "tim.h"
#include "RGB.h"

uint8_t Rx_Buff[3];

uint8_t Tx_Buff[] = "hello word\n";

void Wireless_Control(void)
{
	if(Rx_Buff[0] == '1' && Rx_Buff[1] == '1' && Rx_Buff[2] == '1' ) HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);  //
	else if(strncmp((char*)Rx_Buff, "222",3) == 0) HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
	else if(strcmp((char*)Rx_Buff, "333") == 0) HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
	else if(strcmp((char*)Rx_Buff, "444") == 0) __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2500);
	else if(strcmp((char*)Rx_Buff, "555") == 0) __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
	else if(strcmp((char*)Rx_Buff, "666") == 0) __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
	else if(strcmp((char*)Rx_Buff, "777") == 0) HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
	else if(strcmp((char*)Rx_Buff, "888") == 0) WS2812_Switch();
	else if(strcmp((char*)Rx_Buff, "999") == 0) PID_Switch();
	else if(strcmp((char*)Rx_Buff, "000") == 0) Mode_Switch();
	
//	else if(strcmp((char*)Rx_Buff, "aaa") == 0) Car_Forward_Speed(5);
//	else if(strcmp((char*)Rx_Buff, "bbb") == 0) Car_Backward_Speed(5);
//	else if(strcmp((char*)Rx_Buff, "ccc") == 0) Car_Left_Speed(5);
//	else if(strcmp((char*)Rx_Buff, "ddd") == 0) Car_Right_Speed(5);
//	else if(strcmp((char*)Rx_Buff, "eee") == 0) Car_Forward_Speed(0);
	
	else if(strcmp((char*)Rx_Buff, "aaa") == 0) Car_Control_Timer(80,80);
	else if(strcmp((char*)Rx_Buff, "bbb") == 0) Car_Control_Timer(-80,-80);
	else if(strcmp((char*)Rx_Buff, "ccc") == 0) Car_Control_Timer(-80,80);
	else if(strcmp((char*)Rx_Buff, "ddd") == 0) Car_Control_Timer(80,-80);
	else if(strcmp((char*)Rx_Buff, "eee") == 0) Car_Control_Timer(0,0);

//	else if(strcmp((char*)Rx_Buff, "fff") == 0) printf("电源电压：%f \n",DC);
//	else if(strcmp((char*)Rx_Buff, "ggg") == 0) printf("距离：%f \n",distance);
//	else if(strcmp((char*)Rx_Buff, "hhh") == 0) printf("左轮速度：%d \n",Speed2);
//	else if(strcmp((char*)Rx_Buff, "iii") == 0) printf("右轮速度：%d \n",Speed3);

	else if(strcmp((char*)Rx_Buff, "***") == 0) test_angle = test_angle + 90;
	else if(strcmp((char*)Rx_Buff, "###") == 0) test_angle = test_angle - 90;
	
//	printf("Rx_Buff收到：   %d   %d   %d \r\n", Rx_Buff[0], Rx_Buff[1], Rx_Buff[2]);
	memset(Rx_Buff, 0, sizeof(Rx_Buff));
}
