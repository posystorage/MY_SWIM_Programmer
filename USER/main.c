#include "led.h"
#include "delay.h"
#include "sys.h"
#include "timer.h"
#include "swim.h"



 int main(void)
 {	
	delay_init();	    	 //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	//LED_Init();		  	//初始化与LED连接的硬件接口
	//TIM3_Int_Init(4999,7199);//10Khz的计数频率，计数到5000为500ms  
	 SWIM_Init();
	 SWIM_EnterProgMode();
   	while(1)
	{
	   
	}
}
