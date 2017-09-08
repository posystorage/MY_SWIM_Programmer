#include "led.h"
#include "delay.h"
#include "sys.h"
#include "timer.h"
#include "usart.h"
#include "swim.h"
#include "swim_cmd.h"

#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"	


#include "SEGGER_SYSVIEW.h"


 int main(void)
 {	
	delay_init();	    	 //延时函数初始化
	//SEGGER_SYSVIEW_Conf();//初始化调试组件
	uart_init(115200);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	//LED_Init();		  	//初始化与LED连接的硬件接口
	//TIM3_Int_Init(4999,7199);//10Khz的计数频率，计数到5000为500ms 

	delay_ms(500);
	USB_Port_Set(0); 	//USB先断开
	delay_ms(100);
	USB_Port_Set(1);	//USB再次连接
 	Set_USBClock();   
 	USB_Interrupts_Config();    
 	USB_Init();	

	 
	 SWIM_Init();
//	 SWIM_EnterProgMode();
//	 SWIM_CUT_OFF();
   	while(1)
	{
		SWIM_Service();
	   	
	}
}
