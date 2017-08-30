#include "swim.h"
#include "SEGGER_SYSVIEW.h"

uint16_t SWIN_DMA_DAT_IN[SWIN_DMA_DAT_IN_BUF_SIZE];//输入数据缓存
uint16_t SWIN_DMA_DAT_OUT[SWIN_DMA_DAT_OUT_BUF_SIZE];//输出数据缓存

uint8_t SWIN_DMA_DAT_IN_DONE=0;
uint8_t SWIN_DMA_DAT_OUT_DONE=0;
uint16_t SWIN_CLOCK_128=0;

uint16_t SWIN_OUT_Timer_ARR;//一个swim数据的定时器周期
uint16_t SWIN_OUT_Timer_CN0;//低
uint16_t SWIN_OUT_Timer_CN1;//高


void SWIM_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	
	SWIM_HIGH;
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_SWIM_OUT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	  		//开漏输出，高电平靠外接680R电阻
  GPIO_Init(GPIO_SWIM_PORT, &GPIO_InitStructure);	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_SWIM_IN1_PIN;			
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  		//浮空输入
  GPIO_Init(GPIO_SWIM_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_SWIM_IN2_PIN;			  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  		//浮空输入
  GPIO_Init(GPIO_SWIM_PORT, &GPIO_InitStructure);	
	
	
	SWIM_RST_HIGH;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_SWIM_RSTOUT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIO_SWIM_RST_PORT, &GPIO_InitStructure);
}


void	SYNCSWPWM_TIMER_INIT(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	
	DMA_DeInit(DMA1_Channel1);
	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;	//72MHz
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	
	//Channel 1 - DMA input capture//ti2 映射到tc1
	//Channel 2 - counter reset
		
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection =  TIM_ICSelection_DirectTI;		
	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
	
	
		/* PWM1 Mode configuration: Channel3 out */  
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                          
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
	TIM_OCInitStructure.TIM_Pulse = 1;                                      
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_Low;                    
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);                               
	//TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_DMACmd(TIM4, TIM_DMA_CC1, ENABLE);	
	//TIM_DMACmd(TIM4, TIM_DMA_CC3, ENABLE);
	TIM_Cmd(TIM4,ENABLE);


	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(TIM4->CCR1);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(SWIN_DMA_DAT_IN);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	//DMA_Cmd(DMA1_Channel1, ENABLE);	
	
//	DMA_StructInit(&DMA_InitStructure);
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(TIM4->CCR3);
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
//	DMA_InitStructure.DMA_BufferSize = 0;
//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(SWIN_DMA_DAT_OUT);
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_Init(DMA1_Channel7, &DMA_InitStructure);
//	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
	//DMA_Cmd(DMA1_Channel7, ENABLE);		
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;  //TIM4 dma中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	
	
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;  //TIM4 dma中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	
}


void SWIM_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 	
	
	SWIM_GPIO_Init();
	SYNCSWPWM_TIMER_INIT();

}

//设置SWIM通讯时钟参数
//SWIM_MHz swim时钟 一般为8mhz
//cnt0 低电平时间
//cnt1 高电平时间
void SWIM_Set_Speed(uint8_t SWIM_MHz, uint8_t cnt0, uint8_t cnt1)//cnt0和cnt1构成一个周期
{

	SWIN_OUT_Timer_ARR=SystemCoreClock/1000/1000/SWIM_MHz*(cnt0+cnt1)-1;//计算出定时器重装载值
	SWIN_OUT_Timer_CN0=SystemCoreClock/1000/1000/SWIM_MHz*(cnt0-1);//计算出发送0的时间
	SWIN_OUT_Timer_CN1=SystemCoreClock/1000/1000/SWIM_MHz*cnt1-1;//计算出发送1的时间
}


uint8_t SWIM_Send_Header(uint8_t cmd)
{
	int8_t i,p;
	SWIN_DMA_DAT_OUT[0]=SWIN_OUT_Timer_CN0;
	for (i = 0; i<3; i++)
	{
		if ((cmd >> (3-i-1)) & 1)
		{
			SWIN_DMA_DAT_OUT[i+1] = SWIN_OUT_Timer_CN1;
			p++;
		}
		else
		{
			SWIN_DMA_DAT_OUT[i+1] = SWIN_OUT_Timer_CN0;
		}
	}
	if (p & 1)
	{		
		SWIN_DMA_DAT_OUT[4] = SWIN_OUT_Timer_CN1;
	}
	else
	{
		SWIN_DMA_DAT_OUT[4] = SWIN_OUT_Timer_CN0;
	}
	SWIN_DAT_OUT_ENABLE(SWIN_DMA_DAT_OUT[0]);
	SWIN_DAT_OUT_DAT(SWIN_DMA_DAT_OUT[1]);
	SWIN_DAT_OUT_DAT(SWIN_DMA_DAT_OUT[2]);
	SWIN_DAT_OUT_DAT(SWIN_DMA_DAT_OUT[3]);
	SWIN_DAT_OUT_DAT(SWIN_DMA_DAT_OUT[4]);
	SWIN_DAT_OUT_DISABLE();
	return 0;
}



static uint8_t SWIM_SRST(void)
{
	return SWIM_Send_Header(SWIM_CMD_SRST);
}


uint8_t SWIM_EnterProgMode(void)//swim入口序列
{
	uint8_t i;
	uint16_t time_out=0;
	SWIM_RST_HIGH;
	delay_ms(20);
	SWIM_RST_LOW;
	delay_ms(10);
	SWIN_DMA_DAT_IN_ENABLE(10);	
	SWIN_DMA_DAT_IN_DONE=0;	
//1 - LOW SWIM for 16us	(more)
	SWIM_LOW;
	delay_us(1000);
	SWIM_HIGH;
//2 - 4 pulses 1kHz and 4 pulses 2kHz	
	for (i = 0; i < 4; i++)
	{
		SWIM_HIGH;
		delay_us(500);
		SWIM_LOW;
		delay_us(500);
	}
	
	for (i = 0; i < 4; i++)
	{
		SWIM_HIGH;
		delay_us(250);
		SWIM_LOW;
		delay_us(250);
	}
	if(SWIN_DMA_DAT_IN_DONE==1)
	{
			DMA_Cmd(DMA1_Channel4, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			SWIM_RST_HIGH;
			SWIM_HIGH;
			return 0;//错误	，重复加入时序会造成该错误
	}
	SWIM_HIGH;	
//3 - HSI is turned on . wait 128 swim clock synchronization pulse
	while(SWIN_DMA_DAT_IN_DONE==0)
	{
		delay_us(1);
		time_out++;
		if(time_out>1000) 
		{
			DMA_Cmd(DMA1_Channel4, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			SWIM_RST_HIGH;
			//SEGGER_SYSVIEW_Print("timeout\n");
			return 0;//超时，无设备响应
		}	
	}
//4 - synchronization pulse
	if(SWIN_DMA_DAT_IN[9]<1400&&SWIN_DMA_DAT_IN[9]>900)
	{
		SWIN_CLOCK_128=SWIN_DMA_DAT_IN[9];
	
	}
	else
	{
			DMA_Cmd(DMA1_Channel4, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			SWIM_RST_HIGH;
			return 0;//128swim时钟err	
	}
	SWIN_DMA_DAT_IN_DISABLE();//关闭输入
	
//5 - 300ns delay (more)
	delay_us(350);
	
// Set swim speed   low speed	
	SWIM_Set_Speed(8,20,2);
// soft rst
	SWIM_SRST();
	
	
	
	
	return 1;
}


void DMA1_Channel1_IRQHandler(void)
{
//	uint32_t i;
	DMA_ClearFlag(DMA1_FLAG_TC1);	//clear interrupt flag
	SWIN_DMA_DAT_IN_DONE=1;

}

//void DMA1_Channel7_IRQHandler(void)
//{
//	DMA_ClearFlag(DMA1_FLAG_TC7);	//clear interrupt flag
//	SWIN_DMA_DAT_OUT_DONE=1;
//	//SWIN_DMA_DAT_OUT_DISABLE();
//}

