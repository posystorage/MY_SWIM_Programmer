#include "swim.h"

uint16_t SWIN_DMA_DAT_IN[SWIN_DMA_DAT_IN_BUF_SIZE];//输入数据缓存
uint8_t SWIN_DMA_DAT_IN_DONE=0;

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


void	SYNCSWPWM_IN_TIMER_INIT(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	DMA_DeInit(DMA1_Channel4);
	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;	//72MHz
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
//TIM4->CR1 |= TIM_CR1_CEN;
	
	//Channel 1 - DMA input capture//ti2 映射到tc1
	//Channel 2 - counter reset
		
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection =  TIM_ICSelection_DirectTI;
		
	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
	
	TIM_CCxCmd(TIM4, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM4, TIM_Channel_1, TIM_CCx_Enable);
	
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_DMACmd(TIM4, TIM_DMA_CC1, ENABLE);	
	//TIM_DMACmd(TIM4, TIM_DMA_CC2, ENABLE);

	
	//TIM_Cmd(TIM4, ENABLE);
	
//	DMA_StructInit(&DMA_InitStructure);
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(TIM4->CCR2);
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//	DMA_InitStructure.DMA_BufferSize = 0;
//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(SWIN_DMA_DAT_IN);
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
//	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
//	//DMA_Cmd(DMA1_Channel4, ENABLE);	
//	
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;  //TIM3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	

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
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;  //TIM4 dma中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	
}


void SWIM_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 	
	
	SWIM_GPIO_Init();
	SYNCSWPWM_IN_TIMER_INIT();

}

uint8_t SWIM_EnterProgMode(void)//swim入口序列
{
	uint8_t i;
	uint16_t time_out=0;
	SWIN_DMA_DAT_ENABLE(10);
	SWIM_RST_LOW;
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
	SWIM_HIGH;	

//3 - HSI is turned on
	while(SWIN_DMA_DAT_IN_DONE==0)
	{
		delay_us(1);
		time_out++;
		if(time_out>1000) 
		{
			DMA_Cmd(DMA1_Channel4, DISABLE);
			TIM_Cmd(TIM4, DISABLE);
			SWIM_RST_HIGH;
			return 0;//超时，无设备响应
		}	
	}
//4 - synchronization pulse


//5 - 300ns delay (more)
	delay_us(350);
	
	return 1;
}


void DMA1_Channel1_IRQHandler(void)
{
//	uint32_t i;

	DMA_ClearFlag(DMA1_FLAG_TC1);	//clear interrupt flag
	SWIN_DMA_DAT_IN_DONE=1;
	
}


