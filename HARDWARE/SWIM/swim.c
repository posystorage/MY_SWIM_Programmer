#include "swim.h"
#include "SEGGER_SYSVIEW.h"
#include "usart.h"

uint16_t SWIN_DMA_DAT_IN[SWIN_DMA_DAT_IN_BUF_SIZE];//输入数据缓存
uint16_t SWIN_DMA_DAT_OUT[SWIN_DMA_DAT_OUT_BUF_SIZE];//输出数据缓存

//uint8_t SWIN_DMA_DAT_IN_DONE=0;
//uint8_t SWIN_DMA_DAT_OUT_DONE=0;
uint16_t SWIN_CLOCK_128=0;

uint16_t SWIN_OUT_Timer_ARR;//一个swim数据的定时器周期
uint16_t SWIN_OUT_Timer_CN0;//低
uint16_t SWIN_OUT_Timer_CN1;//高
uint16_t SWIM_PULSE_Threshold;


uint8_t SWIM_ROTF_READ_DMA_NUM_NEXT_TIME=0;//下次读输入缓存的位置

uint8_t ReadBuff[512];
uint8_t WriteBuff[512];

uint8_t err=0;

void SWIM_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable , ENABLE );	
	GPIO_PinRemapConfig( GPIO_PartialRemap_TIM3 , ENABLE );	
	
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
	DMA_DeInit(DMA1_Channel3);
	
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
	TIM_DMACmd(TIM4, TIM_DMA_CC2, ENABLE);
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
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	//DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	//DMA_Cmd(DMA1_Channel1, ENABLE);	
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;  //TIM4 dma中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	



	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;	//72MHz
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity =  TIM_OCPolarity_Low;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM3->CCR1=0;//占空比为0 即一直保持高电平
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_DMACmd(TIM3, TIM_DMA_Update, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(TIM3->CCR1);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(SWIN_DMA_DAT_OUT);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel3, ENABLE);		
	
	TIM_Cmd(TIM3,ENABLE);
	

	
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;  //TIM4 dma中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	
}


void SWIM_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3, ENABLE);	
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
	SWIN_OUT_Timer_CN0=SystemCoreClock/1000/1000/SWIM_MHz*(cnt0)-1;//计算出发送0的时间
	SWIN_OUT_Timer_CN1=SystemCoreClock/1000/1000/SWIM_MHz*cnt1-1;//计算出发送1的时间
	SWIM_OUT_TIMER_SetCycle(SWIN_OUT_Timer_ARR);
	SWIM_PULSE_Threshold=SWIN_OUT_Timer_ARR/2;
}

void SWIM_Set_Low_Speed(void)
{
// Set swim speed   low speed	
	SWIM_Set_Speed(8,20,2);
}

void SWIM_Set_High_Speed(void)
{
// Set swim speed   high speed	
	SWIM_Set_Speed(8,8,2);	
}



uint8_t SWIM_HW_Out(uint8_t cmd, uint8_t bitlen, uint16_t retry_cnt)
{
	int8_t i, p;
	uint32_t dly;
	uint16_t *ptr = SWIN_DMA_DAT_OUT;
	
	//uint16_t test[32];

retry:

	//SYNCSWPWM_IN_TIMER_RISE_DMA_INIT(bitlen + 3, SWIM_DMA_IN_Buffer);
	//SWIN_DMA_DAT_IN_ENABLE(bitlen + 3);
	
	SWIN_DMA_DAT_IN_ENABLE(bitlen + 3 +10);//吧下一个位的内容也收下来
	
	*ptr++ = SWIN_OUT_Timer_CN0;

	p = 0;
	for (i = bitlen - 1; i>=0; i--)
	{
		if ((cmd >> i) & 1)
		{
			*ptr++ = SWIN_OUT_Timer_CN1;
			p++;
		}
		else
		{
			*ptr++ = SWIN_OUT_Timer_CN0;
		}
	}
	// parity bit
	if (p & 1)
	{
		*ptr++ = SWIN_OUT_Timer_CN1;
	}
	else
	{
		*ptr++ = SWIN_OUT_Timer_CN0;
	}
	// wait for last waveform -- parity bit
	// 最后一个位为写pwm占空比为0
	*ptr++ = 0;
	SWIM_OUT_TIMER_DMA_INIT(bitlen + 3, SWIN_DMA_DAT_OUT);
	SWIM_OUT_TIMER_DMA_WAIT();

	dly = SWIM_MAX_DLY;
	SWIM_IN_TIMER_RISE_DMA_WAIT_WITH_NUM(dly,10);//余下10个量下次
	SWIM_ROTF_READ_DMA_NUM_NEXT_TIME=bitlen + 3;	
	//SWIM_IN_TIMER_RISE_DMA_WAIT(dly);
	//for(i=0;i<bitlen + 3;i++)test[i]=SWIN_DMA_DAT_IN[i];
	//SWIM_IN_TIMER_RISE_DMA_INIT(10, SWIN_DMA_DAT_IN + 1);
	//SWIM_IN_TIMER_RISE_DMA_INIT(9, SWIN_DMA_DAT_IN + 1);
	//printf("0x%X %d %d %d %d %d %d %d %d %d %d %d  \r\n",cmd,test[0],test[1],test[2],test[3],test[4],test[5],test[6],test[7],test[8],test[9],test[10]);
	

	if (!dly)
	{
		// timeout
		return 2;
	}
	else if (SWIN_DMA_DAT_IN[bitlen + 2] > SWIM_PULSE_Threshold)		 //判断最后一个ACK应答是否为"1",小于半个周期的低电平在前
	{
		// nack	
		if (retry_cnt)
		{
			retry_cnt--;
			goto retry;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 0;
	}
}


uint8_t SWIM_HW_In(uint8_t* data, uint8_t bitlen)
{
	uint8_t ret = 0;
	uint32_t dly;
//	uint16_t test[32],i;

	dly = SWIM_MAX_DLY;
	SWIM_IN_TIMER_RISE_DMA_WAIT(dly);									//先接收引导bit,目标到主机，这个位必须是1
	*data = 0;
	if (dly && (SWIN_DMA_DAT_IN[SWIM_ROTF_READ_DMA_NUM_NEXT_TIME] < SWIM_PULSE_Threshold))				//如果=1,低电平时间小于4个脉冲
	{
		for (dly = 0; dly < 8; dly++)
		{
			if (SWIN_DMA_DAT_IN[SWIM_ROTF_READ_DMA_NUM_NEXT_TIME +1 + dly] < SWIM_PULSE_Threshold)							
			{
				*data |= 1 << (7 - dly);
			}
		}
//	if (dly )				//如果=1,低电平时间小于4个脉冲
//	{
//		for (dly = 0; dly < 8; dly++)
//		{
//			if (SWIN_DMA_DAT_IN[1 + dly] < SWIM_PULSE_Threshold)							
//			{
//				*data |= 1 << (7 - dly);
//			}
//		}	
		//for(i=0;i<10;i++)test[i]=SWIN_DMA_DAT_IN[i];
		SWIM_IN_TIMER_RISE_DMA_INIT(11, SWIN_DMA_DAT_IN);
		SWIM_ROTF_READ_DMA_NUM_NEXT_TIME=1;

		SWIN_DMA_DAT_OUT[0] = SWIN_OUT_Timer_CN1;
		SWIN_DMA_DAT_OUT[1] = 0;
		SWIM_OUT_TIMER_DMA_INIT(2, SWIN_DMA_DAT_OUT);
		SWIM_OUT_TIMER_DMA_WAIT();
		//printf(" %d %d %d %d %d %d %d %d %d %d %d  \r\n",test[0],test[1],test[2],test[3],test[4],test[5],test[6],test[7],test[8],test[9],test[10]);
	
	}
	else
	{
//		if(dly==0)printf("r7-1");
//		if((SWIN_DMA_DAT_IN[SWIM_ROTF_READ_DMA_NUM_NEXT_TIME] > SWIM_PULSE_Threshold))printf("r7-2  %d  %d\r\n",SWIM_ROTF_READ_DMA_NUM_NEXT_TIME,SWIN_DMA_DAT_IN[SWIM_ROTF_READ_DMA_NUM_NEXT_TIME]);
//		for(i=0;i<10;i++)test[i]=SWIN_DMA_DAT_IN[i];
//		printf(" %d %d %d %d %d %d %d %d %d %d %d  \r\n",test[0],test[1],test[2],test[3],test[4],test[5],test[6],test[7],test[8],test[9],test[10]);
		
		ret = 1;
	}

	return ret;
}


uint8_t SWIM_SRST(void)
{
	uint8_t i;
	i=SWIM_HW_Out(SWIM_CMD_SRST, SWIM_CMD_BITLEN, SWIM_MAX_RESEND_CNT);
	if(SWIM_RST_IN)
	{
		delay_us(20);	
		SWIM_RST_LOW;
		delay_ms(1);	
		SWIM_RST_HIGH;		
	}
	return i;
}


uint8_t SWIM_WOTF(uint32_t addr, uint16_t len, uint8_t *data)
{
	uint16_t processed_len;
	uint8_t cur_len, i;
	uint32_t cur_addr, addr_tmp;
	u8 rtv2;

	if ((0 == len) || ((uint8_t*)0 == data))
	{
		return 1;
	}

	processed_len = 0;
	cur_addr = addr;
	while (processed_len < len)
	{
		if ((len - processed_len) > 255)
		{
			cur_len = 255;
		}
		else
		{
			cur_len = len - processed_len;
		}

		SET_LE_U32(&addr_tmp, cur_addr);

		if(SWIM_HW_Out(SWIM_CMD_WOTF, SWIM_CMD_BITLEN, SWIM_MAX_RESEND_CNT))
		{
			return 1;
		}
		if (SWIM_HW_Out(cur_len, 8, 0))
		{
			return 2;
		}
		rtv2=SWIM_HW_Out((addr_tmp >> 16) & 0xFF, 8, 0);	 //retry=0,出错后不重发
		if (rtv2)
		{
			return 3;
		}
		if (SWIM_HW_Out((addr_tmp >> 8) & 0xFF, 8, 0))
		{
			return 4;
		}
		if (SWIM_HW_Out((addr_tmp >> 0) & 0xFF, 8, 0))
		{
			return 5;
		}
		for (i = 0; i < cur_len; i++)
		{
			if (SWIM_HW_Out(data[processed_len + i], 8, SWIM_MAX_RESEND_CNT))
			{
				return 6;
			}
		}

		cur_addr += cur_len;
		processed_len += cur_len;
	}

	return 0;
}




uint8_t SWIM_WOTF_LONG_DAT_Time_Wheel(uint32_t addr, uint16_t len, uint8_t *data,uint16_t *Sent_num,void (*pf)(void))//地址，长度，要传数据指针，已经传输的数据量指针，命令解析回掉函数
{
	uint16_t processed_len;
	uint8_t cur_len, i;
	uint32_t cur_addr, addr_tmp;
	u8 rtv2;

	if ((0 == len) || ((uint8_t*)0 == data))
	{
		return 4;
	}

	processed_len = 0;
	cur_addr = addr;
	while (processed_len < len)
	{
		if ((len - processed_len) > 255)
		{
			cur_len = 255;
		}
		else
		{
			cur_len = len - processed_len;
		}

		SET_LE_U32(&addr_tmp, cur_addr);

		if(SWIM_HW_Out(SWIM_CMD_WOTF, SWIM_CMD_BITLEN, SWIM_MAX_RESEND_CNT))
		{
			return 4;
		}
		if (SWIM_HW_Out(cur_len, 8, 0))
		{
			return 4;
		}
		rtv2=SWIM_HW_Out((addr_tmp >> 16) & 0xFF, 8, 0);	 //retry=0,出错后不重发
		if (rtv2)
		{
			return 4;
		}
		if (SWIM_HW_Out((addr_tmp >> 8) & 0xFF, 8, 0))
		{
			return 4;
		}
		if (SWIM_HW_Out((addr_tmp >> 0) & 0xFF, 8, 0))
		{
			return 4;
		}
		for (i = 0; i < cur_len; i++)
		{
			rtv2=SWIM_HW_Out(data[processed_len + i], 8, SWIM_MAX_RESEND_CNT);
			*Sent_num=processed_len+i;
			if (rtv2)
			{
				return 4;
			}
			
		}
		pf();
		cur_addr += cur_len;
		processed_len += cur_len;
	}

	return 0;
}


/*uint8_t SWIM_ROTF(uint32_t addr, uint16_t len, uint8_t *data)
{
	uint16_t processed_len;
	uint8_t cur_len, i;
	uint32_t cur_addr, addr_tmp;

	if ((0 == len) || ((uint8_t*)0 == data))
	{
		return 7;
	}

	processed_len = 0;
	cur_addr = addr;
	while (processed_len < len)
	{
		if ((len - processed_len) > 255)
		{
			cur_len = 255;
		}
		else
		{
			cur_len = len - processed_len;
		}

		SET_LE_U32(&addr_tmp, cur_addr);

		if(SWIM_HW_Out(SWIM_CMD_ROTF, SWIM_CMD_BITLEN, SWIM_MAX_RESEND_CNT))
		{
			return 6;
		}
		if (SWIM_HW_Out(cur_len, 8, 0))
		{
			return 5;
		}
		if (SWIM_HW_Out((addr_tmp >> 16) & 0xFF, 8, 0))
		{
			return 4;
		}
		if (SWIM_HW_Out((addr_tmp >> 8) & 0xFF, 8, 0))
		{
			return 3;
		}
		if (SWIM_HW_Out((addr_tmp >> 0) & 0xFF, 8, 0))
		{
			return 2;
		}
		for (i = 0; i < cur_len; i++)
		{
			if (SWIM_HW_In(&data[processed_len + i], 8))
			{
				return 1;
			}
		}

		cur_addr += cur_len;
		processed_len += cur_len;
	}
	return 0;
}*/


uint8_t SWIM_ROTF_LONG_DAT_Time_Wheel(uint32_t addr, uint16_t len, uint8_t *data,uint16_t *Sent_num,void (*pf)(void))//地址，长度，要传数据指针，已经传输的数据量指针，命令解析回掉函数
{
	uint16_t processed_len;
	uint8_t cur_len, i;
	uint32_t cur_addr, addr_tmp;
	u8 rtv2;

	if ((0 == len) || ((uint8_t*)0 == data))
	{
		//printf("r1");
		return 4;
	}

	processed_len = 0;
	cur_addr = addr;
	while (processed_len < len)
	{
		if ((len - processed_len) > 255)
		{
			cur_len = 255;
		}
		else
		{
			cur_len = len - processed_len;
		}

		SET_LE_U32(&addr_tmp, cur_addr);
		__disable_irq();
		if(SWIM_HW_Out(SWIM_CMD_ROTF, SWIM_CMD_BITLEN, SWIM_MAX_RESEND_CNT))
		{
			printf("r2");
			__enable_irq();
			return 4;
		}

		if (SWIM_HW_Out(cur_len, 8, 0))
		{
			__enable_irq();
			printf("r3");
			return 4;
		}
		if (SWIM_HW_Out((addr_tmp >> 16) & 0xFF, 8, 0))
		{
			__enable_irq();
			printf("r4");
			return 4;
		}
		if (SWIM_HW_Out((addr_tmp >> 8) & 0xFF, 8, 0))
		{
			__enable_irq();
			printf("r5");
			return 4;
		}
		if (SWIM_HW_Out((addr_tmp >> 0) & 0xFF, 8, 0))
		{
			__enable_irq();
			printf("r6");
			return 4;
		}
		__enable_irq();
		for (i = 0; i < cur_len; i++)
		{
			rtv2=SWIM_HW_In(&data[processed_len + i], 8);
			*Sent_num=processed_len+i;
			if (rtv2)
			{
				//__enable_irq();
				printf("r7 %d %d\r\n",*Sent_num,i);
				return 4;
			}
		
		}
		//__enable_irq();

		cur_addr += cur_len;
		processed_len += cur_len;
		pf();
	}
	return 0;
}

uint8_t SWIM_EnterProgMode_Time_Wheel(void (*pf)(void))//传入参数  swim命令检测函数
{
	uint8_t i;//,test[32];
	uint32_t dly;
	//uint16_t time_out=0;
	//SWIM_IN_TIMER_RISE_DMA_INIT(10,SWIN_DMA_DAT_IN);	
	//DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	SWIM_OUT_PIN_OD;
	SWIN_DMA_DAT_IN_ENABLE(10);	
	//SWIN_DMA_DAT_IN_DONE=0;	
//1 - LOW SWIM for 16us	(more)
	SWIM_LOW;
	my_delay_us_Part1(500);
	pf();
	my_delay_Part2();
	
	my_delay_us_Part1(500);
	pf();
	my_delay_Part2();
	
	SWIM_HIGH;
//2 - 4 pulses 1kHz and 4 pulses 2kHz	
	for (i = 0; i < 4; i++)
	{
		SWIM_HIGH;
		my_delay_us_Part1(500);
		pf();
		my_delay_Part2();
		SWIM_LOW;
		my_delay_us_Part1(500);
		pf();
		my_delay_Part2();
	}
	
	for (i = 0; i < 4; i++)
	{
		SWIM_HIGH;	
		my_delay_us_Part1(250);
		pf();
		my_delay_Part2();
		SWIM_LOW;
		my_delay_us_Part1(250);
		pf();
		my_delay_Part2();
	}
	if(DMA1->ISR & DMA1_FLAG_TC1)
	{
			DMA_Cmd(DMA1_Channel1, DISABLE);
			SWIM_RST_HIGH;
			SWIM_HIGH;
		printf("e1");
			return 4;//错误	，重复加入时序会造成该错误
		
	}
	SWIM_HIGH;	
//3 - HSI is turned on . wait 128 swim clock synchronization pulse
	dly = SWIM_MAX_DLY;
	SWIM_IN_TIMER_RISE_DMA_WAIT(dly);	
	if(dly==0)
	{
			DMA_Cmd(DMA1_Channel1, DISABLE);
			SWIM_RST_HIGH;
			printf("e2");
			return 4;//超时，无设备响应
	}	
	
//4 - synchronization pulse
	if(SWIN_DMA_DAT_IN[9]<1400&&SWIN_DMA_DAT_IN[9]>900)
	{
		SWIN_CLOCK_128=SWIN_DMA_DAT_IN[9];
	
	}
	else
	{
			DMA_Cmd(DMA1_Channel1, DISABLE);
			SWIM_RST_HIGH;
			printf("e3");
//			for(i=0;i<10;i++)test[i]=SWIN_DMA_DAT_IN[i];
//  		printf(" %d %d %d %d %d %d %d %d %d %d %d  \r\n",test[0],test[1],test[2],test[3],test[4],test[5],test[6],test[7],test[8],test[9],test[10]);
		
			return 4;//128swim时钟err	
	}
	//DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);
	
	SWIM_OUT_PIN_AFOD;
	return 0;
}


/*uint8_t SWIM_EnterProgMode(void)//swim入口序列
{
	uint8_t i;
	uint16_t time_out=0;
	SWIM_RST_HIGH;
	delay_ms(2);
	SWIM_RST_LOW;
	delay_ms(1);
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
	//SWIN_DMA_DAT_IN_DISABLE();//关闭输入
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);
//5 - 300ns delay (more)
	delay_us(350);
	
// Set swim speed   low speed	
	SWIM_Set_Speed(8,20,2);
	
// soft rst
	SWIM_OUT_PIN_AFOD;
	
	if (SWIM_SRST()>0)
	{
		SWIN_DMA_DAT_IN_DISABLE();
		SWIN_DMA_DAT_OUT_DISABLE();
		return 0;
	}
	delay_us(200);
	
//6 - Write 0A0h in the SWIM_CSR	
	WriteBuff[0]=0xA0;
	if (SWIM_WOTF(SWIM_CSR, 1, WriteBuff)>0)
	{
		SWIN_DMA_DAT_IN_DISABLE();
		SWIN_DMA_DAT_OUT_DISABLE();
		return 3;
	}	
	delay_us(200);
//7 - 	Release reset
	SWIM_RST_HIGH;
	delay_ms(20);
	
	


	
	err=SWIM_ROTF(SWIM_CSR, 1, ReadBuff);
	if(err)printf("Derr:%d\r\n",err);
	printf("SWIM_CSR 0x00007F80:0x%X\r\n",ReadBuff[0]);

	err=SWIM_ROTF(DM_CSR2, 1, ReadBuff);
	if(err)printf("Derr:%d\r\n",err);
	printf("DM_CSR2 0x00007F99:0x%X\r\n",ReadBuff[0]);
	
	WriteBuff[0]=0xB0;			
	if (SWIM_WOTF(SWIM_CSR, 1, WriteBuff)>0) //SWIM_CSR,控制寄存器中写入0B1011,0000 换高速
	{
		SWIN_DMA_DAT_IN_DISABLE();
		SWIN_DMA_DAT_OUT_DISABLE();
		return 4;
	}	
// Set swim speed   high speed	
	SWIM_Set_Speed(8,8,2);	
	delay_us(200);
	WriteBuff[0]=0x08;
	if (SWIM_WOTF(DM_CSR2, 1, WriteBuff)>0)  //DM_CSR2,Bit4 STALL =1
	{
		SWIN_DMA_DAT_IN_DISABLE();
		SWIN_DMA_DAT_OUT_DISABLE();
		return 5;
	}	

	err=SWIM_ROTF(SWIM_CSR, 1, ReadBuff);
	if(err)printf("Derr:%d\r\n",err);
	printf("SWIM_CSR 0x00007F80:0x%X\r\n",ReadBuff[0]);

	err=SWIM_ROTF(DM_CSR2, 1, ReadBuff);
	if(err)printf("Derr:%d\r\n",err);
	printf("DM_CSR2 0x00007F99:0x%X\r\n",ReadBuff[0]);	
	
	return 1;
}*/

//void SWIM_CUT_OFF(void)
//{
////	WriteBuff[0]=0x00;
////	SWIM_WOTF(DM_CSR2,1,WriteBuff);

//	WriteBuff[0]=0xB6;
//	SWIM_WOTF(SWIM_CSR,1,WriteBuff);
//	SWIM_SRST();
//	delay_ms(1);
//	SWIM_RST_LOW;
//	delay_ms(1);	
//	SWIM_RST_HIGH;
//}


//void DMA1_Channel1_IRQHandler(void)
//{
//	DMA_ClearFlag(DMA1_FLAG_TC1);	//clear interrupt flag
//	SWIN_DMA_DAT_IN_DONE=1;

//}

//void DMA1_Channel7_IRQHandler(void)
//{
//	DMA_ClearFlag(DMA1_FLAG_TC7);	//clear interrupt flag
//	SWIN_DMA_DAT_OUT_DONE=1;
//	//SWIN_DMA_DAT_OUT_DISABLE();
//}

