#ifndef _SWIM_H_
#define _SWIM_H_

#include "sys.h"
#include "delay.h"


extern uint16_t SWIN_OUT_Timer_ARR;

///////////////swim_pin/////////////////
//swim
#define GPIO_SWIM_PORT 						GPIOB

#define GPIO_SWIM_OUT_PIN         GPIO_Pin_4		//Tim3_CH1映射到PB.4作为SWIM输出
#define GPIO_SWIM_IN1_PIN					GPIO_Pin_7		//PB7,PB9 根线都是作为SWIM输入
#define GPIO_SWIM_IN2_PIN				  GPIO_Pin_9		//Tim4_CH2 PB7// Tim4_CH4 PB9

#define SWIM_OUT_PIN_OD 					GPIO_SWIM_PORT->CRL &= ~GPIO_CRL_CNF4_1  //CNF位 01为开漏  7个指令周期
#define SWIM_OUT_PIN_AFOD				  GPIO_SWIM_PORT->CRL |= GPIO_CRL_CNF4	 //CNF位 11复用为开漏

#define SWIM_HIGH		  						GPIO_SWIM_PORT->BSRR = GPIO_SWIM_OUT_PIN		 
#define SWIM_LOW   								GPIO_SWIM_PORT->BRR = GPIO_SWIM_OUT_PIN


#define SWIM_IN1  								GPIO_ReadInputDataBit(GPIO_SWIM_PORT, GPIO_SWIM_IN1_PIN)
#define SWIM_IN2	  							GPIO_ReadInputDataBit(GPIO_SWIM_PORT, GPIO_SWIM_IN2_PIN)
#define SWIM_IN1_Q  							GPIO_SWIM_PORT->IDR & GPIO_SWIM_IN1_PIN
#define SWIM_IN2_Q  							GPIO_SWIM_PORT->IDR & GPIO_SWIM_IN2_PIN

#define GPIOB_CRH_Addr    (GPIOB_BASE+0)
#define SWIM_OUT_PIN_SET_OD   BIT_ADDR(GPIOB_CRH_Addr,19)=0//3个指令周

//rst
#define GPIO_SWIM_RST_PORT 				GPIOB
#define GPIO_SWIM_RSTOUT_PIN			GPIO_Pin_6		//复位脚 pb6

#define SWIM_RST_HIGH  						GPIO_SWIM_RST_PORT->BSRR = GPIO_SWIM_RSTOUT_PIN		 
#define SWIM_RST_LOW   						GPIO_SWIM_RST_PORT->BRR = GPIO_SWIM_RSTOUT_PIN	

#define	SWIM_RST_IN								GPIO_ReadInputDataBit(GPIO_SWIM_RST_PORT, GPIO_SWIM_RSTOUT_PIN)



////////////////////////////////////////




///////////////swim_cmd/////////////////
#define SWIM_CMD_SRST					0x00	 //复位
#define SWIM_CMD_ROTF					0x01	 //SWIM 飞速读
#define SWIM_CMD_WOTF					0x02	 //SWIM 飞速写
#define SWIM_CMD_BITLEN					3
#define SWIM_SYNC_CYCLES				128

#define SWIM_MAX_DLY					0xfffff
#define SWIM_MAX_RESEND_CNT		20


#define SWIM_CSR					0x00007F80
#define DM_CSR2						0x00007F99
#define STM8_FLASH_KEY		0x00005062
#define STM8_FLASH_CR2		0x0000505B
#define STM8_FLASH_IAPCSR	0x0000505f
#define STM8_FLASH_EOF		0x04
////////////////////////////////////////


///////////////swim_in timer/////////////////
#define SWIN_DMA_DAT_IN_ENABLE(num){\
DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR1_EN);\
DMA1_Channel1->CNDTR = num;\
DMA1->IFCR = DMA1_FLAG_TC1;\
TIM4->CCER |= (TIM_CCER_CC1E|TIM_CCER_CC2E);\
DMA1_Channel1->CCR |= DMA_CCR1_EN;\
}

#define SWIM_IN_TIMER_RISE_DMA_INIT(num,addr){\
DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR1_EN);\
DMA1_Channel1->CNDTR = num;\
DMA1_Channel1->CMAR = (uint32_t)addr;\
DMA1_Channel1->CCR |= DMA_CCR1_EN;\
}

#define SWIN_DMA_DAT_IN_DISABLE(){\
TIM4->CCER &= (uint16_t)~ (TIM_CCER_CC1E|TIM_CCER_CC2E);\
DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR1_EN);\
}


#define SWIM_IN_TIMER_RISE_DMA_WAIT(dly)	{\
while((!(DMA1->ISR & DMA1_FLAG_TC1)) && --dly);\
DMA1->IFCR = DMA1_FLAG_TC1;\
}

#define SWIM_IN_TIMER_RISE_DMA_WAIT_WITH_NUM(dly,num)	{\
while((DMA1_Channel1->CNDTR>num) && --dly);\
}
////////////////////////////////////////
//TIM3->CNT=SWIN_OUT_Timer_ARR-10;\
///////////////swim_out timer/////////////////
#define SWIM_OUT_TIMER_SetCycle(cycle)	{\
TIM3->ARR = (cycle);\
TIM3->EGR = TIM_PSCReloadMode_Immediate;\
} 

#define SWIM_OUT_TIMER_DMA_INIT(num, addr)	{\
DMA1_Channel6->CCR &= ~DMA_CCR1_EN;\
DMA1_Channel6->CNDTR = (num);\
DMA1_Channel6->CMAR = (uint32_t)(addr);\
TIM3->EGR = TIM_PSCReloadMode_Immediate;\
DMA1_Channel6->CCR |= DMA_CCR1_EN;\
TIM3->CCMR1|=TIM_CCMR1_OC1PE;\
}

#define SWIM_OUT_TIMER_DMA_WAIT()	{\
while(!(DMA1->ISR & DMA1_FLAG_TC6));\
DMA1->IFCR = DMA1_FLAG_TC6;\
TIM3->CCMR1&= (uint16_t)~((uint16_t)TIM_CCMR1_OC1PE);\
}
//TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

#define SWIN_DMA_DAT_OUT_DISABLE(){\
TIM3->CCER &= (uint16_t)~ (TIM_CCER_CC1E|TIM_CCER_CC2E);\
DMA1_Channel6->CCR &= (uint16_t)(~DMA_CCR1_EN);\
SWIM_OUT_PIN_OD;\
SWIM_HIGH;\
SWIM_RST_HIGH;\
}
////////////////////////////////////////



///////////////dma/////////////////
#define SWIN_DMA_DAT_IN_BUF_SIZE 32
#define SWIN_DMA_DAT_OUT_BUF_SIZE 32

////////////////////////////////////////


#define SET_U16_MSBFIRST(p, v)		\
	do{\
		*((uint8_t *)(p) + 0) = (((uint16_t)(v)) >> 8) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint16_t)(v)) >> 0) & 0xFF;\
	} while (0)
#define SET_U24_MSBFIRST(p, v)		\
	do{\
		*((uint8_t *)(p) + 0) = (((uint32_t)(v)) >> 16) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint32_t)(v)) >> 8) & 0xFF;\
		*((uint8_t *)(p) + 2) = (((uint32_t)(v)) >> 0) & 0xFF;\
	} while (0)
#define SET_U32_MSBFIRST(p, v)		\
	do{\
		*((uint8_t *)(p) + 0) = (((uint32_t)(v)) >> 24) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint32_t)(v)) >> 16) & 0xFF;\
		*((uint8_t *)(p) + 2) = (((uint32_t)(v)) >> 8) & 0xFF;\
		*((uint8_t *)(p) + 3) = (((uint32_t)(v)) >> 0) & 0xFF;\
	} while (0)
#define SET_U16_LSBFIRST(p, v)		\
	do{\
		*((uint8_t *)(p) + 0) = (((uint16_t)(v)) >> 0) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint16_t)(v)) >> 8) & 0xFF;\
	} while (0)
#define SET_U24_LSBFIRST(p, v)		\
	do{\
		*((uint8_t *)(p) + 0) = (((uint32_t)(v)) >> 0) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint32_t)(v)) >> 8) & 0xFF;\
		*((uint8_t *)(p) + 2) = (((uint32_t)(v)) >> 16) & 0xFF;\
	} while (0)
#define SET_U32_LSBFIRST(p, v)		\
	do{\
		*((uint8_t *)(p) + 0) = (((uint32_t)(v)) >> 0) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint32_t)(v)) >> 8) & 0xFF;\
		*((uint8_t *)(p) + 2) = (((uint32_t)(v)) >> 16) & 0xFF;\
		*((uint8_t *)(p) + 3) = (((uint32_t)(v)) >> 24) & 0xFF;\
	} while (0)

#define GET_LE_U16(p)					GET_U16_LSBFIRST(p)
#define GET_LE_U24(p)					GET_U24_LSBFIRST(p)
#define GET_LE_U32(p)					GET_U32_LSBFIRST(p)
#define GET_BE_U16(p)					GET_U16_MSBFIRST(p)
#define GET_BE_U24(p)					GET_U24_MSBFIRST(p)
#define GET_BE_U32(p)					GET_U32_MSBFIRST(p)
#define SET_LE_U16(p, v)			SET_U16_LSBFIRST(p, v)
#define SET_LE_U24(p, v)			SET_U24_LSBFIRST(p, v)
#define SET_LE_U32(p, v)			SET_U32_LSBFIRST(p, v)
#define SET_BE_U16(p, v)			SET_U16_MSBFIRST(p, v)
#define SET_BE_U24(p, v)			SET_U24_MSBFIRST(p, v)
#define SET_BE_U32(p, v)			SET_U32_MSBFIRST(p, v)




////////
void SWIM_Init(void);
uint8_t SWIM_EnterProgMode_Time_Wheel(void (*pf)(void));	
void SWIM_Set_Low_Speed(void);	
void SWIM_Set_High_Speed(void);		
uint8_t SWIM_SRST(void);	
uint8_t SWIM_WOTF(uint32_t addr, uint16_t len, uint8_t *data);
uint8_t SWIM_WOTF_LONG_DAT_Time_Wheel(uint32_t addr, uint16_t len, uint8_t *data,uint16_t *Sent_num,void (*pf)(void));
uint8_t SWIM_ROTF_LONG_DAT_Time_Wheel(uint32_t addr, uint16_t len, uint8_t *data,uint16_t *Sent_num,void (*pf)(void));

////////
void SWIM_GPIO_Init(void);
uint8_t SWIM_EnterProgMode(void);
void SWIM_CUT_OFF(void);



////////
#endif
