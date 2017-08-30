#ifndef _SWIM_H_
#define _SWIM_H_

#include "sys.h"
#include "delay.h"


extern uint16_t SWIN_OUT_Timer_ARR;

///////////////swim_pin/////////////////
//swim
#define GPIO_SWIM_PORT 						GPIOB

#define GPIO_SWIM_OUT_PIN         GPIO_Pin_8		//Tim4_CH3映射到PB.8作为SWIM输出
#define GPIO_SWIM_IN1_PIN					GPIO_Pin_7		//PB7,PB9 根线都是作为SWIM输入
#define GPIO_SWIM_IN2_PIN				  GPIO_Pin_9		//Tim4_CH2 PB7// Tim4_CH4 PB9

#define SWIM_OUT_PIN_OD 					GPIO_SWIM_PORT->CRH &= ~GPIO_CRH_CNF8_1  //CNF位 01为开漏  7个指令周期
#define SWIM_OUT_PIN_AFOD				  GPIO_SWIM_PORT->CRH |= GPIO_CRH_CNF8	 //CNF位 11复用为开漏

#define SWIM_HIGH		  						GPIO_SWIM_PORT->BSRR = GPIO_SWIM_OUT_PIN		 
#define SWIM_LOW   								GPIO_SWIM_PORT->BRR = GPIO_SWIM_OUT_PIN


#define SWIM_IN1  								GPIO_ReadInputDataBit(GPIO_SWIM_PORT, GPIO_SWIM_IN1_PIN)
#define SWIM_IN2	  							GPIO_ReadInputDataBit(GPIO_SWIM_PORT, GPIO_SWIM_IN2_PIN)
#define SWIM_IN1_Q  							GPIO_SWIM_PORT->IDR & GPIO_SWIM_IN1_PIN
#define SWIM_IN2_Q  							GPIO_SWIM_PORT->IDR & GPIO_SWIM_IN2_PIN

#define GPIOB_CRH_Addr    (GPIOB_BASE+4)
#define SWIM_OUT_PIN_SET_OD   BIT_ADDR(GPIOB_CRH_Addr,3)=0//3个指令周

//rst
#define GPIO_SWIM_RST_PORT 				GPIOB
#define GPIO_SWIM_RSTOUT_PIN			GPIO_Pin_6		//复位脚 pb6

#define SWIM_RST_HIGH  						GPIO_SWIM_RST_PORT->BSRR = GPIO_SWIM_RSTOUT_PIN		 
#define SWIM_RST_LOW   						GPIO_SWIM_RST_PORT->BRR = GPIO_SWIM_RSTOUT_PIN	





////////////////////////////////////////




///////////////swim_cmd/////////////////
#define SWIM_CMD_SRST					0x00	 //复位
#define SWIM_CMD_ROTF					0x01	 //SWIM 飞速读
#define SWIM_CMD_WOTF					0x02	 //SWIM 飞速写

#define SWIM_SYNC_CYCLES				128


////////////////////////////////////////


///////////////swim_in timer/////////////////
#define SWIN_DMA_DAT_IN_ENABLE(num){\
DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR1_EN);\
DMA1_Channel1->CNDTR = num;\
TIM4->CCER |= (TIM_CCER_CC1E|TIM_CCER_CC2E);\
DMA1_Channel1->CCR |= DMA_CCR1_EN;\
}

#define SWIN_DMA_DAT_IN_DISABLE(){\
TIM4->CCER &= (uint16_t)~ (TIM_CCER_CC1E|TIM_CCER_CC2E);\
DMA1_Channel1->CCR &= (uint16_t)(~DMA_CCR1_EN);\
}
////////////////////////////////////////

///////////////swim_out timer/////////////////

//#define SWIN_DMA_DAT_OUT_ENABLE(num){\
//TIM4->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));\
//SWIM_OUT_PIN_AFOD;\
//TIM4->CCER |= TIM_CCER_CC3E;\
//TIM4->ARR=SWIN_OUT_Timer_ARR;\
//DMA1_Channel7->CCR &= (uint16_t)(~DMA_CCR1_EN);\
//DMA1_Channel7->CNDTR = num;\
//DMA1_Channel7->CCR |= DMA_CCR1_EN;\
//TIM4->CCR3=0xb3;\
//TIM4->DIER |= TIM_DMA_Update;\
//TIM4->CNT |= 0;\
//TIM4->CR1 |= TIM_CR1_CEN;\
//}

//#define SWIN_DMA_DAT_OUT_DISABLE(){\
//SWIM_OUT_PIN_OD;\
//TIM4->CCER &= (uint16_t)~ TIM_CCER_CC3E;\
//DMA1_Channel7->CCR &= (uint16_t)(~DMA_CCR1_EN);\
//TIM4->ARR=0xffff;\
//TIM4->DIER &= ~TIM_DMA_Update;\
//}
#define WAIT_SWIN_DAT_OUT_DONE(){\
	while(((TIM4->SR)&TIM_SR_CC3IF)==0);\
	TIM4->SR&=~TIM_SR_CC3IF;\
}

#define SWIN_DAT_OUT_ENABLE(dat1){\
	TIM4->CR1 &= ~((uint16_t)TIM_CR1_CEN);\
	SWIM_OUT_PIN_AFOD;\
	TIM4->CCER |= TIM_CCER_CC3E;\
	TIM4->ARR=SWIN_OUT_Timer_ARR;\
	TIM4->CCR3=dat1;\
	TIM4->CNT |= 0;\
	TIM4->SR&=~TIM_SR_CC3IF;\
	TIM4->CR1 |= TIM_CR1_CEN;\
}

#define SWIN_DAT_OUT_DAT(datx){\
WAIT_SWIN_DAT_OUT_DONE();\
TIM4->CCR3=datx;\
}

#define SWIN_DAT_OUT_DISABLE(){\
while(((TIM4->SR)&TIM_SR_CC3IF)==0);\
SWIM_OUT_PIN_SET_OD;\
TIM4->CCER &= (uint16_t)~ TIM_CCER_CC3E;\
TIM4->ARR=0xffff;\
}

////////////////////////////////////////
//WAIT_SWIN_DAT_OUT_DONE();\
__nop();\


///////////////dma/////////////////
#define SWIN_DMA_DAT_IN_BUF_SIZE 32
#define SWIN_DMA_DAT_OUT_BUF_SIZE 32

////////////////////////////////////////


////////
void SWIM_Init(void);
uint8_t SWIM_EnterProgMode(void);



////////
void SWIM_GPIO_Init(void);




////////
#endif
