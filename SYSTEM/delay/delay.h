#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
	 
void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

void my_delay_ms_Part1(u16 nms);
void my_delay_us_Part1(u16 nus);
void my_delay_Part2(void);

#endif





























