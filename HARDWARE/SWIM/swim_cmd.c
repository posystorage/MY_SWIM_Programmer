#include "swim_cmd.h"
#include "usb_lib.h"
#include "SEGGER_SYSVIEW.h"

uint8_t USB_Rx_Buffer[64];
uint8_t USB_RX_DAT=0,//标记已经收到数据（但是上一个命令还处理完）
USB_RX_CMD_BUSY=0,//标记为1则表示上一个命令还没处理完成，0则完成
LONG_CMD_BUSY=0;//长命令（即超过64字节（一个usb包）的长命令的标记，一般为写一堆数据）
uint16_t USB_Rx_Cnt; //接收长度
uint8_t CMD_READY=0;//用于回复09指令的命令就绪标志位 如果是0 则就绪，1为非就绪 2为带百分百非就绪  4为错误
uint16_t CMD_READY_num=0;//用于回复09指令的命令就绪标志位 当CMD_READY为2的时候，非就绪传输进度记录(已传输的量)

uint8_t SWIM_EnterProgMode_EN=0;//进入时序使能 如果为1则执行进入时序
uint8_t SWIM_WOTF_DAT_EN=0;//写长内容使能

uint8_t WOTF_DAT_Buffer[1024];
uint16_t WOTF_DAT_num=0;
uint16_t WOTF_DAT_num_left=0;//剩下需要传输的量
uint32_t WOTF_DAT_ADDR=0;



uint8_t USB_Tx_Buffer[64];
uint8_t ROTF_DAT_Buffer[6*1024];//每次最多6K的读取量
uint8_t SWIM_ROTF_DAT_EN=0;//读长内容使能


uint16_t ROTF_DAT_num=0;
uint16_t ROTF_DAT_num_left=0;//剩下需要传输的量
uint32_t ROTF_DAT_ADDR=0;

void GET_USB_CMD_Data(uint8_t bEpAddr,uint8_t bEpNum)//断点回掉函数
{
uint16_t i;
	if(USB_RX_CMD_BUSY)//上一个命令还没有被处理
	{
		USB_RX_DAT=1;
	}
	else
	{
		USB_Rx_Cnt = USB_SIL_Read(bEpAddr, USB_Rx_Buffer);	//得到USB接收到的数据及其长度  							
		SetEPRxValid(bEpNum);//使能端点的数据接收
		USB_RX_CMD_BUSY=1;
		USB_RX_DAT=0;
		if(LONG_CMD_BUSY)//一次传不完
		{

			if(WOTF_DAT_num_left>USB_Rx_Cnt)
			{
				for(i=0;i<USB_Rx_Cnt;i++)
				{
					WOTF_DAT_Buffer[WOTF_DAT_num-WOTF_DAT_num_left]=USB_Rx_Buffer[i];		
					WOTF_DAT_num_left--;								
				}						
			}
			else
			{
				for(i=0;i<USB_Rx_Cnt;i++)
				{
					WOTF_DAT_Buffer[WOTF_DAT_num-WOTF_DAT_num_left]=USB_Rx_Buffer[i];		
					WOTF_DAT_num_left--;								
				}							
				LONG_CMD_BUSY=0;
				SWIM_WOTF_DAT_EN=1;//usb接收完成，开始写往设备
			}
			USB_RX_CMD_BUSY=0;
		}		
		
	}
}

void SWIM_Process_USB_CMD(void)
{
	uint16_t i;
	if(USB_RX_CMD_BUSY)//如果有命令
	{
		if(LONG_CMD_BUSY==0)//一次能传完
		{
			if(USB_Rx_Buffer[0]==0xf4)
			{
				switch(USB_Rx_Buffer[1])
				{
					case 0x02:
					{
						USB_Tx_Buffer[0]=0x00;
						USB_Tx_Buffer[1]=0x01;
						USB_Tx_Buffer[2]=0x02;
						USB_Tx_Buffer[3]=0x07;
						USB_Tx_Buffer[4]=0x00;
						USB_Tx_Buffer[5]=0x00;
						USB_Tx_Buffer[6]=0x00;
						USB_Tx_Buffer[7]=0x00;
						SEND_Data_To_USB(USB_Tx_Buffer,8);						
						break;
					}					
					case 0x03:
					{	
						if(USB_Rx_Buffer[2]==0x00)
						{
							SWIM_Set_Low_Speed();
						}
						if(USB_Rx_Buffer[2]==0x01&&USB_Rx_Buffer[3]==0x01)
						{
							SWIM_Set_High_Speed();
						}
						CMD_READY=0;						
						break;
					}
					case 0x04:
					{	
						SWIM_EnterProgMode_EN=1;
						CMD_READY=1;						
						break;
					}
					case 0x05:
					{	
						SWIM_SRST();
						CMD_READY=0;						
						break;
					}
					case 0x06:
					{	
						CMD_READY=SWIM_Communication_Reset();					
						break;
					}
					case 0x07:
					{	
						SWIM_RST_LOW;
						CMD_READY=0;						
						break;
					}
					case 0x08:
					{
						SWIM_RST_HIGH;
						CMD_READY=0;						
						break;
					}
					case 0x09:
					{
						if(CMD_READY==0||CMD_READY==1||CMD_READY==4)	
						{
							SEGGER_SYSVIEW_Print("9-1");
							USB_Tx_Buffer[0]=CMD_READY;
							USB_Tx_Buffer[1]=0x00;
							USB_Tx_Buffer[2]=0x00;
							USB_Tx_Buffer[3]=0x00;
							SEND_Data_To_USB(USB_Tx_Buffer,4);														
						}
						else if(CMD_READY==2)	
						{
							USB_Tx_Buffer[0]=0x01;
							USB_Tx_Buffer[3]=0x00;
							if(CMD_READY_num<0x100)
							{
								USB_Tx_Buffer[1]=CMD_READY_num;
								USB_Tx_Buffer[2]=0x00;												
							}
							else
							{
								USB_Tx_Buffer[1]=CMD_READY_num&0x00ff;
								USB_Tx_Buffer[2]=(CMD_READY_num>>8)&0x00ff;								
							}
							SEGGER_SYSVIEW_Print("9-2");
							SEND_Data_To_USB(USB_Tx_Buffer,4);								
						}							
						break;
					}
					case 0x0a://写
					{
						CMD_READY=1;	
						WOTF_DAT_num=(USB_Rx_Buffer[3]&0x00ff)|((USB_Rx_Buffer[2]<<8)&0xff00);
						WOTF_DAT_ADDR=(USB_Rx_Buffer[7]&0x000000ff)|((USB_Rx_Buffer[6]<<8)&0x0000ff00)|((USB_Rx_Buffer[5]<<16)&0x00ff0000);
						if(WOTF_DAT_num>8)
						{
							LONG_CMD_BUSY=1;
							WOTF_DAT_num_left=WOTF_DAT_num;
							for(i=0;i<8;i++)
							{
								WOTF_DAT_Buffer[WOTF_DAT_num-WOTF_DAT_num_left]=USB_Rx_Buffer[8+i];		
								WOTF_DAT_num_left--;								
							}
							CMD_READY=2;
						}
						else
						{							
							SWIM_WOTF(WOTF_DAT_ADDR, WOTF_DAT_num, &USB_Rx_Buffer[8]);
							CMD_READY=0;
						}							
							
						break;
					}
					case 0x0b://读
					{
						ROTF_DAT_num=(USB_Rx_Buffer[3]&0x00ff)|((USB_Rx_Buffer[2]<<8)&0xff00);
						ROTF_DAT_ADDR=(USB_Rx_Buffer[7]&0x000000ff)|((USB_Rx_Buffer[6]<<8)&0x0000ff00)|((USB_Rx_Buffer[5]<<16)&0x00ff0000);
						ROTF_DAT_num_left=ROTF_DAT_num;
						SWIM_ROTF_DAT_EN=1;
						CMD_READY=2;							
						break;
					}	
					case 0x0c://读 数据回传
					{
						while(ROTF_DAT_num_left)
						{
							if(ROTF_DAT_num_left>64)
							{								
								SEND_Data_To_USB(&ROTF_DAT_Buffer[ROTF_DAT_num-ROTF_DAT_num_left],64);
								ROTF_DAT_num_left-=64;
							}
							else
							{
								SEND_Data_To_USB(&ROTF_DAT_Buffer[ROTF_DAT_num-ROTF_DAT_num_left],ROTF_DAT_num_left);
								ROTF_DAT_num_left=0;
							}					
						}							
						break;
					}	
					case 0x0d:
					{
						USB_Tx_Buffer[0]=0x00;
						USB_Tx_Buffer[1]=0x18;
						SEND_Data_To_USB(USB_Tx_Buffer,2);														
						break;
					}					
					default: break;
				}		
			}
			else if(USB_Rx_Buffer[0]==0xf5)
			{
				if(USB_Rx_Buffer[1]==0x00)
				{
					USB_Tx_Buffer[0]=0x00;
					USB_Tx_Buffer[1]=0x01;
					SEND_Data_To_USB(USB_Tx_Buffer,2);
				}
			}
			USB_RX_CMD_BUSY=0;
		}

	}
}


void SWIM_Service(void)
{
	SWIM_Process_USB_CMD();
	if(SWIM_EnterProgMode_EN==1)
	{
		CMD_READY=SWIM_EnterProgMode_Time_Wheel(SWIM_Process_USB_CMD);//在进入时序里面夹杂检测命令	
		SWIM_EnterProgMode_EN=0;
	}

	if(SWIM_WOTF_DAT_EN==1)
	{
		CMD_READY_num=0;	
		CMD_READY=SWIM_WOTF_LONG_DAT_Time_Wheel(WOTF_DAT_ADDR, WOTF_DAT_num, WOTF_DAT_Buffer, &CMD_READY_num,SWIM_Process_USB_CMD);//在写时序里面夹杂检测命令	(主要是检测09命令)
		SWIM_WOTF_DAT_EN=0;
	}
	
	if(SWIM_ROTF_DAT_EN==1)
	{
		CMD_READY_num=0;	
		CMD_READY=SWIM_ROTF_LONG_DAT_Time_Wheel(ROTF_DAT_ADDR, ROTF_DAT_num, ROTF_DAT_Buffer, &CMD_READY_num,SWIM_Process_USB_CMD);//在读时序里面夹杂检测命令	(主要是检测09命令)
		SWIM_ROTF_DAT_EN=0;
	}
		
	if(USB_RX_DAT==1)GET_USB_CMD_Data(EP2_OUT,ENDP2);
	
}




