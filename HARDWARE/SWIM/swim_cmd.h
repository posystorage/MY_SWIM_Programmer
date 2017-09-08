#ifndef _SWIM_CMD_H_
#define _SWIM_CMD_H_
#include "sys.h"
#include "swim.h"

void GET_USB_CMD_Data(uint8_t bEpAddr,uint8_t bEpNum);
void SEND_Data_To_USB(uint8_t *DAT_Addr,uint8_t DAT_Num);
void SWIM_Process_USB_CMD(void);
void SWIM_Service(void);







#endif





