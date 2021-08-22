#ifndef _PROTOCOL_H_
#define  _PROTOCOL_H_
#include "stm32f4xx.h"


void BasicProtocolAnalysis(u8 const *data_buf,int _len);
extern u8 send_pid1,send_pid2,send_pid3;
extern u8 checkdata_to_send,checksum_to_send,send_check;
#endif
