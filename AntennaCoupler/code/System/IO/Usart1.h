
#pragma once



/*!
 * Sizes of USART1 Rx and Tx buffer are defined here
 */
//#define USART1_RXBUFFER_SIZE 128
//#define USART1_TXBUFFER_SIZE 3072
#define USART1_RXBUFFER_SIZE 128
#define USART1_RXBUFFER_WRAP_MASK 127
#define USART1_TXBUFFER_SIZE 2048
#define USART1_TXBUFFER_WRAP_MASK 2047


#include "types.h"


void Usart1_Setup(void);


uint16	Usart1_GetRxBufferCount();
uint8 	Usart1_GetRxNextByte(void);
char 	Usart1_PeekRxNextChar(void);
bool	Usart1_TryReadLine(char* targetbuf, int targetBufLen, int* outLineLen);


void 	Usart1_SendBytes (const uint8* data, uint8 dataLen);
void 	Usart1_Write (const char* str);
void 	Usart1_WriteLn (const char* str);
void 	Usart1_WriteChar (char c);


