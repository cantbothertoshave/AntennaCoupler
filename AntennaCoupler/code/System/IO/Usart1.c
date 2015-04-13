
#include "Usart1.h"



//#include "stm32lib/stm32f10x.h"
//#include "stm32lib/stm32f10x_rcc.h"
//#include "stm32lib/stm32f10x_gpio.h"
//#include "stm32lib/stm32f10x_usart.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#include "string.h"



//----------------------------------------------------------------------------------------------------------------------------------------
// forward
void USART1_IRQHandler(void); // written in caps since this was default spelling for the startup*.S
void Usart1_Tx_uint8(uint8);


//----------------------------------------------------------------------------------------------------------------------------------------
//Variables
static volatile uint8 USART1_RxBuffer[USART1_RXBUFFER_SIZE];
static uint8 USART1_TxBuffer[USART1_TXBUFFER_SIZE];
static uint16 USART1_Rx_RPtr;
static volatile uint16 USART1_Rx_WPtr;
static volatile uint16 USART1_Rx_cnt;
static volatile uint16 USART1_Tx_RPtr;
static uint16 USART1_Tx_WPtr;
static volatile uint16 USART1_Tx_cnt;




//----------------------------------------------------------------------------------------------------------------------------------------
void Usart1_Setup(void)
{
	USART1_Rx_cnt = 0;
	USART1_Tx_cnt = 0;
	USART1_Rx_RPtr = 0;
	USART1_Rx_WPtr = 0;
	USART1_Tx_RPtr = 0;
	USART1_Tx_WPtr = 0;


	//RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
	//RCC_APB2PeriphClockCmd(	RCC_APB2Periph_USART1, ENABLE );
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1, ENABLE );
	GPIO_PinRemapConfig( AFIO_MAPR_USART1_REMAP, ENABLE );
	USART_Cmd(USART1,DISABLE); //Disable USART1
	USART_DeInit(USART1);

	GPIO_InitTypeDef GPIO_InitStructure; //Configure GPIOs for USART1


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  // Tx pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //Rx pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure); //Init USART1 with above settings


	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE); //Enable RXNE IRQ (Rx buffer NOT empty)

	// temp. disabled
	//USART_ITConfig(USART1,USART_IT_TXE,ENABLE); //Enable TXE IRQ (Tx buffer empty)

	USART_Cmd(USART1,ENABLE); //Enable USART1


	//-----------------------
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		if(USART1_Rx_cnt < USART1_RXBUFFER_SIZE) //space left in RxBuffer?
		{
			/* Read one byte from the receive data register */
			USART1_RxBuffer[USART1_Rx_WPtr] = USART_ReceiveData(USART1);
			//USART1_Rx_WPtr = (USART1_Rx_WPtr + 1) % USART1_RXBUFFER_SIZE; //Ringbuffer pointer
			USART1_Rx_WPtr = (USART1_Rx_WPtr + 1) & USART1_RXBUFFER_WRAP_MASK;
			USART1_Rx_cnt++;
			USART_ClearFlag(USART1, USART_FLAG_RXNE); //clear flag
		}
		else //inputBuffer full, ignore data and clear RXNE-IRQ
		{
			USART_ClearFlag(USART1, USART_FLAG_RXNE);
		}
	}
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		/* Write one byte to the transmit data register */
		if (USART1_Tx_cnt != 0)
		{
			USART_SendData(USART1,USART1_TxBuffer[USART1_Tx_RPtr]);
			//USART1_Tx_RPtr = (USART1_Tx_RPtr + 1) % USART1_TXBUFFER_SIZE;
			USART1_Tx_RPtr = (USART1_Tx_RPtr + 1) & USART1_TXBUFFER_WRAP_MASK;
			USART1_Tx_cnt--;
		}
		else
		{
			/* Disable the USART1 Transmit interrupt */
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
	}
	if(USART_GetITStatus(USART1, USART_IT_ORE) != RESET)
	{
		USART_ReceiveData(USART1); //hopefully ORE is cleared now...
	}
}


uint16	Usart1_GetRxBufferCount()
{
	return USART1_Rx_cnt;
}


bool Usart1_TryReadLine(char* targetbuf, int targetBufLen, int* outLineLen)
{
	const int count = USART1_Rx_cnt;
	const int bufReadStartPos = USART1_Rx_RPtr;

	int get_ringbuf_index(int runningIndex)
	{	return	(bufReadStartPos + runningIndex) & USART1_RXBUFFER_WRAP_MASK;
	}

	*outLineLen = 0;

	for (int i=0; i<count && i<USART1_RXBUFFER_SIZE;	++i)
	{
		if ('\n' == (char)USART1_RxBuffer[ get_ringbuf_index(i) ])
		{
			const int lineLength = i; //  + 1; // omit the LF character
			// if the line is too long for the targetbuf, we copy only as much as fits
			// targbufLen-1 since we need one pos for the null-terminating character
			const int copyCharCount = lineLength <= targetBufLen-1 ? lineLength : targetBufLen-1;
			for (int k=0;	k<copyCharCount;	++k)
			{
				targetbuf[ k ] = USART1_RxBuffer[ get_ringbuf_index(k) ];
			}
			targetbuf[ lineLength ] = '\0'; // null-terminating

			// update the ringbuffer indices
			const int finishedCharCount = lineLength + 1; // also discard the LF char, even though we didn't copy it
			USART1_Rx_cnt -= finishedCharCount;
			USART1_Rx_RPtr = (USART1_Rx_RPtr + finishedCharCount) & USART1_RXBUFFER_WRAP_MASK;

			*outLineLen = lineLength;
		 return true;
		}
	}
 return false;
}

char Usart1_PeekRxNextChar(void)
{
	uint8 Rx_temp = 0;
	if(USART1_Rx_cnt > 0) //Received Bytes in buffer?
	{
		Rx_temp =  USART1_RxBuffer[USART1_Rx_RPtr]; //return value at current Rx_RPtr position
	}
	return Rx_temp;
}

uint8 Usart1_GetRxNextByte (void)
{
	uint8 Rx_temp = 0;
	if(USART1_Rx_cnt > 0) //Received Bytes in buffer?
	{
		USART1_Rx_cnt--; //Decrement Rx counter
		Rx_temp =  USART1_RxBuffer[USART1_Rx_RPtr]; //return value at current Rx_RPtr position
		//USART1_Rx_RPtr = (USART1_Rx_RPtr + 1) % USART1_RXBUFFER_SIZE;
		USART1_Rx_RPtr = (USART1_Rx_RPtr + 1) & USART1_RXBUFFER_WRAP_MASK;
	}
	return Rx_temp;
}

void Usart1_Tx_uint8 (uint8 Tx_byte)
{
	__disable_irq();
	if(USART1_Tx_cnt < USART1_TXBUFFER_SIZE) //space left in TxBuffer?
	{
		USART_ITConfig(USART1,USART_IT_TXE,DISABLE); //Disable TXE interrupt
		USART1_TxBuffer[USART1_Tx_WPtr] = Tx_byte;
		//USART1_Tx_WPtr = (USART1_Tx_WPtr + 1) % USART1_TXBUFFER_SIZE;
		USART1_Tx_WPtr = (USART1_Tx_WPtr + 1) & USART1_TXBUFFER_WRAP_MASK;
		USART1_Tx_cnt++;	//increment Tx counter
		USART_ITConfig(USART1,USART_IT_TXE,ENABLE); //Enable TXE interrupt
	}
	__enable_irq();
}

void Usart1_strout (const char* c)
{
	uint16 i; const uint16 len = strlen(c);
	while(len>(USART1_TXBUFFER_SIZE-USART1_Tx_cnt)) {} //wait for free buffer
	//for(i=0;	i<=len;		++i)
	for(i=0;	i<len;		++i) // do *not* send the '\0' at the end
	{
		Usart1_Tx_uint8(c[i]);
	}
}


void Usart1_SendBytes (const uint8* data, uint8 dataLen)
{
	uint8 i;
	for(i=0;	i<dataLen;		++i)
	{
		Usart1_Tx_uint8(data[i]);
	}
}



void Usart1_WriteChar (char c)
{
	Usart1_Tx_uint8( (uint8) c );
}


void Usart1_Write (const char* str)
{
	Usart1_strout( str );
}



void Usart1_WriteLn (const char* str)
{
	Usart1_Write( str );
	Usart1_Write( "\n" );
}


