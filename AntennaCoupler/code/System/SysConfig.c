#include "SysConfig.h"

#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

#define SYSCLK_FREQ_24MHz
//#define LED_PORT GPIOB // my board has 2 LEDs on GPIOC
#define LED_PORT GPIOC
#define LED_PERIPH RCC_APB2Periph_GPIOC
#define DELAY_COUNT    0x3FFFF


//--------------------------------------------------
//forward
void NVIC_Configuration(void);
void Delay(__IO uint32_t nCount);
void SetSysClock(void);
void SetSysClockTo24(void);
void SetSysClockToHSE(void);

static void	dummy_systick_func();



//--------------------------------------------------------------------
GPIO_InitTypeDef GPIO_InitStructure;
RCC_ClocksTypeDef RCC_ClockFreq;
ErrorStatus HSEStartUpStatus;

static int initialized = 0;

static uint32 			g_timePeriodMecs = 0;
static uint32	volatile	g_elapsedMsecs = 0; // !!! volatile! Or the compiler may optimize away calls of GetTime*(..) in a loop !!!
static SysTickFunc*	g_tickFunc = dummy_systick_func;




void Test_SetLed(int led, int on)
{
	unsigned short pin = led == 0 ? GPIO_Pin_8 : GPIO_Pin_9;

	GPIO_WriteBit(LED_PORT, pin, on ? Bit_SET : Bit_RESET);

	//GPIO_SetBits()
	//GPIO_ResetBits()
	// ?? whats this? GPIO_PinLockConfig()
}


void System_RegisterTickFunc(SysTickFunc* f)
{
	// don't allow setting to null, since we don't want a conditional in the tick handler.
	g_tickFunc  =  (0 == f) ? dummy_systick_func : f;
}



// The time counter will overflow after some days of continuous running.
// E.g., when counting milliseconds, the uint32 counter will overflow after 49.71 days.
// If a device runs that long, and timing critical thinsg are done around the overflow border, funny things might happen,
//	e.g. an event is not triggered before the next ~50 days, which, in case for not switching something off, might be really bad.
//
// If that poses a problem, the counter type can be changed to uint64, overflowing after several hundred million years.
//	But of course, in the library version it's not nice to set this to 64bits counter since it's slower, and if someone
//	wants to use a really quick SysTick in the MHz range, this might have impact.
//	Plus, you'd have to do time calculations in 64bits all the time...
//
//	A ResetTime function, which the user has to call when necessary, would be another solution.
//	E.g. just before use, e.g. when someone connects remotely to a device, with a PC or when a remote controlling device is
//	switched on for use, a ResetTime command could be sent.
//	Or the firmware on the device itself could, at moments where currently nothing is to be timed, reset the sys time.
//
//	An overflow flag could be set here when the overflow happened, which the user would have to query and reset,
//	and perform actions. But that would take conditionals, and time.
//
//	Last but not least, the user could check for time overflow in his code himself.
//	E.g., when using a StopWatch, if timeNow < timeStart, it's clear there's something wrong.
//
uint32 System_GetTimeMsecs()
{
	return g_elapsedMsecs;
}

static
void	dummy_systick_func()
{
	// do nothing.
	//	to avoid an if (func != null) {call func} thing...



	// test

	#define mul 10
	static const unsigned int del = mul * 1000000 / 2;

	while (1)
	{
		// user button pressed?
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
		{

			// bi-stable relay SET coil: power for a short while (~ 30..40ms), then unpower). The relay should be set. Wait another while to keep the state, to be observable.
			GPIO_WriteBit(LED_PORT, GPIO_Pin_8, Bit_SET);
			Delay(del / (10 * mul));
			GPIO_WriteBit(LED_PORT, GPIO_Pin_8, Bit_RESET);
			Delay(del);

			// bi-stable relay RESET coil: power for a short while (~ 30..40ms), then unpower). The relay should be REset. Wait another while to keep the state, to be observable.
			GPIO_WriteBit(LED_PORT, GPIO_Pin_9, Bit_SET);
			Delay(del / (10 * mul));
			GPIO_WriteBit(LED_PORT, GPIO_Pin_9, Bit_RESET);
			Delay(del);

		}

	}

}



void System_Init(uint32 tickPeriodMsecs)
{

	// Debug_Assert( tickPeriodMsecs );
	g_timePeriodMecs = tickPeriodMsecs;

	/* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
	SetSysClock();

	/* This function fills the RCC_ClockFreq structure with the current
	 frequencies of different on chip clocks (for debug purpose) */
	RCC_GetClocksFreq(&RCC_ClockFreq);

	/* Enable Clock Security System(CSS): this will generate an NMI exception
	 when HSE clock fails */
	RCC_ClockSecuritySystemCmd(ENABLE);

	/* NVIC configuration ------------------------------------------------------*/
	NVIC_Configuration();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // GPIO_Speed_10MHz; //GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);




//	//	/* Output HSE clock on MCO pin ---------------------------------------------*/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	RCC_MCOConfig(RCC_MCO_HSE);
//	RCC_MCOConfig( RCC_MCO_SYSCLK ); 		// output full speed sysclock, e.g. 24MHz, on MCO pin
//	RCC_MCOConfig( RCC_MCO_PLLCLK_Div2 );
//	RCC_MCOConfig( RCC_MCO_HSI ); // internal RC oscillator
//	 Well, enough playing / testing.
//	 For emission reasons, let's not output any clock at all
//	RCC_MCOConfig( RCC_MCO_NoClock );
//	It seems, it does so by default (not output anything)





	//-------------------------------------------------------------------
	// user button B1 on PA0
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;// IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	// enable GPIOA (user button uses it)
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


	//
	//	TODO:
	//
	//	this forgetting about enabling a GPIO port only  pin is configured of somewhere sucks...
	//	==> Make a library, where subsystems can request IOs for functions, and all used ports are then
	//		enabled at the end of the config fest, etc... something like that
	//




	//----------
	// TEST
	//RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // GPIO_Speed_10MHz; //GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	//GPIO_Init( GPIOB, &GPIO_InitStructure);


	// ?? does any extra NVIC stuff have to be setup for systick ? I remember setting priorities...
	//-----------------
	// with 24 MHz clock, having the systick timer count 24000 steps between it firing the interrupt,
	//	makes the systick clocked at 1 millisecond, i.e. @ 1000 Hz.
	//SysTick_Config( 24000 * tickPeriodMsecs );
	SysTick_Config( (SystemCoreClock * tickPeriodMsecs) / 1000  ); // make sure SystemCoreClock has been updated by SystemCoreClockUpdate() after setting the sysclock

	//-----------------
	initialized = 1;
}

void SysTick_Handler(void)
{

	g_elapsedMsecs += g_timePeriodMecs;

	// we don't need a conditional (checkfor null ptr) here, since whenever funcptr is set to null by user,
	//	it's set to an internal dummy func.
	g_tickFunc();
}

/***************************************************************************//**
 * @brief  Configures the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers.
 ******************************************************************************/
void SetSysClock(void)
{
	//	The System clock configuration functions defined below assume that:
	//	 - For Low, Medium and High density devices an external 8MHz crystal is
	//	 used to drive the System clock.
	//	 - For Connectivity line devices an external 25MHz crystal is used to drive
	//	 the System clock.
	//	 If you are using different crystal you have to adapt those functions accordingly.

	SetSysClockTo24();

	// Let's the system run with external crystal clock frequency, with no multiplication,
	//	i.e. for a 8MHz crystal, the system clock will then also be 8 MHz.
	//SetSysClockToHSE();


	// If none of the define above is enabled, the HSI is used as System clock
	// source (default after reset)


	// updates SystemCoreClock variable
	SystemCoreClockUpdate();
}

/***************************************************************************//**
 * @brief  Sets System clock frequency to 24MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 ******************************************************************************/
void SetSysClockTo24(void)
{
	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS) {
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 0 wait state */
		FLASH_SetLatency(FLASH_Latency_0);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK */
		RCC_PCLK1Config(RCC_HCLK_Div1);

		/* PLLCLK = 8MHz * 3 = 24 MHz */
		RCC_PLLConfig(0x00010000, 0x00040000);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08) {
		}
	} else { /* If HSE fails to start-up, the application will have wrong clock configuration.
	 User can add here some code to deal with this error */

		/* Go to infinite loop */
		while (1) {
		}
	}
}





/***************************************************************************//**
 * @brief  Selects HSE as System clock source and configure HCLK, PCLK2 and PCLK1 prescalers.
 ******************************************************************************/
void SetSysClockToHSE(void)
{
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

        /* Flash 0 wait state */
        FLASH_SetLatency( FLASH_Latency_0);

        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);

        /* PCLK1 = HCLK */
        RCC_PCLK1Config(RCC_HCLK_Div1);

        /* Select HSE as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_HSE);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x04)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */

        /* Go to infinite loop */
        while (1)
        {
        }
    }
}





/***************************************************************************//**
 * @brief  Configures Vector Table base location.
 ******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable and configure RCC global IRQ channel */
	NVIC_InitStructure.NVIC_IRQChannel = RCC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Inserts a delay time.
 * @param  nCount: specifies the delay time length.
 * @retval None
 */
void Delay(__IO uint32_t nCount) {
	for (; nCount != 0; nCount--)
		;
}

/***************************************************************************//**
 * @brief  This function handles RCC interrupt request.
 ******************************************************************************/
void RCC_IRQHandler(void) {
	if (RCC_GetITStatus(RCC_IT_HSERDY) != RESET) {
		/* Clear HSERDY interrupt pending bit */
		RCC_ClearITPendingBit(RCC_IT_HSERDY);

		/* Check if the HSE clock is still available */
		if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET) {
			/* Enable PLL: once the PLL is ready the PLLRDY interrupt is generated */
			RCC_PLLCmd(ENABLE);
		}
	}

	if (RCC_GetITStatus(RCC_IT_PLLRDY) != RESET) {
		/* Clear PLLRDY interrupt pending bit */
		RCC_ClearITPendingBit(RCC_IT_PLLRDY);

		/* Check if the PLL is still locked */
		if (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != RESET) {
			/* Select PLL as system clock source */
			RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		}
	}
}

/***************************************************************************//**
 * @brief  This function handles NMI exception.
 ******************************************************************************/
void NMI_Handler(void) {
	/* This interrupt is generated when HSE clock fails */

	if (RCC_GetITStatus(RCC_IT_CSS) != RESET) {
		/* At this stage: HSE, PLL are disabled (but no change on PLL config) and HSI
		 is selected as system clock source */

		/* Enable HSE */
		RCC_HSEConfig(RCC_HSE_ON);

		/* Enable HSE Ready interrupt */
		RCC_ITConfig(RCC_IT_HSERDY, ENABLE);

		/* Enable PLL Ready interrupt */
		RCC_ITConfig(RCC_IT_PLLRDY, ENABLE);

		/* Clear Clock Security System interrupt pending bit */
		RCC_ClearITPendingBit(RCC_IT_CSS);

		/* Once HSE clock recover, the HSERDY interrupt is generated and in the RCC ISR
		 routine the system clock will be reconfigured to its previous state (before
		 HSE clock failure) */
	}
}
