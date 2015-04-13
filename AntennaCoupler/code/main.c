
#include "System/SysConfig.h"

#include "System/IO/Usart1.h"
#include "helper.h"


//// TeST
//extern void Usart_Init(void);
//extern void Usart_Print(const char* str);
//extern void Usart_Send(const char* buf, unsigned long bufLen);


// TEST
#include "stm32f10x_gpio.h"



//
//	NOTE:	for more efficient switching of coils, things could be aligned / ports be used such that
//			that writing a while byte to a port switches several relays at once ?
//			--> no, is rather impractical (there are 2 coils each: one off, one on), and error prone.
//			Further more - their switching time is in the tens of milliseconds, so optimizing here is nonsense anyway.
//





//
//	TODO:
//
//	To configure all peripherals for relais automatically, based on the relay array entries,
//	do the following:
//
//	- Create an empty list for activated GPIO ports, named activeList.
//	- foreach relay R in all relay arrays:
//		- if NOT activeList contains R.port, then activate peripheral clock for R.port, and add R.port to activeList
//		- setup R.pin with:
//			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
//
//
// TODO: Check: does the order of enabling matter, e.g., peripheral clock for port enabling first, or pin setup first ?








typedef enum
{
	ActionType_SwitchOff = 0,
	ActionType_SwitchOn,

	ActionType_Invalid = 0xff
}
ActionType;


typedef enum
{
	RelayState_Off = 0,
	RelayState_On,

	RelayState_Unknown // need as array index now, so use the running number // = 0xff
}
RelayState;

// TODO: *perhaps* make one out of ActionType and RelayState, give a name that fits both?


// this will be part of the HAL
typedef struct Relay_
{
	// 2-coil latching relay

	GPIO_TypeDef*	const	setCoilGpioPort;
	uint16			const 	setCoilPin;

	GPIO_TypeDef*	const 	resetCoilGpioPort;
	uint16			const 	resetCoilPin;

	RelayState				lastKnownState;
}
Relay;
// TODO: the pin & port configuration stuff also goes into the HAL module(s)

void	Relay_SwitchCoil(Relay* that, RelayState stateCoil, bool coilPowerOn, bool setLastKnownState)
{
	// DebugAssert( stateCoil != RelayState_Unknown );

	BitAction ba = coilPowerOn ? Bit_SET : Bit_RESET;

	switch (stateCoil)
	{
		case ActionType_SwitchOff:
			GPIO_WriteBit( that->resetCoilGpioPort, that->resetCoilPin, ba );
			break;
		case ActionType_SwitchOn:
			GPIO_WriteBit( that->setCoilGpioPort, that->setCoilPin, ba );
			break;
	}

	if (setLastKnownState)
	{
		that->lastKnownState = stateCoil;
	}
}


#define	InductanceRelays_Count	8
Relay	inductanceRelays[ InductanceRelays_Count ] =
		{
			{GPIOC, GPIO_Pin_8,		GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,		GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},

			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown}
		};

#define	CapacitanceRelays_Count	8
Relay	capacitanceRelays[ CapacitanceRelays_Count ] =
		{
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},

			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown},
			{GPIOC, GPIO_Pin_8,  	GPIOC, GPIO_Pin_9,		RelayState_Unknown}
		};


typedef struct RelayGroup_
{
	Relay*		const	array;
	int			const	arrayLength;
	const char*	const	name;

}
RelayGroup;

#define		RelayGroups_Count	2
RelayGroup	relayGroups[ RelayGroups_Count ] =
			{
					{inductanceRelays,	InductanceRelays_Count,		"L"},
					{capacitanceRelays, CapacitanceRelays_Count,	"C"}
			};






typedef struct PendingAction_
{
	bool			active;
	ActionType		type;
	uint32			coilReleaseDueTimeMsecs; // dueTime = startTime + duration
	Relay*			relay;
}
Action;

void	Action_Construct(Action* a,  ActionType t, Relay* r)
{
	a->active = false;
	a->type = t;
	a->relay = r;
	a->coilReleaseDueTimeMsecs = 0; //	We don't set it here to its real value, it gets set elsewhere
	// TODO: duetime is actually better named "coilReleaseDueTime"
}


static Action	g_Action;

// might be a queue later
static Action*	g_pendingAction = 0;





static void show_status(void)
{
	void write_relay_state(RelayState s)
	{
		static const char* stateNames[] = {"Off", "On", "?"};
		const int i = s % 3;
		Usart1_WriteLn( stateNames[ i ] );
	}

	for (int g=0;	g<RelayGroups_Count;	++g)
	{
		const RelayGroup* rg = & relayGroups[ g ];
		Usart1_Write( "Group: " ); Usart1_WriteLn( rg->name );

		for (int r=0;	r<rg->arrayLength;	++r)
		{
			Usart1_Write( "  " ); Usart1_Write( h_itoa(r) );
			Usart1_Write( ": " ); write_relay_state( rg->array[r].lastKnownState );
		}
	}
}


// In:
//	cmdStr: pointer to null-terminating string which is the command line to interpret (excluding the LineFeed char)
//	cmdStrLen is the length of the null-terminating STRING, NOT the buffer length of cmdStr !
static bool	interpret_command(const char* cmdStr, int cmdStrLen, Action* outAction)
{
	// ASSERT( strlen( cmdStr ) <=

	int foundIndex = -1;
	int	groupIndex = -1;

	if (cmdStrLen < 3)
		goto NOT_RECOGNIZED;

	for (int i=0;	i<cmdStrLen;	++i)
	{
		char c = cmdStr[i];
		switch (c)
		{
			default: continue;

			case 'L':
				foundIndex = i;
				groupIndex = 0; // L
				goto FOREXIT;

			case 'C':
				foundIndex = i;
				groupIndex = 1; // C
				goto FOREXIT;

			case 's': // reading potentially beyond our buffer is not clean, but on this platform and in this context, not harmful
				if (	't'==cmdStr[i+1]
				    &&	'a'==cmdStr[i+2]
				    &&	't'==cmdStr[i+3]
					)
				{
					show_status();
				 return false;
				}
				break;
		}
	}

FOREXIT:

	if (foundIndex < 0)
		goto NOT_RECOGNIZED; // cmd start not found

	if (foundIndex + 2 >= cmdStrLen)
		goto NOT_RECOGNIZED;	// remaining command characters cannot be found, because string too short

	// Debug_Assert( groupIndex>=0 && groupIndex<=1 );
	RelayGroup* group = & relayGroups[ groupIndex ];

	// Debug_Assert( foundIndex>=0 && foundIndex<cmdStrLen );
	int itemIndex = ((int)cmdStr[ foundIndex + 1 ]) - '0'; // expect single-character number, e.g. 0..7, and convert it to index here
	if ( itemIndex < 0 || itemIndex >= group->arrayLength )
		goto RANGE_ERROR;	// invalid relay index relative to array group

	// Set, or Reset the relay?
	char actionChar = cmdStr[ foundIndex + 2 ];
	ActionType type = 'S'==actionChar ? ActionType_SwitchOn : 'R'==actionChar ? ActionType_SwitchOff : ActionType_Invalid;

	Relay* r = & group->array[ itemIndex ];

	Action_Construct( outAction, type, r );

	if ( ActionType_Invalid == type )
		goto NOT_RECOGNIZED;

	Usart1_Write("[Ok] "); Usart1_SendBytes( (uint8*)cmdStr, 3 ); Usart1_WriteLn("");
 return true;


NOT_RECOGNIZED:
	//Usart1_Write("[Error] Command not recognized: "); Usart1_SendBytes( (uint8*)cmdStr, 3 ); Usart1_WriteLn("");
	Usart1_Write("[Error] Command not recognized: ");	Usart1_WriteLn( cmdStr );
	goto SHOW_USAGE;

RANGE_ERROR:
	//Usart1_Write("[Error] Relay index invalid: "); Usart1_SendBytes( (uint8*)cmdStr, 3 ); Usart1_WriteLn("");
	Usart1_Write("[Error] Relay index invalid: ");	Usart1_WriteLn( cmdStr );
	goto SHOW_USAGE;


SHOW_USAGE:
	Usart1_WriteLn("[Hint] Usage: 'C3S' sets capacitor3, 'C3R' resets it, 'L4S' sets coil 4, etc. Index range: 0..7. 'stat' for status.");

 return false;
}




















void	test_systick_func()
{
	// GPIO_Write( GPIOC, 0xFF ); // Test: how fast is writing a byte into the whole port ? Test that on full 24MHz speed

//	// TEST:
//	// output systick (frq / 2) pulse waveform with 50% duty time.
//	//	E.g. when systick has 1000 Hz frq, you should see 500 Hz on the scope.
//	//uint8 bit =  1 & ( ~ GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_7 ));
//	static uint8 bit = 0; // using a variable does a bit less extra stuff than ReadInputDataBit
//	bit = (~bit) & 1;
//	GPIO_WriteBit( GPIOC, GPIO_Pin_7, bit );

	//static const int CoilSwitchTime = 40; // 40 ms switching time (was said to be at least 30ms for the 3V relay, but we use the 5V panasonic one.. time there?)
	//static const int CoilSwitchTime = 25; // but didn't they say "max 10ms" for release and switch.. making it ~ 20 ms? let's try 25
	static const int CoilSwitchTime = 30; // then again... it might fork for most relays but not some, and then we wonder why the device has glitches. Let's keep a safety margin.


	if (0 != g_pendingAction)
	{
		u32 now = System_GetTimeMsecs();

		if ( ! g_pendingAction->active)
		{	// now activate either the relay-SET or relay-REset coil, according to Action

			// switch coil on
			Relay_SwitchCoil( g_pendingAction->relay, g_pendingAction->type, true, false ); // do *not* set lastknowstate here yet, but after full switching time !
			g_pendingAction->coilReleaseDueTimeMsecs = now + CoilSwitchTime;
			g_pendingAction->active = true;
		}
		else // *is* active
		{
			if (now >= g_pendingAction->coilReleaseDueTimeMsecs)
			{
				// switch coil off again
				Relay_SwitchCoil( g_pendingAction->relay, g_pendingAction->type, false, true ); // now, set the last known state, which the relay should be in after long enough switching (coil=on) time
				g_pendingAction = 0; // remoce this action, it's no longer pending
			}
		}

	}

}



int main(void)
{
	//RCC_Exp();

	//DualModeDMA_SineWave();

	System_Init( 1 ); // 1 millisecond tick period
	System_RegisterTickFunc( test_systick_func );



	Usart1_Setup();

	Usart1_WriteLn( "> system running" );


	#define bufLen 16
	char buf[bufLen];

	while (1)
	{
		//if (Usart1_GetRxBufferCount() >= 3)
		//{
		//	buf[0] = Usart1_GetRxNextByte();
		//	buf[1] = Usart1_GetRxNextByte();
		//	buf[2] = Usart1_GetRxNextByte();

		// now we want a LineFeed [LF] character as termination character
		int lineLen;
		if ( Usart1_TryReadLine( buf, bufLen, & lineLen ) )
		{
			if (interpret_command( buf, lineLen, & g_Action ))
			{
				g_pendingAction = & g_Action;
			}
		}


//		if (Usart1_GetRxBufferCount() > 0)
//		{
//			u8 val = Usart1_GetRxNextByte();
//
//			switch (val)
//			{
//				case (u8)'1': // SET relay
//					Action_Construct( & g_Action, ActionType_SwitchOn );
//					g_pendingAction = & g_Action;
//					break;
//				case (u8)'0': // RESET relay
//					Action_Construct( & g_Action, ActionType_SwitchOff );
//					g_pendingAction = & g_Action;
//					break;
//			}
//		}


//		const char str[] = "Test!";
//
//		// TODO: implement this send(data, size) function in module Usart1.c
//		//Usart_Send( str, sizeof(str) );
//
//
//		const int iterations = 2000;
//		int i;
//		for (i=0;	i<iterations;	++i)
//		{
//			Usart1_WriteLn( str );
//
//			Delay( 50000 );
//		}
//
//		//Delay( 500000 );
//		Delay( 50000 );


	}


 return 0;
}
