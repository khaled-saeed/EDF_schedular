

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h> 
#include <string.h> 

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

/*-----------------------------------------------------------*/
#define Button_1_PERIOD 		50 //Button_1 period
#define Button_2_PERIOD 		50 //Button_2 period
#define Transmitter_PERIOD  100 //Transmitter period
#define Receiver_PERIOD 		20 //Receiver period
#define Load_1_PERIOD  			10 //Load_1 period
#define Load_2_PERIOD 			100 //Load_2 period



/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

TaskHandle_t Button_1_Handller = NULL ; 
TaskHandle_t Button_2_Handller = NULL ; 
TaskHandle_t Periodic_TransmitterHandller = NULL ; 
TaskHandle_t Uart_ReceiverHandller = NULL ; 
TaskHandle_t Load_1_SimulationHandller = NULL ; 
TaskHandle_t Load_2_SimulationHandller = NULL ; 

SemaphoreHandle_t xSemaphore_button;
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

//char runTimeStatsBuff[190]; 
char buttonStatusBuffer [200] ; 


#define tickTracePin 					0	
#define idleTracePin 					1
#define button1TracePin 			2
#define button2TracePin 			3
#define TransmitterTracePin 	4
#define ReceiverTracePin 			5
#define load1TracePin 				6
#define load2TracePin 				7


/*********** PORT 1 ************/
#define button1Pin			0
#define button2Pin			1 

typedef enum{
	UNPRESSED,
	PRESSED
}buttonStatus_t; 

buttonStatus_t button_1_status = UNPRESSED,button_2_status= UNPRESSED; 


/* Task to be created. */
void Button_1_Monitor( void * pvParameters )
{
	TickType_t xLastWakeTimeB;
	xLastWakeTimeB = xTaskGetTickCount();
	while(1)
	{
		GPIO_write(PORT_0,PIN4,PIN_IS_HIGH); 
		if(GPIO_read(PORT_0,PIN8) == PIN_IS_HIGH)
		{
			button_1_status = PRESSED ; 
		}
		else
		{
			button_1_status = UNPRESSED ; 
		}
		GPIO_write(PORT_0,PIN4,PIN_IS_LOW); 
		vTaskDelayUntil( &xLastWakeTimeB, Button_1_PERIOD );
		GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
	}
}
void Button_2_Monitor( void * pvParameters )
{
	TickType_t xLastWakeTimeB;
	xLastWakeTimeB = xTaskGetTickCount();
	while(1)
	{
		GPIO_write(PORT_0,PIN5,PIN_IS_HIGH); 
		if(GPIO_read(PORT_0,PIN9) == PIN_IS_HIGH)
		{
			button_2_status = PRESSED ; 
		}
		else
		{
			button_2_status = UNPRESSED ; 
		}
		GPIO_write(PORT_0,PIN5,PIN_IS_LOW); 
		vTaskDelayUntil( &xLastWakeTimeB, Button_2_PERIOD );
		GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
	}
}

void Periodic_Transmitter( void * pvParameters )
{
	//strcpy()
		TickType_t xLastWakeTimeB;
	xLastWakeTimeB = xTaskGetTickCount();
	while(1)
	{
		GPIO_write(PORT_0,PIN6,PIN_IS_HIGH); 
		if(button_1_status == PRESSED)
		{
			if(button_2_status == PRESSED)
			{
				strcpy(buttonStatusBuffer,"Button 1 and Button 2 are pressed");
			}
			else
			{
				strcpy(buttonStatusBuffer,"Button 1 is pressed and Button 2 is unpressed");
			}
		}
		else
		{
			if(button_2_status == PRESSED)
			{
				strcpy(buttonStatusBuffer,"Button 1 is unpressed and Button 2 is pressed");
			}
			else
			{
				strcpy(buttonStatusBuffer,"Button 1 and Button 2 are unpressed");
			}
		}
		GPIO_write(PORT_0,PIN6,PIN_IS_LOW); 
		vTaskDelayUntil( &xLastWakeTimeB, Transmitter_PERIOD );
		GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
	}
}
void Uart_Receiver( void * pvParameters )
{
	TickType_t xLastWakeTimeB;
	xLastWakeTimeB = xTaskGetTickCount();
	
	while(1)
	{
		GPIO_write(PORT_0,PIN7,PIN_IS_HIGH); 
		vSerialPutString((const signed char*)buttonStatusBuffer,50); 
		GPIO_write(PORT_0,PIN7,PIN_IS_LOW); 
		vTaskDelayUntil( &xLastWakeTimeB, Receiver_PERIOD );
		GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
	}
}
void Load_1_Simulation( void * pvParameters )
{
	TickType_t xLastWakeTimeB;
	xLastWakeTimeB = xTaskGetTickCount();
	while(1)
	{
			GPIO_write(PORT_0,PIN2,PIN_IS_HIGH); 
			int i  ; 
			for( i = 0 ; i < 37200 ; i++) 
			{
			}
			GPIO_write(PORT_0,PIN2,PIN_IS_LOW); 
			vTaskDelayUntil( &xLastWakeTimeB, Load_1_PERIOD );
			GPIO_write(PORT_0,PIN1,PIN_IS_LOW); 



	}
}
void Load_2_Simulation( void * pvParameters )
{
	TickType_t xLastWakeTimeA;
	xLastWakeTimeA = xTaskGetTickCount();
	while(1)
	{
			GPIO_write(PORT_0,PIN3,PIN_IS_HIGH); 
			int i  ; 
			for( i = 0 ; i < 89300 ; i++) 
			{
			}
			GPIO_write(PORT_0,PIN3,PIN_IS_LOW); 
			vTaskDelayUntil( &xLastWakeTimeA, Load_2_PERIOD );
			GPIO_write(PORT_0,PIN1,PIN_IS_LOW); 


	}
	
}
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */

void vApplicationTickHook (void)
{
	GPIO_write(PORT_0,PIN0,PIN_IS_HIGH); 
	GPIO_write(PORT_0,PIN0,PIN_IS_LOW); 

}
void vApplicationIdleHook (void)
{
	GPIO_write(PORT_0,PIN1,PIN_IS_HIGH); 

	
}
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	xSemaphore_button = xSemaphoreCreateBinary();
	
    /* Create Tasks here */
		xTaskPeriodicCreate( Button_1_Monitor,
													( const char * ) "Button_1_Monitor",
													configMINIMAL_STACK_SIZE, 
													NULL,
													1, 
													&Button_1_Handller,
													Button_1_PERIOD );
													
		 xTaskPeriodicCreate( Button_2_Monitor, 
													( const char * ) "Button_2_Monitor",
													configMINIMAL_STACK_SIZE
													, NULL,
													1, 
													&Button_2_Handller, 
													Button_2_PERIOD );
													
		 xTaskPeriodicCreate( Periodic_Transmitter,
													( const char * ) "Transmitter",
													configMINIMAL_STACK_SIZE, 
													NULL,
													1, 
													&Periodic_TransmitterHandller,
													Transmitter_PERIOD );
													
		 xTaskPeriodicCreate( Uart_Receiver, 
													( const char * ) "Receiver",
													configMINIMAL_STACK_SIZE
													, NULL,
													1, 
													&Uart_ReceiverHandller, 
													Receiver_PERIOD );
													
		
			xTaskPeriodicCreate( Load_1_Simulation,
													( const char * ) "Load_1_Simulation",
													configMINIMAL_STACK_SIZE, 
													NULL,
													1, 
													&Load_1_SimulationHandller,
													Load_1_PERIOD );
													
		 xTaskPeriodicCreate( Load_2_Simulation, 
													( const char * ) "Load_2_Simulation",
													configMINIMAL_STACK_SIZE
													, NULL,
													1, 
													&Load_2_SimulationHandller, 
													Load_2_PERIOD );

											

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/
void Timer1_reset(void) 
{
	T1TCR |= 0x2 ; 
	T1TCR &= ~0x2 ; 

}
static void Timer1_config(void)
{
	T1PR = 1000 ; 
	T1TCR |= 0x1 ; 
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	Timer1_config(); 
	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/

