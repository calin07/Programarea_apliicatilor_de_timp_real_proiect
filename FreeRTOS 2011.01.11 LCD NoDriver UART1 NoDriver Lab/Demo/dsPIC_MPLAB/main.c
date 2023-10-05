
/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

/* Demo application includes. */
#include "BlockQ.h"
#include "crflash.h"
#include "blocktim.h"
#include "integer.h"
#include "comtest2.h"
#include "partest.h"
#include "semphr.h"
//#include "lcd.h"
#include "timertest.h"

// Includes proprii 
#include "new_lcd.h"
#include "new_serial.h"
#include "libq.h"
#include "ds18s20.h"
#include "adcDrv1.h"
//#include "serial.h"

/* Demo task priorities. */
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY				( 2 )

/* The check task may require a bit more stack as it calls sprintf(). */
#define mainCHECK_TAKS_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )

/* The execution period of the check task. */
#define mainCHECK_TASK_PERIOD				( ( portTickType ) 3000 / portTICK_RATE_MS )

/* The number of flash co-routines to create. */
#define mainNUM_FLASH_COROUTINES			( 5 )

/* Baud rate used by the comtest tasks. */
//#define mainCOM_TEST_BAUD_RATE				( 19200 )
#define mainCOM_TEST_BAUD_RATE				( 9600 )

// Definire lungime coada UART1
#define comBUFFER_LEN						( 10 )

/* We should find that each character can be queued for Tx immediately and we
don't have to block to send. */
#define comNO_BLOCK					( ( portTickType ) 0 )

/* The Rx task will block on the Rx queue for a long period. */
#define comRX_BLOCK_TIME			( ( portTickType ) 0xffff )

/* The LED used by the comtest tasks.  mainCOM_TEST_LED + 1 is also used.
See the comtest.c file for more information. */
#define mainCOM_TEST_LED					( 6 )

/* The frequency at which the "fast interrupt test" interrupt will occur. */
#define mainTEST_INTERRUPT_FREQUENCY		( 20000 )

/* The number of processor clocks we expect to occur between each "fast
interrupt test" interrupt. */
#define mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS ( configCPU_CLOCK_HZ / mainTEST_INTERRUPT_FREQUENCY )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned short ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Dimension the buffer used to hold the value of the maximum jitter time when
it is converted to a string. */
#define mainMAX_STRING_LENGTH				( 20 )

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC);
// Enable Clock Switching and Configure
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF);		// FRC + PLL
//_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);		// XT + PLL
_FWDT(FWDTEN_OFF); 		// Watchdog Timer Enabled/disabled by user software

/*
 * Setup the processor ready for the demo.
 */
static void prvSetupHardware( void );




// Modulul de lucru 1-Automat 0-Manual
int modul=1; // setat automat pe 1
int flag=0;
int comanda=1;

float temperatura=0;
// Transmisie mesaj UART1
/* The queue used to send messages to the LCD task. */
static xQueueHandle xUART1_Queue;
void init_PWM1(void);
xTaskHandle ht1, ht2, ht3, ht4, ht5;
void __attribute__ ((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
 _RB7 = ~_RB7; //Mutare stare LCD pe ON/OFF
 LCD_On_Off(_RB7);
// Initializare Program
 if (flag==0)  // flag=1 starea in care pornim proiectul  
	{
		flag=1;
		xTaskResumeFromISR(ht1);
		xTaskResumeFromISR(ht2);
		xTaskResumeFromISR(ht3);
		xTaskResumeFromISR(ht4);
	}
 else {
		flag=0; // flag = 0 starea in care oprim proiectul	
	  }

_INT0IF = 0;	// Resetam flagul corespunzator intreruperii
 				// INT0 pentru a nu se reapela rutina de intrerupere
} 

void Task1(void *params) { // Mod de lucru Automat:

// Temperatura
int num=0;
int dec=0;
	for (;;)
		{		
		//vParTestToggleLED(15);
		clear();
		temperatura=ds1820_read();
		num=temperatura;
		dec=(temperatura-num)*100;
		LCD_line(0);
		LCD_printf("Modul de lucru: Automat");
		LCD_line(1);
		LCD_printf("Temp C:");
		LCD_line(2);
		LCD_printf("Tensiune ADC:");
		LCD_line(3);
		LCD_printf("Ultima comanda:");
		LCD_Goto(3,16);	
		send_char2LCD(comanda+'0');
		LCD_Goto(1,8);	
		send_char2LCD(num/10+'0');
		send_char2LCD(num%10+'0');
		LCD_Goto(1,10);
		LCD_printf(".");
		LCD_Goto(1,11);
		send_char2LCD(dec/100+'0');
		send_char2LCD((dec/10)%10+'0');
		send_char2LCD(dec%10+'0');
		if(temperatura<=20)
			P1DC3=0x1388;
		if(temperatura>=30)
			P1DC3=0x2710;	
		if(temperatura>=24 && temperatura<=26)
			P1DC3=0x1D4C;
		vTaskDelay(1000);
		}
}

void Task2(void *params) { // Mod de lucru Manual:

// ADC
float adc=0;
int num=0;
int dec=0;
	for (;;)
		{		
		//vParTestToggleLED(15);
		adc=ADC1BUF0;
		adc=(float)adc*3/4095;
		num=adc;
		dec=(adc-num)*100;
		LCD_printf("Modul de lucru: Manual");
		LCD_line(1);
		LCD_printf("Temp C:");
		LCD_line(2);
		LCD_printf("Tensiune ADC:");
		LCD_line(3);
		LCD_printf("Ultima comanda:");
		LCD_Goto(3,17);	
		send_char2LCD(comanda+'0');
		LCD_Goto(2,15);	
		send_char2LCD(num/10+'0');
		send_char2LCD(num%10+'0');
		LCD_Goto(2,16);	
		LCD_printf(".");
		LCD_Goto(2,17);	
		send_char2LCD(dec/100+'0');
		send_char2LCD((dec/10)%10+'0');
		send_char2LCD(dec%10+'0');
		if(adc<1)
			P1DC3=0x1388;
		if(adc>=2.9)
			P1DC3=0x2710;	
		if(adc>=1.9 && adc<=2.1)
			P1DC3=0x1D4C;
		vTaskDelay(1000);
		}
}

void Task3(void *params) {//Citire meniu
	signed char cByteRxed;
	float temperatura=0;
	int temp;
	for (;;)
		{
		/* Block on the queue that contains received bytes until a byte is
		available. */
		if( xSerialGetChar( NULL, &cByteRxed, comRX_BLOCK_TIME ) )
			{
			if(cByteRxed==0+'0') //Modul Automat
				{	
					modul=0;
					comanda=0;
				}
			else if(cByteRxed==1+'0'){//Modul Manual
					modul=1;
					comanda=1;
				}	
			else if(cByteRxed==2+'0'){ // Afisare Temp
					temperatura=ds1820_read();
					_itoaQ15(temperatura,temp);
					comanda=2;
					vSerialPutString( NULL, "Temp:", comNO_BLOCK );
					vSerialPutString( NULL,temp, comNO_BLOCK );
			
				}
			}
		}
}

void Task4(void *params) {//Afisare meniu

	for (;;)
		{		
		vSerialPutString( NULL, "Meniu: 0-M 1-A 2-T", comNO_BLOCK );
		vTaskDelay(4000);
		}
}
void Task5(void *params) {//Initializare
	
	_RB0=0;
	for (;;)
		{
			if(flag==1)
			{
				_RB0=1;
				vTaskResume(ht3);
				vTaskResume(ht4);
			}
		   if(flag==0) 
			{
				vTaskSuspend(ht1);
				vTaskSuspend(ht2);
				vTaskSuspend(ht3);
				vTaskSuspend(ht4);

				_RB0=~_RB0;
			}
		   	
		   if(modul==1 && flag)
			{
				 _RB1=1;
				vTaskSuspend(ht2);
				vTaskResume(ht1);
			}
		   else if(modul==0 && flag)
			{
				_RB1=0;
				vTaskSuspend(ht1);
				vTaskResume(ht2);
			}
		vTaskDelay(250);
		}
}

int main( void )
{
	prvSetupHardware();
	xTaskCreate(Task1, (signed portCHAR *) "Ts1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &ht1);
	xTaskCreate(Task2, (signed portCHAR *) "Ts2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &ht2);
	xTaskCreate(Task3, (signed portCHAR *) "Ts3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &ht3);
	xTaskCreate(Task4, (signed portCHAR *) "Ts4", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &ht4);
	xTaskCreate(Task5, (signed portCHAR *) "Ts5", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, &ht5);
	
	/* Finally start the scheduler. */
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

void initPLL(void)
{
// Configure PLL prescaler, PLL postscaler, PLL divisor
	PLLFBD = 41; 		// M = 43 FRC
	//PLLFBD = 30; 		// M = 32 XT
	CLKDIVbits.PLLPOST=0; 	// N1 = 2
	CLKDIVbits.PLLPRE=0; 	// N2 = 2

// Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
	__builtin_write_OSCCONH(0x01);	// FRC
	//__builtin_write_OSCCONH(0x03);	// XT
	__builtin_write_OSCCONL(0x01);

// Wait for Clock switch to occur
	while (OSCCONbits.COSC != 0b001);	// FRC
	//while (OSCCONbits.COSC != 0b011);	// XT

// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};
}

static void prvSetupHardware( void )
{
	ADPCFG = 0xFFFF;				//make ADC pins all digital - adaugat
	initPLL();

	//Temperatura setari
	ONE_WIRE_PIN=1;//setam protocolul "one-wire"
	PLLFBD = 41; 		// M = 43 FRC
	TRISB = 0x0000;
	//PLLFBD = 30; 		// M = 32 XT
	CLKDIVbits.PLLPOST=0; 	// N1 = 2
	CLKDIVbits.PLLPRE=0; 	// N2 = 2
	CNPU1=0x0040;//PT utilizare rezistenta interna uC
	AD1PCFGLbits.PCFG4=1;//intrare analogica 	//Senzor Temp
	_CN6PUE=1;
	_TRISB2 = 1; // pt senzor tmp
	_RB2=0;
	output_float();

	//Intrerupere
	_TRISB7 = 1; // RB7 este setat ca iesire	
	PORTB = 0xF000;
	_INT0IF = 0; // Resetem flagul coresp. intreruperii INT0
	_INT0IE = 1; // Se permite lucrul cu întreruperea INT0
	_INT0EP = 0; // Se stabileste pe ce front se genereazã INT0

	//ADC
	initAdc1();             	// Initialize ADC AN5 (RB3)
	initTmr3();					// Initialise TIMER 3


	//PWM
	init_PWM1();		// Initializare PWM1_3 RB11-L RB10-H

	//LCD
	LCD_init();
	_RB7=0;      // Pornim in starea de LCD disabled
	PORTB = 0xF000;
	// Initializare interfata UART1
	xSerialPortInitMinimal( mainCOM_TEST_BAUD_RATE, comBUFFER_LEN );
}

void init_PWM1()
{
P1TCONbits.PTOPS = 0; // Timer base output scale
P1TCONbits.PTMOD = 0; // Free running
P1TCONbits.PTCKPS = 0b10; //
P1TMRbits.PTDIR = 0; // Numara in sus pana cand timerul = perioada
P1TMRbits.PTMR = 0; // Baza de timp
P1DC3=0x2710; //  20% Duty cycle
P1TPER=0x61A8;//

 PWM1CON1bits.PMOD1 = 0; // Canalele PWM1H si PWM1L sunt independente
 PWM1CON1bits.PMOD2 = 0; // Canalele PWM2H si PWM2L sunt independente
 PWM1CON1bits.PMOD3 = 0; // Canalele PWM3H si PWM3L sunt complementare

 PWM1CON1bits.PEN1H = 0; // Pinul PWM1H setat pe iesire PWM 
 PWM1CON1bits.PEN1L = 0; // Pinul PWM1L setat pe I/O general purpose
 PWM1CON1bits.PEN2H = 0; // Pinul PWM1H setat pe iesire PWM
 PWM1CON1bits.PEN2L = 0; // Pinul PWM1L setat pe I/O general purpose

 PWM1CON1bits.PEN3H = 1; // Pinul PWM1H setat pe iesire PWM RB10
 PWM1CON1bits.PEN3L = 1; // Pinul PWM1L setat pe iesire PWM RB11
PWM1CON2bits.UDIS = 1; // Disable Updates from duty cycle and period buffers
/* Clock period for Dead Time Unit A is TcY */
P1DTCON1bits.DTAPS = 0b00;
/* Clock period for Dead Time Unit B is TcY */
P1DTCON1bits.DTBPS = 0b00;

/* Dead time value for Dead Time Unit A */
P1DTCON1bits.DTA = 10;
/* Dead time value for Dead Time Unit B */
P1DTCON1bits.DTB = 20;
/* Dead Time Unit selection for PWM signals */
/* Dead Time Unit A selected for PWM active transitions */
P1DTCON2bits.DTS3A = 0;
//P1DTCON2bits.DTS2A = 0;
//P1DTCON2bits.DTS1A = 0;

/* Dead Time Unit B selected for PWM inactive transitions */
P1DTCON2bits.DTS3I = 1;
//P1DTCON2bits.DTS2I = 1;
//P1DTCON2bits.DTS1I = 1;
P1TCONbits.PTEN = 1; /* Enable the PWM Module */
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* Schedule the co-routines from within the idle task hook. */
	vCoRoutineSchedule();
}
/*-----------------------------------------------------------*/
