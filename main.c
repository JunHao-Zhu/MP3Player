/*
 * TFT_test.c
 *
 *  Created on: 2020��12��15��
 *      Author: zjh
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_epi.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/epi.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/fpu.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "grlib/grlib.h"
#include "utils/uartstdio.h"
#include "TFTinit/TFT_400x240_OTM4001A_16bit.h"
#include "TOUCHinit/TOUCH_TSC2046.h"
#include "EPIinit/EPIinit.h"
#include "picture.h"
#include "KalmanFilter.h"


#define M_PI 3.14159265358979323846F
// System clock rate in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;
uint32_t g_ui32SysCount = 0;
uint32_t ulADC0_Value;
int cur_music = 1;
volatile unsigned int count = 0;
bool pauseflag = false; // ��¼ֹͣ���ǿ���
#define TICKS_PER_SECOND 1000

extern uint32_t GetData[6];

float K_v, v;
KFP KFP_V = {0.02, 0, 0, 0, 0.001, 0.543};

uint32_t TouchXData[6];
uint32_t TouchYData[6];
//uint32_t TouchZData[6];   //Z is for pressure measurement
uint8_t play[4] = {0xAA, 0x02, 0x00, 0xAC}; // ����
uint8_t pause[4] = {0xAA, 0x03, 0x00, 0xAD}; // ��ͣ
uint8_t last[4] = {0xAA, 0x05, 0x00, 0xAF}; // ��һ��
uint8_t next[4] = {0xAA, 0x06, 0x00, 0xB0}; // ��һ��
uint8_t sound[5] = {0xAA, 0x13, 0x01, 0x14, 0xD2}; // ����
//=============================================================================
/*
 *  ���ֹܵĲ��������������ݴ���
 */
volatile unsigned int Timernum = 0;

#define _NOP() _nop()

//*********************************************************************
//*********************************************************************
#define I2C0_MASTER_BASE 0x40020000
#define I2C0_SLAVE_BASE 0x40020000

//*********************************************************************
// ��ַ���Ĵ����ȶ��岿��
//*********************************************************************
//*********************************************************************
//
// �趨slave���ӣ�ģ��ĵ�ַ������һ��7-bit�ĵ�ַ����RSλ��������ʽ����:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
// RSλ��һ��ָʾλ�����RS=0����˵�������������ݣ��ӽ������ݣ�RS=1˵�������������ݣ��ӷ�������
//
//*********************************************************************
//U21����4�����ֹܺ�����ܽŵ�����
#define I2C0_ADDR_TUBE_SEL	      0x30  //00110000
//U22�������ֹ�7~14�ܽŶ�Ӧ�����
#define I2C0_ADDR_TUBE_SEG_LOW    0x32  //00110010
//U23�������ֹ�15~18�ܽŶ�Ӧ�����
#define I2C0_ADDR_TUBE_SEG_HIGH  0x34   //00110100
//U24����LED����

//PCA9557�ڲ��Ĵ�����Ҳ���ӵ�ַ
#define PCA9557_REG_INPUT	 0x00
#define PCA9557_REG_OUTPUT	 0x01
#define PCA9557_REG_PolInver 0x02
#define PCA9557_REG_CONFIG	 0x03

//*************************************************************************************
 #define NUM 0
//IIC ����������ʱ������
unsigned char I2C_RECV_DATA[] =
				{
					0x00,
					0x00,
					0x00,
					0x00,
					0x00,
					0x00
				};

/*******************************************
		���� SDA �ź�
********************************************/
void I2C_Set_sda_high( void )
{
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,GPIO_PIN_3);  //����PB3
    _NOP();
    _NOP();
    return;
}

/*******************************************
		����SDA �ź�
********************************************/
void I2C_Set_sda_low ( void )
{
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0X00000000);  //����PB3
    _NOP();
    _NOP();
    return;
}

/*******************************************
		����SCL �ź�
********************************************/
void I2C_Set_scl_high( void )
{
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,GPIO_PIN_2);  //����PB2
    _NOP();
    _NOP();
    return;
}

/*******************************************
		����SCL �ź�
********************************************/
void I2C_Set_scl_low ( void )
{
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,0X00000000);  //����PB2
    _NOP();
    _NOP();
    return;
}

/*******************************************
		IIC �źŽ����źź���
********************************************/
void I2C_STOP(void)
{
    int i;
    I2C_Set_sda_low();
    for(i = NUM;i > 0;i--);
    I2C_Set_scl_low();
    for(i = NUM;i > 0;i--);
    I2C_Set_scl_high();
    for(i = NUM;i > 0;i--);
    I2C_Set_sda_high();
    for(i = NUM+1;i > 0;i--);
    return;
}


/*******************************************
		IIC �źų�ʼ��
********************************************/
void I2C_Initial( void )
{
    I2C_Set_scl_low();
    I2C_STOP();
    return;
}


/*******************************************
		IIC �ź���ʼ�źź���
********************************************/
void I2C_START(void)
{
    int i;

    I2C_Set_sda_high();
    for(i = NUM;i > 0;i--);
    I2C_Set_scl_high();
    for(i = NUM;i > 0;i--);
    I2C_Set_sda_low();
    for(i = NUM;i > 0;i--);
    I2C_Set_scl_low();
    return;
}

/*******************************************
		IIC ��ȡӦ����
********************************************/
int  I2C_GetACK(void)
{
    int j;
    _NOP();
    _NOP();
    I2C_Set_scl_low();
    for(j = NUM;j> 0;j--);
    I2C_Set_scl_high();
    for(j = NUM;j> 0;j--);
    I2C_Set_sda_low();
    for(j = NUM;j > 0;j--);
    I2C_Set_scl_low();
    return 1;
}

/*******************************************
		IIC ����Ӧ����
********************************************/
void I2C_SetNAk(void)
{
    I2C_Set_scl_low();
    I2C_Set_sda_high();
    I2C_Set_scl_high();
    I2C_Set_scl_low();
    return;
}

/*******************************************
		IIC �����ֽں���
		���� 	1��Ҫ�����ֽ�ֵ
		return ���޷���
********************************************/
void I2C_TxByte(unsigned char nValue)
{
    int i;
    int j;
    for(i = 0;i < 8;i++)
    {
    	if(nValue & 0x80)
    	    I2C_Set_sda_high();
    	else
    	    I2C_Set_sda_low();
    	for(j = NUM;j > 0;j--);
    	I2C_Set_scl_high();
    	nValue <<= 1;
    	for(j = NUM;j > 0;j--);
    	I2C_Set_scl_low();
    }

    return;
}


/*******************************************
		IIC �������麯��
	����  	1 num : �����ֽ���
	    2 device_addr : iicĿ���ַ
	    3 *data	�����������ַ
	return ���޷���
********************************************/
void i2c_write(int num, unsigned char device_addr,unsigned char *data)
{
    int i = 0;
    int count = num;
    unsigned char *send_data = data;
    unsigned char write_addr = device_addr;

    I2C_Set_scl_high();
    for(i = NUM;i > 0;i--);
    I2C_Set_sda_high();
    for(i = NUM;i > 0;i--);

    for(i = 0;i < count;i++)
    {
      I2C_START();           //ģ��I2Cд���ݵ�ʱ��
      I2C_TxByte(write_addr);
      I2C_GetACK();
      I2C_TxByte(send_data[i]);
      I2C_GetACK();
      i++;
      I2C_TxByte(send_data[i]);
      I2C_GetACK();
      I2C_STOP();
    }

}

//*********************************************************************
//******����I2C0ģ���IO���ţ�**********************************************
void I2C0GPIOBEnable(void)
{	// Enable GPIO portB containing the I2C pins (PB2&PB3)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2|GPIO_PIN_3);

}

//******����PCA9557оƬ���������ֹܵĸ�����Ϊ���***********************************
void I2C0DeviceInit(void)
{
	unsigned char dataBuf[2] = {PCA9557_REG_CONFIG, 0x00};
	i2c_write(2,I2C0_ADDR_TUBE_SEL,dataBuf);
	i2c_write(2,I2C0_ADDR_TUBE_SEG_LOW,dataBuf);
	i2c_write(2,I2C0_ADDR_TUBE_SEG_HIGH,dataBuf);

}

//*******�������ֹܵĹ�ѡ�ź�**************************************************
void I2C0TubeSelSet(char data)
{   //ѡ��1��2��3��4��5�ĸ����ֹ���
	unsigned char dataBuf[2] = {PCA9557_REG_OUTPUT, data};
	i2c_write(2,I2C0_ADDR_TUBE_SEL,dataBuf);
}
//*******�������ֹܵ���Ӧ���**************************************************
void I2C0TubeLowSet(char data)
{  //����7-14�ܽŶ�Ӧ�����
	unsigned char dataBuf[2] = {PCA9557_REG_OUTPUT, data};
	i2c_write(2,I2C0_ADDR_TUBE_SEG_LOW,dataBuf);
}
void I2C0TubeHighSet(char data)
{  //����15-18�ܽŶ�Ӧ�����
	unsigned char dataBuf[2] = {PCA9557_REG_OUTPUT, data};
	i2c_write(2,I2C0_ADDR_TUBE_SEG_HIGH,dataBuf);
}
//=============================================================================

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);

}

//=============================================================
/*
 * ����UART��GPIO��ģ��
 */
//=============================================================
void InitConsole(void){

	// ʹ��UART0ģ��
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// ��PA0��PA1�������Ź��ܽ���ѡ�� ���ｫ��ѡ��Ϊִ��UART0ģ��
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	// ����PA0��PA1��������UART����
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// UART�ı�׼��ʼ��
	UARTStdioConfig(0, 115200, g_ui32SysClock);
}

//=============================================================
/*
 * MP3ģ���UART��ʼ������
 */
//=============================================================
void UART_initial(void){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

	GPIOPinConfigure(GPIO_PP0_U6RX);
	GPIOPinConfigure(GPIO_PP1_U6TX);

	GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTConfigSetExpClk(UART6_BASE, g_ui32SysClock, 9600,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

//=============================================================
/*
 * ����ADCģ��
 */
//=============================================================
void ADCConfig(void){

	// ��ʼ��UART
//	InitConsole();

	// ��ʼ��ADC0/PE3
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);

	// ����ADC�ɼ�����
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH4 | ADC_CTL_END | ADC_CTL_IE);
	// ʹ��ADC�ɼ�����
	ADCSequenceEnable(ADC0_BASE, 3);

	ADCIntClear(ADC0_BASE, 3);
}

//============================================================
/*
 * ������ʼ������
 */
//============================================================
void GPIOInitial(void){
	// GPIOD�ܽų�ʼ������
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);

	// GPIOD�ж�����
//	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_LOW_LEVEL);
//	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_0);

	// GPION�ܽų�ʼ������
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPION);
	GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_2 | GPIO_PIN_3);

	// GPIOP�ܽų�ʼ������
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
//	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOP);
//	GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// GPIOK�ܽų�ʼ������
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
//	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOK);
//	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
//	GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_3);

	// GPIOM�ܽų�ʼ������
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOM);
//	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_5);


	// GPION�ж�����
//	GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_LOW_LEVEL);
//	GPIOIntEnable(GPIO_PORTN_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3);

}

//=============================================================
/*
 * ���ü������ж�
 */
//=============================================================
void TimerIntInitial(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_PIOSC);

	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR  | TIMER_CFG_B_PERIODIC | TIMER_CFG_A_PERIODIC);//

	TimerLoadSet(TIMER0_BASE, TIMER_B, g_ui32SysClock/TICKS_PER_SECOND ); // set period 1/SysClock * SysClock/1000 = 1/1000s=1ms
	TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock/5000 );

	TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

//=============================================================
/*
 * �������жϺ���--�������ֹ���ʾ
 */
//=============================================================
void Timer0BIntHandler(void){
	unsigned long Status;
	TimerDisable(TIMER0_BASE, TIMER_B);
	Status = TimerIntStatus(TIMER0_BASE, true);


	if(Status == TIMER_TIMB_TIMEOUT){

		Timernum++;
    	I2C0TubeSelSet(~0x10);I2C0TubeLowSet(0x82);I2C0TubeHighSet(0x00);SysCtlDelay(2000);
    	I2C0TubeSelSet(0xff);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		switch((Timernum/100) % 10){
		case 1: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x22);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 2: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x2c);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 3: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x26);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 4: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x60);I2C0TubeHighSet(0x32);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 5: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x16);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 6: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x1e);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 7: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x26);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 8: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x3e);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 9: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x36);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		default: I2C0TubeSelSet(0x36);I2C0TubeLowSet(0x10);I2C0TubeHighSet(0x3e);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
//		default: break;
		}
		switch((Timernum/1000) % 6){
		case 1: I2C0TubeSelSet(0x3a);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x22);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 2: I2C0TubeSelSet(0x3a);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x2c);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 3: I2C0TubeSelSet(0x3a);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x26);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 4: I2C0TubeSelSet(0x3a);I2C0TubeLowSet(0x60);I2C0TubeHighSet(0x32);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 5: I2C0TubeSelSet(0x3a);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x16);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		default: I2C0TubeSelSet(0x3a);I2C0TubeLowSet(0x10);I2C0TubeHighSet(0x3e);SysCtlDelay(20000);
				 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		}
		switch((Timernum/6000) % 10){
		case 1: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x22);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 2: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x2c);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 3: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x26);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 4: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x60);I2C0TubeHighSet(0x32);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 5: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x16);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 6: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x1e);SysCtlDelay(20000);
				I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 7: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x26);SysCtlDelay(20000);
		 	 	I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 8: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x3e);SysCtlDelay(20000);
		 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 9: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x36);SysCtlDelay(20000);
		 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		default: I2C0TubeSelSet(0x3c);I2C0TubeLowSet(0x10);I2C0TubeHighSet(0x3e);SysCtlDelay(20000);
		 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		}
		switch((Timernum/60000) % 6){
		case 1: I2C0TubeSelSet(0x1f);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x22);SysCtlDelay(20000);
		 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 2: I2C0TubeSelSet(0x1f);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x2c);SysCtlDelay(20000);
		 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 3: I2C0TubeSelSet(0x1f);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x26);SysCtlDelay(20000);
		 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 4: I2C0TubeSelSet(0x1f);I2C0TubeLowSet(0x60);I2C0TubeHighSet(0x32);SysCtlDelay(20000);
		 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		case 5: I2C0TubeSelSet(0x1f);I2C0TubeLowSet(0x70);I2C0TubeHighSet(0x16);SysCtlDelay(20000);
		 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		break;
		default: I2C0TubeSelSet(0x1f);I2C0TubeLowSet(0x10);I2C0TubeHighSet(0x3e);SysCtlDelay(20000);
		 I2C0TubeSelSet(0xFF);I2C0TubeLowSet(0x00);I2C0TubeHighSet(0x00);
		}
		if(Timernum == 360000){
			Timernum = 0;
		}
//
	}

	TimerIntClear(TIMER0_BASE, Status);
	TimerLoadSet(TIMER0_BASE, TIMER_B, g_ui32SysClock/TICKS_PER_SECOND);
//	TimerLoadSet(TIMER0_BASE, TIMER_B, g_ui32SysClock);
	TimerEnable(TIMER0_BASE, TIMER_B);
}

//===============================================================
/*
 * �жϿ���ͼƬ��ת
 */
//===============================================================
void Timer0AIntHandler(void){
	unsigned long Status;
	TimerDisable(TIMER0_BASE, TIMER_A);
	Status = TimerIntStatus(TIMER0_BASE, true);

	if(Status == TIMER_TIMA_TIMEOUT){

	    switch(cur_music){
	    case 1:
	        switch(count / 3){
            case 0: DrawPict(70, 80, gImage_jay);break;
            case 1: DrawPict(70, 80, gImage_jay2);break;
            case 2: DrawPict(70, 80, gImage_jay3);break;
            case 3: DrawPict(70, 80, gImage_jay4);break;
            default:
	        }
	        break;
	    case 2:
	        switch(count / 3){
	        case 0: DrawPict(70, 80, gImage_eason);break;
	        case 1: DrawPict(70, 80, gImage_eason2);break;
	        case 2: DrawPict(70, 80, gImage_eason3);break;
	        case 3: DrawPict(70, 80, gImage_eason4);break;
	        default:
	        }
	        break;
	    default: break;
	    }

		count++;
		if(count == 12){
			count = 0;
		}


	}
	TimerIntClear(TIMER0_BASE, Status);
	TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock/5000);
	TimerEnable(TIMER0_BASE, TIMER_A);

	TOUCH_PointAdjust(&TouchXData[5], &TouchYData[5]);
	TOUCH_PressKey(&TouchXData[5], &TouchYData[5]);

	identify_key();

	//===============================================
	// ��ȡADC�еĹ���ֵ
	ADCProcessorTrigger(ADC0_BASE, 3);
	while(!ADCIntStatus(ADC0_BASE, 3, false));

	ADCSequenceDataGet(ADC0_BASE, 3, &ulADC0_Value);
//	SysCtlDelay(2*50000000 / 3000);

	v = ulADC0_Value / 4096.0 * 3.3 * 1000;
//	K_v = kalmanFilter(&KFP_V, v);
	SoundSet((int)v);

}

//==============================================================
/*
 * ������
 */
//==============================================================
void TFTLCD_DrawTriangle(){
	TFTLCD_DrawHorizontalLine(70,200,128,GREEN);
	TFTLCD_DrawHorizontalLine(70,200,193,GREEN);
	TFTLCD_DrawVerticalLine(129,193,71,GREEN);
	TFTLCD_DrawVerticalLine(129,193,199,GREEN);
}

//==============================================================
/*
 * ������
 */
//==============================================================
void DrawBackground(void){
	uint32_t temp, i, j, m;
	m = 0;
	for(i=0; i<400; i++){
		for(j=0; j<240; j++){
			temp = gImage_back[m];
			temp |= gImage_back[m+1] << 8;
			TFTLCD_DrawPoint(j, i, temp);
			m++;
			m++;
		}
	}
}

//==============================================================
/*
 * ��ͼ
 */
//==============================================================
void DrawPict(uint32_t x, uint32_t y,const unsigned char * image){
	uint32_t temp, i, j, m;
	m = 0;
	for(i=y; i<y+100; i++){
		for(j=x; j<x+100; j++){
			temp = image[m];
			temp |= image[m+1] << 8;
			TFTLCD_DrawPoint(j, i, temp);
			m++;
			m++;
		}
	}
}

//==============================================================
/*
 * ����ͣ��
 */
//==============================================================
void DrawPauseKey(uint32_t x, uint32_t y){
	uint32_t i;
	for(i=0; i<30; i++){
//		TFTLCD_DrawLine(y+i, x+i, y+50-i, x+i, WHITE);
		TFTLCD_DrawLine(x+i/2, y+i/2, x+i/2, y+30-i/2, WHITE);
	}
}

//==============================================================
/*
 * ����һ�׼�
 */
//==============================================================
void DrawLastKey(uint32_t x, uint32_t y){
	uint32_t i;
	for(i=0; i<24; i++){
		TFTLCD_DrawLine(x-i/2, y+i/2, x-i/2, y+24-i/2, WHITE);
	}
	TFTLCD_DrawLine(x-(i+1)/2, y, x-(i+1)/2, y+24, WHITE);
}

//==============================================================
/*
 * ����һ�׼�
 */
//==============================================================
void DrawNextKey(uint32_t x, uint32_t y){
	uint32_t i;
	for(i=0; i<24; i++){
		TFTLCD_DrawLine(x+i/2, y+i/2, x+i/2, y+24-i/2, WHITE);
	}
	TFTLCD_DrawLine(x+(i+1)/2, y, x+(i+1)/2, y+24, WHITE);
}

//==============================================================
/*
 * �����ż�
 */
//==============================================================
void DrawStartKey(uint32_t x, uint32_t y){
	uint32_t i;
	for(i=0; i<3; i++){
		TFTLCD_DrawLine(x+i, y, x+i, y+30, WHITE);
		TFTLCD_DrawLine(x+i+15, y, x+i+15, y+30, WHITE);
	}
}

//==============================================================
/*
 * ������ż�
 */
//==============================================================
void ClearStartKey(uint32_t x, uint32_t y){
	uint32_t i;
	for(i=0; i<3; i++){
		TFTLCD_DrawLine(x+i, y, x+i, y+30, WHITE);
		TFTLCD_DrawLine(x+i+15, y, x+i+15, y+30, BLACK);
	}
}

//==============================================================
/*
 * �����ͣ��
 */
//==============================================================
void ClearPauseKey(uint32_t x, uint32_t y){
	uint32_t i;
	for(i=0; i<30; i++){
//		TFTLCD_DrawLine(y+i, x+i, y+50-i, x+i, WHITE);
		TFTLCD_DrawLine(x+i/2, y+i/2, x+i/2, y+30-i/2, BLACK);
	}
}

//==============================================================
/*
 * ������Ӧ�ļ�ʱִ�в�ͬ�Ĺ���
 */
//==============================================================
void TOUCH_PressKey(uint32_t* TouchXData, uint32_t* TouchYData){

	if(*TouchYData >= 290 && *TouchYData <= 350){
		// ����ͣ��
		if(*TouchXData >= 100 && *TouchXData <= 150){
			if(pauseflag){
				TimerEnable(TIMER0_BASE, TIMER_B);
				TimerEnable(TIMER0_BASE, TIMER_A);
				// ��������
				GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x20);
				SysCtlDelay(100000);
				GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x00);

				play_music(1);
				ClearPauseKey(115, 300);
				DrawStartKey(115, 300);

			}
			else{
				TimerDisable(TIMER0_BASE, TIMER_B);
				TimerDisable(TIMER0_BASE, TIMER_A);
				// ��������
				GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x20);
				SysCtlDelay(200000);
				GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x00);

				play_music(2);
				ClearStartKey(115, 300);
				DrawPauseKey(115,300);
			}

			pauseflag = !pauseflag;
			return;
		}
		// ��ǰһ�׸�İ�ť
		if(*TouchXData >= 49 && *TouchXData <= 88){
		    if(cur_music > 1){
		        cur_music --;
		        // ��������
		        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x20);
		        SysCtlDelay(200000);
                GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x00);

                Timernum = 0;
                play_music(3);
		    }

			return;
		}
		// ����һ�׸�İ�ť
		if(*TouchXData >= 155 && *TouchXData <= 173){
		    if(cur_music < 2){
		        cur_music ++;
		        // ��������
                GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x20);
                SysCtlDelay(200000);
                GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x00);

                Timernum = 0;
                play_music(4);

		    }
		    return;

		}
	}
}

//=================================================================
/*
 * �������ж�
 */
//=================================================================
void identify_key(){

	// ��ͣ�� GPIO PN2��
	if((GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) & 0x04)  == 0x00){
		SysCtlDelay(20000);
		if((GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) & 0x04) == 0x00){
			if(pauseflag){
				TimerEnable(TIMER0_BASE, TIMER_A);
				TimerEnable(TIMER0_BASE, TIMER_B);
				play_music(1);
				ClearPauseKey(115, 300);
				DrawStartKey(115, 300);
			}
			else{
				TimerDisable(TIMER0_BASE, TIMER_A);
				TimerDisable(TIMER0_BASE, TIMER_B);
//				stop_music();
				play_music(2);
				ClearStartKey(115, 300);
				DrawPauseKey(115, 300);
			}
			pauseflag = !(pauseflag);
			return ;
		}
	}
	// ��һ�ײ��ż� GPIO PN3��
	else if((GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_3) & 0x08) == 0x00){
		SysCtlDelay(20000);
		if((GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_3) & 0x08) == 0x00){
		    if(cur_music > 1){
		        cur_music--;
		        Timernum = 0;
		        play_music(3);
		    }

			return ;
		}
	}
	// ��һ�ײ��ż�GPIO PD0��
	else if((GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0) & 0x01) == 0x00){
		SysCtlDelay(20000);
		if((GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0) & 0x01) == 0x00){
		    if(cur_music < 2){
		        cur_music++;
		        Timernum = 0;
		        play_music(4);
		    }

			return ;
		}
	}
}

//======================================================================
/*
 * ����MP3
 */
//======================================================================
void play_music(int number){

    int i;
	switch(number){
	case 1:
	    // ����
//	    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x01);
//	    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
//	    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
	    for(i=0; i<4; i++){
	        UARTCharPut(UART6_BASE, play[i]);
	    }
		break;
	case 2:
	    // ��ͣ
//		GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x02);
//		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
//		GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
	    for(i=0; i<4; i++){
	        UARTCharPut(UART6_BASE, pause[i]);
	    }
		break;
	case 3:
	    // ��һ��
//		GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x03);
//		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xFE);
//		GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
	    for(i=0; i<4; i++){
	        UARTCharPut(UART6_BASE, last[i]);
	    }
		break;
	case 4:
	    // ��һ��
//	    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x03);
//	    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
//	    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
	    for(i=0; i<4; i++){
	        UARTCharPut(UART6_BASE, next[i]);
	    }
	    break;
	}

}

//=================================================================
/*
 * ��������
 */
//=================================================================
void SoundSet(int degree){
    int i;
    if(degree < 100){
        sound[3] = 0x01;
        sound[4] = 0xBF;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
    else if(degree < 460){
        sound[3] = 0x04;
        sound[4] = 0xC2;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
    else if(degree < 820){
        sound[3] = 0x07;
        sound[4] = 0xC5;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
    else if(degree < 1180){
        sound[3] = 0x0A;
        sound[4] = 0xC8;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
    else if(degree < 1540){
        sound[3] = 0x0D;
        sound[4] = 0xCB;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
    else if(degree < 1900){
        sound[3] = 0x10;
        sound[4] = 0xCE;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
    else if(degree < 2260){
        sound[3] = 0x13;
        sound[4] = 0xD1;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
    else if(degree < 2620){
        sound[3] = 0x16;
        sound[4] = 0xD4;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
    else if(degree < 2930){
        sound[3] = 0x19;
        sound[4] = 0xD7;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
    else{
        sound[3] = 0x1C;
        sound[4] = 0xDA;
        for(i=0; i<5; i++){
            UARTCharPut(UART6_BASE, sound[i]);
        }
    }
}

void SysTickIntHandler(void){;}

void main()
{
    //====================================================
    // ��ʱ��������
	uint16_t ui32Loop = 0,temp=0;

	//====================================================
	// FPU����
    FPUEnable();
    FPULazyStackingEnable();

    //=====================================================
    // ����ʱ��
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                SYSCTL_CFG_VCO_480), 120000000);
    //======================================================
    // ���ֹܵ�����
	I2C0GPIOBEnable();//����I2C0ģ���IO����
	I2C0DeviceInit();//����PCA9557оƬ���������ֹܵĸ�����Ϊ���

	//======================================================
	// ��ʼ���ж�
	GPIOInitial();

    //======================================================
    // ���ö�ʱ���ж�
    SysTickPeriodSet(g_ui32SysClock / 10000);
    TimerIntInitial();

    //======================================================
    // ����UART�Լ�MP3��Ӧ��UARTģ��
    ConfigureUART();
    UART_initial();

    //=======================================================
    // ��ʾ����ʼ���Լ����ô���
    EPIGPIOinit();
    TOUCH_TSC2046init(g_ui32SysClock);

    GPIOIntEnable(GPIO_PORTB_BASE,GPIO_INT_PIN_0);
    GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);

    TFT_400x240_OTM4001Ainit(g_ui32SysClock);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);

	SSIDataPut(SSI0_BASE,0xd0);

	//====================================================
	// ����ʾ���ϻ��������Լ����ఴť
	DrawBackground();
	DrawStartKey(115, 300);
	DrawLastKey(65, 303);
	DrawNextKey(175, 303);

    //======================================================
    // �����ж����ȼ��Լ�������ʱ���ж�
    IntPrioritySet(INT_TIMER0B, 0x80);
    IntPrioritySet(INT_TIMER0A, 0xe0);
    IntEnable(INT_TIMER0B);
    IntEnable(INT_TIMER0A);
    IntMasterEnable();
    SysTickIntEnable();
    TimerEnable(TIMER0_BASE, TIMER_B);
    TimerEnable(TIMER0_BASE, TIMER_A);

    //======================================================
    // ����ADC����
    ADCConfig();

    //======================================================
    // ��ʼ��������
    play_music(1);

    while(1)
    {

    	temp++;
    	//=========================================================
    	// ��ȡ������λ��
    	for(ui32Loop=0;ui32Loop<=5;ui32Loop++)
    	{
			SSIDataPut(SSI0_BASE,0x90);
			SysCtlDelay(3);
			SSIDataGet(SSI0_BASE,&TouchXData[ui32Loop]);
			SysCtlDelay(3);
			SSIDataPut(SSI0_BASE,0xd0);
			SysCtlDelay(3);
			SSIDataGet(SSI0_BASE,&TouchYData[ui32Loop]);
			SysCtlDelay(3);
		}
		TouchXData[5] = (TouchXData[0]+TouchXData[1]+TouchXData[2]+TouchXData[3]+TouchXData[4])/5;
		TouchYData[5] = (TouchYData[0]+TouchYData[1]+TouchYData[2]+TouchYData[3]+TouchYData[4])/5;


		if(temp >= 200){
		    TOUCH_PointAdjust(&TouchXData[5], &TouchYData[5]);
		    TOUCH_PressKey(&TouchXData[5], &TouchYData[5]);
		}

		//============================================================
		// �жϴ���λ��
		if(temp >= 200){
		    identify_key();
		    temp = 0;
		}
    }
}


