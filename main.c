/*
 * TFT_test.c
 *
 *  Created on: 2020年12月15日
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
bool pauseflag = false; // 记录停止还是开启
#define TICKS_PER_SECOND 1000

extern uint32_t GetData[6];

float K_v, v;
KFP KFP_V = {0.02, 0, 0, 0, 0.001, 0.543};

uint32_t TouchXData[6];
uint32_t TouchYData[6];
//uint32_t TouchZData[6];   //Z is for pressure measurement
uint8_t play[4] = {0xAA, 0x02, 0x00, 0xAC}; // 播放
uint8_t pause[4] = {0xAA, 0x03, 0x00, 0xAD}; // 暂停
uint8_t last[4] = {0xAA, 0x05, 0x00, 0xAF}; // 上一曲
uint8_t next[4] = {0xAA, 0x06, 0x00, 0xB0}; // 下一曲
uint8_t sound[5] = {0xAA, 0x13, 0x01, 0x14, 0xD2}; // 音量
//=============================================================================
/*
 *  米字管的参数、函数、数据传输
 */
volatile unsigned int Timernum = 0;

#define _NOP() _nop()

//*********************************************************************
//*********************************************************************
#define I2C0_MASTER_BASE 0x40020000
#define I2C0_SLAVE_BASE 0x40020000

//*********************************************************************
// 地址、寄存器等定义部分
//*********************************************************************
//*********************************************************************
//
// 设定slave（从）模块的地址。这是一个7-bit的地址加上RS位，具体形式如下:
//                      [A6:A5:A4:A3:A2:A1:A0:RS]
// RS位是一个指示位，如果RS=0，则说明是主发送数据，从接收数据；RS=1说明是主接收数据，从发送数据
//
//*********************************************************************
//U21控制4个米字管和特殊管脚的亮灭
#define I2C0_ADDR_TUBE_SEL	      0x30  //00110000
//U22控制米字管7~14管脚对应的码段
#define I2C0_ADDR_TUBE_SEG_LOW    0x32  //00110010
//U23控制米字管15~18管脚对应的码段
#define I2C0_ADDR_TUBE_SEG_HIGH  0x34   //00110100
//U24控制LED光柱

//PCA9557内部寄存器，也称子地址
#define PCA9557_REG_INPUT	 0x00
#define PCA9557_REG_OUTPUT	 0x01
#define PCA9557_REG_PolInver 0x02
#define PCA9557_REG_CONFIG	 0x03

//*************************************************************************************
 #define NUM 0
//IIC 接受数据临时缓冲区
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
		拉高 SDA 信号
********************************************/
void I2C_Set_sda_high( void )
{
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,GPIO_PIN_3);  //拉高PB3
    _NOP();
    _NOP();
    return;
}

/*******************************************
		拉低SDA 信号
********************************************/
void I2C_Set_sda_low ( void )
{
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0X00000000);  //拉低PB3
    _NOP();
    _NOP();
    return;
}

/*******************************************
		拉高SCL 信号
********************************************/
void I2C_Set_scl_high( void )
{
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,GPIO_PIN_2);  //拉高PB2
    _NOP();
    _NOP();
    return;
}

/*******************************************
		拉低SCL 信号
********************************************/
void I2C_Set_scl_low ( void )
{
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_2,0X00000000);  //拉低PB2
    _NOP();
    _NOP();
    return;
}

/*******************************************
		IIC 信号结束信号函数
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
		IIC 信号初始化
********************************************/
void I2C_Initial( void )
{
    I2C_Set_scl_low();
    I2C_STOP();
    return;
}


/*******************************************
		IIC 信号起始信号函数
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
		IIC 获取应答函数
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
		IIC 设置应答函数
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
		IIC 发送字节函数
		参数 	1：要发送字节值
		return ：无返回
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
		IIC 发送数组函数
	参数  	1 num : 发送字节数
	    2 device_addr : iic目标地址
	    3 *data	：发送数组地址
	return ：无返回
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
      I2C_START();           //模拟I2C写数据的时序
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
//******配置I2C0模块的IO引脚，**********************************************
void I2C0GPIOBEnable(void)
{	// Enable GPIO portB containing the I2C pins (PB2&PB3)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2|GPIO_PIN_3);

}

//******配置PCA9557芯片中连接米字管的各引脚为输出***********************************
void I2C0DeviceInit(void)
{
	unsigned char dataBuf[2] = {PCA9557_REG_CONFIG, 0x00};
	i2c_write(2,I2C0_ADDR_TUBE_SEL,dataBuf);
	i2c_write(2,I2C0_ADDR_TUBE_SEG_LOW,dataBuf);
	i2c_write(2,I2C0_ADDR_TUBE_SEG_HIGH,dataBuf);

}

//*******设置米字管的管选信号**************************************************
void I2C0TubeSelSet(char data)
{   //选择1、2、3、4、5哪个米字管亮
	unsigned char dataBuf[2] = {PCA9557_REG_OUTPUT, data};
	i2c_write(2,I2C0_ADDR_TUBE_SEL,dataBuf);
}
//*******点亮米字管的相应码段**************************************************
void I2C0TubeLowSet(char data)
{  //点亮7-14管脚对应的码段
	unsigned char dataBuf[2] = {PCA9557_REG_OUTPUT, data};
	i2c_write(2,I2C0_ADDR_TUBE_SEG_LOW,dataBuf);
}
void I2C0TubeHighSet(char data)
{  //点亮15-18管脚对应的码段
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
 * 配置UART和GPIO口模块
 */
//=============================================================
void InitConsole(void){

	// 使能UART0模块
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// 对PA0和PA1两个引脚功能进行选择 这里将其选择为执行UART0模块
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	// 配置PA0和PA1两个引脚UART功能
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// UART的标准初始化
	UARTStdioConfig(0, 115200, g_ui32SysClock);
}

//=============================================================
/*
 * MP3模块的UART初始化设置
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
 * 配置ADC模块
 */
//=============================================================
void ADCConfig(void){

	// 初始化UART
//	InitConsole();

	// 初始化ADC0/PE3
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);

	// 配置ADC采集序列
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH4 | ADC_CTL_END | ADC_CTL_IE);
	// 使能ADC采集序列
	ADCSequenceEnable(ADC0_BASE, 3);

	ADCIntClear(ADC0_BASE, 3);
}

//============================================================
/*
 * 按键初始化配置
 */
//============================================================
void GPIOInitial(void){
	// GPIOD管脚初始化配置
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);

	// GPIOD中断配置
//	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_LOW_LEVEL);
//	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_0);

	// GPION管脚初始化配置
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPION);
	GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_2 | GPIO_PIN_3);

	// GPIOP管脚初始化配置
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
//	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOP);
//	GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// GPIOK管脚初始化配置
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
//	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOK);
//	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
//	GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_3);

	// GPIOM管脚初始化配置
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOM);
//	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_5);


	// GPION中断配置
//	GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_LOW_LEVEL);
//	GPIOIntEnable(GPIO_PORTN_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3);

}

//=============================================================
/*
 * 配置计数器中断
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
 * 计数器中断函数--控制米字管显示
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
 * 中断控制图片旋转
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
	// 获取ADC中的滚轮值
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
 * 画三角
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
 * 画背景
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
 * 画图
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
 * 画暂停键
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
 * 画上一首键
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
 * 画下一首键
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
 * 画播放键
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
 * 清除播放键
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
 * 清除暂停键
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
 * 触摸对应的键时执行不同的功能
 */
//==============================================================
void TOUCH_PressKey(uint32_t* TouchXData, uint32_t* TouchYData){

	if(*TouchYData >= 290 && *TouchYData <= 350){
		// 按暂停键
		if(*TouchXData >= 100 && *TouchXData <= 150){
			if(pauseflag){
				TimerEnable(TIMER0_BASE, TIMER_B);
				TimerEnable(TIMER0_BASE, TIMER_A);
				// 蜂鸣器响
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
				// 蜂鸣器响
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
		// 按前一首歌的按钮
		if(*TouchXData >= 49 && *TouchXData <= 88){
		    if(cur_music > 1){
		        cur_music --;
		        // 蜂鸣器响
		        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x20);
		        SysCtlDelay(200000);
                GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x00);

                Timernum = 0;
                play_music(3);
		    }

			return;
		}
		// 按后一首歌的按钮
		if(*TouchXData >= 155 && *TouchXData <= 173){
		    if(cur_music < 2){
		        cur_music ++;
		        // 蜂鸣器响
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
 * 按键的判断
 */
//=================================================================
void identify_key(){

	// 暂停键 GPIO PN2口
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
	// 上一首播放键 GPIO PN3口
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
	// 下一首播放键GPIO PD0口
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
 * 播放MP3
 */
//======================================================================
void play_music(int number){

    int i;
	switch(number){
	case 1:
	    // 播放
//	    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x01);
//	    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
//	    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
	    for(i=0; i<4; i++){
	        UARTCharPut(UART6_BASE, play[i]);
	    }
		break;
	case 2:
	    // 暂停
//		GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x02);
//		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
//		GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
	    for(i=0; i<4; i++){
	        UARTCharPut(UART6_BASE, pause[i]);
	    }
		break;
	case 3:
	    // 上一首
//		GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x03);
//		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xFE);
//		GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0xff);
	    for(i=0; i<4; i++){
	        UARTCharPut(UART6_BASE, last[i]);
	    }
		break;
	case 4:
	    // 下一首
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
 * 音量控制
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
    // 临时变量设置
	uint16_t ui32Loop = 0,temp=0;

	//====================================================
	// FPU配置
    FPUEnable();
    FPULazyStackingEnable();

    //=====================================================
    // 设置时钟
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                SYSCTL_CFG_VCO_480), 120000000);
    //======================================================
    // 米字管的配置
	I2C0GPIOBEnable();//配置I2C0模块的IO引脚
	I2C0DeviceInit();//配置PCA9557芯片中连接米字管的各引脚为输出

	//======================================================
	// 初始化中断
	GPIOInitial();

    //======================================================
    // 设置定时器中断
    SysTickPeriodSet(g_ui32SysClock / 10000);
    TimerIntInitial();

    //======================================================
    // 配置UART以及MP3对应的UART模块
    ConfigureUART();
    UART_initial();

    //=======================================================
    // 显示屏初始化以及设置触屏
    EPIGPIOinit();
    TOUCH_TSC2046init(g_ui32SysClock);

    GPIOIntEnable(GPIO_PORTB_BASE,GPIO_INT_PIN_0);
    GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);

    TFT_400x240_OTM4001Ainit(g_ui32SysClock);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);

	SSIDataPut(SSI0_BASE,0xd0);

	//====================================================
	// 在显示屏上画出背景以及各类按钮
	DrawBackground();
	DrawStartKey(115, 300);
	DrawLastKey(65, 303);
	DrawNextKey(175, 303);

    //======================================================
    // 配置中断优先级以及开启定时器中断
    IntPrioritySet(INT_TIMER0B, 0x80);
    IntPrioritySet(INT_TIMER0A, 0xe0);
    IntEnable(INT_TIMER0B);
    IntEnable(INT_TIMER0A);
    IntMasterEnable();
    SysTickIntEnable();
    TimerEnable(TIMER0_BASE, TIMER_B);
    TimerEnable(TIMER0_BASE, TIMER_A);

    //======================================================
    // 配置ADC滚轮
    ADCConfig();

    //======================================================
    // 开始播放音乐
    play_music(1);

    while(1)
    {

    	temp++;
    	//=========================================================
    	// 读取触摸的位置
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
		// 判断触摸位置
		if(temp >= 200){
		    identify_key();
		    temp = 0;
		}
    }
}


