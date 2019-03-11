/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 */

// SDK11

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

#include "SEGGER_RTT.h"

#define TX2FP              (6) // RX in fingerprint module
#define RX2FP              (7) // TX in fingerprint module

#define TA_FAIL            0x01
#define TA_SUCCESS         0x00

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;

/**
 * @brief Function for application main entry.
 */
//int main(void)
//{
//	int iii = 0;
//    // Configure LED-pins as outputs.
//    LEDS_CONFIGURE(LEDS_MASK);

//    // Toggle LEDs.
//    while (true)
//    {
//        for (int i = 0; i < LEDS_NUMBER; i++)
//        {
//            LEDS_INVERT(1 << leds_list[i]);
//            nrf_delay_ms(100);
//        }
//		//
//		SEGGER_RTT_printf(0, "Start %d\r\n", iii++);
//		
//		
//    }
//}


//#define RX_PIN       (11)
//#define TX_PIN       (9)

void uart_init(void){

    //设置引脚输入输出方向
    nrf_gpio_cfg_input(RX2FP, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(TX2FP);

    //设置输入输出引脚。 流控引脚置位无效值
    NRF_UART0->PSELRXD = RX2FP;      
    NRF_UART0->PSELTXD = TX2FP;
    NRF_UART0->PSELRTS = 0XFFFFFFFF;
    NRF_UART0->PSELCTS = 0XFFFFFFFF;

	
	// 改为 115200
	NRF_UART0->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos);
    //NRF_UART0->BAUDRATE = 0x00275000;      //9600 波特率
    NRF_UART0->CONFIG = 0;             //不使用流控，不使用校验

    //清零一下事件
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;  

    NRF_UART0->ENABLE = 4;          //开启 uart
    NRF_UART0->TASKS_STARTRX = 1;   //使能接收
    NRF_UART0->TASKS_STARTTX = 1;   //使能发送

}

uint8_t get_uart_data(){

    uint8_t temp;

    //轮询等待直到收到数据
    while(NRF_UART0->EVENTS_RXDRDY  == 0);
    
    temp = NRF_UART0->RXD;
    NRF_UART0->EVENTS_RXDRDY = 0;

    return temp;
}

void send_uart_byte(uint8_t data){
    uint8_t temp = data;

    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->TXD = temp;

    //轮询等待直到数据发送完毕
    while(NRF_UART0->EVENTS_TXDRDY == 0);
}

//void make_cmd(uint8_t * buf, ){
//	//
//}

uint8_t get_ack(uint8_t * buf){
	// 先不设超时判断
	//
	
	// fast!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	for(uint_fast8_t i = 0; i < 8; i++) {
		buf[i] = get_uart_data();
	}
	
	return TA_SUCCESS;
}

void send_data(uint8_t * buf){
	// 先不设超时判断
	//
	
	// fast!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	for(uint_fast8_t i = 0; i < 8; i++) {
		send_uart_byte(buf[i]);
	}
}

//int16_t Api_HandShakeFgpModule(uint32_t timeov)
//{
//  int32_t time;
//  int16_t result=TA_FAIL;
//  uint8_t cmd_buf[8], data_buf[8];
//  time=timeov;
//  
//  Make_OneCmd(cmd_buf,0xfe,0,0); // 在 cmd_buf 内生成 0xfe 命令的命令序列
//  // 规定的时间内没获取到即退出 
//  while(time>0) {
//    Api_Send_Data(cmd_buf,8,0); // 串口发送命令，此处是 10ms 循环发送
//    result=Api_GetCommand(data_buf,10); // 获取命令反馈，超时 10ms 一次 
//    if(result==TA_SUCCESS)
//      break;
//    time-=10;
//  }

//  if(result!=TA_SUCCESS)
//    return -result;
//  
//  Api_Clr_Buf(); // 0xfe 命令已有返回，清空底层 BUF
//  Make_OneCmd(cmd_buf,0xfd,0,0); // 生成第二个命令 0xfd
//  Api_Send_Data(cmd_buf,8,0); // 发送 0xfd 命令序列 
//  time=30; 
//  
//  while(time--){
//    result=Api_GetCommand(data_buf,1); // 每隔 1ms 获取一次命令
//    if(result==TA_SUCCESS && data_buf[1]==0xfd && data_buf[4]==TA_FAIL)
//      break; // 判断命令 0xfd 是否正确返回 
//  }
//  
//  if(time)
//    return TA_SUCCESS; // 握手成功退出 
//  else
//    return TA_FAIL; // 握手失败退出 
//}

int main(void){

	
	int iii = 0;
    // Configure LED-pins as outputs.
    LEDS_CONFIGURE(LEDS_MASK);

	SEGGER_RTT_printf(0, "Start: ");
    // Toggle LEDs.
    while (iii < 10)
    {
        for (int i = 0; i < LEDS_NUMBER; i++)
        {
            LEDS_INVERT(1 << leds_list[i]);
            nrf_delay_ms(100);
        }
		//
		SEGGER_RTT_printf(0, "%d ", iii++);
		
    }
	SEGGER_RTT_printf(0, "\n");
	
	
//    uint8_t data;
    uart_init();

//    //死循环等待输入，电脑串口输入 0-9字符后板子会发送回去在电脑串口上    
//    //显示
//    while( 1 ){
//       data = get_uart_data();
//       if ( data <= '9' && data >= '0' ){
//           send_uart_byte(data);
//       }
//    }

//	uint8_t cmd_buf[8];
	uint8_t data_buf[8];
	// 测试命令
	uint8_t cmd_test[8] = {0xF5, 0x09, 0x00, 0x00, 0x00, 0x00, 0x09, 0xF5};
	// 1:N 指纹比对命令
	uint8_t cmd_1_N[8] =  {0xF5, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xF5};
	// 某种灯光命令 F5 C3 03 06 C8 00 0E F5
	uint8_t cmd_LED[8] =  {0xF5, 0xC3, 0x03, 0x06, 0xC8, 0x00, 0x0E, 0xF5};


	
	// 发送命令
	// 测试
	send_data(cmd_test);
	
	SEGGER_RTT_printf(0, "Send Test \"%x\"... ", cmd_test[1]);
	
	get_ack(data_buf);
	
	SEGGER_RTT_printf(0, "ACK \"%x\"\n", data_buf[3]);
	
	nrf_delay_ms(500);
	
	
	
	// LED
	send_data(cmd_LED);	
	SEGGER_RTT_printf(0, "Set LED... ");	
	get_ack(data_buf);	
	SEGGER_RTT_printf(0, "ACK \"%x\"\n", data_buf[3]);	
	nrf_delay_ms(500);
	
	// 1:N 比对
	uint32_t c_time = 1;
	while(1) {
	
		send_data(cmd_1_N);		
//		SEGGER_RTT_printf(0, "(%d) 1:N Compare... ", c_time);		
		get_ack(data_buf);		
//		SEGGER_RTT_printf(0, "ACK %d\n", data_buf[3]);
		SEGGER_RTT_printf(0, "(%d)%d; ", c_time, data_buf[3]);	
		nrf_delay_ms(100);
		
		c_time++;
	}
	
	
//    return 0;
}



/** @} */
