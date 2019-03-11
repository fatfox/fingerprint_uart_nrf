//使用轮询方式工作
#include "nrf51.h"
#include "nrf_gpio.h"

#define RX_PIN       (11)
#define TX_PIN       (9)

void uart_init(void){

    //设置引脚输入输出方向
    nrf_gpio_cfg_input(RX_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(TX_PIN);

    //设置输入输出引脚。 流控引脚置位无效值
    NRF_UART0->PSELRXD = RX_PIN;      
    NRF_UART0->PSELTXD = TX_PIN;
    NRF_UART0->PSELRTS = 0XFFFFFFFF;
    NRF_UART0->PSELCTS = 0XFFFFFFFF;

    NRF_UART0->BAUDRATE = 0x00275000;      //9600波特率
    NRF_UART0->CONFIG = 0;             //不使用流控，不使用校验

    //清零一下事件
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;  

    NRF_UART0->ENABLE = 4;          //开启uart
    NRF_UART0->TASKS_STARTRX = 1;   //使能接收
    NRF_UART0->TASKS_STARTTX = 1;   //使能发送

}

uint8_t get_uart_data(void){

    uint8_t temp;

    //轮询等待直到收到数据
    while(NRF_UART0->EVENTS_RXDRDY  == 0 );
    
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

int main(void){

    uint8_t data;
    uart_init();

    //死循环等待输入，电脑串口输入 0-9字符后板子会发送回去在电脑串口上    
    //显示
    while( 1 ){
       data = get_uart_data();
       if ( data <= '9' && data >= '0' ){
           send_uart_byte(data);
       }
    }
    return 0;
}

#define TA_FAIL 0x01
#define TA_SUCCESS 0x00
int16 Api_HandShakeFgpModule(uint32 timeov)
{
  int32 time;
  int16 result=TA_FAIL;
  uint8 cmd_buf[8],data_buf[8];
  time=timeov;
  
  Make_OneCmd(cmd_buf,0xfe,0,0); // 在 cmd_buf 内生成 0xfe 命令的命令序列
  // 规定的时间内没获取到即退出 
  while(time>0) {
    Api_Send_Data(cmd_buf,8,0); // 串口发送命令，此处是 10ms 循环发送
    result=Api_GetCommand(data_buf,10); // 获取命令反馈，超时 10ms 一次 
    if(result==TA_SUCCESS)
      break;
    time-=10;
  }

  if(result!=TA_SUCCESS)
    return -result;
  
  Api_Clr_Buf(); // 0xfe 命令已有返回，清空底层 BUF
  Make_OneCmd(cmd_buf,0xfd,0,0); // 生成第二个命令 0xfd
  Api_Send_Data(cmd_buf,8,0); // 发送 0xfd 命令序列 
  time=30; 
  
  while(time--){
    result=Api_GetCommand(data_buf,1); // 每隔 1ms 获取一次命令
    if(result==TA_SUCCESS && data_buf[1]==0xfd && data_buf[4]==TA_FAIL)
      break; // 判断命令 0xfd 是否正确返回 
  }
  
  if(time)
    return TA_SUCCESS; // 握手成功退出 
  else
    return TA_FAIL; // 握手失败退出 
}
