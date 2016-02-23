#include "stm32f10x_conf.h"
#include "FreeRtos.h"
#include "task.h"
#include "queue.h"

GPIO_InitTypeDef GPIO_InitStructure;

QueueHandle_t   QueTestId;


 
void RCC_Configuration(void);		//时钟配置函数  8MHz*9=72MHz
void GPIO_Configuration(void);
void USART_Configuration(void);

void delay(u16 t);
void delay_ms(u16 t);

void Task_Led(void *pvParameters)
{
    unsigned char i = 0;
    while (1)
    {
        xQueueSend(QueTestId, &i, 100);

        GPIO_SetBits(GPIOE,GPIO_Pin_4|GPIO_Pin_2);//set GPIOC.0=0delay_ms(0xffff);
        vTaskDelay(0x10);

        GPIO_ResetBits(GPIOE,GPIO_Pin_4|GPIO_Pin_2);
        vTaskDelay(0x10);
        i++;
    }

}


void Task_Led1(void *pvParameters)
{
    unsigned char ucRec = 0;
    while (1)
    {
        if (pdPASS == xQueueReceive(QueTestId, &ucRec, 100))
        {
            USART_SendData(USART1, ucRec);
		    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
        }
        GPIO_ResetBits(GPIOE, GPIO_Pin_5|GPIO_Pin_3);//set GPIOC.0=1 
        vTaskDelay(0x10);
        
        GPIO_SetBits(GPIOE,GPIO_Pin_5|GPIO_Pin_3);//set GPIOC.0=0
        vTaskDelay(0x10);
    }

}

int main(void)
{
    SystemInit();
    GPIO_Configuration();
    USART_Configuration();

    QueTestId = xQueueCreate(10 * sizeof(char), sizeof(char));
    xTaskCreate(Task_Led, "Task_Led", 128, 0, 2, 0);
    xTaskCreate(Task_Led1, "Task_Led1", 128, 0, 3, 0);

    vTaskStartScheduler();
	
	return 0;  
}

void GPIO_Configuration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	/*I/O方向 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	  	/*I/O输出速度=50M*/
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/********************************************************

//函数名称：USART_Configuration(void)
//功能描述：对串口1进行初始化
//输入参数：无
//返回：无

//说明：初始化分为两大块，即串口所使用I/O口的初始化和串口功能的初始化。
//对于USART串口要用的引脚，根据其数据方向，要设置其为GPIO_M_IN_FLOATING浮空输入
//或GPIO_Mode_AF_PP复用推挽输出，其他的和GPIO引脚设置一样。
********************************************************/
void USART_Configuration(void)
{
	USART_InitTypeDef	USART_InitStructure;
	GPIO_InitTypeDef 	GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	//使能串口1时钟		 高速时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//使能串口2时钟		 低速时钟
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//使能串口3时钟		 低速时钟
	//配置USART1 RX （PA.10）为浮空输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//配置USART1 RX （PA.9）为复用推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//******************************************************************************
	// 串口1参数初始化定义部分,串口1参数为38400 ， 8 ，1 ，N  接收中断方式
	//******************************************************************************
	USART_InitStructure.USART_BaudRate= 115200;						//设定传输速率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//传输数据位数
    USART_InitStructure.USART_StopBits = USART_StopBits_1; 			//设定停止位个数	
    USART_InitStructure.USART_Parity = USART_Parity_No; 			//不用检验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
																	//不用流量控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//使用发送和接收端口
	USART_Init(USART1,&USART_InitStructure);						//初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  				//使能串口1接收中断
	USART_Cmd(USART1,ENABLE);										//使能串口1
}

void delay_us(u16 t)
{
    unsigned int i = t * 100;
	while(--i);

}
void delay_ms(u16 t)
{
    char i,j = 0;
    for (i = 0; i < t; i++)
    {
        for (j = 0; j < 10; j++)
        {
            delay_us(t);
        }
    }
}

