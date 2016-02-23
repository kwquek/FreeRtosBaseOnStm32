#include "stm32f10x_conf.h"
#include "FreeRtos.h"
#include "task.h"
#include "queue.h"

GPIO_InitTypeDef GPIO_InitStructure;

QueueHandle_t   QueTestId;


 
void RCC_Configuration(void);		//ʱ�����ú���  8MHz*9=72MHz
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	/*I/O���� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	  	/*I/O����ٶ�=50M*/
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/********************************************************

//�������ƣ�USART_Configuration(void)
//�����������Դ���1���г�ʼ��
//�����������
//���أ���

//˵������ʼ����Ϊ����飬��������ʹ��I/O�ڵĳ�ʼ���ʹ��ڹ��ܵĳ�ʼ����
//����USART����Ҫ�õ����ţ����������ݷ���Ҫ������ΪGPIO_M_IN_FLOATING��������
//��GPIO_Mode_AF_PP������������������ĺ�GPIO��������һ����
********************************************************/
void USART_Configuration(void)
{
	USART_InitTypeDef	USART_InitStructure;
	GPIO_InitTypeDef 	GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	//ʹ�ܴ���1ʱ��		 ����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//ʹ�ܴ���2ʱ��		 ����ʱ��
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//ʹ�ܴ���3ʱ��		 ����ʱ��
	//����USART1 RX ��PA.10��Ϊ��������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//����USART1 RX ��PA.9��Ϊ�����������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//******************************************************************************
	// ����1������ʼ�����岿��,����1����Ϊ38400 �� 8 ��1 ��N  �����жϷ�ʽ
	//******************************************************************************
	USART_InitStructure.USART_BaudRate= 115200;						//�趨��������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//��������λ��
    USART_InitStructure.USART_StopBits = USART_StopBits_1; 			//�趨ֹͣλ����	
    USART_InitStructure.USART_Parity = USART_Parity_No; 			//���ü���λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
																	//������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ʹ�÷��ͺͽ��ն˿�
	USART_Init(USART1,&USART_InitStructure);						//��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  				//ʹ�ܴ���1�����ж�
	USART_Cmd(USART1,ENABLE);										//ʹ�ܴ���1
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

