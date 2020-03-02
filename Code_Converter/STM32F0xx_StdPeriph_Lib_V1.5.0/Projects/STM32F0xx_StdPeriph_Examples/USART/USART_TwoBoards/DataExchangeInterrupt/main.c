/**
  ******************************************************************************
  * @file    USART/USART_TwoBoards/DataExchangeInterrupt/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    24-July-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_DataExchangeInterrupt
  * @{
  */
/* 参数定义 ---------------------------------------------------------*/
	//SDI
#define SDI_TX PA2
#define RS485_TX_EN PA11

uint8_t SDI_12_EN = 0;//默认SDI端口处为接收模式,1发送，0接收
//接收缓存区 	
uint8_t SDI_RX_BUF[64];  	//接收缓冲,最大64个字节.
//接收到的数据长度
uint8_t SDI_RX_LEN=0; 

	//延时
static uint8_t  fac_us=0;//us延时倍乘数
static uint16_t fac_ms=0;//ms延时倍乘数


	//RS485
	
  uint8_t RS485_TX_EN = 0;//默认485端口处为接收模式,1发送，0接收
  
  //接收缓存区 	
uint8_t RS485_RX_BUF[64];  	//接收缓冲,最大64个字节.
//接收到的数据长度
uint8_t RS485_RX_LEN=0;  

/* Private function prototypes -----------------------------------------------*/
static void init(void);
static void delay_init(uint16_t SYSCLK);

void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
void USART2_IRQHandler(void);
void SDI_Receive_Data(uint8_t *buf,uint8_t *len);
void SDI_Send_signal(void);
void SDI_USART2_SendBuf(uint8_t *pBuf, uint8_t Len);
void SDI_CheckResponse(uint8_t *pBuf, uint8_t Len);
void USART1_IRQHandler(void);
void RS485_Receive_Data(uint8_t *buf,uint8_t *len);
void RS485_Send_Data(uint8_t *buf,uint8_t len);
/* Private functions ---------------------------------------------------------*/

int main(void)
{
	//初始化
	init();//串口，中断和IO端
	delay_init(8000000);//延时
  
  /* Enable the USARTx Receive interrupt: this interrupt is generated when the
  USARTx receive data register is not empty */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//使能接收中断
	

	while(1){
	//RS485接收
	
	
	
	
	}
  
  
}




 /* ************初始化 *******************/

static void init(void){
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//GPIOA组时钟使能  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//串口1时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//串口2时钟使能
  
  /* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);//复用串口2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//复用串口1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  
  
  /* Configure USART Tx and Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA9和PA10
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA2和PA3

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11
  
  /* USARTx configuration ----------------------------------------------------*/
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);//串口1初始化
  
  USART_InitStructure.USART_BaudRate = 1200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);//串口2初始化
  
  /* NVIC configuration */
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);//串口1中断初始化
  
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);//串口2中断初始化
  
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);//使能串口
  USART_Cmd(USART2, ENABLE);
	
  
 
}

 /* ************SDI-12 *******************/


//串口2接收服务函数
void USART2_IRQHandler(void){
	
	uint8_t res;
	//判断是否接收使能
	if(SDI_12_EN == 0){
	//首先判断是否是接收中断
	if(USART_GetITStatus(USART2,USART_IT_RXNE)){
		/*判断接收缓冲区是否为非空*/
    while(!USART_GetFlagStatus(USART2,USART_FLAG_RXNE));
		res = USART_ReceiveData(USART2);//全局变量获得接收值
		if(RS485_RX_LEN<64)
		{
			SDI_RX_BUF[RS485_RX_LEN] = res;		//记录接收到的值
			SDI_RX_LEN++;						//接收数据增加1 
		} 
	}
	}
	//如果超过64个字节怎么处理？
}

//SDI查询和接收手到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void SDI_Receive_Data(uint8_t *buf,uint8_t *len)
{
	uint8_t rxlen=RS485_RX_LEN;
	uint8_t i=0;
	*len=0;				//默认为0
	delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
	if(rxlen==SDI_RX_LEN&&rxlen)//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=SDI_RX_BUF[i];	
		}		
		*len=SDI_RX_LEN;	//记录本次数据长度
		RS485_RX_LEN=0;		//清零
	}
}

void SDI_Send_signal(void){
	SDI_TX = 1;//发送中断空号信号，持续12ms
	dalay_ms(12);
	SDI_TX = 0;//发送中断传号信号，持续8.33ms
	dalay_ms(8);
	dalay_us(330);
	SDI_12_EN = 1;//发送使能
}
//串口2发送数据
void SDI_USART2_SendBuf(uint8_t *pBuf, uint8_t Len)
{
	//发送命令
    while(Len--)
    {
        /*判断发送缓冲区是否为空*/
        while(!USART_GetFlagStatus(USART2,USART_FLAG_TXE));
        USART_SendData(USART2,*pBuf++);
    }

	SDI_TX = 0;//发送命令结束
	delay_ms(7);//在停止位后7.5ms内不能发送任何数据
	delay_us(500);
	SDI_12_EN = 0;//接收使能

}

	//检测是否需要重发
void SDI_CheckResponse(uint8_t *pBuf, uint8_t Len) {
	
	//第一种：在不超过87ms没有收到响应——发送函数需要处理，后面两个响应处理
	for(int i=0; i < 3; i++){
	Send_signal();//发送中断信号
	for(i=0; i < 3; i++){
	USART2_SendBuf(*pBuf, Len);//发送消息
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)){
	break;//接收到消息
	}
	delay_ms(25);//等待25ms再次重发
	}
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)){
	break;
	}
	delay_ms(100);//等待100ms后再次发送中断信号重发
	}
	}
	

 /* ************RS485 *******************/
  
  

//串口1接收服务函数
void USART1_IRQHandler(void){

	uint8_t res;
	//首先判断是否是接收中断
	if(USART_GetITStatus(USART1,USART_IT_RXNE)){
	//判断接收缓冲区是否为非空
    while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));
		res = USART_ReceiveData(USART1);//临时变量获得接收值
		if(RS485_RX_LEN<64)
		{
			RS485_RX_BUF[RS485_RX_LEN] = res;		//记录接收到的值
			RS485_RX_LEN++;						//接收数据增加1 
		} 
	}
}

//RS485查询和接收手到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data(uint8_t *buf,uint8_t *len)
{
	uint8_t rxlen=RS485_RX_LEN;
	uint8_t i=0;
	*len=0;				//默认为0
	delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
	if(rxlen==RS485_RX_LEN&&rxlen)//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS485_RX_BUF[i];	
		}		
		*len=RS485_RX_LEN;	//记录本次数据长度
		RS485_RX_LEN=0;		//清零
	}
}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(这里建议不要超过64个字节)
void RS485_Send_Data(uint8_t *buf,uint8_t len)
{
	uint8_t t;
	RS485_TX_EN=1;			//设置为发送模式
  	for(t=0;t<len;t++)		//循环发送数据
	{
	  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);//等待发送结束		
    USART_SendData(USART1,buf[t]); //发送数据
	}	 
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //等待发送结束		  
	RS485_RX_TEN=0;
	RS485_TX_EN=0;				//设置为接收模式	
}

	
	/* ************延时程序 *******************/


//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
static void delay_init(uint16_t SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2清空,选择外部时钟  HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
}								    
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_ms(uint16_t nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
}   
//延时nus
//nus为要延时的us数.		    								   
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}