# 项目介绍
这是一款半双工，SDI-12与RS485协议相互转换的有源信号转换器
# SDI-12转RS485模块

##
# 说明书


目录

第一章、        项目概述        4

产品概述        4

功能描述        4

参考文档        4

第二章、        总体设计路线        4

研发树状图        4

设计方向以及外壳样式        5

第三章、        通讯组件设计        6

第四章、        STM32最小系统设计        7

第五章、        电源组件设计        11

第六章、        模块软件设计        12

第七章、        实物展示        24

第八章、        转换器使用说明        26





## 第一章、项目概述

### 产品概述

设计一款半双工，SDI-12转RS485的有源信号转换模块

### 功能描述

- 小型化设计：50\*30mm，预留安装孔。
- 快速反应设计：数据在单片机停留时间尽可能短，读取到后直接传输出去
- 透明传输设计：双向数据不加修改不做变换直接传输
- 防水设计：安装在IP68防水外壳中
- 硬件接口：4pin接线柱，间距3.96

### 参考文档

- _MINI\_SDI-12转DA-串口TTL-RS485透传模块说明书\_20191014_
- _RELAYSCTRL.SchDoc_
- _光耦.SchDoc_
- _TEROS21传感器调研记录.docx_

## 第二章、总体设计路线

### 研发树状图

![1.png](https://i.loli.net/2020/03/02/9rZ63l7OYiCnjKp.png)

### 设计方向以及外壳样式

![2.png](https://i.loli.net/2020/03/02/cRNp8w9vyfn34hr.png)
![3.png](https://i.loli.net/2020/03/02/IqpGSEUVTjfORMo.png)

## 第三章、通讯组件设计

1. 芯片（模组）硬件手册，AT指令手册等数据资料
2. 规范绘制模组，元件封装并考虑日后重用
3. 依次设计原理图
4. 设计过程中考虑布局以及后期外壳制作的问题，充分考虑信号抗干扰、屏蔽保护的问题 
5. **具体通信模组：**

- TTL转485芯片

使用的是SN65LBC184芯片，功能是实现TTL转RS485协议

![4.png](https://i.loli.net/2020/03/02/6nwKzrCVkUSfEOX.png)
![5.png](https://i.loli.net/2020/03/02/STqOUkgbzEl35cn.png)

- TTL转SDI-12电路

使用的芯片是SN74AVC1T45，功能是使3.3V提升到3.6V输出或反向输入
![6.png](https://i.loli.net/2020/03/02/MgvJt3O6LxdaTeG.png)


## 第四章、STM32最小系统设计

1. 在功能满足，性能满足的条件下首选STM32F030C8T6
2. 外围电路参考最小系统板开发
3. 最小系统包括：主控芯片，复位电路，调试接口，外部晶振，电源，启动方式，去耦电路，指示灯


#### -主控芯片和能源

主控芯片使用的是STM32030C8T6,供电是3.3V能源
![7.png](https://i.loli.net/2020/03/02/hKao41OFv6bUPim.png)

#### -复位电路
![8.png](https://i.loli.net/2020/03/02/OEGyCzR3vMKPx56.png)

#### -调试接口

主芯片程序下载采用SWD接口电路，SWD接口包括VDD,SWIO,WOCLK,GND四线
![9.png](https://i.loli.net/2020/03/02/ZxJdCNAecsMa8vV.png)

#### -外部晶振
![10.png](https://i.loli.net/2020/03/02/RfxwYSjdM5ogC8U.png)

#### -启动方式
![11.png](https://i.loli.net/2020/03/02/HtlBgdesYkx3zwq.png)

#### -去耦电路

用于主控芯片的滤波，PCB设计时应分布在主控芯片能源附近
![12.png](https://i.loli.net/2020/03/02/81h2SFPNEbKnlyv.png)

#### -指示灯

 两盏状态指示灯LED1和LED2，进行系统状态的显示．LED10和KED11分别和主芯片GPIO管脚和地连接．串联电阻为限流电阻，防止电流过大损坏发光二极管
![13.png](https://i.loli.net/2020/03/02/bsJLACGu1IRgWv7.png)



## 第五章、电源组件设计

#### 开关电源12V转换为5V

使用芯片MP2315进行电压的转换
![14.png](https://i.loli.net/2020/03/02/QOz6Yqw5cHBELs7.png)


#### 5V转3.3V

使用芯片RT9193进行电压转换，主要给主控芯片供电
![15.png](https://i.loli.net/2020/03/02/KgDrUpouk2B7iYZ.png)

#### 12V转3.6V

使用芯片LM317进行电压转换，主要给TTL转SDI-12芯片供电
![16.png](https://i.loli.net/2020/03/02/Q84yYtlchrR92Cn.png)

## 第六章、模块软件设计

#### 1.程序设计框架

SDI-12转RS485转换器需要实现的功能是实现两种不同协议的半双工通信转换，其中的难点在于不同协议的实现和半双工。

SDI-12协议：因为SDI-12协议是反逻辑的，所以不能使用常规STM32模块提供的串口进行通信，所以需要通过IO口来模拟串口通信，然后实现SDI-12协议通信。

RS485协议: RS485协议是透明传输，使用STM32提供的串口即可实现

半双工通信：要想实现模块的半双工通信，需要实现三部分内容——SDI-接口端实现半双工收发，RS485接口端实现半双工收发，整体模块的半双工收发

![17.png](https://i.loli.net/2020/03/02/ZRjTNGaKQwgXhWA.png)
#### 2.初始化

##### - 延时初始化	
```C
//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SystemCoreClock:系统时钟
void delay_init(void)

{

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//systick时钟= HCLK/8 
	fac_us = SystemCoreClock/8000000;
	fac_ms = fac_us*1000;
}								    
```

##### - RS485初始化
```C
void RS485_init(void){
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*时钟*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//GPIOA组时钟使能  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);//GPIOC组时钟使能  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//串口1时钟使能

  /* 复用IO口*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//复用串口1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  
  
  /* 初始化复用口 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;//50MHZ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽模式
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA9和PA10
	 
  /*初始化IO口*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA3，控制SDI的输入输出
	GPIO_ResetBits(GPIOA ,GPIO_Pin_3);//默认接收，0接收，1发送

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11，控制RS485输入输出
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);//PA13初始化IO口，显示输入输出标志
	
  
  /* USARTx初始化 ----------------------------------------------------*/
  USART_InitStructure.USART_BaudRate = 9600;//波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//不需要奇偶位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//接收和发送
  USART_Init(USART1, &USART_InitStructure);//串口1初始化
  
  /* NVIC初始化*/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);//串口1中断初始化
  
  
  /* 使能串口 */
  USART_Cmd(USART1, ENABLE);//使能串口1
 
}
```


##### - SDI-12初始化

A.IO模拟串口

因为传感器的时序是反逻辑的，所以STM32的串口是无法使用，所以需要自己使用IO口去模拟串口收发数据

a.接收串口IO
```C
//接收初始化
void VirtualCOM_RX_GPIOConfig(void)

{

EXTI_InitTypeDef EXTI_InitStructure;

NVIC_InitTypeDef NVIC_InitStructure;

GPIO_InitTypeDef GPIO_InitStructure;
	

/* PA2为数据输入，模拟RX */
GPIO_InitStructure.GPIO_Pin = COM_RX_PIN;//PA2
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推免
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉（SDI12是负逻辑）
GPIO_Init(COM_RX_PORT, &GPIO_InitStructure);//PA

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);//PA2映射到中断线2

EXTI_InitStructure.EXTI_Line=EXTI_Line2;	
EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;//上升沿中断
EXTI_InitStructure.EXTI_LineCmd=ENABLE;
EXTI_Init(&EXTI_InitStructure);


NVIC_InitStructure.NVIC_IRQChannel=EXTI2_3_IRQn; //外部中断，边沿触发
NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
NVIC_Init(&NVIC_InitStructure);
EXTI_ClearITPendingBit(EXTI_Line2);//清除中断标志

GPIO_ResetBits(GPIOA ,GPIO_Pin_3);//0是接收

}
```


 b.发送IO初始化
```C
//发送初始化
void VirtualCOM_TX_GPIOConfig(void)
{

GPIO_InitTypeDef GPIO_InitStructure;

/* PA2最为数据输出口，模拟TX */
GPIO_InitStructure.GPIO_Pin = COM_TX_PIN;//PA2
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推免
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉，SDI12是负逻辑
GPIO_Init(COM_RX_PORT, &GPIO_InitStructure);//
GPIO_SetBits(COM_TX_PORT, COM_TX_PIN);
	

}
```


B.SDI-12半双工初始化

因为SDI-12并没有硬件上的半双工管脚，所以使用软件使SDI-12接口半双工

a.发送半双工
```C
//SDI半双工，发送
void Change_TX(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = COM_RX_PIN;//PA2
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推免
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉，SDI12是负逻辑
GPIO_Init(COM_TX_PORT, &GPIO_InitStructure);//PA
	
	GPIO_SetBits(GPIOA ,GPIO_Pin_3);//1是发送
	
	//关闭SDI外部中断
	NVIC_InitStructure.NVIC_IRQChannel=EXTI2_3_IRQn; //外部中断，边沿触发
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}
```


 b.接收半双工
```C
//SDI半双工，接收
void Change_RX(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
GPIO_InitStructure.GPIO_Pin = COM_RX_PIN;//PA2
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推免
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉（SDI12是负逻辑）
GPIO_Init(COM_RX_PORT, &GPIO_InitStructure);//PA
	
	GPIO_ResetBits(GPIOA ,GPIO_Pin_3);//0是接收
	
	//外部中断打开
	NVIC_InitStructure.NVIC_IRQChannel=EXTI2_3_IRQn; //外部中断，边沿触发
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line2);//清除中断标志
}
```


C.定时器初始化

 需要一个定时器来按照一定频率去读取模拟IO口的数据
```C
//定时器初始化
void TIM3_Configuration(u16 period)
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

NVIC_InitTypeDef NVIC_InitStructure;

RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//使能TIM3的时钟


TIM_TimeBaseStructure.TIM_Prescaler = 48 - 1; //预分频系数为48，这样计数器时钟为48MHz/48 = 1MHz
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分频
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//设置计数器模式为向上计数模式
TIM_TimeBaseStructure.TIM_Period = period - 1; //设置计数溢出大小，每计period个数就产生一个更新事件
TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);	//将配置应用到TIM3中

TIM_ClearFlag(TIM3, TIM_FLAG_Update);	//清除溢出中断标志
TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //开启TIM3的中断
TIM_Cmd(TIM3,DISABLE);	//关闭定时器TIM3


NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	//通道设置为TIM3中断
NVIC_InitStructure.NVIC_IRQChannelPriority = 1;//中断优先级1
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//打开中断
NVIC_Init(&NVIC_InitStructure);	
}
```



D.设置波特率

 设置固定的频率给模拟串口IO
```C
//设置波特率
void SDI_init(u16 baudRate)
{
u32 period;
VirtualCOM_TX_GPIOConfig();
VirtualCOM_RX_GPIOConfig();
if(baudRate == _300BuadRate)	//波特率300
period = _300BuadRate + 300;
else if (baudRate == _600BuadRate)	//波特率600
period = _600BuadRate + 300;
else if (baudRate == _1200BuadRate)	//波特率1200
period = _1200BuadRate + 50;//采样
TIM3_Configuration(period);	//设置对应模特率的定时器的定时时间
delayTime = baudRate;	//设置IO串口发送的速率
}
```


#### 3.Delay方法

 延时程序使用非常频繁，使设置波特率的关键，所以需要结合STM32的系统频率来编写准确的延时程序。
```C
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_ms(uint16_t nms)
{	 		  	  
	uint32_t temp;
	
	SysTick->LOAD = nms*fac_ms;//时间加载
	SysTick->VAL = 0x00;        //清除计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;///打开systick定时器，开始倒计时
	
	do
	{
		temp = SysTick->CTRL;
	}while((temp&0x01) && !(temp&(1<<16)));                                
	
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;//关闭systick定时器
	SysTick->VAL = 0x00;//清除计数器


}   
//延时nus
//nus为要延时的us数.		    								   
void delay_us(uint32_t nus)
{		
uint32_t temp;
	
	SysTick->LOAD = nus*fac_us;                  //时间加载
	SysTick->VAL = 0x00;                        //清除计数器

	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;//打开systick定时器，开始倒计时
	do
	{
		temp = SysTick->CTRL;
	}while((temp&0x01) && !(temp&(1<<16)));
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;//关闭systick定时器
	SysTick->VAL = 0x00;//清除计数器
}
```


#### 4.SDI-12方法

 SDI-12的方法分为有三部分：接收数据，解析数据和发送数据。在软件的设计上，我们通过外部中断来相应数据的接收（注意是反逻辑），然后通过定时器按照一定频率来解析接收到的时序，最后按照SDI-12协议发送数据

A.外部中断接收函数
```C
enum{
COM_START_BIT,	//停止位
COM_D0_BIT,	//bit0
COM_D1_BIT,	//bit1
COM_D2_BIT,	//bit2
COM_D3_BIT,	//bit3
COM_D4_BIT,	//bit4
COM_D5_BIT,	//bit5
COM_D6_BIT,	//bit6
COM_D7_BIT,	//bit7
COM_STOP_BIT,	//bit8
};


u8 recvStat = COM_STOP_BIT;	//定义状态机

//外部中断，字节起始位和启动定时中断
void EXTI2_3_IRQHandler(void)
{
	Flag=0;
	SDI_RX_FLAG = 1;//接收到数据 	
	if(EXTI_GetITStatus(EXTI_Line2)!=RESET)
	{
	if(COM_RX_STAT) //检测引脚高低电平，如果是高电平，则说明检测到上升沿
	{
		
	if(recvStat == COM_STOP_BIT)	//状态为停止位
	{
	recvStat = COM_START_BIT;	//接收到开始位
	TIM_Cmd(TIM3, ENABLE);	//打开定时器，接收数据
	}
	}
	EXTI_ClearITPendingBit(EXTI_Line2);	//清除EXTI_Line2中断挂起标志位	
	}
	
}
```


B.定时器解析数据函数
```C
//定时中断，定时接收字节，负逻辑
void TIM3_IRQHandler(void)
{
if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检测是否发生溢出更新事件
{
TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);//清除中断标志
recvStat++;	//改变状态机
len++;
if(recvStat == COM_STOP_BIT) //收到停止位
{
	TIM_Cmd(TIM3, DISABLE);	//关闭定时器
	SDI_BUF[SDI_LEN] = recvData;	
	SDI_LEN++;
	len=0;
	return; //并返回
}
if(len==8 && !COM_RX_STAT){
	recvData |= (1 << (recvStat - 1));//固定第八位为0(负逻辑为1)
	return;
}
if(COM_RX_STAT) //'1'，负逻辑是0
{
recvData |= (1 << (recvStat - 1));
}
else	//'0'负逻辑是1
{
recvData &= ~(1 <<(recvStat - 1));
}
}
}
```


C.SDI-12发送数据函数
```C
//发送一个字节，取反发送
void VirtualCOM_ByteSend(u8 val)
{
u8 i = 0;
GPIO_SetBits(COM_TX_PORT, COM_TX_PIN);	//起始位
delay_us(delayTime);
for(i = 0; i < 8; i++)	//8位数据位
{
if(val & 0x01)
GPIO_ResetBits(COM_TX_PORT, COM_TX_PIN);
else
GPIO_SetBits(COM_TX_PORT, COM_TX_PIN);
delay_us(delayTime);
val >>= 1;
}
GPIO_ResetBits(COM_TX_PORT, COM_TX_PIN);	//停止位
delay_us(delayTime);

}

//发送一串字符串
void VirtualCOM_StringSend(u8 *str,u8 len)
{
	u8 t;
	for(t = 0; t < len; t++)
	{
		VirtualCOM_ByteSend(str[t]);
	}

}
//发送中断空号信号
void SDI_Send_signal(void){
	GPIO_SetBits(COM_TX_PORT, COM_TX_PIN);//发送中断空号信号，持续12ms
	delay_ms(13);
	GPIO_ResetBits(COM_TX_PORT, COM_TX_PIN);//发送中断传号信号，持续8.33ms
	delay_ms(8);
	delay_us(530);
}
	//检测是否需要重发,在不超过87ms不需要发送中断信号，之后仍然没有收到响应,100ms后传感器会进入睡眠状态
void SDI_CheckTimeOutAndSend(uint8_t *pBuf, uint8_t Len) {
	
	int i = 0;
	int k = 0;
	for(k=0; k < 3; k++){
	Change_TX();
	SDI_Send_signal();//发送中断信号
		for(i=0; i < 3; i++){
			Change_TX();
			VirtualCOM_StringSend(pBuf,Len);//PA2发送消息
			Change_RX();
			SDI_RX_FLAG = 0;
			delay_ms(25);//等待25ms后判断传感器是否响应，否则需要重发
			if(SDI_RX_FLAG == 1){
				break;//接收到消息
				}
		}
		
		if(SDI_RX_FLAG == 1){
			break;
		}
		delay_ms(100);//等待100ms后再次发送中断信号重发
	}
	buf_len = 0;//RS485缓冲区清零
	RS485_RX_LEN=0;	//清零
	SDI_RX_FLAG = 0;//接收标志位清0
}

//取反
void NOT(u8 *buf,u8 len){
	int i=0;
	for(i=0;i<len;i++){
	buf[i] = ~buf[i];//取反
	}
}
```



#### 5.RS485方法

RS485的方法分为有三部分：接收数据，解析数据和发送数据。在软件的设计上，我们通过串口来相应数据的接收，然后需要解析接收到的数据，最后按照RS485协议发送数据

A.接收数据
```C
//串口1接收服务函数
void USART1_IRQHandler(void){
	uint8_t res1;
	Flag = 1;//半双工使能
	//首先判断是否是接收中断
	if(USART_GetITStatus(USART1,USART_IT_RXNE)){
	//判断接收缓冲区是否为非空
    while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));
		res1 = USART_ReceiveData(USART1);//临时变量获得接收值
		if(RS485_RX_LEN<36)
		{
			RS485_RX_BUF[RS485_RX_LEN] = res1;		//记录接收到的值
			RS485_RX_LEN++;						//接收数据增加1 
		}
	}

}
```


B.解析数据
```C
/RS485查询和接收手到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data(uint8_t *buf,uint8_t *len)
{
	uint8_t rxlen=RS485_RX_LEN;
	uint8_t i=0;
	*len=0;				//默认为0，*len是方便传参，否则void被调函数无法将变量传递出去
	delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
	if(rxlen==RS485_RX_LEN&&rxlen)//接收到了数据,且接收完成了；==优先级高，先判断是否==再相与是否为空
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS485_RX_BUF[i];	
		}		
		*len=RS485_RX_LEN;	//记录本次数据长度
	}
}
```


C.发送数据
```C
//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(这里建议不要超过64个字节)
void RS485_Send_Data(uint8_t *buf,uint8_t len)
{
	uint8_t t;
	GPIO_SetBits(GPIOA, GPIO_Pin_11);	//设置为发送模式
  	for(t=0;t<len;t++)		//循环发送数据
	{
	  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET)
	  {
	  };//等待发送结束		
    USART_SendData(USART1,buf[t]); //发送数据
	}	 
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //等待发送结束	
	SDI_LEN = 0; //SDI接收缓冲归零	
	GPIO_ResetBits(GPIOA, GPIO_Pin_11);//设置为接收模式	
}

```


#### 主方法

实现RS485接收，SDI-12发送；

SDI-12接收，RS485接收

整体模块半双工通信

##### 1.参数定义和方法声明

```C
/* 头文件------------------------------------------------------------------*/
#include "main.h"
/* 参数定义 ---------------------------------------------------------*/

/*****SDI*************/
//波特率
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define _300BuadRate 3150
#define _600BuadRate 1700
#define _1200BuadRate 833
//采样
#define COM_RX_STAT GPIO_ReadInputDataBit(COM_RX_PORT, COM_RX_PIN)
//缓冲
u8 SDI_BUF[36];
u8  SDI_LEN = 0;
u8 recvData;
u32 delayTime;//1比特需要发送和接收的时间
#define COM_RX_PORT GPIOA
#define COM_RX_PIN GPIO_Pin_2
#define COM_TX_PORT GPIOA
#define COM_TX_PIN GPIO_Pin_2
#define COM_DATA_HIGH()	GPIO_SetBits(COM_TX_PORT, COM_TX_PIN) //高电平
#define COM_DATA_LOW()	GPIO_ResetBits(COM_TX_PORT, COM_TX_PIN) //低电平

uint8_t SDI_12_EN = 0;//默认SDI端口处为接收模式,1发送，0接收
//接收缓存区 	
uint8_t SDI_RX_BUF[36];  	//接收缓冲,最大64个字节.
//接收到的数据长度
uint8_t SDI_RX_LEN=0; 
//接收标志位
uint8_t SDI_RX_FLAG = 0;//1是数据接收到了，0是没有接收到


	//延时
static uint8_t  fac_us=0;//us延时倍乘数
static uint16_t fac_ms=0;//ms延时倍乘数

/*******RS485**********/
  //接收缓存区 	
uint8_t RS485_RX_BUF[36];  	//接收缓冲,最大36个字节.
//接收到的数据长度
uint8_t RS485_RX_LEN=0; 
uint8_t len = 0;

uint8_t Flag = 0;//半双工使能，flag=0，SDI→RS485；flag=1，RS485→SDI
uint8_t buf[36];//缓冲区
uint8_t buf_len = 0; //接收的数据长度

	uint8_t res1 = 0;
/* 函数声明 -----------------------------------------------*/
void RS485_init(void);
void delay_init(void);

void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
void USART2_IRQHandler(void);
void SDI_Receive_Data(uint8_t *buf,uint8_t *len);
void SDI_Send_signal(void);
void SDI_USART2_SendBuf(uint8_t *pBuf, uint8_t Len);
void SDI_CheckTimeOutAndSend(uint8_t *pBuf, uint8_t Len);
void USART1_IRQHandler(void);
void RS485_Receive_Data(uint8_t *buf,uint8_t *len);
void RS485_Send_Data(uint8_t *buf,uint8_t len);

void Change_TX(void);
void Change_RX(void);
void VirtualCOM_RX_GPIOConfig(void);
void VirtualCOM_TX_GPIOConfig(void);
void SDI_init(u16 baudRate);
void TIM3_Configuration(u16 period);
void VirtualCOM_ByteSend(u8 val);
void VirtualCOM_StringSend(u8 *str, u8 len);
void NOT(u8 *buf,u8 len);
```


##### 2.主程序
```C
/* 主函数 ---------------------------------------------------------*/

int main(void)
{	
	
	delay_init();
	RS485_init();
	SDI_init(_1200BuadRate);//设置波特率和SDI接收发送IO模拟串口初始化
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//使能接收中断
	GPIO_ResetBits(GPIOA, GPIO_Pin_11);//默认485端口处为接收模式,1发送，0接收
while(1){

		if(Flag){  //串口使能标志
			//RS485接收，SDI发送
			RS485_Receive_Data(buf,&buf_len);
			if(buf_len != 0){
				SDI_CheckTimeOutAndSend(buf,buf_len);
			}
		} else{
			//SDI接收,RS485发送
			if(SDI_LEN != 0) {
			delay_ms(500);
			NOT(SDI_BUF,SDI_LEN);//取反
			RS485_Send_Data(SDI_BUF, SDI_LEN);                                 
			}
		}
		}
	
}
```


## 第七章、实物展示

#### 1.原理图
![18.png](https://i.loli.net/2020/03/02/JgCslUR1yf63qNH.png)

#### 2.PCB图
![19.png](https://i.loli.net/2020/03/02/PXoeJFvbWYdc7NI.png)

#### 3.实物图
![20.png](https://i.loli.net/2020/03/02/9riYcfSvKV6uLeT.png)
## 第八章、转换器使用说明

1.连接传感器：SDI-12接口有三个连接端——+12V，DATA, GND，分别连接传感器的三端（千万别搭错线，容易烧坏移位电位器芯片）

2.连接RS485器件：RS485接口有4个连接端——+12V，A，B，GND，分别连接RS485器件4端

3.通电，使用电脑发送命令即可

4.**以下为通信的实物展示**

![21.jpg](https://i.loli.net/2020/03/02/4Xat3uzZo2J1cvp.jpg)
