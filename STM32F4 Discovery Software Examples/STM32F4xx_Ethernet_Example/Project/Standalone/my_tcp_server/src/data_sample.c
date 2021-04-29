#include "data_sample.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"
#include "tcp_echoserver.h"
#include <stdio.h>

#define 	SPI_BUFFSIZE 	12
#define   SPI_BUFFER_N	    10
#define 	SPI_DR_BASE 	(SPI3_BASE + 0x0C) //SPI3 数据寄存器地址
#define		SPI_DMA_STREAM DMA1_Stream0
#define		AD_SYNC_GPIO GPIOD
#define		AD_SYNC_GPIO_PIN	GPIO_Pin_8

uint8_t	ad_start_flag = 0;//AD启动标志
extern struct tcp_echoserver_struct *client_es;//接受的客户端连接
extern uint8_t tcp_echoserver_send_data(struct tcp_echoserver_struct *es,void *payload,uint16_t len);

unsigned char arr1[]={'1','2', '3', '4', '5'};
uint8_t spi_send_data[12] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t spi_recv_data[SPI_BUFFSIZE * SPI_BUFFER_N] = {0};//接收缓冲

uint8_t buffer_recv_index = 0;//当前接收缓冲指针
uint8_t buffer_send_index = 0;//当前发送缓冲指针
uint8_t buffer_flip = 0;//缓冲指针翻篇
uint8_t	buffer_overflow = 0;

static void NVIC_ADS1274(void);
static void SPI3_Init(void);
static void EXTI9_Init(void);
static void EXTI9_Enable(void);
static void EXTI9_Disable(void);


void NVIC_ADS1274(void)
{
	NVIC_InitTypeDef NVIC_Structure;

	/*************SPI_DMA_Rx*************/
	NVIC_Structure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Structure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Structure);

	/**************外部中断9************/
	NVIC_Structure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Structure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Structure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_Structure);
	

	
}


void SPI3_Init(void)
{

	GPIO_InitTypeDef SPI_GPIO;
	SPI_InitTypeDef  SPI_InitStruct;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);//使能SPI3时钟
	
/**********************************************************
SPI3:
	[NSS ] :O           PC10[SCK ] :I/O 
	[MOSI] :M-O/S-I     PC11[MISO] :M-I/S-O
**********************************************************/
	SPI_GPIO.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;//PC10~12复用功能输出
	SPI_GPIO.GPIO_Mode = GPIO_Mode_AF;//复用功能
	SPI_GPIO.GPIO_OType = GPIO_OType_PP;//推挽输出
	SPI_GPIO.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	SPI_GPIO.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &SPI_GPIO);//初始化	
	
	//GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3); //PC10复用为 SPI3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3); //PC11复用为 SPI3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3); //PC12复用为 SPI3	
	
	 /* SPI3 configuration */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_RxOnly;  //设置SPI双线双向单收
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     //设置SPI工作模式:设置为主SPI
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;     //设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;       //串行同步时钟的空闲状态为低电平
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;  //串行同步时钟的第一个跳变沿（上升或下降）数据被采样
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;     //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;        //定义波特率预分频的值:波特率预分频值为8
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;    //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStruct.SPI_CRCPolynomial = 7;  //CRC值计算的多项式
	SPI_Init(SPI3, &SPI_InitStruct);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器


	
	
	/******************DMA1_channel0_stream0_SPI3_Rx_Init******************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能
	DMA_DeInit(SPI_DMA_STREAM);
	while (DMA_GetCmdStatus(SPI_DMA_STREAM) != DISABLE){}//等待DMA可配置

  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = SPI_DR_BASE;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)spi_recv_data;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式
  DMA_InitStructure.DMA_BufferSize = SPI_BUFFSIZE;//数据传输量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(SPI_DMA_STREAM, &DMA_InitStructure);//初始化DMA Stream
	
	//SPI_Cmd(SPI3, ENABLE); //使能SPI外设 
	

	/*************SPI_DMA_Rx*************/
	DMA_Cmd(SPI_DMA_STREAM,ENABLE);
	SPI_I2S_DMACmd(SPI3,SPI_I2S_DMAReq_Rx,ENABLE);
	DMA_ITConfig(SPI_DMA_STREAM,DMA_IT_TC,ENABLE);
	DMA_ClearITPendingBit(SPI_DMA_STREAM,DMA_IT_TCIF0);
	
	

}



void EXTI9_Init(void)
{
  /*EXTI 9   -   PD9
	*/
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟 
	
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;                
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource9); 
	EXTI_InitStructure.EXTI_Line=EXTI_Line9;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);

}

void EXTI9_Enable(void)
{    
   EXTI_InitTypeDef EXTI_InitStructure;	
	 EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
}

void EXTI9_Disable(void)
{    
   EXTI_InitTypeDef EXTI_InitStructure;	
	 EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	 EXTI_Init(&EXTI_InitStructure);
}

void ADS1274_Config(void)
{
//	GPIO_InitTypeDef PWDN_GPIO;
	GPIO_InitTypeDef SYNC_GPIO;
	
	NVIC_ADS1274();
	SPI3_Init();
	EXTI9_Init();
	EXTI9_Enable();

	/**************PWDN 控制通道****************/
//	PWDN_GPIO.GPIO_Mode=GPIO_Mode_OUT ;
//	PWDN_GPIO.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
//	PWDN_GPIO.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOE,&PWDN_GPIO);
//	GPIO_SetBits(GPIOE,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);	
	
	/**************SYNC - PD8 控制启停****************/	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOD时钟
	
	SYNC_GPIO.GPIO_Mode=GPIO_Mode_OUT ;
	SYNC_GPIO.GPIO_Pin=AD_SYNC_GPIO_PIN;
	SYNC_GPIO.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&SYNC_GPIO);
	GPIO_ResetBits (AD_SYNC_GPIO,AD_SYNC_GPIO_PIN);	
	
}

/*	ADS1274数据准备好中断	*/
void EXTI9_5_IRQHandler(void)
{   
	EXTI9_Disable();
	DMA_Cmd(SPI_DMA_STREAM,ENABLE);
	EXTI_ClearITPendingBit(EXTI_Line9);
}


/* SPI3 DMA RX 完成中断   */
void SPI_DMA_STREAM_IRQHandler(void)
{
	DMA_Cmd(SPI_DMA_STREAM,DISABLE);
	DMA_ClearITPendingBit(SPI_DMA_STREAM,DMA_IT_TCIF0);
	DMA_ClearFlag(SPI_DMA_STREAM,DMA_FLAG_TCIF0);
	
	if(buffer_flip==1 && (buffer_recv_index + 1 == buffer_send_index))
	{
		buffer_overflow = 1;
		ad_start_flag = 0;
		return;
	}
	
	buffer_recv_index++;
	if(buffer_recv_index == SPI_BUFFER_N)
	{
		buffer_recv_index = 0;
		buffer_flip = 1;
	}
	
	//开始下一次采集
	DMA_SetCurrDataCounter(SPI_DMA_STREAM,SPI_BUFFSIZE);
	DMA_MemoryTargetConfig(SPI_DMA_STREAM, (uint32_t)spi_recv_data + buffer_recv_index * SPI_BUFFSIZE, DMA_Memory_0);
	EXTI9_Enable();
		
	
}

static void ADS1274_Start(void)
{
	uint8_t t = GPIO_ReadOutputDataBit(AD_SYNC_GPIO,AD_SYNC_GPIO_PIN);
	if(t==1)
		return;
	
	buffer_recv_index = 0;
	buffer_send_index = 0;
	buffer_overflow = 0;
	STM_EVAL_LEDOn(LED4);
	printf("START\r\n");	
	GPIO_SetBits(AD_SYNC_GPIO,AD_SYNC_GPIO_PIN);	//拉低禁止转换，拉高开始转换
}

static void ADS1274_Stop(void)
{
	uint8_t t = GPIO_ReadOutputDataBit(AD_SYNC_GPIO,AD_SYNC_GPIO_PIN);
	if(t==0)
		return;
	
	GPIO_ResetBits (AD_SYNC_GPIO,AD_SYNC_GPIO_PIN);	//拉低禁止转换，拉高开始转换
	STM_EVAL_LEDOff(LED4);
	printf("STOP\r\n");
}

void ADS1274_run(void)
{
	if(ad_start_flag == 0)
		ADS1274_Stop();
	else
		ADS1274_Start();
}

void ADS1274_tcp_send_data(void)
{
	uint16_t	n;//要发送的字节个数
	
	if(client_es==NULL || client_es->state!=ES_ACCEPTED)
		return;
	
	EXTI9_Disable();
	
	if(buffer_send_index == buffer_recv_index)
		goto exit;
	
	if(buffer_flip ==1)
	{
		n = (SPI_BUFFER_N - buffer_send_index) * SPI_BUFFSIZE;
		if(tcp_echoserver_send_data(client_es, spi_recv_data + buffer_send_index * SPI_BUFFSIZE ,n)==1)
		{
			buffer_send_index = 0;
			buffer_flip =0;
		}
	}
	else
	{
		n = (buffer_recv_index - buffer_send_index) * SPI_BUFFSIZE;
		if(tcp_echoserver_send_data(client_es, spi_recv_data + buffer_send_index * SPI_BUFFSIZE ,n)==1)
		{
			buffer_send_index = buffer_recv_index;
		}
	}
	
	exit:
		EXTI9_Enable();
	
	//测试发送数据
	//tcp_echoserver_send_data(client_es,arr1,5);
}




