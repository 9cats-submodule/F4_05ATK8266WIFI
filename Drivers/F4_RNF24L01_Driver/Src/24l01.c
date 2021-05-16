#include "24l01.h"
#include "lcd.h"
#include "spi.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//NRF24L01驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/5/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

const u8 TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; //发送地址
const u8 RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; //发送地址

void NRF24L01_SPI_Init(void)
{

	// SPI_InitTypeDef SPI_InitStructure;

	// SPI_Cmd(SPI1, DISABLE); //失能SPI外设

	// SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	 //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	// SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						 //设置SPI工作模式:设置为主SPI
	// SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					 //设置SPI的数据大小:SPI发送接收8位帧结构
	// SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							 //串行同步时钟的空闲状态为低电平
	// SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						 //串行同步时钟的第1个跳变沿（上升或下降）数据被采样
	// SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							 //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	// SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //定义波特率预分频的值:波特率预分频值为256
	// SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					 //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	// SPI_InitStructure.SPI_CRCPolynomial = 7;							 //CRC值计算的多项式
	// SPI_Init(SPI1, &SPI_InitStructure);									 //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	// SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	
  //以上内容用以下代替
  __HAL_SPI_DISABLE(&hspi1);

  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();

  __HAL_SPI_ENABLE(&hspi1);
}

//初始化24L01的IO口
void NRF24L01_Init(void)
{
	// GPIO_InitTypeDef GPIO_InitStructure;

	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOG, ENABLE); //使能GPIOB,G时钟

	// //GPIOB14初始化设置:推挽输出
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   //普通输出模式
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   //推挽输出
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //上拉
	// GPIO_Init(GPIOB, &GPIO_InitStructure);			   //初始化PB14

	// //GPIOG6,7推挽输出
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   //普通输出模式
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   //推挽输出
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //上拉
	// GPIO_Init(GPIOG, &GPIO_InitStructure);			   //初始化PG6,7

	// //GPIOG.8上拉输入
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //输入
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	// GPIO_Init(GPIOG, &GPIO_InitStructure);		 //初始化PG8

	// GPIO_SetBits(GPIOB, GPIO_Pin_14); //PB14输出1,防止SPI FLASH干扰NRF的通信

  //以上内容都已通过CubeMX配置
	SPI1_Init(); //初始化SPI1

	NRF24L01_SPI_Init(); //针对NRF的特点修改SPI的设置

	NRF24L01_CE = 0;  //使能24L01
	NRF24L01_CSN = 1; //SPI片选取消
}
//检测24L01是否存在
//返回值:0，成功;1，失败
u8 NRF24L01_Check(void)
{
	u8 buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
	u8 i;
	SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_8);				 //spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5); //写入5个字节的地址.
	NRF24L01_Read_Buf(TX_ADDR, buf, 5);					 //读出写入的地址
	for (i = 0; i < 5; i++)
		if (buf[i] != 0XA5)
			break;
	if (i != 5)
		return 1; //检测24L01错误
	return 0;	  //检测到24L01
}
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 NRF24L01_Write_Reg(u8 reg, u8 value)
{
	u8 status;
	NRF24L01_CSN = 0;				  //使能SPI传输
	status = SPI1_ReadWriteByte(reg); //发送寄存器号
	SPI1_ReadWriteByte(value);		  //写入寄存器的值
	NRF24L01_CSN = 1;				  //禁止SPI传输
	return (status);				  //返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;
	NRF24L01_CSN = 0;					//使能SPI传输
	SPI1_ReadWriteByte(reg);			//发送寄存器号
	reg_val = SPI1_ReadWriteByte(0XFF); //读取寄存器内容
	NRF24L01_CSN = 1;					//禁止SPI传输
	return (reg_val);					//返回状态值
}
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status, u8_ctr;
	NRF24L01_CSN = 0;				  //使能SPI传输
	status = SPI1_ReadWriteByte(reg); //发送寄存器值(位置),并读取状态值
	for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
		pBuf[u8_ctr] = SPI1_ReadWriteByte(0XFF); //读出数据
	NRF24L01_CSN = 1;							 //关闭SPI传输
	return status;								 //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status, u8_ctr;
	NRF24L01_CSN = 0;				  //使能SPI传输
	status = SPI1_ReadWriteByte(reg); //发送寄存器值(位置),并读取状态值
	for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
		SPI1_ReadWriteByte(*pBuf++); //写入数据
	NRF24L01_CSN = 1;				 //关闭SPI传输
	return status;					 //返回读到的状态值
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
	SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_8); //spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); //写数据到TX BUF  32个字节
	NRF24L01_CE = 1;										//启动发送
	while (NRF24L01_IRQ != 0)
		;											 //等待发送完成
	sta = NRF24L01_Read_Reg(STATUS);				 //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //清除TX_DS或MAX_RT中断标志
	if (sta & MAX_TX)								 //达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX, 0xff); //清除TX FIFO寄存器
		return MAX_TX;
	}
	if (sta & TX_OK) //发送完成
	{
		return TX_OK;
	}
	return 0xff; //其他原因发送失败
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;
	SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_8);			 //spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）
	sta = NRF24L01_Read_Reg(STATUS);				 //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //清除TX_DS或MAX_RT中断标志
	if (sta & RX_OK)								 //接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH); //读取数据
		NRF24L01_Write_Reg(FLUSH_RX, 0xff);					   //清除RX FIFO寄存器
		return 0;
	}
	return 1; //没收到任何数据
}
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (u8 *)RX_ADDRESS, RX_ADR_WIDTH); //写RX节点地址

	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);			  //使能通道0的自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);		  //使能通道0的接收地址
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 32);				  //设置RF通信频率
	NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //选择通道0的有效数据宽度
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);			  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);			  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式
	NRF24L01_CE = 1;											  //CE为高,进入接收模式
}
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了
//CE为高大于10us,则启动发送.
void NRF24L01_TX_Mode(void)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (u8 *)TX_ADDRESS, TX_ADR_WIDTH);	//写TX节点地址
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (u8 *)RX_ADDRESS, RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK

	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);	  //使能通道0的自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);  //使能通道0的接收地址
	NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1a); //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 32);		  //设置RF通道为40
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);	  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);	  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF24L01_CE = 1;									  //CE为高,10us后启动发送
}

//以下是SPI模块的初始化代码，配置成主机模式
//SPI口初始化
//这里针是对SPI1的初始化
void SPI1_Init(void)
{
	// GPIO_InitTypeDef GPIO_InitStructure;
	// SPI_InitTypeDef SPI_InitStructure;

	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOA时钟
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  //使能SPI1时钟

	// //GPIOFB3,4,5初始化设置
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; //PB3~5复用功能输出
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						//复用功能
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//推挽输出
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					//100MHz
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						//上拉
	// GPIO_Init(GPIOB, &GPIO_InitStructure);								//初始化

	// GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1); //PB3复用为 SPI1
	// GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1); //PB4复用为 SPI1
	// GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1); //PB5复用为 SPI1

	// //这里只针对SPI口初始化
	// RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);  //复位SPI1
	// RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE); //停止复位SPI1

	// SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	 //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	// SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						 //设置SPI工作模式:设置为主SPI
	// SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					 //设置SPI的数据大小:SPI发送接收8位帧结构
	// SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							 //串行同步时钟的空闲状态为高电平
	// SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						 //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	// SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							 //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	// SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //定义波特率预分频的值:波特率预分频值为256
	// SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					 //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	// SPI_InitStructure.SPI_CRCPolynomial = 7;							 //CRC值计算的多项式
	// SPI_Init(SPI1, &SPI_InitStructure);									 //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	// SPI_Cmd(SPI1, ENABLE); //使能SPI外设

  //以上内容都已通过CubeMX配置
  __HAL_SPI_ENABLE(&hspi1); //使能SPI1
	SPI1_ReadWriteByte(0xff); //启动传输
}
//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
//fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  __HAL_SPI_DISABLE(&hspi1);                    //??SPI1
	SPI1->CR1 &= 0XFFC7;											//位3-5清零，用来设置波特率
	SPI1->CR1 |= SPI_BaudRatePrescaler;								//设置SPI1速度
  __HAL_SPI_ENABLE(&hspi1);                    //??SPI1
}
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{
	while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);

	SPI1->DR = TxData;

	while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET); //等待接收完一个byte

	return SPI1->DR; //返回通过SPIx最近接收的数据
}

