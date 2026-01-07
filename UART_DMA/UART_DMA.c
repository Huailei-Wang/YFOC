#include "UART_DMA.h"
#include <stdio.h>
#include <stdarg.h>
// extern DMA_HandleTypeDef hdma_usart1_tx;
char updata_info1[50];
char Serial_RxPacket[100];				//定义接收数据包数组，数据包格式"@MSG\r\n"
uint8_t Serial_RxFlag;					//定义接收数据包标志位
void Serial_SendByte(uint8_t Byte)
{
	HAL_UART_Transmit(&huart1, &Byte, 1, HAL_MAX_DELAY);  // 发送 1 个字节，使用最大超时值
}
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}
void Serial_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial_SendString(String);		//串口发送字符数组（字符串）
}

void Float_to_Byte(float Fdata, uint8_t *ArrayByte)
{
    Vofa_Type Vofa;                  // 定义Vofa_Type类型的Vofa变量
    Vofa.Fdata   = Fdata;            // 把需要操作的浮点数复制到共同体的Fdata变量中
    ArrayByte[0] = Vofa.Adata;       // 0-7位移到数组元素0
    ArrayByte[1] = Vofa.Adata >> 8;  // 8-15位移动到数组元素1
    ArrayByte[2] = Vofa.Adata >> 16; // 16-23位移动到数组元素2
    ArrayByte[3] = Vofa.Adata >> 24; // 24-31位移动到数组元素3
}
void DMA_to_Vofa(float angle, float speed, float value)
{
    static uint8_t Byte[4];
    static uint8_t Byte1[4];
    static uint8_t Byte2[4];  // 新增的用于存储第三个浮点数的字节数组
    static uint8_t Tail[4] = {0x00, 0x00, 0x80, 0x7f}; // 定义包尾数组

    // 定义用于发送的总数组，长度为 16 字节（3个浮点数 + 包尾）
    static uint8_t All[16];

    // 将浮点数转换为字节存放到 Byte, Byte1, Byte2 中
    Float_to_Byte(angle, Byte);  // 转换第一个浮点数
    Float_to_Byte(speed, Byte1); // 转换第二个浮点数
    Float_to_Byte(value, Byte2); // 转换第三个浮点数

    // 合并三个数组到 All 数组中
    for (int i = 0; i < 4; i++) {
        All[i]     = Byte[i];   // 复制 Byte 中的4个字节到 All
        All[i + 4] = Byte1[i];  // 复制 Byte1 中的4个字节到 All
        All[i + 8] = Byte2[i];  // 复制 Byte2 中的4个字节到 All
        All[i + 12] = Tail[i];  // 复制 Tail 中的4个字节到 All
    }

    // 使用 DMA 一次性发送整个 All 数组
    HAL_UART_Transmit_DMA(&huart1, All, 16);
}
void DMA_to_Vofa_v5(float angle, float speed, float value1, float value2, float value3)
{
    static uint8_t Byte[4];
    static uint8_t Byte1[4];
    static uint8_t Byte2[4];
    static uint8_t Byte3[4];  // 新增的字节数组用于存储第四个浮动数字
    static uint8_t Byte4[4];  // 新增的字节数组用于存储第五个浮动数字
    static uint8_t Tail[4] = {0x00, 0x00, 0x80, 0x7f}; // 定义包尾数组

    // 定义用于发送的总数组，长度为 24 字节（5个浮点数 + 包尾）
    static uint8_t All[24];

    // 将浮动数字转换为字节并存储到相应的字节数组
    Float_to_Byte(angle, Byte);   // 转换第一个浮动数
    Float_to_Byte(speed, Byte1);  // 转换第二个浮动数
    Float_to_Byte(value1, Byte2); // 转换第三个浮动数
    Float_to_Byte(value2, Byte3); // 转换第四个浮动数
    Float_to_Byte(value3, Byte4); // 转换第五个浮动数

    // 合并五个浮动数字的字节到 All 数组中
    for (int i = 0; i < 4; i++) {
        All[i]     = Byte[i];     // 复制 Byte 中的4个字节到 All
        All[i + 4] = Byte1[i];    // 复制 Byte1 中的4个字节到 All
        All[i + 8] = Byte2[i];    // 复制 Byte2 中的4个字节到 All
        All[i + 12] = Byte3[i];   // 复制 Byte3 中的4个字节到 All
        All[i + 16] = Byte4[i];   // 复制 Byte4 中的4个字节到 All
        All[i + 20] = Tail[i];    // 复制 Tail 中的4个字节到 All
    }

    // 使用 DMA 一次性发送整个 All 数组
    HAL_UART_Transmit_DMA(&huart1, All, 24);
}
void DMA_Send_Debug(int _data1)
{
    // 预定义字符数组，用于存储要发送的数据

    // 处理正负号
    if (_data1 >= 0)
        updata_info1[0] = '+';
    else {
        updata_info1[0] = '-';
        _data1          = -_data1; // 将负数转为正数处理
    }

    // 将数值转换为字符，确保显示到100000的位
    updata_info1[1] = (_data1 / 100000) % 10 + '0'; // 十万位
    updata_info1[2] = (_data1 / 10000) % 10 + '0';  // 万位
    updata_info1[3] = (_data1 / 1000) % 10 + '0';   // 千位
    updata_info1[4] = (_data1 / 100) % 10 + '0';    // 百位
    updata_info1[5] = (_data1 / 10) % 10 + '0';     // 十位
    updata_info1[6] = _data1 % 10 + '0';            // 个位

    // 结束字符，可以是回车换行符，如果需要的话
    updata_info1[7] = '\r';
    updata_info1[8] = '\n';

    // 通过DMA发送数据，根据结束字符调整长度
    HAL_UART_Transmit_DMA(&huart1,updata_info1, 9); // 发送数据，长度包含回车换行
}
