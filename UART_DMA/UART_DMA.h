#ifndef __UART_DMA_H
#define __UART_DMA_H
#include "main.h"
#include "usart.h"
#include "stdio.h"
typedef union
{
    float Fdata;
    uint32_t Adata;
    
}Vofa_Type;
void Float_to_Byte(float Fdata, uint8_t *ArrayByte);
void DMA_to_Vofa(float angle, float speed,float value);
void DMA_Send_Debug(int _data1);
void DMA_to_Vofa_v5(float angle, float speed, float value1, float value2, float value3);
void Serial_SendByte(uint8_t Byte);
void Serial_SendString(char *String);
uint32_t Serial_Pow(uint32_t X, uint32_t Y);
int fputc(int ch, FILE *f);
void Serial_Printf(char *format, ...);
#endif
