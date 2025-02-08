#ifndef _SENSOR_H
#define _SENSOR_H

#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "main.h"
#include "SEGGER_RTT.h"

// fetch references
float* fetch_flowrate();
int* fetch_temperature();
int* fetch_pressure();

// water flow rate
void flowrate_nvic_init();

// Temperature sensors onewire communication implementation
void uart_init(UART_HandleTypeDef* huart, int baudrate);
void DS18B20_WriteByte(UART_HandleTypeDef* huart, uint8_t data);
uint8_t DS18B20_ReadByte(UART_HandleTypeDef* huart);
uint8_t DS18B20_Init(UART_HandleTypeDef* huart);
void DS18B20_SampleTemp(UART_HandleTypeDef* huart);
int DS18B20_ReadTemp(UART_HandleTypeDef* huart);
void get_temp(UART_HandleTypeDef* haurt);

// ADC (Pressure Transducer)
void init_ADC_DMA();
void get_pressure();
#endif