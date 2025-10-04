#include "commu.h"

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static float* flow_rate;
static int* DS18B20_Temp;
static int* pressure;

static uint32_t txmailbox;
static CAN_TxHeaderTypeDef tx_handler = 
{
  .StdId = 0xFF,
  .ExtId = 0x01,
  .RTR = CAN_RTR_DATA,
  .IDE = CAN_ID_STD,
  .DLC = 8,
  .TransmitGlobalTime = DISABLE,
};

void can_init() {
  init_ADC_DMA();
  flowrate_nvic_init();
  *flow_rate = 0;
}

void can_fetch_reference() {
  flow_rate = fetch_flowrate();
  DS18B20_Temp = fetch_temperature();
  pressure = fetch_pressure();
}

// void can_send_data(int data) {
//   uint8_t buffer[8] = {0, 1, 2, 3, 4, 5, 6, 7};
//   if (HAL_CAN_AddTxMessage(&hcan,&tx_handler,buffer, &txmailbox) != HAL_OK) {
//     SEGGER_RTT_printf(0, "Failed to write parameter\n");
//   }
// }

void can_send_data() {
  // uint8_t buffer[8];

  get_temp(&huart1);
  uint8_t temp1 = *DS18B20_Temp;

  get_temp(&huart2);
  uint8_t temp2 = *DS18B20_Temp;

  uint8_t flow1 = (*(int*)flow_rate & 0xFF);
  uint8_t flow2 = (*(int*)flow_rate >> 8);
  *flow_rate = 0;

  get_pressure();
  uint8_t pressure1 = (*pressure & 0xFF);
  uint8_t pressure2 = (*pressure >> 8);

  uint8_t buffer[8] = {temp1, temp2, flow1, flow2, pressure1, pressure2, 0, 0};

  if (HAL_CAN_AddTxMessage(&hcan,&tx_handler,buffer, &txmailbox) != HAL_OK) {
    SEGGER_RTT_printf(0, "Failed to write parameter\n");
  }
}

void can_receive_data(CAN_RxHeaderTypeDef *rx_header,uint8_t *rx_data) {
  uint8_t temp1 = rx_data[0];
  uint8_t temp2 = rx_data[1];
  uint16_t flow = (rx_data[2] << 8) | rx_data[3];
  uint16_t pressure = (rx_data[4] << 8) | rx_data[5];

  SEGGER_RTT_printf(0, "temp1: %d, temp2: %d, flow: %d, pressure: %d\n", temp1, temp2, flow, pressure);
}