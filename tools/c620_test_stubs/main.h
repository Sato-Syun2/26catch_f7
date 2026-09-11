#ifndef C620_TEST_MAIN_H
#define C620_TEST_MAIN_H
#include <stdint.h>
typedef struct { int unused; } GPIO_TypeDef;
typedef struct { int unused; } CAN_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef enum { HAL_OK } HAL_StatusTypeDef;
extern GPIO_TypeDef test_gpio;
#define sensor2_GPIO_Port (&test_gpio)
#define sensor2_Pin 16U
uint32_t HAL_GetTick(void);
int HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
#endif
