/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define BT_SLV_MST_Pin GPIO_PIN_13
#define BT_SLV_MST_GPIO_Port GPIOC
#define BT_SLV_MST_SW_HW_Pin GPIO_PIN_14
#define BT_SLV_MST_SW_HW_GPIO_Port GPIOC
#define BT_INT_Pin GPIO_PIN_15
#define BT_INT_GPIO_Port GPIOC
#define SPI2_CS4_Pin GPIO_PIN_1
#define SPI2_CS4_GPIO_Port GPIOB
#define SPI2_CS3_Pin GPIO_PIN_2
#define SPI2_CS3_GPIO_Port GPIOB
#define EXT_UART_TX_Pin GPIO_PIN_10
#define EXT_UART_TX_GPIO_Port GPIOB
#define EXT_UART_RX_Pin GPIO_PIN_11
#define EXT_UART_RX_GPIO_Port GPIOB
#define SPI2_CS2_Pin GPIO_PIN_12
#define SPI2_CS2_GPIO_Port GPIOB
#define SPI2_CS1_Pin GPIO_PIN_8
#define SPI2_CS1_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_9
#define BT_RX_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_10
#define BT_TX_GPIO_Port GPIOA
#define BT_AWAKE_Pin GPIO_PIN_11
#define BT_AWAKE_GPIO_Port GPIOA
#define BT_NRST_Pin GPIO_PIN_12
#define BT_NRST_GPIO_Port GPIOA
#define IO_NCS_Pin GPIO_PIN_15
#define IO_NCS_GPIO_Port GPIOA
#define IO_CLK_Pin GPIO_PIN_3
#define IO_CLK_GPIO_Port GPIOB
#define IO_MISO_Pin GPIO_PIN_4
#define IO_MISO_GPIO_Port GPIOB
#define IO_MOSI_Pin GPIO_PIN_5
#define IO_MOSI_GPIO_Port GPIOB
#define IO_INT_Pin GPIO_PIN_6
#define IO_INT_GPIO_Port GPIOB
#define IO_RST_Pin GPIO_PIN_7
#define IO_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
