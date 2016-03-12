/*
 * driver_config.h
 *
 *  Created on: Aug 31, 2010
 *      Author: nxp28548
 */

#ifndef DRIVER_CONFIG_H_
#define DRIVER_CONFIG_H

#if defined(lpc11c14)
#include <cmsis/LPC11xx.h>
#endif

#if defined(lpc1768)
#include <cmsis/LPC17xx.h>
#endif

#undef CAN_UART_DEBUG

#define CONFIG_ENABLE_DRIVER_CRP						1
#define CONFIG_CRP_SETTING_NO_CRP						1

#define CONFIG_ENABLE_DRIVER_TIMER32					1
#define CONFIG_TIMER32_DEFAULT_TIMER32_0_IRQHANDLER		1

#define CONFIG_ENABLE_DRIVER_GPIO						1

#define CONFIG_ENABLE_DRIVER_UART						1
#define CONFIG_UART_DEFAULT_UART_IRQHANDLER				1
#define CONFIG_UART_ENABLE_INTERRUPT					1

#define CONFIG_ENABLE_DRIVER_CAN						1
#define CONFIG_CAN_DEFAULT_CAN_IRQHANDLER				1

//I2
#define CONFIG_ENABLE_DRIVER_I2C						1
#define CONFIG_I2C_DEFAULT_I2C_IRQHANDLER				1

//SSP
#define CONFIG_ENABLE_DRIVER_SSP	                    1
#define	USE_SCK_P0_10									1
//#define	USE_CS											1

#define CONFIG_ENABLE_DRIVER_TIMER16                    1

 /* DRIVER_CONFIG_H_ */
#endif
