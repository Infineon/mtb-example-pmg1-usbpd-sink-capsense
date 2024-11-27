/******************************************************************************
* File Name: config.h
*
* Description: This header file defines the application configuration for the PMG1
*              MCU USBPD Sink with Capsense Example for ModusToolBox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef _CONFIG_H_
#define _CONFIG_H_

/*******************************************************************************
 * Header files
 ******************************************************************************/
#include "cybsp.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
/* The ADC which should be used to measure VBus voltage on the Type-C side. */
#define APP_VBUS_POLL_ADC_ID                    (CY_USBPD_ADC_ID_0)

/*
 * The Analog-MUX bus input which is used to measure VBus voltage. Choose AMUXBUS_A on PMG1-S2 and AMUXBUS_B on
 * other devices.
 */
#if defined(CY_DEVICE_CCG3)
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_A)
#else
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_B)
#endif /* defined(CY_DEVICE_CCG3) */

/*
 * Enable/Disable firmware active LED operation.
 *
 * The blinking LED is enabled by default but it is recommended to disable it
 * for production designs to save power.
 */
#define APP_FW_LED_ENABLE                       (1u)

/*
 * Port-0 activity indicator LED timer. The timer is used to indicate that the firmware
 * is functional. The feature is controlled by APP_FW_LED_ENABLE.
 */
#define LED_TIMER_ID                            (CY_PDUTILS_TIMER_USER_START_ID)

/*
 * Port-1 activity indicator LED timer. The timer is used to indicate that the firmware
 * is functional. The feature is controlled by APP_FW_LED_ENABLE.
 */
#define LED2_TIMER_ID                           (CY_PDUTILS_TIMER_USER_START_ID + 3u)

/*
 * The LED toggle period (ms) to be used when Type-C connection hasn't been detected.
 */
#define LED_TIMER_PERIOD_DETACHED               (4000u)

/*
 * The LED toggle period (ms) to be used when a Type-C power source is connected.
 */
#define LED_TIMER_PERIOD_TYPEC_SRC              (5000u)

/*
 * The LED toggle period (ms) to be used when a USB-PD power source is connected.
 */
#define LED_TIMER_PERIOD_PD_SRC                 (10000u)

/*
 * The LED toggle period (ms) to be used when a BC 1.2 DCP (Downstream Charging Port) source without PD support is connected.
 */
#define LED_TIMER_PERIOD_DCP_SRC                (7500u)

/*
 * The LED toggle period (ms) to be used when a BC 1.2 CDP (Charging Downstream Port) source without PD support is connected.
 */
#define LED_TIMER_PERIOD_CDP_SRC                (10000u)

/*
 * The LED toggle period (ms) to be used when an Apple charging source without PD support is connected.
 */
#define LED_TIMER_PERIOD_APPLE_SRC              (12500u)

/*
 * Time period for EPR Exit mode
 */
#define EPR_MODE_EXIT_TIMER_PERIOD              (5u)

/*
 * 9.0V Vbus voltage in 50mV units
 */
#define VSAFE_9V_IN_50MV                        (180u)

/*
 * 15.0V Vbus voltage in 50mV units
 */
#define VSAFE_15V_IN_50MV                       (300u)

/*
 * 20.0V Vbus voltage in 50mV units
 */
#define VSAFE_20V_IN_50MV                       (400u)

#endif /* _CONFIG_H_ */

/* End of file */
