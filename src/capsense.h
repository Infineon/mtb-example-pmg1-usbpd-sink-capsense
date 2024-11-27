/*******************************************************************************
* File Name: capsense.h
*
* Description:
*  This file contains the structure declaration and function prototypes used in
*  the USB PD Sink Capsense Code example.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#ifndef SRC_CAPSENSE_H_
#define SRC_CAPSENSE_H_

/*******************************************************************************
 * Header files
 ******************************************************************************/
#include "cy_pdutils_sw_timer.h"
#include "cy_pdstack_common.h"
#include "cycfg_capsense.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
/*
 * Time interval(ms) for the Capsense widget scan.
 */
#define CAPSENSE_FAST_SCAN_INTERVAL             (100u)
#define CAPSENSE_SLOW_SCAN_INTERVAL             (150u)

/*
 * The timer is used to initiate Capsense scan on all widgets periodically.
 */
#define CAPSENSE_SLOW_TIMER_ID                  (CY_PDUTILS_TIMER_USER_START_ID + 1u)
#define CAPSENSE_FAST_TIMER_ID                  (CY_PDUTILS_TIMER_USER_START_ID + 2u)

/*
 * The total count of Capsense scan when fast scan mode is active.
 */
#define FAST_SCAN_COUNT                         (50u)

/*
 * Capsense slider widget step has 5 segments with a step size of 20.
 */
#define STEP_SIZE                               (100u/5u)

/*
 * Used to indicate the number of touch detected on the Capsense slider widget.
 */
#define ONE_TOUCH                               (1u)

/*
 * The drive signal of the GPIO connected to Capsense Widget LED's to turn it ON.
 */
#define LED_ON                                  (0u)

/*
 * The drive signal of the GPIO connected to Capsense Widget LED's to turn it OFF.
 */
#define LED_OFF                                 (1u)

/*
 * Mask to select PDO type.
 */
#define PDO_MASK                                (0x0F)

/*
 * Mask to select APDO type.
 */
#define APDO_MASK                               (0xF0)

/*****************************************************************************
 * Data struct definition
 ****************************************************************************/
/**
 * @typedef en_supply_type_t
 * @brief Types of power supply.
 */
typedef enum {
    FIXED_SUPPLY                     = 0x00, /**< Fixed Supply */
    BATTERY_SUPPLY                   = 0x01, /**< Battery Supply */
    VARIABLE_SUPPLY                  = 0x02, /**< Variable Supply */
    PROGRAMMABLE_POWER_SUPPLY        = 0x03, /**< Programmable Power Supply */
    EPR_ADJUSTABLE_VOLTAGE_SUPPLY    = 0x13, /**< EPR Adjustable Voltage Supply */
    SPR_ADJUSTABLE_VOLTAGE_SUPPLY    = 0x23, /**< SPR Adjustable Voltage Supply */
} en_supply_type_t;

/******************************************************************************
 * Global function declaration
 ******************************************************************************/
/**
 * @brief This function handles interrupts from CapSense block.
 *
 * @param None
 * @return void
 */
void capsense_isr (void);

/**
 * @brief This is a Callback function for CapSense block.
 *
 * @param cy_stc_active_scan_sns_t - Active sensor details
 * @return void
 */
void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan);

/**
 * @brief This function scans all the CapSense widgets
 *
 * @param None
 * @return uint8_t
 */
uint8_t scan_touch(void);

/**
 * @brief This function scans all the CapSense widgets and starts the
 * slow timer
 *
 * @param cy_timer_id_t - Timer ID
 * @param callbackContext - Callback Context
 * @return void
 */
void capsense_slow_timer_cb (cy_timer_id_t id, void *callbackContext);

/**
 * @brief This function scans all the CapSense widgets and decrements the scan
 * count if the scan was initiated in the previous step and starts the timer
 * (slow/fast) depending on the scan count
 *
 * @param cy_timer_id_t - Timer ID
 * @param callbackContext - Callback Context
 * @return void
 */
void capsense_fast_timer_cb (cy_timer_id_t id, void *callbackContext);

/**
 * @brief This function initializes the CapSense and configure the CapSense
 * interrupt.
 *
 * @param None
 * @return uint32_t
 */
uint32_t initialize_capsense(void);

/**
 * @brief This function starts the fast timer
 *
 * @param None
 * @return void
 */
void enable_fast_timer(void);

/**
 * @brief This function checks if the CapSense widgets are touched and glows the
 * corresponding LED if the widget is touched
 *
 * @param None
 * @return void
 */
void process_touch(void);

#endif /* SRC_CAPSENSE_H_ */
