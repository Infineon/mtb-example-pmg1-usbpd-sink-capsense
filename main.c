/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USBPD Sink with Capsense Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"

#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "instrumentation.h"
#include "app.h"
#include "pdo.h"
#include "psink.h"
#include "swap.h"
#include "vdm.h"
#include "charger_detect.h"
#include "mtbcfg_ezpd.h"
#include "cycfg_capsense.h"

/* The variable is set to true once all the capsense widgets are scanned */
volatile bool gl_capsense_scan_complete = false;
/* The variable is set to true once all the capsense widgets are processed */
volatile bool gl_capsense_process_complete = true;
/* Flag to indicated fast scanning of capsense widgets */
volatile bool gl_capsense_fast_scan = false;
/* Keep tracks of number of times fast scan has been performed */
static uint8_t  gl_capsense_fast_scan_count = FAST_SCAN_COUNT;

/* LED blink rate in milliseconds */
static uint16_t gl_LedBlinkRate = LED_TIMER_PERIOD_DETACHED;

cy_stc_pdutils_sw_timer_t        gl_TimerCtx;
cy_stc_usbpd_context_t   gl_UsbPdPort0Ctx;

cy_stc_pdstack_context_t gl_PdStackPort0Ctx;
#if PMG1_PD_DUALPORT_ENABLE
cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;
cy_stc_pdstack_context_t gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
        .dpmSnkWaitCapPeriod = 335,
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 0,
        .defCur = 90
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_pdstack_dpm_params_t pdstack_port1_dpm_params =
{
        .dpmSnkWaitCapPeriod = 335,
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 0,
        .defCur = 90
};
#endif /* PMG1_PD_DUALPORT_ENABLE */
cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
        &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
        &gl_PdStackPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

bool mux_ctrl_init(uint8_t port)
{
    /* No MUXes to be controlled on the PMG1 proto kits. */
    CY_UNUSED_PARAMETER(port);
    return true;
}

const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_sysint_t usbpd_port1_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port1_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_DS_IRQ,
    .intrPriority = 0U,
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/* Solution PD event handler */
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    (void)ctx;

    if(evt == APP_EVT_HANDLE_EXTENDED_MSG)
    {
        cy_stc_pd_packet_extd_t * ext_mes = (cy_stc_pd_packet_extd_t * )data;
        if ((ext_mes->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (ext_mes->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ctx, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
    }
}

void instrumentation_cb(uint8_t port, inst_evt_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier));
#endif /* (TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler (&(gl_TimerCtx));
}

static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

#if PMG1_PD_DUALPORT_ENABLE
static void cy_usbpd1_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort1Ctx);
}

static void cy_usbpd1_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort1Ctx);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
void led_timer_cb (
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)callbackContext;
#if BATTERY_CHARGING_ENABLE
    const chgdet_status_t    *chgdet_stat;
#endif /* #if BATTERY_CHARGING_ENABLE */

    /* Toggle the User LED and re-start timer to schedule the next toggle event. */
    Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);

    /* Calculate the desired LED blink rate based on the correct Type-C connection state. */
    if (stack_ctx->dpmConfig.attach)
    {
        if (stack_ctx->dpmConfig.contractExist)
        {
            gl_LedBlinkRate = LED_TIMER_PERIOD_PD_SRC;
        }
        else
        {
#if BATTERY_CHARGING_ENABLE
            chgdet_stat = chgdet_get_status(stack_ctx);
            if (chgdet_stat->chgdet_fsm_state == CHGDET_FSM_SINK_DCP_CONNECTED)
            {
                gl_LedBlinkRate = LED_TIMER_PERIOD_DCP_SRC;
            }
            else if (chgdet_stat->chgdet_fsm_state == CHGDET_FSM_SINK_CDP_CONNECTED)
            {
                gl_LedBlinkRate = LED_TIMER_PERIOD_CDP_SRC;
            }
            else
#endif /* BATTERY_CHARGING_ENABLE */
            {
                gl_LedBlinkRate = LED_TIMER_PERIOD_TYPEC_SRC;
            }
        }
    }
    else
    {
        gl_LedBlinkRate = LED_TIMER_PERIOD_DETACHED;
    }

    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, callbackContext, id, gl_LedBlinkRate, led_timer_cb);
}
#endif /* APP_FW_LED_ENABLE */

cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

#if PMG1_PD_DUALPORT_ENABLE
cy_stc_pd_dpm_config_t* get_dpm_port1_connect_stat()
{
    return &(gl_PdStackPort1Ctx.dpmConfig);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    app_event_handler,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
    vbus_get_value,
};

cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    gl_capsense_scan_complete = true;
}

/*******************************************************************************
* Function Name: scan_touch
********************************************************************************
* Summary:
*  Initiated all Capsense widget scan if the previous processing was complete.
*
*******************************************************************************/
uint8_t scan_touch(void)
{
    uint8_t scanStart = 0u;
    /* Initiate widget scan only when the previous scan processing is complete */
    if(gl_capsense_process_complete == true)
    {
        /* Initiate scan of all the widgets */
        Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

        /* Set to false to indicate scan is in progress */
        gl_capsense_scan_complete = false;

        /* Set to false to indicate the processing is pending on this scan */
        gl_capsense_process_complete = false;

        /* Return flag to indicated scan is started */
        scanStart = 1u;
    }
    return scanStart;
}

void capsense_slow_timer_cb (
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    /* Initiate scanning of all widgets */
    scan_touch();

    /* Start the slow timer to initiate the next scan when the timer expires */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, NULL, (cy_timer_id_t)CAPSENSE_SLOW_TIMER_ID,
            (uint16_t)CAPSENSE_SLOW_SCAN_INTERVAL, capsense_slow_timer_cb);
}

void capsense_fast_timer_cb (
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    uint8_t scanStart = 0u;
    /* Initiate scanning of all widgets */
    scanStart = scan_touch();
    /* Decrement the scan count only if the scan was initiated in prev step */
    if(scanStart == 1u)
    {
        /* Fast scan count is decremented every time a scan is performed */
        gl_capsense_fast_scan_count--;
    }
    /* Once the the scan count lapses, the slow timer is started */
    if(gl_capsense_fast_scan_count == 0u)
    {
        gl_capsense_fast_scan = false;
        Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, NULL, (cy_timer_id_t)CAPSENSE_SLOW_TIMER_ID,
                (uint16_t)CAPSENSE_SLOW_SCAN_INTERVAL, capsense_slow_timer_cb);
    }
    else
    {
        /* If the scan count is non-zero, keep the fast timer running */
        Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, NULL, (cy_timer_id_t)CAPSENSE_FAST_TIMER_ID,
                (uint16_t)CAPSENSE_FAST_SCAN_INTERVAL, capsense_fast_timer_cb);
    }
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
*
*******************************************************************************/
static uint32_t initialize_capsense(void)
{
    uint32_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
        {
            .intrSrc = CYBSP_CSD_IRQ,
            .intrPriority = 3u,
        };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CY_CAPSENSE_STATUS_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
    NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
    NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CY_CAPSENSE_STATUS_SUCCESS != status)
    {
        return status;
    }

    /* Assign a callback function to indicate end of CapSense scan. */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
            capsense_callback, &cy_capsense_context);
    if (CY_CAPSENSE_STATUS_SUCCESS != status)
    {
        return status;
    }

    /* Start a timer that will initiate all the widget scan periodically. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, NULL, (cy_timer_id_t)CAPSENSE_SLOW_TIMER_ID,
            (uint16_t)CAPSENSE_SLOW_SCAN_INTERVAL, capsense_slow_timer_cb);

    return status;
}

void enable_fast_timer(void)
{
    /* Since touch is detected, enable fast scan flag & reset count */
    gl_capsense_fast_scan = true;
    gl_capsense_fast_scan_count = FAST_SCAN_COUNT;
    /* If the fast timer is not yet active, start the fast timer */
    if(Cy_PdUtils_SwTimer_IsRunning(&gl_TimerCtx, (cy_timer_id_t)CAPSENSE_FAST_TIMER_ID)==false)
    {
        /* Stop the slow timer */
        Cy_PdUtils_SwTimer_Stop(&gl_TimerCtx, (cy_timer_id_t)CAPSENSE_SLOW_TIMER_ID);

        /* Start the fast timer */
        Cy_PdUtils_SwTimer_Start(&gl_TimerCtx, NULL, (cy_timer_id_t)CAPSENSE_FAST_TIMER_ID,
                (uint16_t)CAPSENSE_FAST_SCAN_INTERVAL, capsense_fast_timer_cb);
    }
}
/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
static void process_touch(void)
{
    uint8_t touchDetected = 0u;
    uint32_t button0_status;
    uint32_t button1_status;
    cy_stc_capsense_touch_t *ptrSliderPosition;
    uint32_t position = 0u;

    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON0_WDGT_ID,
        CY_CAPSENSE_BUTTON0_SNS0_ID,
        &cy_capsense_context);
    if(button0_status!=0u)
    {
        /* Switch ON the LED when touch is detected */
        Cy_GPIO_Write(CYBSP_LED_BTN0_PORT, CYBSP_LED_BTN0_PIN, LED_ON);
        /* Set the flag to indicate touch was detected */
        touchDetected = 1u;
    }
    else
    {
        /* Switch OFF the LED when button is not active */
        Cy_GPIO_Write(CYBSP_LED_BTN0_PORT, CYBSP_LED_BTN0_PIN, LED_OFF);
    }

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON1_WDGT_ID,
        CY_CAPSENSE_BUTTON1_SNS0_ID,
        &cy_capsense_context);
    if(button1_status!=0u)
    {
        /* Switch ON the LED when touch is detected */
        Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_PIN, LED_ON);
        /* Set the flag to indicate touch was detected */
        touchDetected = 1u;
    }
    else
    {
        /* Switch OFF the LED when button is not active */
        Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_PIN, LED_OFF);
    }

    /* Get slider status */
    ptrSliderPosition = Cy_CapSense_GetTouchInfo(
        CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    if (ptrSliderPosition->numPosition == ONE_TOUCH)
    {
        position = ptrSliderPosition->ptrPosition->x;
        Cy_GPIO_Write(CYBSP_LED_SLD0_PORT, CYBSP_LED_SLD0_PIN,
                (position <= ( 1 * STEP_SIZE)) ? LED_ON : LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD1_PORT, CYBSP_LED_SLD1_PIN,
                (((position > ( 1 * STEP_SIZE))) && ((position <= ( 2 * STEP_SIZE)))) ? LED_ON : LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD2_PORT, CYBSP_LED_SLD2_PIN,
                (((position > ( 2 * STEP_SIZE))) && ((position <= ( 3 * STEP_SIZE)))) ? LED_ON : LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD3_PORT, CYBSP_LED_SLD3_PIN,
                (((position > ( 3 * STEP_SIZE))) && ((position <= ( 4 * STEP_SIZE)))) ? LED_ON : LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD4_PORT, CYBSP_LED_SLD4_PIN,
                (((position > ( 4 * STEP_SIZE))) && ((position <= ( 5 * STEP_SIZE)))) ? LED_ON : LED_OFF);

        /* Set the flag to indicate touch was detected */
        touchDetected = 1u;
    }
    else
    {
        Cy_GPIO_Write(CYBSP_LED_SLD0_PORT, CYBSP_LED_SLD0_PIN, LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD1_PORT, CYBSP_LED_SLD1_PIN, LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD2_PORT, CYBSP_LED_SLD2_PIN, LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD3_PORT, CYBSP_LED_SLD3_PIN, LED_OFF);
        Cy_GPIO_Write(CYBSP_LED_SLD4_PORT, CYBSP_LED_SLD4_PIN, LED_OFF);
    }

    if(touchDetected == 1u)
    {
        /* Start fast timer since the touch is detected */
        enable_fast_timer();
    }
}




int main(void)
{
    cy_rslt_t result;
    cy_stc_pdutils_timer_config_t timerConfig;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
    timerConfig.hw_timer_ctx = NULL;

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize Capsense */
    result = initialize_capsense();

    if (CY_CAPSENSE_STATUS_SUCCESS != result)
    {
        /* Halt the CPU if CapSense initialization failed */
        CY_ASSERT(0);
    }

    /* Initialize the instrumentation related data structures. */
    instrumentation_init();

    /* Register callback function to be executed when instrumentation fault occurs. */
    instrumentation_register_cb((instrumentation_cb_t)instrumentation_cb);

    /* Configure and enable the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

#if PMG1_PD_DUALPORT_ENABLE
    /* Configure and enable the USBPD interrupts for Port #1. */
    Cy_SysInt_Init(&usbpd_port1_intr0_config, &cy_usbpd1_intr0_handler);
    NVIC_EnableIRQ(usbpd_port1_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port1_intr1_config, &cy_usbpd1_intr1_handler);
    NVIC_EnableIRQ(usbpd_port1_intr1_config.intrSrc);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the USBPD driver */
#if defined(CY_DEVICE_CCG3)
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, NULL,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#else
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_USBPD_Init(&gl_UsbPdPort1Ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port1_config, get_dpm_port1_connect_stat);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif

    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                       &gl_UsbPdPort0Ctx,
                       &mtb_usbpd_port0_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort0Ctx),
                       &pdstack_port0_dpm_params,
                       &gl_TimerCtx);

#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Init(&gl_PdStackPort1Ctx,
                       &gl_UsbPdPort1Ctx,
                       &mtb_usbpd_port1_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort1Ctx),
                       &pdstack_port1_dpm_params,
                       &gl_TimerCtx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Perform application level initialization. */
    app_init(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    app_init(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the fault configuration values */
    fault_handler_init_vars(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    fault_handler_init_vars(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    instrumentation_start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)LED_TIMER_ID,
            gl_LedBlinkRate, led_timer_cb);
#endif /* APP_FW_LED_ENABLE */

    /*
     * After the initialization is complete, keep processing the USB-PD device
     * policy manager task in a loop.  Since this application does not have any
     * other function, the PMG1 device can be placed in "deep sleep" mode for
     * power saving whenever the PD stack and drivers are idle.
     */
    for (;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_PdStack_Dpm_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        if((gl_capsense_scan_complete == true) && (gl_capsense_process_complete == false))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Process touch input */
            process_touch();

            /* Set to true to indicate capsense processing is complete */
            gl_capsense_process_complete = true;
        }

        /* Perform any application level tasks. */
        app_task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        app_task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Perform tasks associated with instrumentation. */
        instrumentation_task();

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter deep sleep mode for power saving. */
        system_sleep(&gl_PdStackPort0Ctx, 
#if PMG1_PD_DUALPORT_ENABLE
                &gl_PdStackPort1Ctx
#else
                NULL
#endif /* PMG1_PD_DUALPORT_ENABLE */
                );
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* [] END OF FILE */
