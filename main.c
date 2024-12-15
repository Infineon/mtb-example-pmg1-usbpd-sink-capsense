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

/*******************************************************************************
 * Header files
 ******************************************************************************/
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
#include "cy_app_instrumentation.h"
#include "cy_app.h"
#include "cy_app_pdo.h"
#include "cy_app_sink.h"
#include "cy_app_swap.h"
#include "cy_app_vdm.h"
#include "cy_app_battery_charging.h"
#include "cy_app_fault_handlers.h"
#include "mtbcfg_ezpd.h"
#include "capsense.h"
#include "cy_app_timer_id.h"

/*******************************************************************************
* Structure definitions
*******************************************************************************/
/* Structure to hold the user LED status. */
typedef struct
{
    GPIO_PRT_Type* gpioPort;    /* User LED port base address */
    uint32_t gpioPin;           /* User LED pin GPIO number */
    uint16_t blinkRate;         /* User LED blink rate in millisecond */
}cy_stc_user_led_status;

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* This variable is set to true once all the capsense widgets are scanned */
volatile bool gl_capsense_scan_complete = false;

/* This variable is set to true once all the capsense widgets are processed */
volatile bool gl_capsense_process_complete = true;

/* Variable for indicating to start scanning of capsense widgets. */
volatile uint8_t gl_capsense_scan_start = 0u;

cy_stc_pdutils_sw_timer_t gl_TimerCtx;
cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;
cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

#if PMG1_PD_DUALPORT_ENABLE
cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;
cy_stc_pdstack_context_t gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if (CY_PD_EPR_ENABLE)
extern bool gl_epr_exit;
#endif /* CY_PD_EPR_ENABLE */

/* Variable to store the user LED status */
static cy_stc_user_led_status gl_userLedStatus[NO_OF_TYPEC_PORTS] =
{
    {CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, LED_TIMER_PERIOD_DETACHED},
#if PMG1_PD_DUALPORT_ENABLE
    {CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_PIN, LED_TIMER_PERIOD_DETACHED},
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

/******************************************************************************
 * Structure type declaration
 ******************************************************************************/
/* PD Stack DPM Parameters for Port 0 */
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
/* PD Stack DPM Parameters for Port 1 */
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

uint32_t gl_discIdResp[7] = {0xFF00A841, 0x184004B4, 0x00000000, 0xF5030000};

/* App Parameters for Port 0 */
const cy_stc_app_params_t port0_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
    .prefPowerRole = 2u, /* 0-Sink, 1-Source, 2-No Preference */
    .prefDataRole = 2u, /* 0-UFP, 1- DFP, 2-No Preference */
    .discIdResp = (cy_pd_pd_do_t *)&gl_discIdResp[0],
    .discIdLen = 0x14,
    .swapResponse = 0x3F
};

#if PMG1_PD_DUALPORT_ENABLE
/* App Parameters for Port 1 */
const cy_stc_app_params_t port1_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
    .prefPowerRole = 2u, /* 0-Sink, 1-Source, 2-No Preference */
    .prefDataRole = 2u, /* 0-UFP, 1- DFP, 2-No Preference */
    .discIdResp = (cy_pd_pd_do_t *)&gl_discIdResp[0],
    .discIdLen = 0x14,
    .swapResponse = 0x3F
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

/* PD Stack Contexts */
cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
        &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
        &gl_PdStackPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

/* Watch Dog Timer Interrupt Configuration */
const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};

/* USB PD Port 0 Interrupt 0 Configuration */
const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};

/* USB PD Port 0 Interrupt 1 Configuration */
const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

#if PMG1_PD_DUALPORT_ENABLE
/* USB PD Port 1 Interrupt 0 Configuration */
const cy_stc_sysint_t usbpd_port1_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_IRQ,
    .intrPriority = 0U,
};

/* USB PD Port 1 Interrupt 1 Configuration */
const cy_stc_sysint_t usbpd_port1_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_DS_IRQ,
    .intrPriority = 0U,
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*******************************************************************************
* Function Name: get_pdstack_context
********************************************************************************
* Summary:
*   Returns the respective port PD Stack Context
*
* Parameters:
*  portIdx - Port Index
*
* Return:
*  cy_stc_pdstack_context_t
*
*******************************************************************************/
cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

#if (CY_PD_EPR_ENABLE)
/*******************************************************************************
* Function Name: epr_mode_exit_cb
********************************************************************************
* Summary:
*  Callback function for EPR exit
*
* Parameters:
*  id - Timer ID
*  ptrContext - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void epr_mode_exit_cb (cy_timer_id_t id, void *ptrContext)
{
    (void)id;
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *)ptrContext;

    cy_stc_pdstack_dpm_pd_cmd_buf_t cmdBuf;
    cmdBuf.noOfCmdDo = 1u;
    cmdBuf.cmdDo[0].eprmdo.action = CY_PDSTACK_EPR_MODE_EXIT;
    cmdBuf.cmdDo[0].eprmdo.data = 0u;
    cmdBuf.cmdDo[0].eprmdo.rsvd =  0u;

    if( Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_EPR_MODE, &cmdBuf, false, NULL) == CY_PDSTACK_STAT_SUCCESS )
    {
        gl_epr_exit = false;
    }
}
#endif /* (CY_PD_EPR_ENABLE) */

/*******************************************************************************
* Function Name: sln_pd_event_handler
********************************************************************************
* Summary:
*   Solution PD Event Handler
*   Handles the Extended message event
*
* Parameters:
*  ctx - PD Stack Context
*  evt - App Event
*  data - Data
*
* Return:
*  None
*
*******************************************************************************/
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    (void)ctx;

    if (evt == APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE)
    {
#if (CY_PD_EPR_ENABLE)
        if(gl_epr_exit == true)
        {
            Cy_PdUtils_SwTimer_Start(ctx->ptrTimerContext, ctx, CY_APP_GET_TIMER_ID(ctx, CY_APP_EPR_EXT_CMD_TIMER),
                    (uint16_t)EPR_MODE_EXIT_TIMER_PERIOD, epr_mode_exit_cb);
        }
#endif /* (CY_PD_EPR_ENABLE) */
    }
}

/*******************************************************************************
* Function Name: soln_sleep
********************************************************************************
* Summary:
*  This function prepares the solution for deepsleep entry.
*
* Parameters:
*  None
*
* Return:
*  true if deepsleep entry is possible otherwise false
*
*******************************************************************************/
bool soln_sleep()
{
    bool retVal = true;

    /* If either the capsense widget scan or the widget process is not complete, don't go to sleep. */
    if((gl_capsense_scan_complete == false) || (gl_capsense_process_complete == false))
    {
        retVal = false;
    }

    return retVal;
}

/*******************************************************************************
* Function Name: instrumentation_cb
********************************************************************************
* Summary:
*  Callback function for handling instrumentation faults
*
* Parameters:
*  port - Port
*  evt - Event
*
* Return:
*  None
*
*******************************************************************************/
void instrumentation_cb(uint8_t port, uint8_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

/*******************************************************************************
* Function Name: wdt_interrupt_handler
********************************************************************************
* Summary:
*  Interrupt Handler for Watch Dog Timer
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
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

/*******************************************************************************
* Function Name: cy_usbpd0_intr0_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

/*******************************************************************************
* Function Name: cy_usbpd0_intr1_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 1
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: cy_usbpd1_intr0_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD1 Interrupt 0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd1_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort1Ctx);
}

/*******************************************************************************
* Function Name: cy_usbpd1_intr1_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD1 Interrupt 1
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd1_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort1Ctx);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
/*******************************************************************************
* Function Name: led_timer_cb
********************************************************************************
* Summary:
*  Sets the desired LED blink rate based on the Type-C connection
*
* Parameters:
*  id - Timer ID
*  callbackContext - Context
*
* Return:
*  None
*
*******************************************************************************/
void led_timer_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)callbackContext;
    cy_stc_user_led_status *user_led = &gl_userLedStatus[stack_ctx->port];
#if BATTERY_CHARGING_ENABLE
    const cy_stc_bc_status_t    *bc_stat;
#endif /* #if BATTERY_CHARGING_ENABLE */

    /* Toggle the User LED and re-start timer to schedule the next toggle event. */
    Cy_GPIO_Inv(user_led->gpioPort, user_led->gpioPin);

    /* Calculate the desired LED blink rate based on the correct Type-C connection state. */
    if (stack_ctx->dpmConfig.attach)
    {
        if (stack_ctx->dpmConfig.contractExist)
        {
            if(stack_ctx ->dpmStat.contract.minVolt == CY_PD_VSAFE_5V)
            {
                /* Toggle the LED 5 times in 10 seconds for a 5V USBPD Contract */
                user_led->blinkRate = LED_TIMER_PERIOD_PD_SRC / 5;
            }
            else if(stack_ctx ->dpmStat.contract.minVolt == CY_PD_VSAFE_9V)
            {
                /* Toggle the LED 9 times in 10 seconds for a 9V USBPD Contract */
                user_led->blinkRate = LED_TIMER_PERIOD_PD_SRC / 9;
            }
            else if(stack_ctx ->dpmStat.contract.minVolt == CY_PD_VSAFE_12V)
            {
                /* Toggle the LED 12 times in 10 seconds for a 12V USBPD Contract */
                user_led->blinkRate = LED_TIMER_PERIOD_PD_SRC / 12;
            }
            else if(stack_ctx ->dpmStat.contract.minVolt == CY_PD_VSAFE_15V)
            {
                /* Toggle the LED 15 times in 10 seconds for a 15V USBPD Contract */
                user_led->blinkRate = LED_TIMER_PERIOD_PD_SRC / 15;
            }
            else if(stack_ctx ->dpmStat.contract.minVolt == CY_PD_VSAFE_20V)
            {
                /* Toggle the LED 20 times in 10 seconds for a 20V USBPD Contract */
                user_led->blinkRate = LED_TIMER_PERIOD_PD_SRC / 20;
            }
            else  if(stack_ctx ->dpmStat.contract.minVolt > CY_PD_VSAFE_20V)
            {
                /* Toggle the LED 28 times in 10 seconds for a EPR USBPD Contract */
                user_led->blinkRate = LED_TIMER_PERIOD_PD_SRC / 28;
            }
        }
        else
        {
#if BATTERY_CHARGING_ENABLE
            bc_stat = Cy_App_Bc_GetStatus(stack_ctx->ptrUsbPdContext);
            if (bc_stat->bc_fsm_state == BC_FSM_SINK_DCP_CONNECTED)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_DCP_SRC;
            }
            else if (bc_stat->bc_fsm_state == BC_FSM_SINK_CDP_CONNECTED)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_CDP_SRC;
            }
            else if (bc_stat->bc_fsm_state == BC_FSM_SINK_APPLE_BRICK_ID_DETECT)
            {
                user_led->blinkRate = LED_TIMER_PERIOD_APPLE_SRC;
            }
            else
#endif /* BATTERY_CHARGING_ENABLE */
            {
                user_led->blinkRate = LED_TIMER_PERIOD_TYPEC_SRC;
            }
        }
    }
    else
    {
        user_led->blinkRate = LED_TIMER_PERIOD_DETACHED;
    }

    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, callbackContext, id, user_led->blinkRate, led_timer_cb);
}
#endif /* APP_FW_LED_ENABLE */

/*******************************************************************************
* Function Name: get_dpm_connect_stat
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port 0
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: get_dpm_port1_connect_stat
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port 1
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
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
    .app_event_handler = Cy_App_EventHandler,
    .vconn_enable = Cy_App_VconnEnable,
    .vconn_disable = Cy_App_VconnDisable,
    .vconn_is_present = Cy_App_VconnIsPresent,
    .vbus_is_present = Cy_App_VbusIsPresent,
    .vbus_discharge_on = Cy_App_VbusDischargeOn,
    .vbus_discharge_off = Cy_App_VbusDischargeOff,
    .psnk_set_voltage = Cy_App_Sink_SetVoltage,
    .psnk_set_current = Cy_App_Sink_SetCurrent,
    .psnk_enable = Cy_App_Sink_Enable,
    .psnk_disable = Cy_App_Sink_Disable,
    .eval_src_cap = Cy_App_Pdo_EvalSrcCap,
    .eval_dr_swap = Cy_App_Swap_EvalDrSwap,
    .eval_pr_swap = Cy_App_Swap_EvalPrSwap,
    .eval_vconn_swap = Cy_App_Swap_EvalVconnSwap,
    .eval_vdm = Cy_App_Vdm_EvalVdmMsg,
    .vbus_get_value = Cy_App_VbusGetValue
};

/*******************************************************************************
* Function Name: app_get_callback_ptr
********************************************************************************
* Summary:
*  Returns pointer to the structure holding the application callback functions
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  cy_stc_pdstack_app_cbk_t
*
*******************************************************************************/
cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

/*******************************************************************************
* Function Name: soln_sink_fet_off
********************************************************************************
* Summary:
*  Turns off the consumer FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_sink_fet_off(cy_stc_pdstack_context_t * context)
{
#if CY_APP_SINK_FET_CTRL_GPIO_EN
    if (context->port == 0u)
    {
        Cy_GPIO_Clr (VBUS_C_CTRL_P0_PORT, VBUS_C_CTRL_P0_PIN);
    }
#if PMG1_PD_DUALPORT_ENABLE
    else
    {
        Cy_GPIO_Clr (VBUS_C_CTRL_P1_PORT, VBUS_C_CTRL_P1_PIN);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_SINK_FET_CTRL_GPIO_EN */
}

/*******************************************************************************
* Function Name: soln_sink_fet_on
********************************************************************************
* Summary:
*  Turns on the consumer FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_sink_fet_on(cy_stc_pdstack_context_t * context)
{
#if CY_APP_SINK_FET_CTRL_GPIO_EN
    if (context->port == 0u)
    {
        Cy_GPIO_Set (VBUS_C_CTRL_P0_PORT, VBUS_C_CTRL_P0_PIN);
    }
#if PMG1_PD_DUALPORT_ENABLE
    else
    {
        Cy_GPIO_Set (VBUS_C_CTRL_P1_PORT, VBUS_C_CTRL_P1_PIN);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_APP_SINK_FET_CTRL_GPIO_EN */
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - Initial setup of device
*  - Enables Watchdog timer, USB PD Port 0 and Port 1 interrupt
*  - Initializes Capsense and USB PD Port 0 and Port 1
*  - Checks for the Touch in Capsense
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
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

    /* Initialize the timerconfig with sysclk. */
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
    Cy_App_Instrumentation_Init(&gl_TimerCtx);

    /* Register callback function to be executed when instrumentation fault occurs. */
    Cy_App_Instrumentation_RegisterCb((cy_app_instrumentation_cb_t)instrumentation_cb);

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
    Cy_App_Init(&gl_PdStackPort0Ctx, &port0_app_params);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Init(&gl_PdStackPort1Ctx, &port1_app_params);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the fault configuration values */
    Cy_App_Fault_InitVars(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Fault_InitVars(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    Cy_App_Instrumentation_Start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if APP_FW_LED_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, (void *)&gl_PdStackPort0Ctx, (cy_timer_id_t)LED_TIMER_ID,
            LED_TIMER_PERIOD_DETACHED, led_timer_cb);
#if PMG1_PD_DUALPORT_ENABLE
    /* Start a timer that will blink the FW ACTIVE LED. */
    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, (void *)&gl_PdStackPort1Ctx, (cy_timer_id_t)LED2_TIMER_ID,
            LED_TIMER_PERIOD_DETACHED, led_timer_cb);
#endif /* PMG1_PD_DUALPORT_ENABLE */
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

        if(gl_capsense_scan_start != 0u)
        {
            /* Start scanning of capsense widgets. */
            scan_touch();
        }
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
        Cy_App_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_App_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Perform tasks associated with instrumentation. */
        Cy_App_Instrumentation_Task();

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter deep sleep mode for power saving. */
        Cy_App_SystemSleep(&gl_PdStackPort0Ctx,
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
