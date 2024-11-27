/******************************************************************************
* File Name: capsense.c
*
* Description: This file contains the functions related to CAPSENSE
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

/*******************************************************************************
 * Header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "capsense.h"
#include "cy_pdutils.h"
#include "config.h"
#include "cy_app.h"

/******************************************************************************
 * Macro definitions
 ******************************************************************************/
/* Capsense slow scan. */
#define SLOW_SCAN                           (1u)
/* Capsense fast scan. */
#define FAST_SCAN                           (2u)

/******************************************************************************
 * Global variables declaration
 ******************************************************************************/
/* This variable is set to true once all the capsense widgets are scanned */
extern volatile bool gl_capsense_scan_complete;

/* This variable is set to true once all the capsense widgets are processed */
extern volatile bool gl_capsense_process_complete;

/* Variable for indicating to start scanning of capsense widgets. */
extern volatile uint8_t gl_capsense_scan_start;

/* Variable used to keep tracks of number of times fast scan has been performed */
static uint8_t  gl_capsense_fast_scan_count = FAST_SCAN_COUNT;

/* Variable to store the current contract voltage */
static uint16_t gl_cur_voltage;

/* CapSense interrupt configuration */
const cy_stc_sysint_t CapSense_interrupt_config =
{
    .intrSrc = CYBSP_CSD_IRQ,
    .intrPriority = 3u,
};

extern cy_stc_pdutils_sw_timer_t gl_TimerCtx;
extern cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

#if (CY_PD_EPR_ENABLE)
bool gl_epr_exit = false;
#endif /* CY_PD_EPR_ENABLE */
uint8_t get_src_data_buffer[2];

/*******************************************************************************
* Function Name: is_request_valid
********************************************************************************
* Summary:
*  Validates user request
*
* Parameters:
*  context - PdStack context
*  volt - Voltage in 50mV
*  cur - Current in 10mA
*
* Return:
*  true if the request is valid otherwise false
*
*******************************************************************************/
static bool is_request_valid(cy_stc_pdstack_context_t *context, uint16_t volt, uint16_t cur)
{
    cy_stc_pdstack_dpm_status_t *dpm_stat = &(context->dpmStat);
#if CY_PD_EPR_ENABLE
    cy_stc_pdstack_dpm_ext_status_t *dpmExt = &(context->dpmExtStat);
#endif /* CY_PD_EPR_ENABLE */
    cy_en_pdstack_pdo_t supply_type;
    uint8_t snk_pdo_idx;
    uint8_t snk_pdo_len = dpm_stat->curSnkPdocount;
    cy_pd_pd_do_t* pdo_snk;

#if CY_PD_EPR_ENABLE
    if(dpmExt->eprActive)
    {
        snk_pdo_len = CY_PD_MAX_NO_OF_PDO + dpmExt->curEprSnkPdoCount;
    }
#endif /* CY_PD_EPR_ENABLE */

    for(snk_pdo_idx = 0; snk_pdo_idx < snk_pdo_len; snk_pdo_idx++)
    {
        pdo_snk = (cy_pd_pd_do_t*)&(dpm_stat->curSnkPdo[snk_pdo_idx]);
        supply_type = (cy_en_pdstack_pdo_t)pdo_snk->fixed_snk.supplyType;

        switch(supply_type)
        {
            case CY_PDSTACK_PDO_FIXED_SUPPLY:
                if((volt == pdo_snk->fixed_snk.voltage) && (cur <= pdo_snk->fixed_snk.opCurrent))
                {
                    return true;
                }
                break;
            case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                if((volt >= pdo_snk->var_snk.minVoltage) && (volt <= pdo_snk->var_snk.maxVoltage))
                {
                    if(cur <= pdo_snk->var_snk.opCurrent)
                    {
                        return true;
                    }
                }
                break;
            case CY_PDSTACK_PDO_BATTERY:
                if((volt >= pdo_snk->bat_snk.minVoltage) && (volt <= pdo_snk->bat_snk.maxVoltage))
                {
                    uint16_t power = CY_PDUTILS_DIV_ROUND_UP(volt * cur, 500);
                    if(power <= pdo_snk->bat_snk.opPower)
                    {
                        return true;
                    }
                }
                break;
            case CY_PDSTACK_PDO_AUGMENTED:
#if (CY_PD_EPR_AVS_ENABLE)
                if(pdo_snk->epr_avs_snk.apdoType == CY_PDSTACK_APDO_AVS)
                {
                    /* Convert PDO voltage to 50 mV from 100 mV unit */
                    if((volt >= pdo_snk->epr_avs_snk.minVolt * 2u) && (volt <= pdo_snk->epr_avs_snk.maxVolt * 2u))
                    {
                        /* Calculate the power in 250mW units */
                        uint16_t power = CY_PDUTILS_DIV_ROUND_UP(volt * cur, 500);
                        /* Convert PDP to 250mW units */
                        if(power <= pdo_snk->epr_avs_snk.pdp * 4u)
                        {
                            return true;
                        }
                    }
                }
#endif /* CY_PD_EPR_AVS_ENABLE */
                break;
            default:
                /* Do Nothing */
                break;
        }
    }
    return false;
}

/*******************************************************************************
* Function Name: select_src_pdo
********************************************************************************
* Summary:
*  Choose the source PDO that can provide requested voltage and current
*
* Parameters:
*  context - PdStack context
*  supply_type - Supply type
*  volt - Voltage in 50mV
*  Cur - Current in 10mA
*  srcCap - Pointer to source capabilities message
*
* Return:
*  uint8_t - Object position
*
*******************************************************************************/
static uint8_t select_src_pdo(cy_stc_pdstack_context_t *context, en_supply_type_t supply_type, uint16_t volt, uint16_t cur,
                              cy_stc_pdstack_pd_packet_t* srcCap)
{
    bool status = false;
#if CY_PD_EPR_ENABLE
    cy_stc_pdstack_dpm_ext_status_t *dpmExt = &(context->dpmExtStat);
#endif /* CY_PD_EPR_ENABLE */
    uint8_t src_pdo_idx;
    uint8_t src_pdo_len = srcCap->len;
    cy_pd_pd_do_t* pdo_src;
    uint8_t obj_pos = 0u;

#if CY_PD_EPR_ENABLE
    if(srcCap->hdr.hdr.extd && dpmExt->eprActive)
    {
        src_pdo_len = srcCap->hdr.hdr.dataSize / 4u;
    }
#endif /* CY_PD_EPR_ENABLE */

    for(src_pdo_idx = 0; src_pdo_idx < src_pdo_len; src_pdo_idx++)
    {
        pdo_src = (cy_pd_pd_do_t*)(&srcCap->dat[src_pdo_idx]);
        if((supply_type & PDO_MASK) == (cy_en_pdstack_pdo_t)pdo_src->fixed_src.supplyType)
        {
            switch((cy_en_pdstack_pdo_t)pdo_src->fixed_src.supplyType)
            {
                case CY_PDSTACK_PDO_FIXED_SUPPLY:
                    if((volt == pdo_src->fixed_src.voltage) && (cur <= pdo_src->fixed_src.maxCurrent))
                    {
                        status = true;
                    }
                    break;
                case CY_PDSTACK_PDO_VARIABLE_SUPPLY:
                    if((volt >= pdo_src->var_src.minVoltage) && (volt <= pdo_src->var_src.maxVoltage))
                    {
                        if(cur <= pdo_src->var_src.maxCurrent)
                        {
                            status = true;
                        }
                    }
                    break;
                case CY_PDSTACK_PDO_BATTERY:
                    if((volt >= pdo_src->bat_src.minVoltage) && (volt <= pdo_src->bat_src.maxVoltage))
                    {
                        uint16_t power = CY_PDUTILS_DIV_ROUND_UP(volt * cur, 500);
                        if(power <= pdo_src->bat_src.maxPower)
                        {
                            status = true;
                        }
                    }
                    break;
                case CY_PDSTACK_PDO_AUGMENTED:
                    if((supply_type >> 4u) == pdo_src->spr_avs_src.apdoType)
                    {
                        if(pdo_src->spr_avs_src.apdoType == CY_PDSTACK_APDO_SPR_AVS)
                        {
                            if((volt >= VSAFE_9V_IN_50MV) && (volt <= VSAFE_15V_IN_50MV))
                            {
                                if(cur <= pdo_src->spr_avs_src.maxCur1)
                                {
                                    status = true;
                                }
                            }
                            else if((volt > VSAFE_15V_IN_50MV) && (volt <= VSAFE_20V_IN_50MV))
                            {
                                if(cur <= pdo_src->spr_avs_src.maxCur2)
                                {
                                    status = true;
                                }
                            }
                        }
#if (CY_PD_EPR_AVS_ENABLE)
                        if(pdo_src->epr_avs_src.apdoType == CY_PDSTACK_APDO_AVS)
                        {
                            /* Convert PDO voltage to 50 mV from 100 mV unit */
                            if((volt >= pdo_src->epr_avs_src.minVolt * 2u) && (volt <= pdo_src->epr_avs_src.maxVolt * 2u))
                            {
                                /* Calculate the power in 250mW units */
                                uint16_t power = CY_PDUTILS_DIV_ROUND_UP(volt * cur, 500);
                                /* Convert PDP to 250mW units */
                                if(power <= pdo_src->epr_avs_src.pdp * 4u)
                                {
                                    status = true;
                                }
                            }
                        }
#endif /* CY_PD_EPR_AVS_ENABLE */
                    }
                    break;
                default:
                    /* Do Nothing */
                    break;
            }
            if(status == true)
            {
                obj_pos = src_pdo_idx + 1u;
                break;
            }
        }
    }

    return obj_pos;
}

/*******************************************************************************
* Function Name: send_request
********************************************************************************
* Summary:
*  Forms RDO and sends request message
*
* Parameters:
*  context - PdStack context
*  pdo_no - PDO number
*  volt - Voltage in 50mV
*  Cur - Current in 10mA
*  srcCap - Pointer to source capabilities message
*
* Return:
* CY_PDSTACK_STAT_SUCCESS if the request is successful
* CY_PDSTACK_STAT_FAILURE if the request is failed
*
*******************************************************************************/
static cy_en_pdstack_status_t send_request(cy_stc_pdstack_context_t *context, uint8_t pdo_no, uint16_t volt, uint16_t cur, cy_stc_pdstack_pd_packet_t* srcCap)
{
    cy_en_pdstack_status_t status;
#if (CY_PD_EPR_ENABLE)
    const cy_stc_pdstack_dpm_ext_status_t *dpmExtStat = &(context->dpmExtStat);
#endif /* (CY_PD_EPR_ENABLE) */
    cy_stc_pdstack_dpm_pd_cmd_buf_t cmd_buf;
    cy_pd_pd_do_t* pdo_src = &srcCap->dat[pdo_no - 1u];
    cy_pd_pd_do_t snkRdo;

    snkRdo.val = 0u;
    snkRdo.rdo_gen.noUsbSuspend = context->dpmStat.snkUsbSuspEn;
    snkRdo.rdo_gen.usbCommCap = context->dpmStat.snkUsbCommEn;
    snkRdo.rdo_gen.capMismatch = 0u;
#if (CY_PD_EPR_ENABLE)
    /* In request PDO index is SPR 1...7, EPR 8...13 */
    if(pdo_no > CY_PD_MAX_NO_OF_PDO)
    {
        /* if PDO index > 7, set the bit 31 and limit EPR obj_pos in 0...5 range */
        snkRdo.rdo_gen.eprPdo = true;
    }
    snkRdo.rdo_gen.objPos = (pdo_no & CY_PD_MAX_NO_OF_PDO);
#else
    snkRdo.rdo_gen.objPos = pdo_no;
#endif /* CY_PD_EPR_ENABLE */

    if(pdo_src->fixed_src.supplyType != CY_PDSTACK_PDO_AUGMENTED)
    {
        snkRdo.rdo_gen.giveBackFlag = false;
        if(pdo_src->fixed_src.supplyType == CY_PDSTACK_PDO_BATTERY)
        {
            uint16_t power = CY_PDUTILS_DIV_ROUND_UP(volt * cur, 500);
            snkRdo.rdo_gen.opPowerCur = power;
            snkRdo.rdo_gen.minMaxPowerCur = power;
        }
        else
        {
            snkRdo.rdo_gen.opPowerCur = cur;
            snkRdo.rdo_gen.minMaxPowerCur = cur;
        }
    }
    else
    {
        if((pdo_src->spr_avs_src.apdoType == CY_PDSTACK_APDO_SPR_AVS) ||
           (pdo_src->epr_avs_src.apdoType == CY_PDSTACK_APDO_AVS))
        {
            /* Convert voltage to 25 mV unit */
            snkRdo.rdo_spr_avs.outVolt = volt * 2u;
            /* Convert current to 50 mA unit */
            snkRdo.rdo_spr_avs.opCur = cur / 5u;
        }
    }

#if (CY_PD_REV3_ENABLE)
    /* Supports unchunked extended messages in PD 3.0 mode. */
    if (context->dpmConfig.specRevSopLive >= CY_PD_REV3)
    {
        snkRdo.rdo_gen.unchunkSup = true;
    }

#if (CY_PD_EPR_ENABLE)
    if (context->dpmConfig.specRevSopLive >= CY_PD_REV3)
    {
        snkRdo.rdo_gen.eprModeCapable = dpmExtStat->epr.snkEnable;
    }
#endif /* CY_PD_EPR_ENABLE */
#endif /* CY_PD_REV3_ENABLE */

    /* Prepare the DPM command buffer */
    cmd_buf.cmdSop = (cy_en_pd_sop_t)CY_PD_SOP;
    cmd_buf.noOfCmdDo = 1u;
    cmd_buf.cmdDo[0] = snkRdo;
#if (CY_PD_EPR_ENABLE)
    if(dpmExtStat->eprActive == true)
    {
        cmd_buf.noOfCmdDo = 2u;
        cmd_buf.cmdDo[1].val = pdo_src->val;

        status = Cy_PdStack_Dpm_SendPdCommand(context, CY_PDSTACK_DPM_CMD_SEND_EPR_REQUEST, &cmd_buf, false, NULL);
    }
    else
#endif /* (CY_PD_EPR_ENABLE) */
    {
        status = Cy_PdStack_Dpm_SendPdCommand(context, CY_PDSTACK_DPM_CMD_SEND_REQUEST, &cmd_buf, false, NULL);
    }

    return status;
}

/*******************************************************************************
* Function Name: snk_request_new_contract
********************************************************************************
* Summary:
*  Evaluates the user request and sends request message to the source
*
* Parameters:
*  context - PdStack context
*  supply_type - Supply type
*  volt - Voltage in mV
*  Cur - Current in mA
*
* Return:
* CY_PDSTACK_STAT_SUCCESS if the request is successful.
* CY_PDSTACK_STAT_FAILURE if the request is failed.
*
*******************************************************************************/
static cy_en_pdstack_status_t snk_request_new_contract(cy_stc_pdstack_context_t *context, en_supply_type_t supply_type, uint16_t volt, uint16_t cur)
{
    cy_en_pdstack_status_t status = CY_PDSTACK_STAT_FAILURE;
    uint8_t obj_pos = 0u;
    /* Convert voltage to 50mV units */
    volt = volt / 50u;
    /* Convert current to 10mA units */
    cur = cur / 10u;

    if(is_request_valid(context, volt, cur))
    {
        obj_pos = select_src_pdo(context, supply_type, volt, cur, context->dpmStat.srcCapP);
        if(obj_pos != 0u)
        {
            status = send_request(context, obj_pos, volt, cur, context->dpmStat.srcCapP);
        }
    }

    return status;
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: capsense_callback
********************************************************************************
* Summary:
*  Capsense Callback function
*
* Parameters:
*  ptrActiveScan
*
* Return:
*  None
*
*******************************************************************************/
void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    gl_capsense_scan_complete = true;
}

/*******************************************************************************
* Function Name: scan_touch
********************************************************************************
* Summary:
*  Scans all the Widgets if the previous processing was complete
*
* Parameters:
*  None
*
* Return:
*  None
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

    if(gl_capsense_scan_start == FAST_SCAN)
    {
        /* Decrement the scan count only if the scan was initiated in prev step */
        if(scanStart == 1u)
        {
            /* Fast scan count is decremented every time a scan is performed */
            gl_capsense_fast_scan_count--;
        }

        /* Once the the scan count lapses, the slow timer is started */
        if(gl_capsense_fast_scan_count == 0u)
        {
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
    else
    {
        /* Start the slow timer to initiate the next scan when the timer expires */
        Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, NULL, (cy_timer_id_t)CAPSENSE_SLOW_TIMER_ID,
                (uint16_t)CAPSENSE_SLOW_SCAN_INTERVAL, capsense_slow_timer_cb);
    }

    /* Clear the scan start flag. */
    gl_capsense_scan_start = 0u;

    return scanStart;
}

/*******************************************************************************
* Function Name: capsense_slow_timer_cb
********************************************************************************
* Summary:
*  Scans all the Widgets
*  Starts the Timer
*
* Parameters:
*  id - Timer ID
*  callbackContext - Timer Context
*
* Return:
*  None
*
*******************************************************************************/
void capsense_slow_timer_cb (cy_timer_id_t id, void *callbackContext)
{
    gl_capsense_scan_start = SLOW_SCAN;
}

/*******************************************************************************
* Function Name: capsense_fast_timer_cb
********************************************************************************
* Summary:
*  Scans all the Widgets
*  Decrement the scan count only if the scan was initiated in prev step
*  Starts the Timer (Slow/Fast) depending on the scan count
*
* Parameters:
*  id - Timer ID
*  callbackContext - Timer Context
*
* Return:
*  None
*
*******************************************************************************/
void capsense_fast_timer_cb (cy_timer_id_t id, void *callbackContext)
{
    gl_capsense_scan_start = FAST_SCAN;
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
*
* Parameters:
*  None
*
* Return:
*  uint32_t
*
*******************************************************************************/
uint32_t initialize_capsense(void)
{
    uint32_t status = CY_CAPSENSE_STATUS_SUCCESS;

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

/*******************************************************************************
* Function Name: enable_fast_timer
********************************************************************************
* Summary:
*  Starts the Fast Timer
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void enable_fast_timer(void)
{
    /* Since touch is detected, enable fast scan reset count */
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
*  Checks if Capsense Widgets are touched
*  Glows the corresponding LED if widget is touched
*  PDO is changed depending on the slider touched
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void process_touch(void)
{
    uint8_t touchDetected = 0u;
    uint32_t button0_status;
    uint32_t button1_status;
    cy_stc_capsense_touch_t *ptrSliderPosition;
    uint32_t position = 0u;
#if (CY_PD_EPR_ENABLE)
    bool enterEprMode = false;
    bool exitEprMode = false;
#endif /* (CY_PD_EPR_ENABLE) */
    bool newContractReq = false;
    cy_stc_pdstack_context_t *ptrPdStackContext = &gl_PdStackPort0Ctx;
    cy_stc_pdstack_dpm_ext_status_t *dpmExt = &(ptrPdStackContext->dpmExtStat);
    uint16_t volt = 0u;
    /* The current can be updated with respect to the voltage requested */
    uint16_t cur = 900u;
    en_supply_type_t supply_type = FIXED_SUPPLY;

    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON0_WDGT_ID,
        CY_CAPSENSE_BUTTON0_SNS0_ID,
        &cy_capsense_context);
    if(button0_status != 0u)
    {
        /* Switch ON the LED when touch is detected */
        Cy_GPIO_Write(CYBSP_LED_BTN0_PORT, CYBSP_LED_BTN0_PIN, LED_ON);
        /* Set the flag to indicate touch was detected */
        touchDetected = 1u;
#if (CY_PD_EPR_ENABLE)
        if(dpmExt->eprActive == true)
        {
            /* Exit EPR mode */
            exitEprMode = true;
        }
#endif /* (CY_PD_EPR_ENABLE) */
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
    if(button1_status != 0u)
    {
        /* Switch ON the LED when touch is detected */
        Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_PIN, LED_ON);
        /* Set the flag to indicate touch was detected */
        touchDetected = 1u;

#if (CY_PD_EPR_ENABLE)
        if ((dpmExt->eprActive == false) &&
            (ptrPdStackContext->dpmStat.srcCapP->dat[0].fixed_src.eprModeCapable == true) &&
            (ptrPdStackContext->dpmExtStat.epr.snkEnable == true))
         {

             enterEprMode = true;
         }
#endif /* CY_PD_EPR_ENABLE */
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

        /* Set new contract request flag */
        newContractReq = true;

        if(position <= ( 1 * STEP_SIZE))
        {
            /* Request for 5V fixed supply */
            volt = CY_PD_VSAFE_5V;
        }
        else if( (position > ( 1 * STEP_SIZE)) && (position <= ( 2 * STEP_SIZE)))
        {
            /* Request for 9V fixed supply */
            volt = CY_PD_VSAFE_9V;
        }
        else if( (position > ( 2 * STEP_SIZE)) && (position <= (3 * STEP_SIZE)))
        {
            /* Request for 12V fixed supply */
            volt = CY_PD_VSAFE_12V;
        }
        else if( (position > ( 3 * STEP_SIZE)) && (position <= ( 4 * STEP_SIZE)))
        {
            /* Request for 15V fixed supply */
            volt = CY_PD_VSAFE_15V;
        }
        else if( (position > ( 4 * STEP_SIZE)) && (position <= ( 5 * STEP_SIZE)))
        {
            /* Request for 20V fixed supply */
            volt = CY_PD_VSAFE_20V;
        }
        else
        {
            /* Do nothing */
            newContractReq = false;
        }
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
        if(newContractReq == true)
        {
            newContractReq = false;
            if(volt != gl_cur_voltage)
            {
                if(snk_request_new_contract(ptrPdStackContext, supply_type, volt, cur) == CY_PDSTACK_STAT_SUCCESS)
                {
                    gl_cur_voltage = volt;
                }
            }
        }
#if (CY_PD_EPR_ENABLE)
        else if(enterEprMode == true)
        {
            enterEprMode = false;
            Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SNK_EPR_MODE_ENTRY, NULL, false, NULL);
        }
        else if(exitEprMode == true)
        {
            exitEprMode = false;
            bool isEprSpr = false;
            Cy_PdStack_Dpm_IsEprSpr(ptrPdStackContext, &isEprSpr);
            if(isEprSpr != true)
            {
                cy_stc_pdstack_dpm_pd_cmd_buf_t dpm_cmd_buf;
                if(Cy_App_GetStatus(ptrPdStackContext->port)->psnk_volt <= CY_PD_VSAFE_20V)
                {
                    dpm_cmd_buf.noOfCmdDo = 1u;
                    dpm_cmd_buf.cmdDo[0].val = 0u;
                    dpm_cmd_buf.cmdDo[0].eprmdo.action = CY_PDSTACK_EPR_MODE_EXIT;

                    Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_EPR_MODE, &dpm_cmd_buf, false, NULL);
                }
                else
                {
                    get_src_data_buffer[0] = CY_PDSTACK_EPR_GET_SRC_CAP;
                    get_src_data_buffer[1] = 0x0u;
                    dpm_cmd_buf.cmdSop = CY_PD_SOP;
                    dpm_cmd_buf.extdType = CY_PDSTACK_EXTD_MSG_EXTD_CTRL_MSG;
                    dpm_cmd_buf.extdHdr.extd.dataSize = 0x02u;
                    dpm_cmd_buf.extdHdr.extd.chunked = !(ptrPdStackContext->dpmStat.unchunkSupLive);
                    dpm_cmd_buf.datPtr = (uint8_t*)&get_src_data_buffer[0];
                    dpm_cmd_buf.cmdDo[0].val = CY_PDSTACK_EPR_GET_SRC_CAP;

                    if(CY_PDSTACK_STAT_SUCCESS == Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, CY_PDSTACK_DPM_CMD_SEND_EXTENDED, &dpm_cmd_buf, false, NULL))
                    {
                        gl_epr_exit = true;
                        Cy_PdStack_Dpm_ChangeEprToSpr(ptrPdStackContext, true);
                    }
                }
            }
        }
#endif /* CY_PD_EPR_ENABLE */
        else
        {
            /* Do Nothing */
        }

        /* Start fast timer since the touch is detected */
        enable_fast_timer();
    }
}

/* [] END OF FILE */

