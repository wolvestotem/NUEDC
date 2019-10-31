/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIESREGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2015, 2016 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_main.c
* Version      : Code Generator for RX23T V1.00.04.02 [29 Nov 2016]
* Device(s)    : R5F523T5AxFM
* Tool-Chain   : CCRX
* Description  : This file implements main function.
* Creation Date: 2017/8/7
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_cmt.h"
#include "r_cg_rspi.h"
#include "r_cg_sci.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */


static void R_MAIN_UserInit(void);
/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    /* Start user code. Do not edit comment generated here */
    while (1U)
    {
    	int mode = -1;
    	mode = Set_Mode_And_Safety_Switch();
    	switch(mode)
    	{
    	case 0:
    		Take_Off();
    		Mission_One();
    		break;
    	case 1:
			//Take_Off();
			Mission_Two();
			break;
    	case 2:
			Take_Off();
			Mission_Three();
			break;
    	case 3:
			Take_Off();
			Mission_Four_and_Five();
			break;
    	default:
    		break;
    	}
    	Land_Reset();
    	Enable_Software();
    	//Copter_Land();
    }
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
	SCI1_Start();
	SCI5_Start();
	R_RSPI0_Start();

	MavLink_Start();
	Decoder_Init();

	R_CMT0_Start();
	R_CMT1_Start();    //heartbeat

	delay_ms(4000);

	if(OPENMV_CHECK_SWITCH == 0U && BIT0_SWITCH_PIN == 0U && BIT1_SWITCH_PIN == 0U)
		Copter_Level_Calibration();

	Follow_Car_Self_Check();    // Move Down After Debug

	while(OPENMV_CHECK_SWITCH == 0U)
		OpenMV_Assert();
	BIT0_INDICATOR = 0U;
	BIT0_INDICATOR = 0U;
	BUZZER_PIN = 0U;

	delay_ms(2000);
	Copter_PreArm();

	Wait_For_Take_Off();

	while(OPENMV_CHECK_SWITCH == 0U)
		OpenMV_Assert();
	BIT0_INDICATOR = 0U;
	BIT0_INDICATOR = 0U;
	BUZZER_PIN = 0U;
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
