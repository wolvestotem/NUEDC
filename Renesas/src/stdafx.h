/*
 * stdafx.h
 *
 *  Created on: 2017年8月5日
 *      Author: msi
 */

#ifndef __STDAFX__
#define __STDAFX__

#define COPTER_HEIGHT 0.8f

/* DEBUG Text Type */
//#define SCSELFCHECKTEXT
//#define SCPIDTEXT
//#define SCCOPTERTEXT
//#define SCVELOCITYTEXT
#define SCSPIDATAASSERT
//#define SCOPENMVTEXT
//#define SCPATHTEXT
#define SCB2C
//#define STOPWHENNOTFOUND
#define EMERGENCYSTOP
//#define SCDISTANCETEXT
#define ALWAYSFRESHDISTANCEINDICATOR

/* Pin Assignment */
#define SAFE_SWITCH_PIN PORT4.PIDR.BIT.B7  // pin49
#define BIT0_SWITCH_PIN PORT4.PIDR.BIT.B5  // pin51
#define BIT1_SWITCH_PIN PORT4.PIDR.BIT.B3  // pin53
#define OPENMV_CHECK_SWITCH PORT4.PIDR.BIT.B1 // pin55
#define BUZZER_PIN PORT3.PODR.BIT.B2       // pin41
#define BIT0_INDICATOR PORT7.PODR.BIT.B0   // pin39
#define BIT1_INDICATOR PORT7.PODR.BIT.B2   // pin37
#define READY_TO_TAKEOFF_LED PORT7.PODR.BIT.B0 // pin39
#define DISTANCE_INDICATOR PORT7.PODR.BIT.B4 // pin35
//#define SELF_CHECKED_LED PORT7.PODR.BIT.B1 // pin38 (disabled)

#include "cg_src/r_cg_macrodriver.h"
#include "math.h"
#include "cg_src/r_cg_cmt.h"
#include "cg_src/r_cg_sci.h"
#include "cg_src/r_cg_rspi.h"
#include "Mavlink/Mavlink_Head.h"
#include "Bluetooth_Decoder/Bluetooth_Decoder.h"
#include "Sc_CopterControl/Sc_CopterControl.h"
#include "Sc_AlgorithmTools/Sc_AlgorithmTools.h"
#include "Sc_Serial_Library_new/Sc_Serial_Library_new.h"
#include "PathSeeking/PathSeeking.h"

short Software_Enabled(void);
void Enable_Software(void);
void Disable_SoftWare(void);

#endif /* STDAFX_H_ */
