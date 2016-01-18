/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <linux/xlog.h>
#include <mach/mt_pm_ldo.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  			(720)
#define FRAME_HEIGHT 			(1280)
#define LCM_OTM1283_ID			(0x1283)

#define REGFLAG_DELAY          		(0xFE)
#define REGFLAG_END_OF_TABLE      	(0xFB)	// END OF REGISTERS MARKER

#ifndef FALSE
#define FALSE 0
#endif
#define GPIO_LCD_RST_EN      (GPIO131)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    		(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 					(lcm_util.udelay(n))
#define MDELAY(n) 					(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)        (lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update))
#define dsi_set_cmdq(pdata, queue_size, force_update)		(lcm_util.dsi_set_cmdq(pdata, queue_size, force_update))
#define wrtie_cmd(cmd)						(lcm_util.dsi_write_cmd(cmd))
#define write_regs(addr, pdata, byte_nums)			(lcm_util.dsi_write_regs(addr, pdata, byte_nums))
#define read_reg						(lcm_util.dsi_read_reg())
#define read_reg_v2(cmd, buffer, buffer_size)   		(lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size))

static struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
{0x00,1,{0x00}},
	{0xff,3,{0x12,0x83,0x01}},	//EXTC=1
	{0x00,1,{0x80}},	        //Orise mode enable
	{0xff,2,{0x12,0x83}},

//-------------------------------------------------------// 
	{0x00,1,{0xB9}},             
	{0xB0,1,{0x51}},
	{0x00,1,{0xC6}},
	{0xB0,1,{0x03}}, 
//-------------------------------------------------------// 
//-------------------- panel setting --------------------//

	{0x00,1,{0x80}},             //TCON Setting
	{0xc0,9,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},
        
	{0x00,1,{0x90}},             //Panel Timing Setting
	{0xc0,6,{0x00,0x5c,0x00,0x01,0x00,0x04}},
        
//	{0x00,1,{0xa2}},
//	{0xC0,3,{0x01,0x00,0x00}},
         
	{0x00,1,{0xb3}},             //Interval Scan Frame: 0 frame, column inversion
	{0xc0,2,{0x00,0x50}},
         
	{0x00,1,{0x81}},             //frame rate:60Hz
	{0xc1,1,{0x55}},
         
	{0x00,1,{0x82}},             
	{0xc4,1,{0x02}},

	{0x00,1,{0x8B}},             
	{0xc4,1,{0x40}},
	
	{0x00,1,{0x90}},             //clock delay for data latch 
	{0xc4,1,{0x49}},

//-------------------- power setting --------------------//
	{0x00,1,{0xa0}},             //dcdc setting
	{0xc4,14,{0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}},

	{0x00,1,{0xb0}},             //clamp voltage setting
	{0xc4,2,{0x00,0x00}},
         
	{0x00,1,{0x90}},             //VGH=13V, VGL=-12V, pump ratio:VGH=6x, VGL=-5x
	{0xc5,4,{0x50,0xA6,0xD0,0x66}},
        
	{0x00,1,{0xB0}},             //GVDD=4.204V, NGVDD=-4.204V 
	{0xC5,2,{0x04,0x38}},             
       
	{0x00,1,{0xB4}},             //VCOM=0.240V
	{0xC5,1,{0xC0}},             
         
	{0x00,1,{0xB5}},             //source bias 0.75uA
	{0xc5,6,{0x0B,0x95,0xFF,0x0B,0x95,0xFF}},
        
//	{0x00,1,{0xb0}},             //VDD_18V=1.6V, LVDSVDD=1.55V
//	{0xc5,2,{0x04,0xb8}},
        
	{0x00,1,{0xbb}},             //LVD voltage level setting
	{0xc5,1,{0x80}},
         
	{0x00,1,{0x90}},		//chopper
	{0xF5,4,{0x02,0x11,0x02,0x11}},
        
	{0x00,1,{0x94}},		//debounce
	{0xF5,1,{0x02}},

//-------------------- control setting --------------------//
	{0x00,1,{0xB2}},             //ID1
	{0xF5,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
        
	{0x00,1,{0xBA}},             //ID2, ID3
	{0xF5,1,{0x03}},

//-------------------- power on setting --------------------//
/*	{0x00,1,{0x80}},             //source blanking frame = black, defacult='30'
	{0xc4,1,{0x00}},
       
	{0x00,1,{0x98}},             //vcom discharge=gnd:'10', '00'=disable
	{0xc5,1,{0x10}},
        
	{0x00,1,{0x81}},
	{0xf5,1,{0x15}},  // ibias off
	{0x00,1,{0x83}}, 
	{0xf5,1,{0x15}},  // lvd off
	{0x00,1,{0x85}},
	{0xf5,1,{0x15}},  // gvdd off
	{0x00,1,{0x87}}, 
	{0xf5,1,{0x15}},  // lvdsvdd off
	{0x00,1,{0x89}},
	{0xf5,1,{0x15}},  // nvdd_18 off
	{0x00,1,{0x8b}}, 
	{0xf5,1,{0x15}},  // en_vcom off
      
	{0x00,1,{0x95}},
	{0xf5,1,{0x15}},  // pump3 off
	{0x00,1,{0x97}}, 
	{0xf5,1,{0x15}},  // pump4 off
	{0x00,1,{0x99}},
	{0xf5,1,{0x15}},  // pump5 off
       
	{0x00,1,{0xa1}}, 
	{0xf5,1,{0x15}},  // gamma off
	{0x00,1,{0xa3}},
	{0xf5,1,{0x15}},  // sd ibias off
	{0x00,1,{0xa5}}, 
	{0xf5,1,{0x15}},  // sdpch off
	{0x00,1,{0xa7}}, 
	{0xf5,1,{0x15}},  // sdpch bias off
	{0x00,1,{0xab}},
	{0xf5,1,{0x18}},  // ddc osc off
       
	{0x00,1,{0x94}},  //VCL on  	
	{0xF5,1,{0x02}},

	{0x00,1,{0xBA}},  //VSP on   	
	{0xF5,1,{0x03}},

	{0x00,1,{0xb1}},             //VGLO, VGHO setting
	{0xf5,13,{0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x15,0x08,0x15}},

	{0x00,1,{0xb4}},             //VGLO1/2 Pull low setting
	{0xc5,1,{0xc0}},		//d[7] vglo1 d[6] vglo2 => 0: pull vss, 1: pull vgl
	


//-------------------- for Power IC ---------------------------------
	{0x00,1,{0x90}},             //Mode-3
	{0xf5,4,{0x02,0x11,0x02,0x11}},

	{0x00,1,{0x90}},             //2xVPNL, 1.5*=00, 2*=50, 3*=a0
	{0xc5,1,{0x50}},

	{0x00,1,{0x94}},             //Frequency
	{0xc5,1,{0x66}},
*/

//-------------------- panel timing state control --------------------//
	{0x00,1,{0x80}},             //panel timing state control
	{0xCB,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x90}},             //panel timing state control
	{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xA0}},             //panel timing state control
	{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xB0}},             //panel timing state control
	{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xC0}},             //panel timing state control
	{0xCB,15,{0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,0x04,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xD0}},             //panel timing state control
	{0xCB,15,{0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04}},

	{0x00,1,{0xE0}},             //panel timing state control
	{0xCB,14,{0x04,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00}},

	{0x00,1,{0xF0}},             //panel timing state control
	{0xCB,11,{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},

//  0003:8F60:
//-------------------- panel pad mapping control --------------------//
	{0x00,1,{0x80}},             //panel pad mapping control
	{0xcc,15,{0x29,0x2A,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x06,0x00,0x08,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x90}},             //panel pad mapping control
	{0xcc,15,{0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x29,0x2A,0x09,0x0B,0x0D,0x0F,0x11,0x13}},

	{0x00,1,{0xa0}},             //panel pad mapping control
	{0xcc,14,{0x05,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00}},

	{0x00,1,{0xb0}},             //panel pad mapping control
	{0xcc,15,{0x29,0x2A,0x13,0x11,0x0F,0x0D,0x0B,0x09,0x01,0x00,0x07,0x00,0x00,0x00,0x00}},

	{0x00,1,{0xc0}},             //panel pad mapping control
	{0xcc,15,{0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x29,0x2A,0x14,0x12,0x10,0x0E,0x0C,0x0A}},

	{0x00,1,{0xd0}},             //panel pad mapping control
	{0xcc,14,{0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00}},

//-------------------- panel timing setting --------------------//
	{0x00,1,{0x80}},             //panel VST setting
	{0xCE,12,{0x89,0x05,0x10,0x88,0x05,0x10,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00,1,{0x90}},             //panel VEND setting
	{0xCE,14,{0x54,0xFD,0x10,0x54,0xFE,0x10,0x55,0x01,0x10,0x55,0x02,0x10,0x00,0x00}},

	{0x00,1,{0xa0}},             //panel CLKA1/2 setting
	{0xCE,14,{0x58,0x07,0x04,0xFD,0x00,0x10,0x00,0x58,0x06,0x04,0xFE,0x00,0x10,0x00}},

	{0x00,1,{0xb0}},             //panel CLKA3/4 setting
	{0xCE,14,{0x58,0x05,0x04,0xFF,0x00,0x10,0x00,0x58,0x04,0x05,0x00,0x00,0x10,0x00}},

	{0x00,1,{0xc0}},             //panel CLKb1/2 setting
	{0xCE,14,{0x58,0x03,0x05,0x01,0x00,0x10,0x00,0x58,0x02,0x05,0x02,0x00,0x10,0x00}},

	{0x00,1,{0xd0}},             //panel CLKb3/4 setting
	{0xCE,14,{0x58,0x01,0x05,0x03,0x00,0x10,0x00,0x58,0x00,0x05,0x04,0x00,0x10,0x00}},

	{0x00,1,{0x80}},             //panel CLKc1/2 setting
	{0xCF,14,{0x50,0x00,0x05,0x05,0x00,0x10,0x00,0x50,0x01,0x05,0x06,0x00,0x10,0x00}},

	{0x00,1,{0x90}},             //panel CLKc3/4 setting
	{0xCF,14,{0x50,0x02,0x05,0x07,0x00,0x10,0x00,0x50,0x03,0x05,0x08,0x00,0x10,0x00}},

//	{0x00,1,{0xa0}},             //panel CLKd1/2 setting
//	{0xcf,14,{0x38,0x03,0x05,0x04,0x00,0x18,0x25,0x38,0x02,0x05,0x05,0x00,0x18,0x25}},

//	{0x00,1,{0xb0}},             //panel CLKd3/4 setting
//	{0xcf,14,{0x38,0x01,0x05,0x06,0x00,0x18,0x25,0x38,0x00,0x05,0x07,0x00,0x18,0x25}},

	{0x00,1,{0xc0}},             //panel ECLK setting
	{0xcf,11,{0x39,0x39,0x20,0x20,0x00,0x00,0x01,0x01,0x20,0x00,0x00}}, //gate pre. ena.

	{0x00,1,{0x00}},			//GVDD=4.204V, NGVDD=-4.204V 
	{0xD8,2,{0xBE,0xBE}},

	{0x00,1,{0x00}},			//VCOM=0.240V 
	{0xD9,1,{0x82}},
	
	{0x00,1,{0x00}},
	{0xE1,16,{0x02,0x12,0x18,0x0E,0x07,0x0F,0x0B,0x09,0x04,0x07,0x0E,0x08,0x0F,0x12,0x0C,0x08}}, 
	{0x00,1,{0x00}},
	{0xE2,16,{0x02,0x12,0x18,0x0E,0x07,0x10,0x0B,0x0A,0x04,0x08,0x0E,0x08,0x0F,0x12,0x0C,0x08}}, 
  {0x00,1,{0x00}},             //Orise mode disable
  {0xff,3,{0xff,0xff,0xff}},

  {0x35,1,{0x00}},//TE ON                                                                                                 
                                                                                                                        
{0x11,0,{0x00}},//  
  {REGFLAG_DELAY,120,{}},                                                                                                 
                                   				                                                                                
{0x29,0,{0x00}}, //Display on (}},
  {REGFLAG_DELAY,20,{}},  
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 100, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {

		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}

	}

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS * params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;	//BURST_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = 720 * 3;

		params->dsi.vertical_sync_active				= 0x3;// 3    2
		params->dsi.vertical_backporch					= 0x0E;// 20   1
		params->dsi.vertical_frontporch					= 0x10; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 0x04;// 50  2
		params->dsi.horizontal_backporch				= 0x22 ;
		params->dsi.horizontal_frontporch				= 0x18 ;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	// Bit rate calculation
	//1 Every lane speed
	params->dsi.pll_div1 = 0;	// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2 = 1;	// div2=0,1,2,3;div1_real=1,2,4,4
	params->dsi.fbk_div = 14;	//12;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)

}

static void lcm_init(void)
{
	unsigned int data_array[16];

#ifdef BUILD_LK
	upmu_set_rg_vgp6_vosel(6);
	upmu_set_rg_vgp6_en(1);
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP6, VOL_3300, "LCM");
#endif
	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	MDELAY(20);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(120);

	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) /
		   sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00280500;	// Display Off
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00100500;	// Sleep In
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(200);
#ifdef BUILD_LK
	upmu_set_rg_vgp6_en(0);
#else
	hwPowerDown(MT65XX_POWER_LDO_VGP6, "LCM");
#endif
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
	lcm_init();
#endif
}


static unsigned int lcm_compare_id(void)
{
	unsigned int id0, id1, id2, id3, id4;
	unsigned char buffer[5];
	unsigned int array[5];
#ifdef BUILD_LK
	upmu_set_rg_vgp6_vosel(6);
	upmu_set_rg_vgp6_en(1);
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP6, VOL_2800, "LCM");
#endif

	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(100);

	// Set Maximum return byte = 1
	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, buffer, 5);
	id0 = buffer[0];
	id1 = buffer[1];
	id2 = buffer[2];
	id3 = buffer[3];
	id4 = buffer[4];

#if defined(BUILD_LK)
	printf("%s, Module ID = {%x, %x, %x, %x, %x} \n", __func__, id0,
	       id1, id2, id3, id4);
#else
	printk("%s, Module ID = {%x, %x, %x, %x,%x} \n", __func__, id0,
	       id1, id2, id3, id4);
#endif

	return (LCM_OTM1283_ID == ((id2 << 8) | id3)) ? 1 : 0;
}

//

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER otm1283a_ykl_LP050CPKP245A_CMI_dsi_vdo_hd_lcm_drv = {
	.name = "otm1283a_ykl_LP050CPKP245A_CMI_dsi_vdo_hd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
};
