/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 * 
 * MediaTek Inc. (C) 2010. All rights reserved.
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

/******************************************************************************
 * mt6516_gpio.h - The file is the gpio header file !
 * 
 * Copyright 2008-2009 MediaTek Co.,Ltd.
 * 
 * DESCRIPTION: - 
 *     This file expose an in-kernel gpio modules to other device drivers
 *
 * modification history
 * ----------------------------------------
 * 01a, 08 oct 2008, Koshi,Chiu written
 * ----------------------------------------
 ******************************************************************************/
#include <cust_gpio_usage.h>

/*----------------------------------------------------------------------------*/
//  Error Code No.
#define RSUCCESS        0
#define ERACCESS        1
#define ERINVAL         2
#define ERWRAPPER		3
/*----------------------------------------------------------------------------*/
/******************************************************************************
* Enumeration for GPIO pin
******************************************************************************/
typedef enum GPIO_PIN
{    
    GPIO_UNSUPPORTED = -1,    
        
    GPIO0  , GPIO1  , GPIO2  , GPIO3  , GPIO4  , GPIO5  , GPIO6  , GPIO7  ,
    GPIO8  , GPIO9  , GPIO10 , GPIO11 , GPIO12 , GPIO13 , GPIO14 , GPIO15 ,
    GPIO16 , GPIO17 , GPIO18 , GPIO19 , GPIO20 , GPIO21 , GPIO22 , GPIO23 ,
    GPIO24 , GPIO25 , GPIO26 , GPIO27 , GPIO28 , GPIO29 , GPIO30 , GPIO31 ,
    GPIO32 , GPIO33 , GPIO34 , GPIO35 , GPIO36 , GPIO37 , GPIO38 , GPIO39 ,
    GPIO40 , GPIO41 , GPIO42 , GPIO43 , GPIO44 , GPIO45 , GPIO46 , GPIO47 ,
    GPIO48 , GPIO49 , GPIO50 , GPIO51 , GPIO52 , GPIO53 , GPIO54 , GPIO55 ,
    GPIO56 , GPIO57 , GPIO58 , GPIO59 , GPIO60 , GPIO61 , GPIO62 , GPIO63 ,
    GPIO64 , GPIO65 , GPIO66 , GPIO67 , GPIO68 , GPIO69 , GPIO70 , GPIO71 ,
    GPIO72 , GPIO73 , GPIO74 , GPIO75 , GPIO76 , GPIO77 , GPIO78 , GPIO79 ,
    GPIO80 , GPIO81 , GPIO82 , GPIO83 , GPIO84 , GPIO85 , GPIO86 , GPIO87 ,
    GPIO88 , GPIO89 , GPIO90 , GPIO91 , GPIO92 , GPIO93 , GPIO94 , GPIO95 ,
    GPIO96 , GPIO97 , GPIO98 , GPIO99 , GPIO100, GPIO101, GPIO102, GPIO103,
    GPIO104, GPIO105, GPIO106, GPIO107, GPIO108, GPIO109, GPIO110, GPIO111,
    GPIO112, GPIO113, GPIO114, GPIO115, GPIO116, GPIO117, GPIO118, GPIO119,
    GPIO120, GPIO121, GPIO122, GPIO123, GPIO124, GPIO125, GPIO126, GPIO127,
    GPIO128, GPIO129, GPIO130, GPIO131, GPIO132, GPIO133, GPIO134, GPIO135,
    GPIO136, GPIO137, GPIO138, GPIO139, GPIO140, GPIO141, GPIO142, GPIO143,
    GPIO144, GPIO145, GPIO146, GPIO147, GPIO148, GPIO149, GPIO150, GPIO151,
    GPIO152, GPIO153, GPIO154, GPIO155, GPIO156, GPIO157, GPIO158, GPIO159,
    GPIO160, GPIO161, GPIO162, GPIO163, GPIO164, GPIO165, GPIO166, GPIO167,
    GPIO168, GPIO169, GPIO170, GPIO171, GPIO172, GPIO173, GPIO174, GPIO175,
    GPIO176, GPIO177, GPIO178, GPIO179, GPIO180, GPIO181, GPIO182, GPIO183,
    GPIO184, GPIO185, GPIO186, GPIO187, GPIO188, GPIO189, GPIO190, GPIO191,
    GPIO192, GPIO193, GPIO194, GPIO195, GPIO196, GPIO197, GPIO198, GPIO199,
    GPIO200, GPIO201, GPIO202, GPIO203, GPIO204, GPIO205, GPIO206, GPIO207,
    GPIO208, GPIO209, GPIO210, GPIO211, GPIO212, GPIO213, GPIO214, GPIO215,
    GPIO216, GPIO217, GPIO218, GPIO219, GPIO220, GPIO221, GPIO222, GPIO223,
    GPIO224, GPIO225, GPIO226, GPIO227, GPIO228, GPIO229, GPIO230, GPIO231,
    GPIOEXT0 , GPIOEXT1 , GPIOEXT2 , GPIOEXT3 , GPIOEXT4 , GPIOEXT5 , GPIOEXT6 , GPIOEXT7,	
    GPIOEXT8 , GPIOEXT9 , GPIOEXT10, GPIOEXT11, GPIOEXT12, GPIOEXT13, GPIOEXT14, GPIOEXT15,	
    GPIOEXT16, GPIOEXT17, GPIOEXT18, GPIOEXT19, GPIOEXT20, GPIOEXT21, GPIOEXT22, GPIOEXT23,	
    GPIOEXT24, GPIOEXT25, GPIOEXT26, GPIOEXT27, GPIOEXT28, GPIOEXT29, GPIOEXT30, GPIOEXT31,	
    GPIOEXT32, GPIOEXT33, GPIOEXT34, GPIOEXT35, GPIOEXT36, GPIOEXT37, GPIOEXT38, GPIOEXT39,	
    GPIOEXT40, GPIOEXT41, GPIOEXT42, GPIOEXT43, GPIOEXT44, GPIOEXT45, GPIOEXT46, GPIOEXT47,	
    GPIOEXT48, GPIO_MAX
}GPIO_PIN;         

#define MAX_GPIO_PIN    (GPIO_MAX)
#define GPIO_EXTEND_START GPIOEXT0
typedef enum GPIO_PIN_EXT
{    
    GPIO232 = GPIO_EXTEND_START,    
    GPIO233, GPIO234, GPIO235, GPIO236, GPIO237, GPIO238, GPIO239,	
    GPIO240, GPIO241, GPIO242, GPIO243, GPIO244, GPIO245, GPIO246, GPIO247,	
    GPIO248, GPIO249, GPIO250, GPIO251, GPIO252, GPIO253, GPIO254, GPIO255,	
    GPIO256, GPIO257, GPIO258, GPIO259, GPIO260, GPIO261, GPIO262, GPIO263,	
    GPIO264, GPIO265, GPIO266, GPIO267, GPIO268, GPIO269, GPIO270, GPIO271,	
    GPIO272, GPIO273, GPIO274, GPIO275, GPIO276, GPIO277, GPIO278, GPIO279,	
    GPIO280
}GPIO_PIN_EXT;    
/******************************************************************************
* Enumeration for Clock output
******************************************************************************/
/* GPIO MODE CONTROL VALUE*/
typedef enum {
    GPIO_MODE_UNSUPPORTED = -1,
    GPIO_MODE_GPIO  = 0,
    GPIO_MODE_00    = 0,
    GPIO_MODE_01    = 1,
    GPIO_MODE_02    = 2,
    GPIO_MODE_03    = 3,
    GPIO_MODE_04    = 4,
    GPIO_MODE_05    = 5,
    GPIO_MODE_06    = 6,
    GPIO_MODE_07    = 7,

    GPIO_MODE_MAX,
    GPIO_MODE_DEFAULT = GPIO_MODE_01,
} GPIO_MODE;
/*----------------------------------------------------------------------------*/
/* GPIO DIRECTION */
typedef enum {
    GPIO_DIR_UNSUPPORTED = -1,
    GPIO_DIR_IN     = 0,
    GPIO_DIR_OUT    = 1,

    GPIO_DIR_MAX,
    GPIO_DIR_DEFAULT = GPIO_DIR_IN,
} GPIO_DIR;
/*----------------------------------------------------------------------------*/
/* GPIO PULL ENABLE*/
typedef enum {
    GPIO_PULL_EN_UNSUPPORTED = -1,
    GPIO_PULL_DISABLE = 0,
    GPIO_PULL_ENABLE  = 1,

    GPIO_PULL_EN_MAX,
    GPIO_PULL_EN_DEFAULT = GPIO_PULL_ENABLE,
} GPIO_PULL_EN;
/*----------------------------------------------------------------------------*/
/* GPIO PULL-UP/PULL-DOWN*/
typedef enum {
    GPIO_PULL_UNSUPPORTED = -1,
    GPIO_PULL_DOWN  = 0,
    GPIO_PULL_UP    = 1,

    GPIO_PULL_MAX,
    GPIO_PULL_DEFAULT = GPIO_PULL_DOWN
} GPIO_PULL;
/*----------------------------------------------------------------------------*/
/* GPIO INVERSION */
typedef enum {
    GPIO_DATA_INV_UNSUPPORTED = -1,
    GPIO_DATA_UNINV = 0,
    GPIO_DATA_INV   = 1,

    GPIO_DATA_INV_MAX,
    GPIO_DATA_INV_DEFAULT = GPIO_DATA_UNINV
} GPIO_INVERSION;
/*----------------------------------------------------------------------------*/
/* GPIO OUTPUT */
typedef enum {
    GPIO_OUT_UNSUPPORTED = -1,
    GPIO_OUT_ZERO = 0,
    GPIO_OUT_ONE  = 1,

    GPIO_OUT_MAX,
    GPIO_OUT_DEFAULT = GPIO_OUT_ZERO,
    GPIO_DATA_OUT_DEFAULT = GPIO_OUT_ZERO,  /*compatible with DCT*/
} GPIO_OUT;
/* GPIO INPUT */
typedef enum {
    GPIO_IN_UNSUPPORTED = -1,
    GPIO_IN_ZERO = 0,
    GPIO_IN_ONE  = 1,

    GPIO_IN_MAX,
} GPIO_IN;
/*CLOCK OUT*/
typedef enum {
    CLK_OUT_UNSUPPORTED = -1,
    CLK_OUT0,
    CLK_OUT1,
    CLK_OUT2,
    CLK_OUT3,
    CLK_OUT4,
    CLK_OUT5,
    CLK_MAX	
}GPIO_CLKOUT;

typedef enum CLK_SRC
{
    CLK_SRC_UNSUPPORTED = -1,	
    CLK_SRC_F32K 	= 0x2,
    CLK_SRC_F26M 	= 0x4,
    CLK_SRC_FAXI0 	= 0x6,
    CLK_SRC_FAXI1 	= 0x8,
    CLK_SRC_FAXI2,
    CLK_SRC_FAXI3,
    CLK_SRC_FAXI4,
    CLK_SRC_DPI_PIXELPLL,
    CLK_SRC_3GPLL_123M,
    CLK_SRC_FMM,

    CLK_SRC_MAX
}GPIO_CLKSRC;
    
/* GPIO POWER*/
typedef enum {
    GPIO_VIO28 = 0,
    GPIO_VIO18 = 1,

    GPIO_VIO_MAX,
} GPIO_POWER;
/*----------------------------------------------------------------------------*/
typedef struct {    
    u16 val;        
    u16 _align1;
    u16 set;
    u16 _align2;
    u16 rst;
    u16 _align3[3];
} VAL_REGS;
/*----------------------------------------------------------------------------*/
typedef struct {
    VAL_REGS    dir[15];            /*0x0000 ~ 0x00EF: 240 bytes*/
    u8          rsv00[272];         /*0x00F0 ~ 0x01FF: 272 bytes*/
    VAL_REGS    pullen[15];         /*0x0200 ~ 0x02CF: 240 bytes*/
    u8          rsv01[272];         /*0x02F0 ~ 0x03FF: 272 bytes*/
    VAL_REGS    pullsel[15];        /*0x0400 ~ 0x04CF: 240 bytes*/
    u8          rsv02[272];         /*0x04F0 ~ 0x05FF: 272 bytes*/    
    VAL_REGS    dinv[15];           /*0x0600 ~ 0x06CF: 240 bytes*/    
    u8          rsv03[272];         /*0x06F0 ~ 0x07FF: 272 bytes*/    
    VAL_REGS    dout[15];           /*0x0800 ~ 0x08CF: 240 bytes*/
    u8          rsv04[272];         /*0x08F0 ~ 0x09FF: 272 bytes*/
    VAL_REGS    din[15];            /*0x0A00 ~ 0x0ACF: 240 bytes*/
    u8          rsv05[272];         /*0x0AF0 ~ 0x0BFF: 272 bytes*/
    VAL_REGS    mode[47];           /*0x0C00 ~ 0x0EF0: 752 bytes*/  
} GPIO_REGS;
/*----------------------------------------------------------------------------*/
typedef struct {
    u16 val;        
    u16 set;
    u16 rst;
    u16 _align;
} EXT_VAL_REGS;
/*----------------------------------------------------------------------------*/
typedef struct {
    EXT_VAL_REGS    dir[4];            /*0x0000 ~ 0x001F: 32 bytes*/
    EXT_VAL_REGS    pullen[4];         /*0x0020 ~ 0x003F: 32 bytes*/
    EXT_VAL_REGS    pullsel[4];        /*0x0040 ~ 0x005F: 32 bytes*/
    EXT_VAL_REGS    dinv[4];           /*0x0060 ~ 0x007F: 32 bytes*/    
    EXT_VAL_REGS    dout[4];           /*0x0080 ~ 0x009F: 32 bytes*/
    EXT_VAL_REGS    din[4];            /*0x00A0 ~ 0x00BF: 32 bytes*/
    EXT_VAL_REGS    mode[10];          /*0x00C0 ~ 0x010F: 80 bytes*/  
} GPIOEXT_REGS;
/*----------------------------------------------------------------------------*/
typedef struct {
    unsigned int no     : 16;
    unsigned int mode   : 3;    
    unsigned int pullsel: 1;
    unsigned int din    : 1;
    unsigned int dout   : 1;
    unsigned int pullen : 1;
    unsigned int dir    : 1;
    unsigned int dinv   : 1;
    unsigned int _align : 7; 
} GPIO_CFG; 
/******************************************************************************
* GPIO Driver interface 
******************************************************************************/
/*direction*/
s32 mt_set_gpio_dir(u32 pin, u32 dir);
s32 mt_get_gpio_dir(u32 pin);

/*pull enable*/
s32 mt_set_gpio_pull_enable(u32 pin, u32 enable);
s32 mt_get_gpio_pull_enable(u32 pin);
/*pull select*/
s32 mt_set_gpio_pull_select(u32 pin, u32 select);    
s32 mt_get_gpio_pull_select(u32 pin);

/*data inversion*/
s32 mt_set_gpio_inversion(u32 pin, u32 enable);
s32 mt_get_gpio_inversion(u32 pin);

/*input/output*/
s32 mt_set_gpio_out(u32 pin, u32 output);
s32 mt_get_gpio_out(u32 pin);
s32 mt_get_gpio_in(u32 pin);

/*mode control*/
s32 mt_set_gpio_mode(u32 pin, u32 mode);
s32 mt_get_gpio_mode(u32 pin);

/*clock output setting*/
s32 mt_set_clock_output(u32 num, u32 src, u32 div);
s32 mt_get_clock_output(u32 num, u32 *src, u32 *div);
void mt_gpio_init(void);
