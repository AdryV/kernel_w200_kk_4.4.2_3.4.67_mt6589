#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//RGK add
// ---------------------------------------------------------------------------
#include <cust_adc.h>    	// zhoulidong  add for lcm detect
#define MIN_VOLTAGE (500)     // zhoulidong  add for lcm detect
#define MAX_VOLTAGE (700)     // zhoulidong  add for lcm detect
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define LCM_ID_OTM1283A					0x1283

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

 unsigned static int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util ;

#define SET_RESET_PIN(v)    				(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 					(lcm_util.udelay(n))
#define MDELAY(n) 					(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        	lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE					0

// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);


static LCM_setting_table_V3 lcm_initialization_setting[] = {






	{0x15,0x00,1,{0x00}},
	{0x39,0xff,3,{0x12,0x83,0x01}},	//EXTC=1

	{0x15,0x00,1,{0x80}},	        //Orise mode enable
	{0x39,0xff,2,{0x12,0x83}},
	
	{0x15,0x00,1,{0x92}},	        //MIPI 3 lane setting
	{0x39,0xff,2,{0x20,0x02}},

//-------------------- panel setting --------------------//
	{0x15,0x00,1,{0x80}},             //TCON Setting
	{0x39,0xc0,9,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},

	{0x15,0x00,1,{0x90}},             //Panel Timing Setting
	{0x39,0xc0,6,{0x00,0x5c,0x00,0x01,0x00,0x04}},

	{0x15,0x00,1,{0xa4}},             //source pre. 
	{0x15,0xc0,1,{0x22}},

	{0x15,0x00,1,{0xb3}},             //Interval Scan Frame: 0 frame, column inversion
	{0x39,0xc0,2,{0x00,0x50}},

	{0x15,0x00,1,{0x81}},             //frame rate:60Hz
	{0x15,0xc1,1,{0x55}},

	{0x15,0x00,1,{0x90}},             //clock delay for data latch 
	{0x15,0xc4,1,{0x49}},

//-------------------- power setting --------------------//
	{0x15,0x00,1,{0xa0}},             //dcdc setting
	{0x39,0xc4,14,{0x05,0x10,0x06,0x02,0x05,0x15,0x10,0x05,0x10,0x07,0x02,0x05,0x15,0x10}},

	{0x15,0x00,1,{0xb0}},             //clamp voltage setting
	{0x39,0xc4,2,{0x00,0x00}},

	{0x15,0x00,1,{0x91}},             //VGH=15V, VGL=-10V, pump ratio:VGH=6x, VGL=-5x
	{0x39,0xc5,2,{0x46,0x40}},

	{0x15,0x00,1,{0x00}},             //GVDD=5.008V, NGVDD=-5.008V
	{0x39,0xd8,2,{0x97,0x97}},

	{0x15,0x00,1,{0x00}},             //VCOMDC=-1.584
	{0x15,0xd9,1,{0x7c}},//7d

	//{0x15,0x00,1,{0xB4}},      
	//{0x15,0xc0,1,{0x10}},


	{0x15,0x00,1,{0x81}},             //source bias 0.75uA
	{0x15,0xc4,1,{0x82}},

	{0x15,0x00,1,{0xb0}},             //VDD_18V=1.6V, LVDSVDD=1.55V
	{0x39,0xc5,2,{0x04,0xb8}},

	{0x15,0x00,1,{0xbb}},             //LVD voltage level setting
	{0x15,0xc5,1,{0x80}},

	{0x15,0x00,1,{0x82}},		// chopper 0: frame 2: line 4: disable
	{0x15,0xC4,1,{0x02}}, 

	{0x15,0x00,1,{0xc6}},		// debounce
	{0x15,0xB0,1,{0x03}}, 

//-------------------- control setting --------------------//
	{0x15,0x00,1,{0x00}},             //ID1
	{0x15,0xd0,1,{0x40}},

	{0x15,0x00,1,{0x00}},             //ID2, ID3
	{0x39,0xd1,2,{0x00,0x00}},

//-------------------- panel timing state control --------------------//
	{0x15,0x00,1,{0x80}}, 
	{0x39,0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0x90}}, 
	{0x39,0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0xa0}}, 
	{0x39,0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0xb0}},  
	{0x39,0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0xc0}}, 
	{0x39,0xcb,15,{0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x05,0x05,0x00,0x05,0x05}},

	{0x15,0x00,1,{0xd0}}, 
	{0x39,0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00}},

	{0x15,0x00,1,{0xe0}},
	{0x39,0xcb,14,{0x00,0x00,0x05,0x05,0x00,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0xf0}},
	{0x39,0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},

//-------------------- panel pad mapping control --------------------//
	{0x15,0x00,1,{0x80}}, 
	{0x39,0xcc,15,{0x0E,0x10,0x0A,0x0C,0x02,0x04,0x00,0x00,0x00,0x00,0x2E,0x2D,0x00,0x29,0x2A}},

	{0x15,0x00,1,{0x90}},    
	{0x39,0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x0F,0x09,0x0B,0x01,0x03,0x00,0x00}},

	{0x15,0x00,1,{0xa0}}, 
	{0x39,0xcc,14,{0x00,0x00,0x2E,0x2D,0x00,0x29,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0xb0}}, 
	{0x39,0xcc,15,{0x0B,0x09,0x0F,0x0D,0x03,0x01,0x00,0x00,0x00,0x00,0x2D,0x2E,0x00,0x29,0x2A}},

	{0x15,0x00,1,{0xc0}}, 
	{0x39,0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x0A,0x10,0x0E,0x04,0x02,0x00,0x00}},

	{0x15,0x00,1,{0xd0}},  
	{0x39,0xcc,14,{0x00,0x00,0x2D,0x2E,0x00,0x29,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

//-------------------- panel timing setting --------------------//
	{0x15,0x00,1,{0x80}},             //panel VST setting
	{0x39,0xce,12,{0x8B,0x03,0x18,0x8A,0x03,0x18,0x89,0x03,0x18,0x88,0x03,0x18}},

	{0x15,0x00,1,{0x90}},             //panel VEND setting
	{0x39,0xce,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0xa0}},             //panel CLKA1/2 setting
	{0x39,0xce,14,{0x38,0x07,0x05,0x00,0x00,0x18,0x00,0x38,0x06,0x05,0x01,0x00,0x18,0x00}},

	{0x15,0x00,1,{0xb0}},             //panel CLKA3/4 setting
	{0x39,0xce,14,{0x38,0x05,0x05,0x02,0x00,0x18,0x00,0x38,0x04,0x05,0x03,0x00,0x18,0x00}},

	{0x15,0x00,1,{0xc0}},             //panel CLKb1/2 setting
	{0x39,0xce,14,{0x38,0x03,0x05,0x04,0x00,0x18,0x00,0x38,0x02,0x05,0x05,0x00,0x18,0x00}},

	{0x15,0x00,1,{0xd0}},             //panel CLKb3/4 setting
	{0x39,0xce,14,{0x38,0x01,0x05,0x06,0x00,0x18,0x00,0x38,0x00,0x05,0x07,0x00,0x18,0x00}},

	{0x15,0x00,1,{0x80}},             //panel CLKc1/2 setting
	{0x39,0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0x90}},             //panel CLKc3/4 setting
	{0x39,0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0xa0}},             //panel CLKd1/2 setting
	{0x39,0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0xb0}},             //panel CLKd3/4 setting
	{0x39,0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x15,0x00,1,{0xc0}},             //panel ECLK setting
	{0x39,0xcf,11,{0x3d,0x02,0x15,0x20,0x00,0x00,0x01,0x81,0x00,0x03,0x08}}, //gate pre. ena.

	{0x15,0x00,1,{0xb5}},             //TCON_GOA_OUT Setting
	{0x39,0xc5,6,{0x00,0x6f,0xfF,0x00,0x6f,0xfF}},

//-------------------- for Power IC --------------------//
	{0x15,0x00,1,{0x90}},             //Mode-3
	{0x39,0xf5,4,{0x02,0x11,0x02,0x11}},

	{0x15,0x00,1,{0x90}},             //2xVPNL
	{0x15,0xc5,1,{0xA0}},

	{0x15,0x00,1,{0x94}},             //Freq.
	{0x15,0xc5,1,{0x66}},

//------------------VGLO1/O2 disable----------------
	{0x15,0x00,1,{0xb2}},             //VGLO1
	{0x39,0xf5,2,{0x00,0x00}},

	{0x15,0x00,1,{0xb4}},             //VGLO1_S
	{0x39,0xf5,2,{0x00,0x00}},

	{0x15,0x00,1,{0xb6}},             //VGLO2
	{0x39,0xf5,2,{0x00,0x00}},

	{0x15,0x00,1,{0xb8}},             //VGLO2_S
	{0x39,0xf5,2,{0x00,0x00}},

	{0x15,0x00,1,{0x94}},  		//VCL on  	
	{0x15,0xF5,1,{0x02}},

	{0x15,0x00,1,{0xBA}},  		//VSP on   	
	{0x15,0xF5,1,{0x03}},

	{0x15,0x00,1,{0xb4}},             //VGLO1/2 Pull low setting
	{0x15,0xc5,1,{0xc0}},		//d[7] vglo1 d[6] vglo2 => 0: pull vss, 1: pull vgl

//-------------------- Gamma --------------------//
	{0x15,0x00,1,{0x00}},
	{0x39,0xE1,16,{0x00,0x08,0x0e,0x0d,0x06,0x0e,0x0a,0x0a,0x04,0x07,0x0f,0x08,0x0f,0x13,0x0d,0x06}},

	{0x15,0x00,1,{0x00}},
	{0x39,0xE2,16,{0x00,0x08,0x0e,0x0d,0x06,0x0e,0x0a,0x0a,0x04,0x07,0x0f,0x08,0x0f,0x13,0x0d,0x06}},

	{0x15, 0x00,1,{0xa0}},             ////////////////////////////////////////////////////////////////		
        {0x15, 0xC1,1,{0x02}}, 
	
	{0x15,0x00,1,{0x00}},             //Orise mode disable
	{0x39,0xff,3,{0xff,0xff,0xff}},
		 

	{0x05,0x11,0,{}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},
	{0x05, 0x29,0,{}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

};



// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));

		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
		#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
		#endif

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_THREE_LANE;

		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 6;  //4
		params->dsi.vertical_backporch					= 14; //16
		params->dsi.vertical_frontporch					= 20; //15
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 10; //6
		params->dsi.horizontal_backporch				= 37; //37
		params->dsi.horizontal_frontporch				= 37; //37
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		//params->dsi.LPX=8; 

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_select=1;
		//params->dsi.PLL_CLOCK  = LCM_DSI_6589_PLL_CLOCK_377;
		params->dsi.PLL_CLOCK=255;
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
		#if (LCM_DSI_CMD_MODE)
		params->dsi.fbk_div =7;
		#else
		params->dsi.fbk_div =7;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		#endif
		//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's

}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);

}


static LCM_setting_table_V3  lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x05, 0x28, 0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

    // Sleep Mode On
	{0x05, 0x10, 0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},
};
static void lcm_suspend(void)
{
	dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting)/sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);

	SET_RESET_PIN(1);	
	SET_RESET_PIN(0);
	MDELAY(20); // 1ms

	SET_RESET_PIN(1);
	MDELAY(120);
}


static void lcm_resume(void)
{
	
	lcm_init();

}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif





// zhoulidong  add for lcm detect (start)
static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);	

	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;
	
	#ifdef BUILD_LK
		printf("OTM1283A uboot %s \n", __func__);
	       printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("OTM1283A kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif
	   
	return (LCM_ID_OTM1283A == id)?1:0;
}

// zhoulidong  add for lcm detect (start)
static unsigned int rgk_lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    { 
	#ifdef BUILD_LK
	printf("[adc_uboot]: get data error\n");
	#endif
	return 0;
		   
    }
#endif

    lcm_vol = data[0]*1000+data[1]*10;

    #ifdef BUILD_LK
    printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
	
    #endif

    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE)
    {
	return 1;
    }

    return 0;
	
}



// zhoulidong add for eds(start)
static unsigned int lcm_esd_check(void)
{
	#ifdef BUILD_LK
		//printf("lcm_esd_check()\n");
	#else
		//printk("lcm_esd_check()\n");
	#endif 
 #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	if(buffer[0]==0x9c)
	{
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif
		return FALSE;
	}
	else
	{	
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif		 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	
	#ifndef BUILD_LK
	printk("lcm_esd_recover  otm1283a_dsi_vdo_djn_hd \n");
	#endif
	return TRUE;
}

LCM_DRIVER otm1283a_dsi_vdo_djn_hd_lcm_drv = 
{
    .name			= "otm1283a_dsi_vdo_djn_hd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = rgk_lcm_compare_id,	
//	.esd_check = lcm_esd_check,
//	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};

