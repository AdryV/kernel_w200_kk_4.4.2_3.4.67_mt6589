#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>
#include <mach/mt_gpio.h>

#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/delay.h>
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int disp_bls_set_backlight(unsigned int level);

// Only support 64 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT 64 
// Support 256 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256 

// Configure the support type "BACKLIGHT_LEVEL_PWM_256_SUPPORT" or "BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT" !!
#define BACKLIGHT_LEVEL_PWM_MODE_CONFIG BACKLIGHT_LEVEL_PWM_256_SUPPORT

unsigned int Cust_GetBacklightLevelSupport_byPWM(void)
{
	return BACKLIGHT_LEVEL_PWM_MODE_CONFIG;
}

unsigned int brightness_mapping(unsigned int level)
{
  unsigned int v_level = level;

  if(level>255)
		v_level = 255;

  v_level = 50 - level/6;  

    printk("MYCAT high light level:v_level:[%d,%d]\n",level,v_level);
    return v_level;
}


 
int Cust_SetRedlight(int level)
{
	printk("MYCAT Cust_SetRedlight level=%d\n", level);
	
	mt_set_gpio_mode(126, 0);
	mt_set_gpio_dir(126, 1);
	mt_set_gpio_out(126, level == 0);
	
	return 0;
}

static bool g_backlight_enabled;
unsigned int Cust_SetBacklight(int level, int div)
{
	struct pwm_spec_config pwm_setting;

	unsigned int l_level;

	mt_set_gpio_mode(129, 0);
	mt_set_gpio_dir(129, 1);

  if(level) 
  {

	if ( !g_backlight_enabled )
		g_backlight_enabled = true;

	mt_set_gpio_mode(74, 1);

	if (level >= 255)
		level = 255;

	mt_set_gpio_out(129, 1);

        pwm_setting.pwm_no = PWM2; //PWM2=1;//=1;
		pwm_setting.mode = PWM_MODE_FIFO; //=1;
		pwm_setting.clk_div = div;
        pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK; //=2;
        pwm_setting.pmic_pad = FALSE; //=0; 

		pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
        pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
        pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
        pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
        pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
        pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 4;
        pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 4;

	l_level = level / 6;

	if (50 - l_level <= 32)
	{
		pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = (1 << 50 - l_level) - 1;
		pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0;
	}
	else
	{
		pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0xFFFFFFFF;
		pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = (1 << 18 - l_level) - 1;
	}

	printk("MYCAT level:l_level:div = %d,%d,%d\n", level,l_level, div);

	pwm_set_spec_config(&pwm_setting);
	}
  else
  {
	g_backlight_enabled = false;
	mt_set_gpio_out(129, 0);
	mt_set_gpio_mode(74, 0);
  }
  return 0;
}
 
static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_GPIO, (int)Cust_SetRedlight,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_BUTTON,{0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_LCM, (int)Cust_SetBacklight,{0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

