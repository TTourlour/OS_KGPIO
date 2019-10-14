/*  Example  kernel module driver 
    by Ludovic Saint-Bauzel (saintbauzel@isir.upmc.fr)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>. 
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <asm/io.h>

#include <asm/uaccess.h>	/* copy_{from,to}_user() */
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/cdev.h>  

#define  DEVICE_NAME "kgpio"    ///< The device will appear at /dev/ebbchar using this value
#define  CLASS_NAME  "kgpio"        ///< The device class -- this is a character device driver
static struct class*  charClass  = NULL; ///< The device-driver class struct pointer
static struct device* charDevice = NULL; ///< The device-driver device struct pointer
int major;


#define PERIOD 1200

#define AM33XX_CONTROL_BASE		0x44e10000
// spruh73m.pdf  Issue de L3 memory et L4 memory p179 p181 p182
#define GPIO0_REGISTER 0x44e07000
#define GPIO1_REGISTER 0x4804C000
#define GPIO2_REGISTER 0x481AC000
#define GPIO3_REGISTER 0x481AE000


// p. 183 
#define PWMSS0_REG 0x48300000
#define PWMSS1_REG 0x48302000
#define PWMSS2_REG 0x48304000

#define ECAP_OFF 0x100
#define EQEP_OFF 0x180
#define EPWM_OFF 0x200



// spruh73m.pdf Issue de register description GPIO p 4881 ...
#define GPIO_OE 0x134
#define GPIO_DATAIN 0x138
#define GPIO_DATAOUT 0x13C
#define GPIO_CTRL 0x130
#define GPIO_CLRDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194


// spruh73m.pdf Issue de p 1369
#define OFFSET_PWMSS_CTRL 0x664


#define OFFSET_PIN9_12 0x878
#define GPIO1_28_PIN9_12 28

#define GPIO_PIN9_12 60

#define OFFSET_PIN_9_14 0x848
#define GPIO1_18_PIN9_14 18

#define GPIO_PIN9_14 50
#define GPIO_PIN8_17 27
#define EPWM_TBCTL 0x00
// Bits in TBCTL 
#define CTRMODE 0x0
#define PHSEN 0x2
#define PRDLD 0x3
#define SYNCOSEL 0x4
#define HSPCLKDIV 0x7
#define CLKDIV 0xA 
// Values in TBCTL
#define TB_UP 0x0
#define TB_UPDOWN 0x2 
#define TB_DISABLE 0x0 
#define TB_SHADOW 0x0
#define TB_SYNC_DISABLE 0x3
#define TB_DIV1 0x0
#define TB_DIV2 0x1
#define TB_DIV4 0x2

#define EPWM_CMPCTL 0x0E
// Bits in CMPCTL
#define SHDWAMODE 0x4
#define SHDWBMODE 0x6
#define LOADAMODE 0x0
#define LOADBMODE 0x2
// Values in CMPCTL
#define CC_SHADOW 0x0
#define CC_CTR_ZERO 0x0

#define EPWM_TBCNT 0x08
#define EPWM_TBPRD 0x0A
#define EPWM_TBPHS 0x06

#define EPWM_CMPA 0x12
#define EPWM_CMPB 0x14

#define EPWM_AQCTLA 0x16
// Bits in AQCTL
#define ZRO 0
#define PRD 2
#define CAU 4
#define CAD 6
#define CBU 8
#define CBD 10
// Values
#define AQ_CLEAR 0x1
#define AQ_SET 0x2



//#define ECAP_OFF 0x100
/* ECAP registers and bits definitions */
#define CAP1			0x08
#define CAP2			0x0C
#define CAP3			0x10
#define CAP4			0x14
#define ECCTL2			0x2A
#define ECCTL2_APWM_POL_LOW     (0x1 << 10)
#define ECCTL2_APWM_MODE        (0x1 << 9)
#define ECCTL2_SYNC_SEL_DISA	((0x1 << 7) |(0x1 << 6))
#define ECCTL2_TSCTR_FREERUN	(0x1 << 4)

#define INTC_MIR_CLEAR2 0xC8
#define INTC_MIR_SET2 0xCC
#define EPWM0INT 86 
#define EPWM1INT 87


#define PWM_SET 0
#define GPIO_SET 1
#define GPIO_CLEAR 4


MODULE_DESCRIPTION("Simple ioctl gpio driver (char)");
MODULE_AUTHOR("Ludovic Saint-Bauzel, ISIR UPMC");
MODULE_LICENSE("GPL");


static long char_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  unsigned short val;
  printk( KERN_DEBUG "char: ioctl() GPIOSET/CLEAR(%ud)\n",cmd);
  
  val=(unsigned short)arg;
  if(val > PERIOD)
    val = PERIOD;
  
  switch (cmd) {
  case PWM_SET :
    break;
  case GPIO_SET:
    gpio_set_value(GPIO_PIN9_14, 1);
    break;
  case GPIO_CLEAR:
    gpio_set_value(GPIO_PIN9_14, 0);
    break;
  default :
    printk(KERN_WARNING "kgpio: %ud unsupported ioctl command\n", cmd);  
    return -EINVAL;
  }
  return 0;
}

static struct file_operations char_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl =	char_ioctl,
};



static int __init pinmuxGPIO(void)
{
 int ret=0;
  int regval ;                 
  void __iomem* cm_per_gpio;
  
  /********************* PINMUX spruh73m.pdf p. 179 - 1370 - 1426 ****************/
  cm_per_gpio = ioremap( /*A REMPLIR*/0x44E10848,4);
  if(!cm_per_gpio) {
    printk("cm_per_gpio_reg ioremap: error\n");
  }
  else {
    regval = ioread32(cm_per_gpio) ; //get GPIO MUX register value
    iowrite32( 0X00000017, cm_per_gpio);// p. 1426 : GPIO MUX to GPIO OUTPUT (Mode 7)
    //                                             Pullup (sel. and enabled)
    regval = ioread32(cm_per_gpio) ; //enabled? PWM GPIO MUX
    printk("GPX1CON register mux (1e) : 0x%08x\n", regval);
    
  }

  ret = gpio_request_one(  /*A REMPLIR*/GPIO_PIN9_14 , GPIOF_OUT_INIT_HIGH,
			 DEVICE_NAME " gpio");
  if (ret < 0) {
    printk(KERN_ALERT DEVICE_NAME " : failed to request GPIO pin %d.\n",
	   GPIO_PIN9_14);
    return ret;
  }

  return ret;
  
}


// this gets called on module init
static int __init kernmodex_init(void)
{
  int ret;
  /* int regval ;                  */
  /* short sregval; */
  /* void __iomem* cm_per_gpio; */
  
  printk(KERN_INFO "Loading example driver by Ludo...\n");

  ret = pinmuxGPIO();
  /*if (ret < 0)
    {
      printk("problem in pinmux of GPIO");
      return ret;
    }
*/

  /************** Char Device creation *****************************/
  /*A REMPLIR register_chrdev*/
  ret = register_chrdev(major,"kgpio",&char_fops);
  if (major == 0)
    major = ret; /* dynamic value */
  
  printk(KERN_INFO "KGPIO: successfully loaded with major %d\n", major);
  
  charClass = class_create(THIS_MODULE, CLASS_NAME);
  if (IS_ERR(charClass)){                // Check for error and clean up if there is
    unregister_chrdev(major, DEVICE_NAME);
    printk(KERN_ALERT "Failed to register device class\n");
    return PTR_ERR(charClass);          // Correct way to return an error on a pointer
  }
  printk(KERN_INFO "KGPIO device class registered correctly\n");
  
  
  // Register the device driver
  
  
  charDevice = device_create(charClass, NULL, MKDEV(major, 0), NULL, DEVICE_NAME );
  if (IS_ERR(charDevice)){               // Clean up if there is an error
    class_destroy(charClass);           // Repeated code but the alternative is goto statements
    unregister_chrdev(major, DEVICE_NAME);
    printk(KERN_ALERT "Failed to create the device\n");
    return PTR_ERR(charDevice);
  }
  printk(KERN_INFO "KGPIO: device class created correctly\n"); // Made it! device was initialized
  
  return 0;
  
}

// this gets called when module is getting unloaded
static void __exit kernmodex_exit(void)
{
  device_destroy(charClass, MKDEV(major, 0));

  class_unregister(charClass);  
  class_destroy(charClass);
  unregister_chrdev(major, CLASS_NAME);

  gpio_free(GPIO_PIN9_14);

  printk(KERN_INFO "Example driver by Ludo removed.\n");
  
  
}

// setting which function to call on module init and exit
module_init(kernmodex_init);
module_exit(kernmodex_exit);


MODULE_VERSION("0.3");
