/*
 * a simple char device driver
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/major.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <asm/io.h>


#define GROUP(x) (x>>16)
#define PIN(x)   (x&0xFFFF)

#define LED_MAJOR 230
static int g_ledcnt = 0;
static int led_major = LED_MAJOR;
module_param(led_major, int, S_IRUGO);

struct led_dev {
	struct cdev cdev;
	struct mutex mutex;
	int pin[8];
};

static struct led_dev *led_devp;
static struct class *led_class;

static volatile unsigned int *CCM_CCGR1                              ;
static volatile unsigned int *IOMUXC_SNVS_SW_MUX_CTL_PAD_SNVS_TAMPER3;
static volatile unsigned int *GPIO5_GDIR                             ;
static volatile unsigned int *GPIO5_DR                               ;

#define LED_MAJOR	230
#define DEVICE_NUM	1

static ssize_t led_read (struct file *filp, char __user *buf, size_t size, loff_t *off)
{
	printk("ytest %s %d\n", __FUNCTION__, __LINE__);

	return 0;
}

static ssize_t led_write (struct file *filp, const char __user *buf, size_t szie, loff_t *off)
{
	int err;
	char status;
	struct inode *inode = file_inode(filp);
	struct led_dev *dev = filp->private_data;
	int minor = iminor(inode);
	
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	err = copy_from_user(&status, buf, 1);

	if (status) /* on: output 0*/
	{
		/* d. 设置GPIO5_DR输出低电平
			* set GPIO5_DR to configure GPIO5_IO03 output 0
			* GPIO5_DR 0x020AC000 + 0
			* bit[3] = 0b0
			*/
		*GPIO5_DR &= ~(1<<PIN(dev->pin[minor]));
	}
	else  /* off: output 1*/
	{
		/* e. 设置GPIO5_IO3输出高电平
			* set GPIO5_DR to configure GPIO5_IO03 output 1
			* GPIO5_DR 0x020AC000 + 0
			* bit[3] = 0b1
			*/ 
		*GPIO5_DR |= (1<<PIN(dev->pin[minor]));
	}

	return 0;
}

static int led_release (struct inode *inode, struct file *filp)
{
	printk("ytest %s %d\n", __FUNCTION__, __LINE__);
	iounmap(CCM_CCGR1);
	CCM_CCGR1 = NULL;
	iounmap(IOMUXC_SNVS_SW_MUX_CTL_PAD_SNVS_TAMPER3);
	IOMUXC_SNVS_SW_MUX_CTL_PAD_SNVS_TAMPER3 = NULL;
	iounmap(GPIO5_GDIR);
	GPIO5_GDIR = NULL;
	iounmap(GPIO5_DR);
	GPIO5_DR = NULL;

	return 0;
}

static int led_open (struct inode *inode, struct file *filp)
{
	unsigned int val;
	int minor = iminor(inode);
	struct led_dev *dev = container_of(inode->i_cdev,
										struct led_dev, cdev);
	filp->private_data = dev;


	printk("init led group %d, pin %d\n", GROUP(dev->pin[minor]), PIN(dev->pin[minor]));

	if (!CCM_CCGR1) {
		CCM_CCGR1								= ioremap(0x20C406C, 4);
		IOMUXC_SNVS_SW_MUX_CTL_PAD_SNVS_TAMPER3 = ioremap(0x2290014, 4);
		GPIO5_GDIR								= ioremap(0x020AC000 + 0x4, 4);
		GPIO5_DR								= ioremap(0x020AC000 + 0, 4);
	}
	/* GPIO5_IO03 */
	/* a. 使能GPIO5
		* set CCM to enable GPIO5
		* CCM_CCGR1[CG15] 0x20C406C
		* bit[31:30] = 0b11
		*/
	*CCM_CCGR1 |= (PIN(dev->pin[minor]) << 30);
	
	/* b. 设置GPIO5_IO03用于GPIO
		* set IOMUXC_SNVS_SW_MUX_CTL_PAD_SNVS_TAMPER3
		*      to configure GPIO5_IO03 as GPIO
		* IOMUXC_SNVS_SW_MUX_CTL_PAD_SNVS_TAMPER3  0x2290014
		* bit[3:0] = 0b0101 alt5
		*/
	val = *IOMUXC_SNVS_SW_MUX_CTL_PAD_SNVS_TAMPER3;
	val &= ~(0xf);
	val |= (GROUP(dev->pin[minor]));
	*IOMUXC_SNVS_SW_MUX_CTL_PAD_SNVS_TAMPER3 = val;

	/* b. 设置GPIO5_IO03作为output引脚
		* set GPIO5_GDIR to configure GPIO5_IO03 as output
		* GPIO5_GDIR  0x020AC000 + 0x4
		* bit[3] = 0b1
		*/
	*GPIO5_GDIR |= (1 << PIN(dev->pin[minor]));

	return 0;
}

static struct file_operations led_fops = {
	.owner = THIS_MODULE,
	.open = led_open,
	.read = led_read,
	.write = led_write,
	.release = led_release,
};

static void led_setup_cdev(struct led_dev *dev, int index)
{
	int err, devno = MKDEV(led_major, index);

	cdev_init(&dev->cdev, &led_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding led%d", err, index);
}

static int led_probe(struct platform_device *pdev)
{
	int ret = 0;
	dev_t devno = 0;

	devno = MKDEV(led_major, 0);
	if (led_major)
		ret = register_chrdev_region(devno, 1, "led");
	else
		ret = alloc_chrdev_region(&devno, 0, 1, "led");


	if (ret < 0)
		return ret;

	led_major = MAJOR(devno);

	if (!led_devp) {
		led_devp = kzalloc(sizeof(struct led_dev), GFP_KERNEL);
		if (!led_devp) {
			ret = -ENOMEM;
			goto fail_malloc;
		}
	}

	if (!led_class) {
		led_class = class_create(THIS_MODULE, "led_class");
		if (IS_ERR(led_class)) {
			printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
			ret = -1;
			goto fail_malloc;
		}
	}

	led_setup_cdev(led_devp, 0);

	device_create(led_class, NULL, MKDEV(led_major, g_ledcnt),
					NULL, "led%d", g_ledcnt);

	of_property_read_s32(pdev->dev.of_node, "pint", &(led_devp->pin[g_ledcnt]));
	g_ledcnt++;
	mutex_init(&(led_devp->mutex));


	return 0;

fail_malloc:
	unregister_chrdev_region(MKDEV(led_major, 0), 1);
	return ret;
}

static int led_remove(struct platform_device *device)
{
	int ret = 0;
	int i = 0;

	for (i = 0; i < g_ledcnt; i++)
		device_destroy(led_class, MKDEV(led_major, i));

	cdev_del(&(led_devp->cdev));
	class_destroy(led_class);
	unregister_chrdev_region(MKDEV(led_major, 0), 1);
	kfree(led_devp);

	return ret;
}

static const struct of_device_id of_match_leds[] = {
	{ .compatible = "adrian,led", .data = NULL},
	{}
};

static struct platform_driver led_drv = {
	.probe = led_probe,
	.remove = led_remove,
	.driver = {
		.name = "adrian_led",
		.of_match_table = of_match_leds,
	}
};

static int led_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&led_drv);

	return ret;
}


static void led_exit(void)
{
	platform_driver_unregister(&led_drv);
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Adrian");
module_init(led_init);
module_exit(led_exit);