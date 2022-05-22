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
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#define AP3216_MAJOR 230
static int ap3216_major = AP3216_MAJOR;
module_param(ap3216_major, int, S_IRUGO);
static struct class *ap3216_class = NULL;

struct ap3216_dev {
	struct cdev cdev;
	struct mutex mutex;
	struct device *dev;
	struct i2c_client *client;
	char reg;

};

static int ap3216_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ap3216_open(struct inode *inode, struct file *filp)
{
	struct ap3216_dev *dev = container_of(inode->i_cdev, struct ap3216_dev, cdev);

	filp->private_data = dev;

	i2c_smbus_write_byte_data(dev->client, 0, 0x4);
	/* delay for reset */
	mdelay(20);
	i2c_smbus_write_byte_data(dev->client, 0, 0x3);
	mdelay(250);

	return 0;
}

static ssize_t ap3216_read(struct file *filp, char __user *buf, size_t size, loff_t *off)
{
	int err;
	char kernel_buf[6];
	int val;
	struct ap3216_dev *dev = filp->private_data;
	
	if (size != 6)
		return -EINVAL;

	val = i2c_smbus_read_word_data(dev->client, 0xA); /* read IR */
	kernel_buf[0] = val & 0xff;
	kernel_buf[1] = (val>>8) & 0xff;
	
	val = i2c_smbus_read_word_data(dev->client, 0xC); /* read 光强 */
	kernel_buf[2] = val & 0xff;
	kernel_buf[3] = (val>>8) & 0xff;

	val = i2c_smbus_read_word_data(dev->client, 0xE); /* read 距离 */
	kernel_buf[4] = val & 0xff;
	kernel_buf[5] = (val>>8) & 0xff;
	
	err = copy_to_user(buf, kernel_buf, size);
	return size;

	return 0;
}

static ssize_t ap3216_write(struct file *filp, const char __user *buf, size_t size, loff_t *off)
{

	return 0;
}


static struct file_operations ap3216_fops = {
	.owner = THIS_MODULE,
	.open = ap3216_open,
	.read = ap3216_read,
	.write = ap3216_write,
	.release = ap3216_release,
};

static int ap3216_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -1;
	dev_t devno = 0;
	struct ap3216_dev *ap3216_devp = NULL;

	devno = MKDEV(ap3216_major, 0);

	ap3216_devp = kzalloc(sizeof(struct ap3216_dev), GFP_KERNEL);
	if (!ap3216_devp) {
		ret = -ENOMEM;
		return ret;
	}

	ap3216_devp->client = client;
	ap3216_class = class_create(THIS_MODULE, "ap3216");
	if (IS_ERR(ap3216_class))
		return IS_ERR(ap3216_class);

	if (ap3216_major)
		ret = register_chrdev_region(devno, 1, "ap3216");
	else
		ret = alloc_chrdev_region(&devno, 0, 1, "ap3216");
	if (ret)
		goto err_dev;

	ap3216_major = MAJOR(devno);

	cdev_init(&ap3216_devp->cdev, &ap3216_fops);
	ap3216_devp->cdev.owner = THIS_MODULE;
	ret = cdev_add(&ap3216_devp->cdev, devno, 1);
	if (ret)
		goto err_cdev;

	ap3216_devp->dev = device_create(ap3216_class, NULL, devno, NULL, "ap3216");
	if (IS_ERR(ap3216_devp->dev)) {
		printk("ytest device_create error\n");
		ret = PTR_ERR(ap3216_devp->dev);
		goto err_device;
	}
	i2c_set_clientdata(client, ap3216_devp);
	mutex_init(&ap3216_devp->mutex);

	return 0;

err_device:
	cdev_del(&ap3216_devp->cdev);
err_cdev:
	unregister_chrdev_region(devno, 1);
err_dev:
	class_destroy(ap3216_class);
	kfree(ap3216_devp);

	return ret;
}

static int ap3216_remove(struct i2c_client *client)
{
	struct ap3216_dev *ap3216_devp = i2c_get_clientdata(client);

	device_destroy(ap3216_class, MKDEV(ap3216_major, 0));
	cdev_del(&ap3216_devp->cdev);
	unregister_chrdev_region(MKDEV(ap3216_major, 0), 1);
	class_destroy(ap3216_class);
	kfree(ap3216_devp);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct of_device_id of_match_ids_ap3216[] = {
	{.compatible = "lite-on,ap3216", .data = NULL},
	{}
};

static const struct i2c_device_id ap3216_ids[] = {
	{"ap3216c-1", (kernel_ulong_t)NULL},
	{}
};

static struct i2c_driver i2c_ap3216_driver = {
	.probe = ap3216_probe,
	.remove = ap3216_remove,
	.driver = {
		.name = "ap3216",
		.of_match_table = of_match_ids_ap3216,
	},
	.id_table = ap3216_ids,
};

module_i2c_driver(i2c_ap3216_driver);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Adrian");