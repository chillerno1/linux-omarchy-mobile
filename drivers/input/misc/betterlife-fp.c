// SPDX-License-Identifier: GPL-2.0-only
/*
 * Betterlife (Blestech) fingerprint sensor platform driver
 *
 * The sensor's SPI bus belongs to the secure world, which reads the
 * images. This driver only handles what Linux owns: the supply, the reset
 * line and the finger-detect interrupt. It keeps the userspace interface
 * the vendor fingerprint library expects: a "blfp" misc device with the
 * vendor's ioctls, interrupt notification over a netlink socket, and an
 * optional input device for navigation gestures.
 */

#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netlink.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <net/net_namespace.h>
#include <net/netlink.h>
#include <net/sock.h>

#define BF_FP_DEV_NAME			"blfp"

/* ioctl interface of the vendor library; numbers and encodings are ABI */
#define BF_IOC_MAGIC			0xFC
#define BF_IOC_INIT_ARGS		_IOWR(BF_IOC_MAGIC, 0, u32)
#define BF_IOC_REGISTER_READ_WRITE	_IOWR(BF_IOC_MAGIC, 1, u32)
#define BF_IOC_RESET			_IO(BF_IOC_MAGIC, 2)
#define BF_IOC_DISABLE_INTERRUPT	_IO(BF_IOC_MAGIC, 3)
#define BF_IOC_GAIN_ADJUST		_IOWR(BF_IOC_MAGIC, 4, u32)
#define BF_IOC_ENABLE_POWER		_IO(BF_IOC_MAGIC, 5)
#define BF_IOC_DISABLE_POWER		_IO(BF_IOC_MAGIC, 6)
#define BF_IOC_ENABLE_SPI_CLOCK		_IOW(BF_IOC_MAGIC, 7, u32)
#define BF_IOC_DISABLE_SPI_CLOCK	_IOW(BF_IOC_MAGIC, 8, u32)
#define BF_IOC_GET_ID			_IOWR(BF_IOC_MAGIC, 9, u32)
#define BF_IOC_INIT_DEVICE		_IOW(BF_IOC_MAGIC, 10, u32)
#define BF_IOC_REMOVE_DEVICE		_IOW(BF_IOC_MAGIC, 11, u32)
#define BF_IOC_INPUT_KEY		_IOW(BF_IOC_MAGIC, 12, u32)
#define BF_IOC_ENBACKLIGHT		_IOW(BF_IOC_MAGIC, 13, u32)
#define BF_IOC_ISBACKLIGHT		_IOWR(BF_IOC_MAGIC, 14, u32)
#define BF_IOC_DISPLAY_STATUS		_IOW(BF_IOC_MAGIC, 15, u32)
#define BF_IOC_SET_PID			_IOW(BF_IOC_MAGIC, 16, u32)
#define BF_IOC_INPUT_KEY_DOWN		_IOW(BF_IOC_MAGIC, 17, u32)
#define BF_IOC_INPUT_KEY_UP		_IOW(BF_IOC_MAGIC, 18, u32)
#define BF_IOC_LOW_RESET		_IO(BF_IOC_MAGIC, 19)
#define BF_IOC_HIGH_RESET		_IO(BF_IOC_MAGIC, 20)
#define BF_IOC_NETLINK_INIT		_IOW(BF_IOC_MAGIC, 21, u32)
#define BF_IOC_TRANS_IC_INFO		_IOW(BF_IOC_MAGIC, 22, u32)
#define BF_IOC_ENABLE_INTERRUPT		_IO(BF_IOC_MAGIC, 23)
#define BF_IOC_RESET_FLAG		_IOW(BF_IOC_MAGIC, 24, u32)
#define BF_IOC_IS_OPT_POWER_ON2V8	_IOWR(BF_IOC_MAGIC, 25, u32)
#define BF_IOC_CREATE_INPUT		_IO(BF_IOC_MAGIC, 26)
#define BF_IOC_COMPATIBLE_IN_HAL	_IOWR(BF_IOC_MAGIC, 27, u32)
#define BF_IOC_POWER_GPIO_STATUS	_IOWR(BF_IOC_MAGIC, 28, u32)
#define BF_IOC_RESET_GPIO_STATUS	_IOWR(BF_IOC_MAGIC, 29, u32)
#define BF_IOC_WAKE_LOCK		_IO(BF_IOC_MAGIC, 30)
#define BF_IOC_WAKE_UNLOCK		_IO(BF_IOC_MAGIC, 31)

/* Netlink messages to the library: a fixed 16-byte payload, command first */
#define BF_NL_DEFAULT_PROTO		26
#define BF_NL_MSG_LEN			16
#define BF_NL_CMD_IRQ			102

/* What the library reports when it has identified the chip */
struct bf_ic_info {
	s32 chip_id;
	char name[128];
	char ca_ver[128];
	char ta_ver[128];
} __aligned(4);

/* INPUT_KEY gestures: click, double click, long press, then swipes */
static const unsigned short bf_fp_keys[] = {
	KEY_F10, KEY_F11, KEY_F12, KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT,
};

static bool report_keys;
module_param(report_keys, bool, 0644);
MODULE_PARM_DESC(report_keys,
		 "Report the library's navigation keys on the input device (default: off)");

struct bf_fp {
	struct device *dev;
	struct gpio_desc *reset;
	int irq;
	struct miscdevice misc;

	atomic_long_t irq_total;

	/* Everything below is protected by @lock */
	struct mutex lock;
	struct sock *nl_sk;
	int nl_proto;
	u32 nl_portid;		/* 0: no listener registered */
	bool armed;		/* forward the next interrupt */
	bool doing_reset;	/* keep forwarding while the library resets */
	bool suspended;
	bool key_down;
	u32 backlight;
	struct input_dev *input;
	struct bf_ic_info ic_info;
	bool ic_info_valid;
	struct file *wake_holder;	/* file that took the wake lock */
};

static void bf_fp_reset_pulse(struct bf_fp *fp)
{
	gpiod_set_value_cansleep(fp->reset, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(fp->reset, 0);
}

static void bf_fp_send_cmd(struct bf_fp *fp, u8 cmd)
{
	struct nlmsghdr *nlh;
	struct sk_buff *skb;
	int ret;

	lockdep_assert_held(&fp->lock);

	if (!fp->nl_sk || !fp->nl_portid)
		return;

	skb = nlmsg_new(BF_NL_MSG_LEN, GFP_KERNEL);
	if (!skb)
		return;

	nlh = nlmsg_put(skb, 0, 0, 0, BF_NL_MSG_LEN, 0);
	if (!nlh) {
		kfree_skb(skb);
		return;
	}
	memset(nlmsg_data(nlh), 0, BF_NL_MSG_LEN);
	*(u8 *)nlmsg_data(nlh) = cmd;

	ret = netlink_unicast(fp->nl_sk, skb, fp->nl_portid, MSG_DONTWAIT);
	if (ret < 0)
		dev_dbg(fp->dev, "netlink send to %u failed: %d\n",
			fp->nl_portid, ret);
}

static irqreturn_t bf_fp_irq(int irq, void *data)
{
	struct bf_fp *fp = data;

	atomic_long_inc(&fp->irq_total);
	pm_wakeup_event(fp->dev, 5000);

	mutex_lock(&fp->lock);
	if (fp->armed) {
		/* One shot, unless the library is in the middle of a reset */
		if (!fp->doing_reset)
			fp->armed = false;
		bf_fp_send_cmd(fp, BF_NL_CMD_IRQ);
	}
	mutex_unlock(&fp->lock);

	return IRQ_HANDLED;
}

/*
 * The library registers itself by sending any message on the socket. It
 * sends that message from a short-lived socket bound to its PID, closes it,
 * then listens on a second socket bound to the same PID. So the port must
 * outlive the socket that registered it: it is kept until the next
 * registration, and a send to a port nobody holds just fails.
 */
static void bf_fp_nl_recv(struct sk_buff *skb)
{
	struct bf_fp *fp;
	struct nlmsghdr *nlh;

	if (skb->len < NLMSG_HDRLEN)
		return;
	nlh = nlmsg_hdr(skb);
	if (nlh->nlmsg_len < NLMSG_HDRLEN || nlh->nlmsg_len > skb->len)
		return;

	rcu_read_lock();
	fp = rcu_dereference_sk_user_data(skb->sk);
	rcu_read_unlock();
	if (!fp)
		return;

	mutex_lock(&fp->lock);
	fp->nl_portid = NETLINK_CB(skb).portid;
	fp->armed = true;
	mutex_unlock(&fp->lock);

	dev_dbg(fp->dev, "netlink listener %u\n", NETLINK_CB(skb).portid);
}

static void bf_fp_nl_close(struct bf_fp *fp)
{
	lockdep_assert_held(&fp->lock);

	if (fp->nl_sk) {
		netlink_kernel_release(fp->nl_sk);
		fp->nl_sk = NULL;
	}
}

static int bf_fp_nl_open(struct bf_fp *fp, unsigned long proto)
{
	struct netlink_kernel_cfg cfg = {
		.input = bf_fp_nl_recv,
	};
	struct sock *sk;

	lockdep_assert_held(&fp->lock);

	if (!proto || proto >= MAX_LINKS)
		return -EINVAL;

	bf_fp_nl_close(fp);
	if (proto != fp->nl_proto)
		fp->nl_portid = 0;

	sk = netlink_kernel_create(&init_net, proto, &cfg);
	if (!sk) {
		dev_err(fp->dev, "cannot create netlink socket %lu\n", proto);
		return -EBUSY;
	}
	rcu_assign_sk_user_data(sk, fp);

	fp->nl_sk = sk;
	fp->nl_proto = proto;

	return 0;
}

static void bf_fp_nl_cleanup(void *data)
{
	struct bf_fp *fp = data;

	mutex_lock(&fp->lock);
	bf_fp_nl_close(fp);
	mutex_unlock(&fp->lock);
}

static int bf_fp_create_input(struct bf_fp *fp)
{
	struct input_dev *input;
	unsigned int i;
	int ret;

	lockdep_assert_held(&fp->lock);

	if (fp->input)
		return 0;

	input = devm_input_allocate_device(fp->dev);
	if (!input)
		return -ENOMEM;

	input->name = "betterlife_inputdev";
	input->id.bustype = BUS_HOST;
	for (i = 0; i < ARRAY_SIZE(bf_fp_keys); i++)
		input_set_capability(input, EV_KEY, bf_fp_keys[i]);

	ret = input_register_device(input);
	if (ret)
		return ret;

	fp->input = input;
	return 0;
}

static bool bf_fp_key_supported(unsigned long code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(bf_fp_keys); i++)
		if (bf_fp_keys[i] == code)
			return true;
	return false;
}

static void bf_fp_report_key(struct bf_fp *fp, unsigned int code, int value)
{
	input_report_key(fp->input, code, value);
	input_sync(fp->input);
}

static int bf_fp_put_u32(unsigned long arg, u32 val)
{
	return put_user(val, (u32 __user *)arg);
}

static long bf_fp_ioctl_locked(struct bf_fp *fp, struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	struct bf_ic_info info;

	switch (cmd) {
	case BF_IOC_RESET:
		bf_fp_reset_pulse(fp);
		return 0;
	case BF_IOC_LOW_RESET:
		gpiod_set_raw_value_cansleep(fp->reset, 0);
		return 0;
	case BF_IOC_HIGH_RESET:
		gpiod_set_raw_value_cansleep(fp->reset, 1);
		return 0;
	case BF_IOC_RESET_FLAG:
		fp->doing_reset = !!(u8)arg;
		return 0;
	case BF_IOC_RESET_GPIO_STATUS:
		return bf_fp_put_u32(arg,
				     gpiod_get_raw_value_cansleep(fp->reset));

	case BF_IOC_ENABLE_INTERRUPT:
		fp->armed = true;
		return 0;
	case BF_IOC_DISABLE_INTERRUPT:
		fp->armed = false;
		return 0;

	case BF_IOC_NETLINK_INIT:
		return bf_fp_nl_open(fp, arg);

	case BF_IOC_TRANS_IC_INFO:
		if (copy_from_user(&info, (void __user *)arg, sizeof(info)))
			return -EFAULT;
		info.name[sizeof(info.name) - 1] = '\0';
		info.ca_ver[sizeof(info.ca_ver) - 1] = '\0';
		info.ta_ver[sizeof(info.ta_ver) - 1] = '\0';
		fp->ic_info = info;
		fp->ic_info_valid = true;
		dev_info(fp->dev, "sensor %s (0x%x), CA %s, TA %s\n",
			 info.name, info.chip_id, info.ca_ver, info.ta_ver);
		return 0;

	case BF_IOC_CREATE_INPUT:
		return bf_fp_create_input(fp);
	case BF_IOC_INPUT_KEY:
		if (!report_keys || !fp->input)
			return 0;
		if (arg < 1 || arg > ARRAY_SIZE(bf_fp_keys))
			return 0;
		bf_fp_report_key(fp, bf_fp_keys[arg - 1], 1);
		bf_fp_report_key(fp, bf_fp_keys[arg - 1], 0);
		return 0;
	case BF_IOC_INPUT_KEY_DOWN:
		if (!report_keys || !fp->input || !bf_fp_key_supported(arg))
			return 0;
		if (fp->backlight && !fp->suspended && !fp->key_down) {
			fp->key_down = true;
			bf_fp_report_key(fp, arg, 1);
		}
		return 0;
	case BF_IOC_INPUT_KEY_UP:
		if (!report_keys || !fp->input || !bf_fp_key_supported(arg))
			return 0;
		if (fp->key_down) {
			fp->key_down = false;
			bf_fp_report_key(fp, arg, 0);
		}
		return 0;
	case BF_IOC_ENBACKLIGHT:
		fp->backlight = arg;
		return 0;
	case BF_IOC_ISBACKLIGHT:
		return bf_fp_put_u32(arg, fp->backlight);

	case BF_IOC_GET_ID:
		/* The vendor driver never identifies the chip itself */
		return bf_fp_put_u32(arg, 0);
	case BF_IOC_COMPATIBLE_IN_HAL:
		return bf_fp_put_u32(arg, 0);
	case BF_IOC_POWER_GPIO_STATUS:
		/* No power GPIO: the supply is a regulator */
		return bf_fp_put_u32(arg, 2);

	case BF_IOC_WAKE_LOCK:
		pm_stay_awake(fp->dev);
		fp->wake_holder = file;
		return 0;
	case BF_IOC_WAKE_UNLOCK:
		pm_relax(fp->dev);
		fp->wake_holder = NULL;
		return 0;

	/*
	 * Power and device setup belong to probe and remove; the library's
	 * requests for them succeed without doing anything.
	 */
	case BF_IOC_INIT_DEVICE:
	case BF_IOC_REMOVE_DEVICE:
	case BF_IOC_ENABLE_POWER:
	case BF_IOC_DISABLE_POWER:
	case BF_IOC_IS_OPT_POWER_ON2V8:
	default:
		/* Unknown commands succeed, as the library expects */
		return 0;
	}
}

static long bf_fp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct bf_fp *fp = container_of(file->private_data, struct bf_fp, misc);
	long ret;

	if (_IOC_TYPE(cmd) != BF_IOC_MAGIC)
		return -ENOTTY;

	dev_dbg(fp->dev, "ioctl %u\n", _IOC_NR(cmd));

	mutex_lock(&fp->lock);
	ret = bf_fp_ioctl_locked(fp, file, cmd, arg);
	mutex_unlock(&fp->lock);

	return ret;
}

/* A library that exits or crashes must not keep the system awake. */
static int bf_fp_release(struct inode *inode, struct file *file)
{
	struct bf_fp *fp = container_of(file->private_data, struct bf_fp, misc);

	mutex_lock(&fp->lock);
	if (fp->wake_holder == file) {
		fp->wake_holder = NULL;
		pm_relax(fp->dev);
	}
	mutex_unlock(&fp->lock);

	return 0;
}

static const struct file_operations bf_fp_fops = {
	.owner = THIS_MODULE,
	.release = bf_fp_release,
	.unlocked_ioctl = bf_fp_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static ssize_t irq_count_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct bf_fp *fp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%lu\n", atomic_long_read(&fp->irq_total));
}
static DEVICE_ATTR_RO(irq_count);

static ssize_t reset_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct bf_fp *fp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", gpiod_get_raw_value_cansleep(fp->reset));
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct bf_fp *fp = dev_get_drvdata(dev);
	bool level;
	int ret;

	ret = kstrtobool(buf, &level);
	if (ret)
		return ret;

	mutex_lock(&fp->lock);
	gpiod_set_raw_value_cansleep(fp->reset, level);
	mutex_unlock(&fp->lock);

	return count;
}
static DEVICE_ATTR_RW(reset);

static ssize_t chipid_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct bf_fp *fp = dev_get_drvdata(dev);
	ssize_t len = 0;

	mutex_lock(&fp->lock);
	if (fp->ic_info_valid)
		len = sysfs_emit(buf, "0x%x\n", fp->ic_info.chip_id);
	mutex_unlock(&fp->lock);

	return len;
}
static DEVICE_ATTR_RO(chipid);

static ssize_t version_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct bf_fp *fp = dev_get_drvdata(dev);
	ssize_t len = 0;

	mutex_lock(&fp->lock);
	if (fp->ic_info_valid)
		len = sysfs_emit(buf, "%s\n", fp->ic_info.ca_ver);
	mutex_unlock(&fp->lock);

	return len;
}
static DEVICE_ATTR_RO(version);

static struct attribute *bf_fp_attrs[] = {
	&dev_attr_irq_count.attr,
	&dev_attr_reset.attr,
	&dev_attr_chipid.attr,
	&dev_attr_version.attr,
	NULL
};
ATTRIBUTE_GROUPS(bf_fp);

static void bf_fp_misc_deregister(void *data)
{
	misc_deregister(data);
}

static int bf_fp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bf_fp *fp;
	int ret;

	fp = devm_kzalloc(dev, sizeof(*fp), GFP_KERNEL);
	if (!fp)
		return -ENOMEM;

	fp->dev = dev;
	fp->backlight = 1;
	fp->nl_proto = BF_NL_DEFAULT_PROTO;
	atomic_long_set(&fp->irq_total, 0);
	ret = devm_mutex_init(dev, &fp->lock);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, fp);

	ret = devm_regulator_get_enable(dev, "vcc");
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable vcc\n");

	/* Released (high), then a 5 ms reset pulse once the supply is up */
	fp->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(fp->reset))
		return dev_err_probe(dev, PTR_ERR(fp->reset),
				     "cannot get reset GPIO\n");
	usleep_range(5000, 6000);
	bf_fp_reset_pulse(fp);

	/* Runs after the IRQ is freed, so the handler never sees a stale socket */
	ret = devm_add_action_or_reset(dev, bf_fp_nl_cleanup, fp);
	if (ret)
		return ret;

	fp->irq = platform_get_irq(pdev, 0);
	if (fp->irq < 0)
		return fp->irq;

	ret = devm_request_threaded_irq(dev, fp->irq, NULL, bf_fp_irq,
					IRQF_ONESHOT, BF_FP_DEV_NAME, fp);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request IRQ\n");

	if (device_property_read_bool(dev, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;
		ret = devm_pm_set_wake_irq(dev, fp->irq);
		if (ret)
			return ret;
	}

	fp->misc.minor = MISC_DYNAMIC_MINOR;
	fp->misc.name = BF_FP_DEV_NAME;
	fp->misc.fops = &bf_fp_fops;
	fp->misc.parent = dev;
	ret = misc_register(&fp->misc);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register misc device\n");

	return devm_add_action_or_reset(dev, bf_fp_misc_deregister, &fp->misc);
}

static int bf_fp_suspend(struct device *dev)
{
	struct bf_fp *fp = dev_get_drvdata(dev);

	mutex_lock(&fp->lock);
	fp->suspended = true;
	mutex_unlock(&fp->lock);

	return 0;
}

static int bf_fp_resume(struct device *dev)
{
	struct bf_fp *fp = dev_get_drvdata(dev);

	mutex_lock(&fp->lock);
	fp->suspended = false;
	mutex_unlock(&fp->lock);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(bf_fp_pm_ops, bf_fp_suspend, bf_fp_resume);

static const struct of_device_id bf_fp_of_match[] = {
	{ .compatible = "betterlife,bf83112" },
	{ }
};
MODULE_DEVICE_TABLE(of, bf_fp_of_match);

static struct platform_driver bf_fp_driver = {
	.probe = bf_fp_probe,
	.driver = {
		.name = "betterlife-fp",
		.of_match_table = bf_fp_of_match,
		.dev_groups = bf_fp_groups,
		.pm = pm_sleep_ptr(&bf_fp_pm_ops),
		/* Open files of the misc device refer to the bound device */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(bf_fp_driver);

MODULE_DESCRIPTION("Betterlife fingerprint sensor platform driver");
MODULE_LICENSE("GPL");
