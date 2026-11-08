// SPDX-License-Identifier: GPL-2.0
/*
 * Temporary camera bring-up probe: power one sensor, run MCLK, release reset,
 * scan the CCI bus and read chip-id registers. Re-run via sysfs "run".
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

static unsigned int mclk_hz = 24000000;
static unsigned int rst_delay_ms = 20;
static bool rst_first;
static bool scan = true;
module_param(mclk_hz, uint, 0644);
module_param(rst_delay_ms, uint, 0644);
module_param(rst_first, bool, 0644);
module_param(scan, bool, 0644);

struct cb {
	struct device *dev;
	struct i2c_adapter *adap;
	struct clk *clk;
	struct regulator *vio, *vana, *vdig;
	struct gpio_desc *rst, *avdd;
	struct delayed_work work;
	struct mutex lock;
	bool vio_on, vana_on, vdig_on, clk_on;
};

static int cb_read16(struct i2c_adapter *a, u16 addr, u16 reg, u16 *val)
{
	u8 w[2] = { reg >> 8, reg & 0xff }, r[2] = { };
	struct i2c_msg m[2] = {
		{ .addr = addr, .len = 2, .buf = w },
		{ .addr = addr, .flags = I2C_M_RD, .len = 2, .buf = r },
	};
	int ret = i2c_transfer(a, m, 2);

	if (ret != 2)
		return ret < 0 ? ret : -EIO;
	*val = r[0] << 8 | r[1];
	return 0;
}

static void cb_ids(struct cb *c, u16 addr)
{
	static const u16 regs[] = { 0x0000, 0x0002, 0x0016, 0x0018 };
	u16 v;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = cb_read16(c->adap, addr, regs[i], &v);
		if (ret)
			dev_info(c->dev, "cambring: 0x%02x reg 0x%04x -> err %d\n",
				 addr, regs[i], ret);
		else
			dev_info(c->dev, "cambring: 0x%02x reg 0x%04x -> 0x%04x\n",
				 addr, regs[i], v);
	}
}

static void cb_power(struct cb *c)
{
	struct device *d = c->dev;
	int ret;

	gpiod_set_value_cansleep(c->rst, 0);
	if (c->clk_on) {
		clk_disable_unprepare(c->clk);
		c->clk_on = false;
	}
	msleep(5);

	ret = regulator_set_voltage(c->vana, 2800000, 2800000);
	dev_info(d, "cambring: set vana 2.8V -> %d\n", ret);
	ret = regulator_set_voltage(c->vdig, 1050000, 1050000);
	dev_info(d, "cambring: set vdig 1.05V -> %d\n", ret);

	if (!c->vio_on) {
		ret = regulator_enable(c->vio);
		c->vio_on = !ret;
		dev_info(d, "cambring: vio on -> %d\n", ret);
		usleep_range(1000, 2000);
	}
	if (!c->vana_on) {
		ret = regulator_enable(c->vana);
		c->vana_on = !ret;
		dev_info(d, "cambring: vana on -> %d\n", ret);
		usleep_range(1000, 2000);
	}
	if (!c->vdig_on) {
		ret = regulator_enable(c->vdig);
		c->vdig_on = !ret;
		dev_info(d, "cambring: vdig on -> %d\n", ret);
		usleep_range(1000, 2000);
	}
	gpiod_set_value_cansleep(c->avdd, 1);
	msleep(2);

	if (rst_first) {
		gpiod_set_value_cansleep(c->rst, 1);
		msleep(rst_delay_ms);
	}

	ret = clk_set_rate(c->clk, mclk_hz);
	dev_info(d, "cambring: mclk set %u -> %d, now %lu\n", mclk_hz, ret,
		 clk_get_rate(c->clk));
	ret = clk_prepare_enable(c->clk);
	c->clk_on = !ret;
	dev_info(d, "cambring: mclk enable -> %d\n", ret);
	msleep(5);

	if (!rst_first) {
		gpiod_set_value_cansleep(c->rst, 1);
		msleep(rst_delay_ms);
	}
}

static void cb_run(struct work_struct *w)
{
	struct cb *c = container_of(to_delayed_work(w), struct cb, work);
	struct i2c_adapter *a = c->adap;
	u8 b;
	int addr, ret, n = 0;
	struct i2c_msg m = { .flags = I2C_M_RD, .len = 1, .buf = &b };

	mutex_lock(&c->lock);
	dev_info(c->dev, "cambring: start, adapter %s (i2c-%d)\n", a->name, a->nr);
	cb_power(c);

	cb_ids(c, 0x1a);

	if (scan) {
		for (addr = 0x08; addr <= 0x77; addr++) {
			m.addr = addr;
			ret = i2c_transfer(a, &m, 1);
			if (ret == 1) {
				n++;
				dev_info(c->dev, "cambring: ACK at 0x%02x (byte 0x%02x)\n",
					 addr, b);
				if (addr != 0x1a)
					cb_ids(c, addr);
			}
		}
		dev_info(c->dev, "cambring: scan done, %d device(s) ACKed\n", n);
	}
	mutex_unlock(&c->lock);
}

static ssize_t run_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t len)
{
	struct cb *c = dev_get_drvdata(dev);

	schedule_delayed_work(&c->work, 0);
	return len;
}
static DEVICE_ATTR_WO(run);

static struct attribute *cb_attrs[] = { &dev_attr_run.attr, NULL };
ATTRIBUTE_GROUPS(cb);

static int cb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct cb *c;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->dev = dev;
	mutex_init(&c->lock);

	np = of_parse_phandle(dev->of_node, "i2c-bus", 0);
	if (!np)
		return -EINVAL;
	c->adap = of_find_i2c_adapter_by_node(np);
	of_node_put(np);
	if (!c->adap)
		return -EPROBE_DEFER;

	c->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(c->clk))
		return dev_err_probe(dev, PTR_ERR(c->clk), "clk\n");
	c->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(c->vio))
		return dev_err_probe(dev, PTR_ERR(c->vio), "vio\n");
	c->vana = devm_regulator_get(dev, "vana");
	if (IS_ERR(c->vana))
		return dev_err_probe(dev, PTR_ERR(c->vana), "vana\n");
	c->vdig = devm_regulator_get(dev, "vdig");
	if (IS_ERR(c->vdig))
		return dev_err_probe(dev, PTR_ERR(c->vdig), "vdig\n");
	c->rst = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(c->rst))
		return dev_err_probe(dev, PTR_ERR(c->rst), "reset\n");
	c->avdd = devm_gpiod_get(dev, "avdd", GPIOD_OUT_LOW);
	if (IS_ERR(c->avdd))
		return dev_err_probe(dev, PTR_ERR(c->avdd), "avdd\n");

	INIT_DELAYED_WORK(&c->work, cb_run);
	platform_set_drvdata(pdev, c);
	dev_info(dev, "cambring: ready, running in 10s (echo 1 > run to repeat)\n");
	schedule_delayed_work(&c->work, 10 * HZ);
	return 0;
}

static const struct of_device_id cb_of_match[] = {
	{ .compatible = "wojtekojtek,cam-bringup" },
	{ }
};

static struct platform_driver cb_driver = {
	.probe = cb_probe,
	.driver = {
		.name = "cam-bringup",
		.of_match_table = cb_of_match,
		.dev_groups = cb_groups,
	},
};
builtin_platform_driver(cb_driver);
