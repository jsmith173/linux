/*
 * MCP4822 Digital to analog converters  driver
 *
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

enum mcp4822_supported_device_ids {
	ID_MCP4802,
	ID_MCP4812,
	ID_MCP4822,
};

enum mcp4822_current_range {
	MCP4822_CURRENT_RANGE_4mA_20mA,
	MCP4822_CURRENT_RANGE_3mA8_21mA,
	MCP4822_CURRENT_RANGE_3mA2_24mA,
};

/**
 * struct mcp4822_platform_data - MCP4822 DAC driver platform data
 * @external_vref: whether an external reference voltage is used or not
 * @current_range: Current range the MCP4822 is configured for
 */

struct mcp4822_platform_data {
	bool external_vref;
	enum mcp4822_current_range current_range;
};

const int int_vref = 2048; // in milli volts

#define MCP4822_REG_UNDEF		    0x0

#define MCP4822_REG_DAC_DATA		0x1
#define MCP4822_REG_CTRL			0x2
#define MCP4822_REG_OFFSET		    0x3
#define MCP4822_REG_GAIN			0x4
/* load dac and fault shared the same register number. Writing to it will cause
 * a dac load command, reading from it will return the fault status register */
#define MCP4822_REG_LOAD_DAC		0x5
#define MCP4822_REG_FAULT		    0x5
#define MCP4822_REG_FORCE_ALARM_CURRENT	0x6
#define MCP4822_REG_RESET		        0x7
#define MCP4822_REG_START_CONVERSION	0x8
#define MCP4822_REG_NOOP		   	    0x9

struct mcp4822_chip_info {
	unsigned int bit_num;
};

/**
 * struct mcp4822_state - driver instance specific data
 * @spi:		spi_device
 * @ctrl:		control register cache
 * @current_range:	current range which the device is configured for
 * @data:		spi transfer buffers
 * @fault_mask:		software masking of events
 */
struct mcp4822_state {
	struct spi_device		*spi;
	const struct mcp4822_chip_info	*chip_info;
	unsigned int			ctrl;
	enum mcp4822_current_range	current_range;
	unsigned int			fault_mask;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		__be32 d32;
		u8 d8[4];
	} data[2] ____cacheline_aligned;
};

static const struct iio_event_spec mcp4822_current_event[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_event_spec mcp4822_temp_event[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec mcp4822_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_CALIBSCALE) |
			BIT(IIO_CHAN_INFO_CALIBBIAS),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_OFFSET),
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
		},
	},
};

static const struct mcp4822_chip_info mcp4822_chip_infos[] = {
	[ID_MCP4802] = {
		.bit_num = 8,
	},
	[ID_MCP4812] = {
		.bit_num = 10,
	},
	[ID_MCP4822] = {
		.bit_num = 12,
	},
};

static inline int i_pow(int a, int N)
{
 int tmp = 1;
 int i;
 for (i=0; i<N; i++) tmp *= a;
 return tmp;
}

// Set 1.8V output
// IIO="/sys/devices/soc0/amba/e0006000.spi/spi_master/spi1/spi1.0/iio:device1"
// echo 1800 > $IIO/out_voltage0_raw
static int mcp4822_write_unlocked(struct iio_dev *indio_dev,
	unsigned int reg, unsigned int val)
{
	struct mcp4822_state *st = iio_priv(indio_dev);
	int full_scale = i_pow(2, st->chip_info->bit_num);
	
	int vout = val;
	int tmp = vout*full_scale/int_vref;
	
	uint16_t spi_data = tmp;	
	if (spi_data >= full_scale) spi_data = full_scale-1;
	spi_data |= 0x03000;	// A/B=0 (DACA),-=dont care,GA=1,SHDN=1
	dev_notice(&indio_dev->dev, "mcp4822: mcp4822_write_unlocked (1): 0x%04x\n", spi_data);
	//
	
    spi_data = (spi_data << 8) | (spi_data>> 8);	
	dev_notice(&indio_dev->dev, "mcp4822: mcp4822_write_unlocked (2): 0x%04x\n", spi_data);
	
	st->data[0].d8[0] = spi_data & 0xff;
	st->data[0].d8[1] = (spi_data >> 8) & 0xff;
	st->data[0].d8[2] = 0;
	st->data[0].d8[3] = 0;
	
	dev_notice(&indio_dev->dev, "mcp4822: mcp4822_write_unlocked (3): 0x%04x\n", st->data[0].d32);

	return spi_write(st->spi, &st->data[0].d8[0], 4);
}

static int mcp4822_write(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int val)
{
	int ret;

	mutex_lock(&indio_dev->mlock);
	ret = mcp4822_write_unlocked(indio_dev, reg, val);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int mcp4822_read(struct iio_dev *indio_dev, unsigned int reg)
{
	struct mcp4822_state *st = iio_priv(indio_dev);
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[1],
			.len = 3,
			.cs_change = 1,
		}, {
			.rx_buf = &st->data[1].d8[1],
			.len = 3,
		},
	};

	mutex_lock(&indio_dev->mlock);

	st->data[0].d32 = cpu_to_be32((1 << 23) | (reg << 16));

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret >= 0)
		ret = be32_to_cpu(st->data[1].d32) & 0xffff;

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int mcp4822_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long m)
{
	struct mcp4822_state *st = iio_priv(indio_dev);
	unsigned int min, max;
	int ret;

	if (chan->type != IIO_VOLTAGE)
		return -EINVAL;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = mcp4822_read(indio_dev, MCP4822_REG_DAC_DATA);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = mcp4822_read(indio_dev, MCP4822_REG_OFFSET);
		if (ret < 0)
			return ret;
		*val = ret - 32768;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = mcp4822_read(indio_dev, MCP4822_REG_GAIN);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int mcp4822_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	const unsigned int max_val = 1 << 16;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val >= max_val || val < 0)
			return -EINVAL;

		return mcp4822_write(indio_dev, MCP4822_REG_DAC_DATA, val);
	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_info mcp4822_info = {
	.read_raw =		mcp4822_read_raw,
	.write_raw =	mcp4822_write_raw,
};

static int mcp4822_probe(struct spi_device *spi)
{
	enum mcp4822_supported_device_ids type = spi_get_device_id(spi)->driver_data;
	struct mcp4822_platform_data *pdata = dev_get_platdata(&spi->dev);
	struct iio_dev *indio_dev;
	struct mcp4822_state *st;
	int ret;

	dev_notice(&spi->dev, "mcp4822: probe... type: %d\n", type);
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL) {
		dev_err(&spi->dev, "Failed to allocate iio device\n");
		return  -ENOMEM;
	}

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;
	st->chip_info = &mcp4822_chip_infos[type];
	dev_notice(&spi->dev, "mcp4822: probe... bit_num: %d\n", st->chip_info->bit_num);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "mcp4822";
	indio_dev->info = &mcp4822_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = mcp4822_channels;
	indio_dev->num_channels = ARRAY_SIZE(mcp4822_channels);

	dev_notice(&spi->dev, "mcp4822: registering...\n");
	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id mcp4822_id[] = {
	{"mcp4802", ID_MCP4802},
	{"mcp4812", ID_MCP4812},
	{"mcp4822", ID_MCP4822},
	{}
};
MODULE_DEVICE_TABLE(spi, mcp4822_id);

#ifdef CONFIG_OF

static const struct of_device_id mcp4822_dt_ids[] = {
	{ .compatible = "microchip,mcp4802" },
	{ .compatible = "microchip,mcp4812" },
	{ .compatible = "microchip,mcp4822" },
	{},
};
MODULE_DEVICE_TABLE(of, mcp4822_dt_ids);

#endif

static struct spi_driver mcp4822_driver = {
	.driver = {
		   .name = "mcp4822",
		   .of_match_table = mcp4822_dt_ids,
	},
	.probe = mcp4822_probe,
	.id_table = mcp4822_id,
};
module_spi_driver(mcp4822_driver);

MODULE_AUTHOR("user <localhost>");
MODULE_DESCRIPTION("Microchip MCP4822 DAC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:mcp4822");
