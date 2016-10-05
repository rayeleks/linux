/*
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


#define CHAN_COUNT		( 8 )


static int usrm_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct i2c_client **client = iio_priv(indio_dev);
	int ret;
	u8 rxData[CHAN_COUNT * 2];

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = i2c_smbus_read_i2c_block_data(*client, 0, CHAN_COUNT * 2, rxData);
		if (ret < 0)
			return ret;
		if (chan->channel >= 0 && chan->channel < CHAN_COUNT) {
			*val = rxData[chan->channel * 2];				/* LSB */
			*val = (u16)rxData[chan->channel * 2 + 1] << 8;	/* MSB */
		}
		else {
			*val = 0;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 8333; /* = 8.333 * 1000 */
		*val2 = 1000;
		return IIO_VAL_FRACTIONAL;
	default:
		break;
	}

	return -EINVAL;
}

#define USRM_CHAN(_channel)			\
		{							\
			.type = IIO_DISTANCE,	\
			.indexed = 1,			\
			.channel = _channel,	\
			.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),		\
			.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ)	\
		}


static const struct iio_chan_spec usrm_channels[] = {
		USRM_CHAN(0),
		USRM_CHAN(1),
		USRM_CHAN(2),
		USRM_CHAN(3),
		USRM_CHAN(4),
		USRM_CHAN(5),
		USRM_CHAN(6),
		USRM_CHAN(7)
};

static const struct iio_info usrm_info = {
	.read_raw = usrm_read_raw,
	.driver_module = THIS_MODULE,
};

static int usrm_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct i2c_client **data;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	*data = client;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &usrm_info;
	indio_dev->channels = usrm_channels;
	indio_dev->num_channels = ARRAY_SIZE(usrm_channels);

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct i2c_device_id usrm_id[] = {
	{ "usrm_hub", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, usrm_id);

#ifdef CONFIG_OF
static const struct of_device_id usrm_of_id[] = {
		{ .compatible = "luftronix,usrm_hub" },
		{ }
};
MODULE_DEVICE_TABLE(of, usrm_of_id);
#endif

static struct i2c_driver usrm_driver = {
	.driver.name	= "usrm_hub",
	.driver.of_match_table = of_match_ptr(usrm_of_id),
	.probe		= usrm_probe,
	.id_table	= usrm_id,
};

module_i2c_driver(usrm_driver);

MODULE_DESCRIPTION("Luftronix Ultrasonic Ranging Modules Hub");
MODULE_AUTHOR("Volodymyr Yerashok <Volodymyr.Yerashok@eleks.com>");
MODULE_LICENSE("GPL");
