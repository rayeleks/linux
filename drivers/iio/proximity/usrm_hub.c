/*
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>


#define USRM_DRIVER_NAME	"usrm_hub"
#define USRM_IRQ_NAME		"usrm_irq"

#define CHAN_COUNT		( 8 )


struct usrm_data {
	struct i2c_client *client;
	struct iio_trigger *trig;
	bool trigger_enabled;
};


static int usrm_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct usrm_data *data = iio_priv(indio_dev);
	u8 rxData[CHAN_COUNT * 2];
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = i2c_smbus_read_i2c_block_data(data->client, 0, CHAN_COUNT * 2, rxData);
		if (ret < 0)
			return ret;
		if (chan->channel >= 0 && chan->channel < CHAN_COUNT) {
			*val = rxData[chan->channel * 2];					/* LSB */
			*val += (u16)rxData[chan->channel * 2 + 1] << 8;	/* MSB */
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

static irqreturn_t usrm_irq_handler(int irq, void *private)
{

	struct iio_dev *indio_dev = private;
	struct usrm_data *data = iio_priv(indio_dev);

	if (data->trigger_enabled) {
		iio_trigger_poll(data->trig);
	}

	return IRQ_HANDLED;
}

static irqreturn_t usrm_trigger_handler(int irq, void *private)
{
	struct iio_poll_func *pf = private;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct usrm_data *data = iio_priv(indio_dev);
	u8 rxData[CHAN_COUNT * 2];
	u8 buffer[CHAN_COUNT * 2 + sizeof(s64)];	// Data plus timestamp.
	int bit, ret, i = 0;

//	mutex_lock(&data->mutex);

	dev_dbg(&indio_dev->dev, "active_scan_mask = %lu, masklength = %u\n", *indio_dev->active_scan_mask, indio_dev->masklength);

	ret = i2c_smbus_read_i2c_block_data(data->client, 0, CHAN_COUNT * 2, rxData);
	if (ret >= 0) {

		for_each_set_bit(bit, indio_dev->active_scan_mask, indio_dev->masklength) {
			// Fill data buffer with active scan mask channels.
			buffer[i++] = rxData[bit * 2];
			buffer[i++] = rxData[bit * 2 + 1];
		}

		iio_push_to_buffers_with_timestamp(indio_dev, buffer, pf->timestamp);
	}

//	mutex_unlock(&data->mutex);

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int usrm_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct usrm_data *data = iio_priv(indio_dev);

	data->trigger_enabled = state;

	return 0;
}

#define USRM_CHAN(_channel, _si)	\
		{							\
			.type = IIO_DISTANCE,	\
			.indexed = 1,			\
			.channel = _channel,	\
			.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),			\
			.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
			.scan_index = _si,			\
			.scan_type = {					\
					.sign = 'u',			\
					.realbits = 16,			\
					.storagebits = 16,		\
					.shift = 0,				\
					.endianness = IIO_LE,	\
			},								\
		}


static const struct iio_chan_spec usrm_channels[] = {
		IIO_CHAN_SOFT_TIMESTAMP(0),
		USRM_CHAN(0, 1),
		USRM_CHAN(1, 2),
		USRM_CHAN(2, 3),
		USRM_CHAN(3, 4),
		USRM_CHAN(4, 5),
		USRM_CHAN(5, 6),
		USRM_CHAN(6, 7),
		USRM_CHAN(7, 8)
};

static const struct iio_info usrm_info = {
	.read_raw = usrm_read_raw,
	.driver_module = THIS_MODULE,
};

static const struct iio_trigger_ops usrm_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &usrm_set_trigger_state,	
};

static int usrm_probe(
		struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct usrm_data *data;
	int result;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "required i2c bus functionality not supported\n");
		return -ENOSYS;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev) {
		dev_err(&client->dev, "devm_iio_device_alloc() failed\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	data->client = client;
	data->trigger_enabled = false;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = USRM_DRIVER_NAME;
	indio_dev->modes = INDIO_BUFFER_TRIGGERED;
	indio_dev->info = &usrm_info;
	indio_dev->channels = usrm_channels;
	indio_dev->num_channels = ARRAY_SIZE(usrm_channels);
	i2c_set_clientdata(client, indio_dev);

	if (client->irq <= 0)
		dev_warn(&client->dev, "no valid irq found\n");
	else {
		dev_dbg(&client->dev, "client irq = %d\n", client->irq);
		result = devm_request_irq(&client->dev, client->irq, usrm_irq_handler,
				0, USRM_IRQ_NAME, indio_dev);
		if (result < 0) {
			dev_err(&client->dev, "devm_request_irq() error %d\n", result);
			return result;
		}

		data->trig = devm_iio_trigger_alloc(&client->dev, "%s-dev%d",
				indio_dev->name, indio_dev->id);
		if (!data->trig) {
			dev_err(&client->dev, "devm_iio_trigger_alloc() failed\n");
			return -ENOMEM;
		}

		data->trig->dev.parent = &client->dev;
		data->trig->ops =&usrm_trigger_ops;
		iio_trigger_set_drvdata(data->trig, indio_dev);

		result = iio_trigger_register(data->trig);
		if (result) {
			dev_err(&client->dev, "iio_trigger_register() error %d\n", result);
			return result;
		}
	}

	result = iio_triggered_buffer_setup(indio_dev, iio_pollfunc_store_time,
			usrm_trigger_handler, NULL);

	if (result == 0) {

		result = iio_device_register(indio_dev);
		if (result == 0) {
			return 0;
		}
		else {
			dev_err(&client->dev, "iio_device_register() error %d\n", result);
		}

		iio_triggered_buffer_cleanup(indio_dev);

	}
	else {
		dev_err(&client->dev, "iio_triggered_buffer_setup() error %d\n", result);
	}

	if (client->irq > 0)
		iio_trigger_unregister(data->trig);

	return result;
}

static int usrm_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct usrm_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	if (client->irq > 0) {
		iio_trigger_unregister(data->trig);
	}

	return 0;
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
	.remove		= usrm_remove,
	.id_table	= usrm_id,
};

module_i2c_driver(usrm_driver);

MODULE_DESCRIPTION("Luftronix Ultrasonic Ranging Modules Hub");
MODULE_AUTHOR("Volodymyr Yerashok <Volodymyr.Yerashok@eleks.com>");
MODULE_LICENSE("GPL");
