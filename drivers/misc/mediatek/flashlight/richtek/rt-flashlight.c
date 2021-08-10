/*
 *  Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>
#include "rt-flashlight.h"

static const char * const flashlight_type_string[] = {
	[FLASHLIGHT_TYPE_XENON] = "Xenon",
	[FLASHLIGHT_TYPE_LED] = "LED",
	[FLASHLIFHT_TYPE_BULB] = "Bulb",
};

static const char * const flashlight_mode_string[] = {
	[FLASHLIGHT_MODE_OFF] = "Off",
	[FLASHLIGHT_MODE_TORCH] = "Torch",
	[FLASHLIGHT_MODE_FLASH] = "Flash",
	[FLASHLIGHT_MODE_MIXED] = "Mixed",
	[FLASHLIGHT_MODE_DUAL_FLASH] = "Flash",
	[FLASHLIGHT_MODE_DUAL_TORCH] = "Torch",
	[FLASHLIGHT_MODE_DUAL_OFF] = "Off",
};

static ssize_t flashlight_show_name(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
		       flashlight_dev->props.alias_name ?
		       flashlight_dev->props.alias_name : "anonymous");
}

static ssize_t flashlight_show_type(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
		       flashlight_type_string[flashlight_dev->props.type]);
}

static ssize_t flashlight_show_mode(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
		       flashlight_mode_string[flashlight_dev->props.mode]);
}

static ssize_t flashlight_show_torch_max_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		       flashlight_dev->props.torch_max_brightness);
}

static ssize_t flashlight_show_strobe_max_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		       flashlight_dev->props.strobe_max_brightness);
}

static ssize_t flashlight_show_color_temperature(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		       flashlight_dev->props.color_temperature);
}

static ssize_t flashlight_show_strobe_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		       flashlight_dev->props.strobe_delay);
}

static ssize_t flashlight_store_strobe_timeout(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);
	long timeout;

	rc = kstrtol(buf, 0, &timeout);
	if (rc)
		return rc;
	rc =  flashlight_dev->ops->set_strobe_timeout(flashlight_dev, timeout);
	if (rc == 0)
		rc = count;
	return rc;
}

static ssize_t flashlight_show_strobe_timeout(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		       flashlight_dev->props.strobe_timeout);
}


static ssize_t flashlight_store_torch_brightness(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);
	long brightness;

	rc = kstrtol(buf, 0, &brightness);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock(&flashlight_dev->ops_lock);
	if (flashlight_dev->ops &&
	    flashlight_dev->ops->set_torch_brightness) {
		if (brightness > flashlight_dev->props.torch_max_brightness)
			rc = -EINVAL;
		else {
			pr_debug("flashlight: set torch brightness to %ld\n",
				 brightness);
			flashlight_dev->props.torch_brightness = brightness;
			flashlight_dev->ops->set_torch_brightness(
				flashlight_dev, brightness);
			rc = count;
		}
	}
	mutex_unlock(&flashlight_dev->ops_lock);

	return rc;
}

static ssize_t flashlight_show_torch_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		       flashlight_dev->props.torch_brightness);
}

static ssize_t flashlight_store_strobe_brightness(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);
	long brightness;

	rc = kstrtol(buf, 0, &brightness);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock(&flashlight_dev->ops_lock);
	if (flashlight_dev->ops &&
	    flashlight_dev->ops->set_strobe_brightness) {
		if (brightness > flashlight_dev->props.strobe_max_brightness)
			rc = -EINVAL;
		else {
			pr_debug("flashlight: set strobe brightness to %ld\n",
				 brightness);
			flashlight_dev->props.strobe_brightness = brightness;
			flashlight_dev->ops->set_strobe_brightness(
				flashlight_dev, brightness);
			rc = count;
		}
	}
	mutex_unlock(&flashlight_dev->ops_lock);
	return rc;
}

static ssize_t flashlight_show_strobe_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		       flashlight_dev->props.strobe_brightness);
}



static struct class *flashlight_class;

static int flashlight_suspend(struct device *dev, pm_message_t state)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	if (flashlight_dev->ops)
		flashlight_dev->ops->suspend(flashlight_dev, state);
	return 0;
}

static int flashlight_resume(struct device *dev)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	if (flashlight_dev->ops)
		flashlight_dev->ops->resume(flashlight_dev);
	return 0;
}

static void flashlight_device_release(struct device *dev)
{
	struct flashlight_device *flashlight_dev = to_flashlight_device(dev);

	kfree(flashlight_dev);
}


static DEVICE_ATTR(name, 0444, flashlight_show_name, NULL);
static DEVICE_ATTR(type, 0444, flashlight_show_type, NULL);
static DEVICE_ATTR(mode, 0444, flashlight_show_mode, NULL);
static DEVICE_ATTR(torch_max_brightness, 0444,
	flashlight_show_torch_max_brightness, NULL);
static DEVICE_ATTR(strobe_max_brightness, 0444,
	flashlight_show_strobe_max_brightness, NULL);
static DEVICE_ATTR(color_temperature, 0444,
	flashlight_show_color_temperature, NULL);
static DEVICE_ATTR(strobe_delay, 0664,
	flashlight_show_strobe_delay, NULL);
static DEVICE_ATTR(strobe_timeout, 0664,
	flashlight_show_strobe_timeout,
	flashlight_store_strobe_timeout);
static DEVICE_ATTR(torch_brightness, 0664,
	flashlight_show_torch_brightness,
	flashlight_store_torch_brightness);
static DEVICE_ATTR(strobe_brightness, 0664,
	flashlight_show_strobe_brightness,
	flashlight_store_strobe_brightness);

static struct attribute *flashlight_class_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_type.attr,
	&dev_attr_mode.attr,
	&dev_attr_torch_max_brightness.attr,
	&dev_attr_strobe_max_brightness.attr,
	&dev_attr_color_temperature.attr,
	&dev_attr_strobe_delay.attr,
	&dev_attr_strobe_timeout.attr,
	&dev_attr_torch_brightness.attr,
	&dev_attr_strobe_brightness.attr,
	NULL,
};

static const struct attribute_group flashlight_group = {
	.attrs = flashlight_class_attrs,
};

static const struct attribute_group *flashlight_groups[] = {
	&flashlight_group,
	NULL,
};



/**
 * flashlight_device_register - create and register a new object of
 *   flashlight_device class.
 * @name: the name of the new object(must be the same as the name of the
 *   respective framebuffer device).
 * @parent: a pointer to the parent device
 * @devdata: an optional pointer to be stored for private driver use. The
 *   methods may retrieve it by using flashlight_get_data(flashlight_dev).
 * @ops: the flashlight operations structure.
 *
 * Creates and registers new flashlight device. Returns either an
 * ERR_PTR() or a pointer to the newly allocated device.
 */
struct flashlight_device *flashlight_device_register(const char *name,
		struct device *parent, void *devdata,
		const struct flashlight_ops *ops,
		const struct flashlight_properties *props)
{
	struct flashlight_device *flashlight_dev;
	int rc;

	pr_debug("%s: name=%s\n", __func__, name);
	flashlight_dev = kzalloc(sizeof(*flashlight_dev), GFP_KERNEL);
	if (!flashlight_dev)
		return ERR_PTR(-ENOMEM);

	mutex_init(&flashlight_dev->ops_lock);
	flashlight_dev->dev.class = flashlight_class;
	flashlight_dev->dev.parent = parent;
	flashlight_dev->dev.release = flashlight_device_release;
	dev_set_name(&flashlight_dev->dev, name);
	dev_set_drvdata(&flashlight_dev->dev, devdata);
	/* Copy properties */
	if (props) {
		memcpy(&flashlight_dev->props, props,
		       sizeof(struct flashlight_properties));
	}
	rc = device_register(&flashlight_dev->dev);
	if (rc) {
		kfree(flashlight_dev);
		return ERR_PTR(rc);
	}
	flashlight_dev->ops = ops;
	return flashlight_dev;
}
EXPORT_SYMBOL(flashlight_device_register);

/**
 * flashlight_device_unregister - unregisters a flashlight device object.
 * @flashlight_dev: the flashlight device object to be unregistered and freed.
 *
 * Unregisters a previously registered via flashlight_device_register object.
 */
void flashlight_device_unregister(struct flashlight_device *flashlight_dev)
{
	if (!flashlight_dev)
		return;

	mutex_lock(&flashlight_dev->ops_lock);
	flashlight_dev->ops = NULL;
	mutex_unlock(&flashlight_dev->ops_lock);
	device_unregister(&flashlight_dev->dev);
}
EXPORT_SYMBOL(flashlight_device_unregister);

int flashlight_list_color_temperature(
	struct flashlight_device *flashlight_dev,
	int selector)
{
	if (flashlight_dev->ops &&
	    flashlight_dev->ops->list_color_temperature) {
		return flashlight_dev->ops->list_color_temperature(
			       flashlight_dev,
			       selector);
	}
	return -EINVAL;
}
EXPORT_SYMBOL(flashlight_list_color_temperature);

int flashlight_set_color_temperature(
	struct flashlight_device *flashlight_dev,
	int minK, int maxK)
{
	int selector = 0;
	int rc;

	if ((flashlight_dev->ops ==  NULL) ||
	    (flashlight_dev->ops->set_color_temperature == NULL))
		return -EINVAL;
	for (selector = 0; ; selector++) {
		rc = flashlight_list_color_temperature(flashlight_dev,
								selector);
		if (rc < 0)
			return rc;
		if (rc >= minK && rc <= maxK) {
			mutex_lock(&flashlight_dev->ops_lock);
			rc = flashlight_dev->ops->set_color_temperature(
				     flashlight_dev, rc);
			mutex_unlock(&flashlight_dev->ops_lock);
			if (rc == 0)
				flashlight_dev->props.color_temperature = rc;
			return rc;
		}

	}
	return -EINVAL;
}
EXPORT_SYMBOL(flashlight_set_color_temperature);

int flashlight_set_torch_brightness(
	struct flashlight_device *flashlight_dev,
	int brightness_level)
{
	int rc;

	if ((flashlight_dev->ops ==  NULL) ||
	    (flashlight_dev->ops->set_torch_brightness == NULL))
		return -EINVAL;
	if (brightness_level > flashlight_dev->props.torch_max_brightness)
		return -EINVAL;
	mutex_lock(&flashlight_dev->ops_lock);
	rc = flashlight_dev->ops->set_torch_brightness(flashlight_dev,
			brightness_level);
	mutex_unlock(&flashlight_dev->ops_lock);
	if (rc < 0)
		return rc;
	flashlight_dev->props.torch_brightness = brightness_level;
	return rc;

}
EXPORT_SYMBOL(flashlight_set_torch_brightness);

int flashlight_set_strobe_brightness(
	struct flashlight_device *flashlight_dev,
	int brightness_level)
{
	int rc;

	if ((flashlight_dev->ops ==  NULL) ||
	    (flashlight_dev->ops->set_strobe_brightness == NULL))
		return -EINVAL;
	if (brightness_level > flashlight_dev->props.strobe_max_brightness)
		return -EINVAL;
	mutex_lock(&flashlight_dev->ops_lock);
	rc = flashlight_dev->ops->set_strobe_brightness(flashlight_dev,
			brightness_level);
	mutex_unlock(&flashlight_dev->ops_lock);
	if (rc < 0)
		return rc;
	flashlight_dev->props.strobe_brightness = brightness_level;
	return rc;
}
EXPORT_SYMBOL(flashlight_set_strobe_brightness);

int flashlight_list_strobe_timeout(
	struct flashlight_device *flashlight_dev,
	int selector)
{
	if (flashlight_dev->ops &&
	    flashlight_dev->ops->list_strobe_timeout) {
		return flashlight_dev->ops->list_strobe_timeout(flashlight_dev,
				selector);
	}
	return -EINVAL;
}
EXPORT_SYMBOL(flashlight_list_strobe_timeout);

int flashlight_set_strobe_timeout(
	struct flashlight_device *flashlight_dev,
	int min_ms, int max_ms)
{
	int selector = 0;
	int rc = -EINVAL;
	int timeout;

	if ((flashlight_dev->ops ==  NULL) ||
	    (flashlight_dev->ops->set_strobe_timeout == NULL))
		return -EINVAL;
	for (selector = 0; ; selector++) {
		timeout = flashlight_list_strobe_timeout(flashlight_dev,
								selector);
		if (timeout < 0)
			return timeout;
		if (timeout >= min_ms && timeout <= max_ms) {
			mutex_lock(&flashlight_dev->ops_lock);
			rc = flashlight_dev->ops->set_strobe_timeout(
				     flashlight_dev, timeout);
			mutex_unlock(&flashlight_dev->ops_lock);
			if (rc == 0)
				flashlight_dev->props.strobe_timeout = timeout;
			return rc;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(flashlight_set_strobe_timeout);

int flashlight_set_mode(struct flashlight_device *flashlight_dev,
			int mode)
{
	int rc;

	if (mode >= FLASHLIGHT_MODE_MAX || mode < 0)
		return -EINVAL;
	if ((flashlight_dev->ops ==  NULL) ||
	    (flashlight_dev->ops->set_mode == NULL)) {
		flashlight_dev->props.mode = mode;
		return 0;
	}
	mutex_lock(&flashlight_dev->ops_lock);
	rc = flashlight_dev->ops->set_mode(flashlight_dev,
					   mode);
	mutex_unlock(&flashlight_dev->ops_lock);
	if (rc < 0)
		return rc;
	flashlight_dev->props.mode = mode;
	return rc;
}
EXPORT_SYMBOL(flashlight_set_mode);

int flashlight_strobe(struct flashlight_device *flashlight_dev)
{
	if (flashlight_dev->props.mode == FLASHLIGHT_MODE_FLASH
	    || flashlight_dev->props.mode == FLASHLIGHT_MODE_MIXED) {
		if (flashlight_dev->ops == NULL ||
		    flashlight_dev->ops->strobe == NULL)
			return -EINVAL;
		return flashlight_dev->ops->strobe(flashlight_dev);
	}
	return -EINVAL;
}
EXPORT_SYMBOL(flashlight_strobe);

int flashlight_is_ready(struct flashlight_device *flashlight_dev)
{
	if (flashlight_dev->ops == NULL ||
			flashlight_dev->ops->is_ready == NULL)
		return -EINVAL;
	return flashlight_dev->ops->is_ready(flashlight_dev);
}
EXPORT_SYMBOL(flashlight_is_ready);

static int flashlight_match_device_by_name(struct device *dev, const void *data)
{
	const char *name = data;

	return strcmp(dev_name(dev), name) == 0;
}

struct flashlight_device *find_flashlight_by_name(const char *name)
{
	struct device *dev;

	if (!name)
		return (struct flashlight_device *)NULL;
	dev = class_find_device(flashlight_class, NULL, name,
				flashlight_match_device_by_name);

	return dev ? to_flashlight_device(dev) : NULL;

}
EXPORT_SYMBOL(find_flashlight_by_name);

int flashlight_strobe_charge(struct flashlight_device *flashlight_dev,
			flashlight_charge_event_cb cb, void *data, int start)
{

	if (flashlight_dev->ops->strobe_charge)
		return flashlight_dev->ops->strobe_charge(flashlight_dev,
				cb, data, start);
	if (flashlight_dev->props.type == FLASHLIGHT_TYPE_LED) {
		if (cb)
			cb(data, 0);
		return 0;
	}
	return -EINVAL;
}
EXPORT_SYMBOL(flashlight_strobe_charge);

static void __exit flashlight_class_exit(void)
{
	class_destroy(flashlight_class);
}

static int __init flashlight_class_init(void)
{
	flashlight_class = class_create(THIS_MODULE, "flashlight");
	if (IS_ERR(flashlight_class)) {
		pr_info("Unable to create flashlight class; errno = %ld\n",
		       PTR_ERR(flashlight_class));
		return PTR_ERR(flashlight_class);
	}
	flashlight_class->dev_groups = flashlight_groups;
	flashlight_class->suspend = flashlight_suspend;
	flashlight_class->resume = flashlight_resume;
	return 0;
}

subsys_initcall(flashlight_class_init);
module_exit(flashlight_class_exit);

MODULE_DESCRIPTION("Flashlight Class Device");
MODULE_AUTHOR("Patrick Chang <patrick_chang@richtek.com>");
MODULE_VERSION("1.0.2_G");
MODULE_LICENSE("GPL");
