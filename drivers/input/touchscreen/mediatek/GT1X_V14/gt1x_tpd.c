/* drivers/input/touchscreen/gt1x_tpd.c
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * Version: 1.4   
 * Release Date:  2015/07/10
 */

#include "gt1x_tpd_custom.h"
#include "gt1x_generic.h"
#ifdef GTP_CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif
#if TPD_SUPPORT_I2C_DMA
#include <linux/dma-mapping.h>
#endif
#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif

#if (GTP_HAVE_TOUCH_KEY && TPD_HAVE_BUTTON)
#error GTP_HAVE_TOUCH_KEY and TPD_HAVE_BUTTON are mutually exclusive.
#endif

extern struct tpd_device *tpd;
static spinlock_t irq_lock; 
static int tpd_flag = 0;
static int tpd_irq_flag;
static int tpd_eint_mode = 1;
static int tpd_polling_time = 50;
static struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);

#if 0// TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT] = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#ifdef GTP_CONFIG_OF
static unsigned int tpd_touch_irq;
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc);
#else
static void tpd_eint_interrupt_handler(void);
#endif
static int tpd_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);

#ifndef GTP_CONFIG_OF
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En, kal_bool ACT_Polarity, void (EINT_FUNC_PTR) (void), kal_bool auto_umask);
#endif

#define GTP_DRIVER_NAME  "gt1x"
static const struct i2c_device_id tpd_i2c_id[] = { {GTP_DRIVER_NAME, 0}, {} };
static unsigned short force[] = { 0, GTP_I2C_ADDRESS, I2C_CLIENT_END, I2C_CLIENT_END };
static const unsigned short *const forces[] = { force, NULL };

//static struct i2c_client_address_data addr_data = { .forces = forces,};
//static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO(GTP_DRIVER_NAME, (GTP_I2C_ADDRESS >> 1)) };

static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{.compatible = "goodix,gt9xx"},
	{},
};

static struct i2c_driver tpd_i2c_driver = {
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.detect = tpd_i2c_detect,
	.driver = {
		   .name = GTP_DRIVER_NAME,
		   .of_match_table = tpd_of_match,
		  },
	.id_table = tpd_i2c_id,
	.address_list = (const unsigned short *)forces,
};

#if TPD_SUPPORT_I2C_DMA
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
struct mutex dma_mutex;

static s32 i2c_dma_write_mtk(u16 addr, u8 * buffer, s32 len)
{
	s32 ret = 0;
	s32 pos = 0;
	s32 transfer_length;
	u16 address = addr;
	struct i2c_msg msg = {
		.flags = !I2C_M_RD,
		.ext_flag = (gt1x_i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG),
		.timing = I2C_MASTER_CLOCK,
		.buf = (u8 *) gpDMABuf_pa,
	};

	mutex_lock(&dma_mutex);
	while (pos != len) {
		if (len - pos > (IIC_DMA_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH)) {
			transfer_length = IIC_DMA_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH;
		} else {
			transfer_length = len - pos;
		}

		gpDMABuf_va[0] = (address >> 8) & 0xFF;
		gpDMABuf_va[1] = address & 0xFF;
		memcpy(&gpDMABuf_va[GTP_ADDR_LENGTH], &buffer[pos], transfer_length);

		msg.len = transfer_length + GTP_ADDR_LENGTH;

		ret = i2c_transfer(gt1x_i2c_client->adapter, &msg, 1);
		if (ret != 1) {
			GTP_ERROR("I2c Transfer error! (%d)", ret);
			ret = ERROR_IIC;
			break;
		}
		ret = 0;
		pos += transfer_length;
		address += transfer_length;
	}
	mutex_unlock(&dma_mutex);
	return ret;
}

static s32 i2c_dma_read_mtk(u16 addr, u8 * buffer, s32 len)
{
	s32 ret = ERROR;
	s32 pos = 0;
	s32 transfer_length;
	u16 address = addr;
	u8 addr_buf[GTP_ADDR_LENGTH] = { 0 };
	struct i2c_msg msgs[2] = {
		{
		 .flags = 0,	//!I2C_M_RD,
		 .addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG),
		 .timing = I2C_MASTER_CLOCK,
		 .len = GTP_ADDR_LENGTH,
		 .buf = addr_buf,
		 },
		{
		 .flags = I2C_M_RD,
		 .ext_flag = (gt1x_i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		 .addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG),
		 .timing = I2C_MASTER_CLOCK,
		 .buf = (u8 *) gpDMABuf_pa,
		 },
	};
	mutex_lock(&dma_mutex);
	while (pos != len) {
		if (len - pos > IIC_DMA_MAX_TRANSFER_SIZE) {
			transfer_length = IIC_DMA_MAX_TRANSFER_SIZE;
		} else {
			transfer_length = len - pos;
		}

		msgs[0].buf[0] = (address >> 8) & 0xFF;
		msgs[0].buf[1] = address & 0xFF;
		msgs[1].len = transfer_length;

		ret = i2c_transfer(gt1x_i2c_client->adapter, msgs, 2);
		if (ret != 2) {
			GTP_ERROR("I2C Transfer error! (%d)", ret);
			ret = ERROR_IIC;
			break;
		}
		ret = 0;
		memcpy(&buffer[pos], gpDMABuf_va, transfer_length);
		pos += transfer_length;
		address += transfer_length;
	};
	mutex_unlock(&dma_mutex);
	return ret;
}

#else

static s32 i2c_write_mtk(u16 addr, u8 * buffer, s32 len)
{
	s32 ret;

	struct i2c_msg msg = {
		.flags = 0,
#ifdef CONFIG_MTK_I2C_EXTENSION
		.addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG),	/*remain*/
		.timing = I2C_MASTER_CLOCK,
#else
		.addr = gt1x_i2c_client->addr,  /*remain*/
#endif
	};

	ret = _do_i2c_write(&msg, addr, buffer, len);
	return ret;
}

static s32 i2c_read_mtk(u16 addr, u8 * buffer, s32 len)
{
	int ret;
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };

	struct i2c_msg msgs[2] = {
		{
#ifdef CONFIG_MTK_I2C_EXTENSION
		 .addr = ((gt1x_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
		 .timing = I2C_MASTER_CLOCK,
#else
		 .addr = gt1x_i2c_client->addr,
#endif
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH,
		},
		{
#ifdef CONFIG_MTK_I2C_EXTENSION
		 .addr = ((gt1x_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
		 .timing = I2C_MASTER_CLOCK,
#else
		 .addr = gt1x_i2c_client->addr,
#endif
		 .flags = I2C_M_RD,
		},
	};

	ret = _do_i2c_read(msgs, addr, buffer, len);
	return ret;
}
#endif /* TPD_SUPPORT_I2C_DMA */

/**
 * @return: return 0 if success, otherwise return a negative number
 *          which contains the error code.
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
#if TPD_SUPPORT_I2C_DMA
	return i2c_dma_read_mtk(addr, buffer, len);
#else
	return i2c_read_mtk(addr, buffer, len);
#endif
}

/**
 * @return: return 0 if success, otherwise return a negative number
 *          which contains the error code.
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
#if TPD_SUPPORT_I2C_DMA
	return i2c_dma_write_mtk(addr, buffer, len);
#else
	return i2c_write_mtk(addr, buffer, len);
#endif
}

#ifdef TPD_REFRESH_RATE
/**
 * gt1x_set_refresh_rate - Write refresh rate
 * @rate: refresh rate N (Duration=5+N ms, N=0~15)
 * Return: 0---succeed.
 */
static u8 gt1x_set_refresh_rate(u8 rate)
{
	u8 buf[1] = { rate };

	if (rate > 0xf) {
		GTP_ERROR("Refresh rate is over range (%d)", rate);
		return ERROR_VALUE;
	}

	GTP_INFO("Refresh rate change to %d", rate);
	return gt1x_i2c_write(GTP_REG_REFRESH_RATE, buf, sizeof(buf));
}

/**
  *gt1x_get_refresh_rate - get refresh rate
  *Return: Refresh rate or error code
 */
static u8 gt1x_get_refresh_rate(void)
{
	int ret;
	u8 buf[1] = { 0x00 };
	ret = gt1x_i2c_read(GTP_REG_REFRESH_RATE, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	GTP_INFO("Refresh rate is %d", buf[0]);
	return buf[0];
}

static ssize_t show_refresh_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = gt1x_get_refresh_rate();
	if (ret < 0)
		return 0;
	else
		return sprintf(buf, "%d\n", ret);
}

static ssize_t store_refresh_rate(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	gt1x_set_refresh_rate(simple_strtoul(buf, NULL, 16));
	return size;
}

static DEVICE_ATTR(tpd_refresh_rate, 0664, show_refresh_rate, store_refresh_rate);

static struct device_attribute *gt1x_attrs[] = {
	&dev_attr_tpd_refresh_rate,
};
#endif

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, "mtk-tpd");
	return 0;
}

static int tpd_power_on(void)
{
	gt1x_power_switch(SWITCH_ON);

    gt1x_reset_guitar();

	if (gt1x_get_chip_type() != 0) {
		return -1;
	}

	if (gt1x_reset_guitar() != 0) {
		return -1;
	}
	return 0;
}

void gt1x_irq_enable(void)
{
    unsigned long flag;
    
    spin_lock_irqsave(&irq_lock, flag);
    if (!tpd_irq_flag) { // 0-disabled
        tpd_irq_flag = 1;  // 1-enabled
#ifdef GTP_CONFIG_OF
        enable_irq(tpd_touch_irq);
#else
	    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
    }
    spin_unlock_irqrestore(&irq_lock, flag);
}

void gt1x_irq_disable(void)
{
    unsigned long flag;

    spin_lock_irqsave(&irq_lock, flag);
    if (tpd_irq_flag) {
        tpd_irq_flag = 0;
#ifdef GTP_CONFIG_OF
        disable_irq(tpd_touch_irq);
#else
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
    }
    spin_unlock_irqrestore(&irq_lock, flag);
}

int gt1x_power_switch(s32 state)
{
    static int power_state = 0;
    
	tpd_gpio_output(GTP_RST_PORT, 0);
	tpd_gpio_output(GTP_INT_PORT, 0);
	msleep(10);

	switch (state) {
	case SWITCH_ON:
        if (power_state == 0) {    
    		GTP_DEBUG("Power switch on!");
#ifdef MT6573
    		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#else // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )
#ifdef TPD_POWER_SOURCE_CUSTOM
             // set 2.8v
            if (regulator_set_voltage(tpd->reg, 2800000, 2800000))   
                GTP_ERROR("regulator_set_voltage() failed!");
            //enable regulator
            if (regulator_enable(tpd->reg))
                GTP_ERROR("regulator_enable() failed!");
#else
    		hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif

#ifdef TPD_POWER_SOURCE_1800
    		hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif
#endif
            power_state = 1;
        }
    		break;
	case SWITCH_OFF:
        if (power_state == 1) {
		    GTP_DEBUG("Power switch off!");
#ifdef MT6573
    		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
#else
#ifdef TPD_POWER_SOURCE_1800
    		//hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_CUSTOM
            //disable regulator
            if (regulator_disable(tpd->reg)){
				GTP_ERROR("regulator_disable() failed!");
			}
#else
    		//hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP")
#endif
#endif
            power_state = 0;
        }
		break;
	default:
		GTP_ERROR("Invalid power switch command!");
		break;
	}
	return 0;
}

static int tpd_irq_registration(void)
{
#ifdef GTP_CONFIG_OF
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};
	GTP_INFO("Device Tree Tpd_irq_registration!");
	
//	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	node = of_find_matching_node(NULL, touch_of_match);
	if(node){
		of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		tpd_touch_irq = irq_of_parse_and_map(node, 0);
		GTP_INFO("Device gt1x_int_type = %d!", gt1x_int_type);
		if (!gt1x_int_type)	//EINTF_TRIGGER
		{
			ret = request_irq(tpd_touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, IRQF_TRIGGER_RISING, "TOUCH_PANEL-eint", NULL);
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		} else {
			ret = request_irq(tpd_touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		}
	}else{
		GTP_ERROR("tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}
	GTP_INFO("irq:%d, debounce:%d-%d:", tpd_touch_irq, ints[0], ints[1]);
	return ret;
    
#else

    #ifndef MT6589
	if (!gt1x_int_type) {	/*EINTF_TRIGGER */
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);
	} else {
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	}

    #else
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);

	if (!gt1x_int_type) {
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1);
	} else {
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, tpd_eint_interrupt_handler, 1);
	}
    #endif
    return 0;
#endif
}

static s32 tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 err = 0;
#if GTP_HAVE_TOUCH_KEY
	s32 idx = 0;
#endif

	// important
    if (RECOVERY_BOOT == get_boot_mode()){
        return 0;		
	}
	
	if(client->addr != 0x5d)
	{
		client->addr = 0x5d;
		//printk("frank_zhonghua:i2c_client_FT->addr=%d\n",i2c_client->addr);
	}
	gt1x_i2c_client = client;
    spin_lock_init(&irq_lock);

	if (gt1x_init()) {
		/* TP resolution == LCD resolution, no need to match resolution when initialized fail */
		gt1x_abs_x_max = 0;
		gt1x_abs_y_max = 0;
		tpd_load_status = 0;//xielei
		return -1;
	}

	thread = kthread_run(tpd_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		GTP_ERROR(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}
#if GTP_HAVE_TOUCH_KEY
	for (idx = 0; idx < GTP_MAX_KEY_NUM; idx++) {
		input_set_capability(tpd->dev, EV_KEY, gt1x_touch_key_array[idx]);
	}
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(tpd->dev, EV_KEY, KEY_P);
	input_set_capability(tpd->dev, EV_KEY, KEY_UP);
	input_set_capability(tpd->dev, EV_KEY, KEY_DOWN);
	input_set_capability(tpd->dev, EV_KEY, KEY_RIGHT);
	input_set_capability(tpd->dev, EV_KEY, KEY_LEFT);
	input_set_capability(tpd->dev, EV_KEY, KEY_C);
	input_set_capability(tpd->dev, EV_KEY, KEY_O);
   	input_set_capability(tpd->dev, EV_KEY, KEY_E);
	 input_set_capability(tpd->dev, EV_KEY, KEY_M);
	  input_set_capability(tpd->dev, EV_KEY, KEY_W);
	input_set_capability(tpd->dev, EV_KEY, KEY_GES_CUSTOM);
    input_set_capability(tpd->dev, EV_KEY, KEY_GES_REGULAR);
	//add by gandy
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_EVENT);
#endif

	tpd_gpio_as_int(GTP_INT_PORT);
	msleep(50);

    /* interrupt registration */
    tpd_irq_registration();
	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	/*  must before auto update */
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
	thread = kthread_run(gt1x_auto_update_proc, (void *)NULL, "gt1x auto update");
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		GTP_ERROR(TPD_DEVICE "failed to create auto-update thread: %d\n", err);
	}
#endif

	tpd_load_status = 1;
	return 0;
}

#ifdef GTP_CONFIG_OF
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
    TPD_DEBUG_PRINT_INT;
	
	tpd_flag = 1;

	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
    spin_lock(&irq_lock);
    tpd_irq_flag = 0;
	disable_irq_nosync(tpd_touch_irq);
    spin_unlock(&irq_lock);

 	wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
static void tpd_eint_interrupt_handler(void)
{
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
    gt1x_irq_disable();
	wake_up_interruptible(&waiter);
}
#endif

void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(tpd->dev, id);
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
#else
	input_report_key(tpd->dev, BTN_TOUCH, 1);
	if ((!size) && (!id)) {
		/* for virtual button */
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	}
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
#endif

	if (tpd_dts_data.use_tpd_button) {
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(x, y, 1);
    }
	}
}

void gt1x_touch_up(s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(tpd->dev, id);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
#else
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
#endif
	if (tpd_dts_data.use_tpd_button) {

    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(0, 0, 0);
    }
	}
}

#if GTP_CHARGER_SWITCH
#ifdef MT6573
#define CHR_CON0      (0xF7000000+0x2FA00)
#else
extern kal_bool upmu_is_chr_det(void);
#endif

u32 gt1x_get_charger_status(void)
{
	u32 chr_status = 0;
#ifdef MT6573
	chr_status = *(volatile u32 *)CHR_CON0;
	chr_status &= (1 << 13);
#else /* ( defined(MT6575) || defined(MT6577) || defined(MT6589) ) */
	chr_status = upmu_is_chr_det();
#endif
	return chr_status;
}
#endif

static int tpd_event_handler(void *unused)
{
	u8 finger = 0;
	u8 end_cmd = 0;
	s32 ret = 0;
	u8 point_data[11] = { 0 };
	
	struct sched_param param = {.sched_priority = 4 };
	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);

		if (tpd_eint_mode) {
			wait_event_interruptible(waiter, tpd_flag != 0);
			tpd_flag = 0;
		} else {
			GTP_DEBUG("Polling coordinate mode!");
			msleep(tpd_polling_time);
		}

		set_current_state(TASK_RUNNING);

        if (update_info.status) {
            GTP_DEBUG("Ignore interrupts during fw updating.");
            continue;
        }
        
		mutex_lock(&i2c_access);
		/* don't reset before "if (gt1x_halt..."  */

#if GTP_GESTURE_WAKEUP
		ret = gesture_event_handler(tpd->dev);
		if (ret >= 0) {
			gt1x_irq_enable();
			mutex_unlock(&i2c_access);
			continue;
		}
#endif
		if (gt1x_halt) {
			mutex_unlock(&i2c_access);
			GTP_DEBUG("Ignore interrupts after suspend.");
			continue;
		}

		/* read coordinates */
		ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
		if (ret < 0) {
			GTP_ERROR("I2C transfer error!");
#if !GTP_ESD_PROTECT
			gt1x_power_reset();
#endif
			gt1x_irq_enable();
			mutex_unlock(&i2c_access);
            continue;
		}
		finger = point_data[0];

		/* response to a ic request */
		if (finger == 0x00) {
			gt1x_request_event_handler();
		}

		if ((finger & 0x80) == 0) {
#if HOTKNOT_BLOCK_RW
			if (!hotknot_paired_flag)
#endif
			{
				gt1x_irq_enable();
				mutex_unlock(&i2c_access);
				//GTP_ERROR("buffer not ready:0x%02x", finger);
				continue;
			}
		}
#if HOTKNOT_BLOCK_RW
		ret = hotknot_event_handler(point_data);
		if (!ret) {
			goto exit_work_func;
		}
#endif

#if GTP_PROXIMITY
		ret = gt1x_prox_event_handler(point_data);
		if (ret > 0) {
			goto exit_work_func;
		}
#endif

#if GTP_WITH_STYLUS
		ret = gt1x_touch_event_handler(point_data, tpd->dev, pen_dev);
#else
		ret = gt1x_touch_event_handler(point_data, tpd->dev, NULL);
#endif

#if (HOTKNOT_BLOCK_RW || GTP_PROXIMITY)
exit_work_func:
#endif
		if (!gt1x_rawdiff_mode && (ret >= 0 || ret == ERROR_VALUE)) {
			ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
			if (ret < 0) {
				GTP_INFO("I2C write end_cmd  error!");
			}
		}
		gt1x_irq_enable();
		mutex_unlock(&i2c_access);

	} while (!kthread_should_stop());

	return 0;
}

int gt1x_debug_proc(u8 * buf, int count)
{
	char mode_str[50] = { 0 };
	int mode;

	sscanf(buf, "%s %d", (char *)&mode_str, &mode);

	/***********POLLING/EINT MODE switch****************/
	if (strcmp(mode_str, "polling") == 0) {
		if (mode >= 10 && mode <= 200) {
			GTP_INFO("Switch to polling mode, polling time is %d", mode);
			tpd_eint_mode = 0;
			tpd_polling_time = mode;
			tpd_flag = 1;
			wake_up_interruptible(&waiter);
		} else {
			GTP_INFO("Wrong polling time, please set between 10~200ms");
		}
		return count;
	}
	if (strcmp(mode_str, "eint") == 0) {
		GTP_INFO("Switch to eint mode");
		tpd_eint_mode = 1;
		return count;
	}
	/**********************************************/
	if (strcmp(mode_str, "switch") == 0) {
		if (mode == 0)	// turn off
			tpd_off();
		else if (mode == 1)	//turn on
			tpd_on();
		else
			GTP_ERROR("error mode :%d", mode);
		return count;
	}

	return -1;
}

static u16 convert_productname(u8 * name)
{
	int i;
	u16 product = 0;
	for (i = 0; i < 4; i++) {
		product <<= 4;
		if (name[i] < '0' || name[i] > '9') {
			product += '*';
		} else {
			product += name[i] - '0';
		}
	}
	return product;
}

static int tpd_i2c_remove(struct i2c_client *client)
{
	gt1x_deinit();
	return 0;
}

static int tpd_local_init(void)
{
#ifdef TPD_POWER_SOURCE_CUSTOM
#ifdef GTP_CONFIG_OF
	#if defined(CONFIG_PRIZE_VTOUCH_USE_MT6370_LDO)
		tpd->reg = regulator_get(tpd->tpd_dev, "vtouch_ldo"); // get pointer to regulator structure
	#else
    	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch"); // get pointer to regulator structure
	#endif
    if (IS_ERR(tpd->reg)) {
        GTP_ERROR("regulator_get() failed.");
    }
#endif
#endif

#if TPD_SUPPORT_I2C_DMA
	mutex_init(&dma_mutex);
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *) dma_alloc_coherent(&tpd->dev->dev, IIC_DMA_MAX_TRANSFER_SIZE, &gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va) {
		GTP_ERROR("Allocate DMA I2C Buffer failed!");
		return -1;
	}
	memset(gpDMABuf_va, 0, IIC_DMA_MAX_TRANSFER_SIZE);
#endif
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		GTP_ERROR("unable to add i2c driver.");
		return -1;
	}

	if (tpd_load_status == 0)	// disable auto load touch driver for linux3.0 porting
	{
		GTP_ERROR("add error touch panel driver.");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (GTP_MAX_TOUCH - 1), 0, 0);

	if (tpd_dts_data.use_tpd_button) {
		/*initialize tpd button data */
		tpd_button_setting(tpd_dts_data.tpd_key_num,
				   tpd_dts_data.tpd_key_local,
				   tpd_dts_data.tpd_key_dim_local);
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif

	// set vendor string
	tpd->dev->id.vendor = 0x00;
	tpd->dev->id.product = convert_productname(gt1x_version.product_id);
	tpd->dev->id.version = (gt1x_version.patch_id >> 8);

	GTP_INFO("end %s, %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

/* Function to manage low power suspend */
static void tpd_suspend(struct device *h)
{
	gt1x_suspend();
}

/* Function to manage power-on resume */
static void tpd_resume(struct device *h)
{
	gt1x_resume();
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "gt1x",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

void tpd_off(void)
{
	gt1x_power_switch(SWITCH_OFF);
	gt1x_halt = 1;
	gt1x_irq_disable();
}

void tpd_on(void)
{
	s32 ret = -1, retry = 0;

	while (retry++ < 5) {
		ret = tpd_power_on();
		if (ret < 0) {
			GTP_ERROR("I2C Power on ERROR!");
		}

		ret = gt1x_send_cfg(gt1x_config, gt1x_cfg_length);
		if (ret == 0) {
			GTP_DEBUG("Wakeup sleep send gt1x_config success.");
			break;
		}
	}
	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
	//gt1x_irq_enable();
	gt1x_halt = 0;
}

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	GTP_INFO("Goodix touch panel driver init.");

	tpd_get_dts_info();

	if (tpd_driver_add(&tpd_device_driver) < 0) {
		GTP_ERROR("add generic driver failed.");
	}

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	GTP_INFO("MediaTek GT1x touch panel driver exit.");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
