/* BOSCH BMI160 STEP COUNTER Sensor Driver
 * (C) Copyright 2011~2016 Bosch Sensortec GmbH All Rights Reserved
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 * VERSION: V2.1
 * Date: 2017/1/18
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <cust_acc.h>
#include <step_counter.h>
#include "bmi160_stc.h"


struct acc_hw acc_cust;

struct pedometer_data_t {
	u8 wkar_step_detector_status;
	u_int32_t last_step_counter_value;
};

/* bmg i2c client data */
struct step_c_i2c_data {
	struct i2c_client *client;
	u8 sensor_name[MAX_SENSOR_NAME];
	enum SENSOR_TYPE_ENUM sensor_type;
	enum STC_POWERMODE_ENUM power_mode;
	int datarate;
	struct mutex lock;
	atomic_t	trace;
	atomic_t	suspend;
	atomic_t	filter;
	int sigmotion_enable;
	int stepdet_enable;
	int stepcounter_enable;
	struct pedometer_data_t pedo_data;
};

/* 0 = OK, -1 = fail */
static int step_c_init_flag = -1;
static struct step_c_i2c_data *obj_i2c_data;
static int step_c_set_powermode(
enum STC_POWERMODE_ENUM power_mode);

static const struct i2c_device_id step_c_i2c_id[] = {
	{STC_DEV_NAME, 0},
	{}
};

static void bmi160_stc_delay(u32 msec)
{
	if (msec <= 20)
		usleep_range(msec * 1000, (msec * 1000) + 1000);
	else
		msleep(msec);
}

static int stc_i2c_read_block(struct i2c_client *client, u8 addr,
				u8 *data, u8 len)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &beg
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,	.buf = data,
		}
	};
	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		pr_err(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		pr_err("i2c_transfer error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	msleep(1);/*delay 1ms for every read*/
	return err;
}

static int stc_i2c_write_block(struct i2c_client *client, u8 addr,
				u8 *data, u8 len)
{
	int err, idx = 0, num = 0;
	char buf[C_I2C_FIFO_SIZE];
	if (!client)
		return -EINVAL;
	else if (len >= C_I2C_FIFO_SIZE) {
		pr_err("length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];
	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		pr_err("send command error!!\n");
		return -EFAULT;
	} else {
		err = 0;
	}
	msleep(1);/*delay 1ms for every write*/
	return err;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_counter_enable(u8 v_step_counter_u8)
{
	struct step_c_i2c_data *obj = obj_i2c_data;
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;

	/* write the step counter */
	com_rslt = stc_i2c_read_block(obj->client,
	BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG,
	&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
	if (com_rslt == SUCCESS) {
		v_data_u8 =
		BMI160_SET_BITSLICE(v_data_u8,
		BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE,
		v_step_counter_u8);
		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
	}
	return com_rslt;
}

/*!
 *	@brief  This API is used to set
 *	interrupt enable from the register 0x50 bit 0 to 7
 *
 *	@param v_enable_u8 : Value to decided to select interrupt
 *   v_enable_u8   |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_ANY_MOTION_X_ENABLE
 *       1         | BMI160_ANY_MOTION_Y_ENABLE
 *       2         | BMI160_ANY_MOTION_Z_ENABLE
 *       3         | BMI160_DOUBLE_TAP_ENABLE
 *       4         | BMI160_SINGLE_TAP_ENABLE
 *       5         | BMI160_ORIENT_ENABLE
 *       6         | BMI160_FLAT_ENABLE
 *
 *	@param v_intr_enable_zero_u8 : The interrupt enable value
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
static BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_0(
u8 v_enable_u8, u8 v_intr_enable_zero_u8)
{
	struct step_c_i2c_data *obj = obj_i2c_data;
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	if (obj == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	} else {
		switch (v_enable_u8) {
		case BMI160_ANY_MOTION_X_ENABLE:
			/* write any motion x*/
			com_rslt = stc_i2c_read_block(obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE,
				v_intr_enable_zero_u8);
				com_rslt += stc_i2c_write_block(
				obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG,
				&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			}
			break;
		case BMI160_ANY_MOTION_Y_ENABLE:
			/* write any motion y*/
			com_rslt = stc_i2c_read_block(obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE,
				v_intr_enable_zero_u8);
				com_rslt += stc_i2c_write_block(
				obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG,
				&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			}
			break;
		case BMI160_ANY_MOTION_Z_ENABLE:
			/* write any motion z*/
			com_rslt = stc_i2c_read_block(obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE,
				v_intr_enable_zero_u8);
				com_rslt += stc_i2c_write_block(
				obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG,
				&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			}
			break;
		case BMI160_DOUBLE_TAP_ENABLE:
			/* write double tap*/
			com_rslt = stc_i2c_read_block(obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE,
				v_intr_enable_zero_u8);
				com_rslt += stc_i2c_write_block(
				obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG,
				&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			}
			break;
		case BMI160_SINGLE_TAP_ENABLE:
			/* write single tap */
			com_rslt = stc_i2c_read_block(obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE,
				v_intr_enable_zero_u8);
				com_rslt += stc_i2c_write_block(
				obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG,
				&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			}
			break;
		case BMI160_ORIENT_ENABLE:
			/* write orient interrupt*/
			com_rslt = stc_i2c_read_block(obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE,
				v_intr_enable_zero_u8);
				com_rslt += stc_i2c_write_block(
				obj_i2c_data->client,
				BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG,
				&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			}
			break;
		case BMI160_FLAT_ENABLE:
			/* write flat interrupt*/
			com_rslt = stc_i2c_read_block(obj_i2c_data->client,
			BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			if (com_rslt == SUCCESS) {
				v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
				BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE,
				v_intr_enable_zero_u8);
				com_rslt += stc_i2c_write_block(
				obj_i2c_data->client,
				BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG,
				&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
			}
			break;
		default:
			com_rslt = E_BMI160_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API reads step counter value
 *	form the register 0x78 and 0x79
 *  @param v_step_cnt_s16 : The value of step counter
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_read_step_count(u16 *v_step_cnt_s16)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 a_data_u8r[2] = {BMI160_INIT_VALUE, BMI160_INIT_VALUE};
	com_rslt = stc_i2c_read_block(obj_i2c_data->client,
	BMI160_USER_STEP_COUNT_LSB__REG,
	a_data_u8r, BMI160_STEP_COUNTER_LENGTH);

	*v_step_cnt_s16 = (u16)
	((((u32)((u8)a_data_u8r[BMI160_STEP_COUNT_MSB_BYTE]))
	<< BMI160_SHIFT_BIT_POSITION_BY_08_BITS)
	| (a_data_u8r[BMI160_STEP_COUNT_LSB_BYTE]));
	return com_rslt;
}
/*!
 *	@brief  Configure trigger condition of interrupt1
 *	and interrupt2 pin from the register 0x53
 *	@brief interrupt1 - bit 0
 *	@brief interrupt2 - bit 4
 *
 *  @param v_channel_u8: The value of edge trigger selection
 *   v_channel_u8  |   Edge trigger
 *  ---------------|---------------
 *       0         | BMI160_INTR1_EDGE_CTRL
 *       1         | BMI160_INTR2_EDGE_CTRL
 *
 *	@param v_intr_edge_ctrl_u8 : The value of edge trigger enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_EDGE
 *  0x00     |  BMI160_LEVEL
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_edge_ctrl_stc(
u8 v_channel_u8, u8 v_intr_edge_ctrl_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	switch (v_channel_u8) {
	case BMI160_INTR1_EDGE_CTRL:
		/* write the edge trigger interrupt1*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR1_EDGE_CTRL__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR1_EDGE_CTRL,
			v_intr_edge_ctrl_u8);
			com_rslt += stc_i2c_write_block(obj_i2c_data->client,
			BMI160_USER_INTR1_EDGE_CTRL__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
		break;
	case BMI160_INTR2_EDGE_CTRL:
		/* write the edge trigger interrupt2*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR2_EDGE_CTRL__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR2_EDGE_CTRL,
			v_intr_edge_ctrl_u8);
			com_rslt += stc_i2c_write_block(obj_i2c_data->client,
			BMI160_USER_INTR2_EDGE_CTRL__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
	return com_rslt;
}
/*!
 *	@brief  API used for set the Configure level condition of interrupt1
 *	and interrupt2 pin form the register 0x53
 *	@brief interrupt1 - bit 1
 *	@brief interrupt2 - bit 5
 *
 *  @param v_channel_u8: The value of level condition selection
 *   v_channel_u8  |   level selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_LEVEL
 *       1         | BMI160_INTR2_LEVEL
 *
 *	@param v_intr_level_u8 : The value of level of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_LEVEL_HIGH
 *  0x00     |  BMI160_LEVEL_LOW
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_level_stc(
u8 v_channel_u8, u8 v_intr_level_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	switch (v_channel_u8) {
	case BMI160_INTR1_LEVEL:
		/* write the interrupt1 level*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR1_LEVEL__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR1_LEVEL, v_intr_level_u8);
			com_rslt += stc_i2c_write_block(obj_i2c_data->client,
			BMI160_USER_INTR1_LEVEL__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
		break;
	case BMI160_INTR2_LEVEL:
		/* write the interrupt2 level*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR2_LEVEL__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR2_LEVEL, v_intr_level_u8);
			com_rslt += stc_i2c_write_block(obj_i2c_data->client,
			BMI160_USER_INTR2_LEVEL__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
	return com_rslt;
}

 /*!
 *	@brief API used to set the Output enable for interrupt1
 *	and interrupt1 pin from the register 0x53
 *	@brief interrupt1 - bit 3
 *	@brief interrupt2 - bit 7
 *
 *  @param v_channel_u8: The value of output enable selection
 *   v_channel_u8  |   level selection
 *  ---------------|---------------
 *       0         | BMI160_INTR1_OUTPUT_TYPE
 *       1         | BMI160_INTR2_OUTPUT_TYPE
 *
 *	@param v_output_enable_u8 :
 *	The value of output enable of interrupt enable
 *	value    | Behaviour
 * ----------|-------------------
 *  0x01     |  BMI160_INPUT
 *  0x00     |  BMI160_OUTPUT
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_output_enable_stc(
u8 v_channel_u8, u8 v_output_enable_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	/* check the p_bmi160 structure as NULL*/
	switch (v_channel_u8) {
	case BMI160_INTR1_OUTPUT_ENABLE:
		/* write the output enable of interrupt1*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR1_OUTPUT_ENABLE__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR1_OUTPUT_ENABLE,
			v_output_enable_u8);
			com_rslt += stc_i2c_write_block(obj_i2c_data->client,
			BMI160_USER_INTR1_OUTPUT_ENABLE__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
	break;
	case BMI160_INTR2_OUTPUT_ENABLE:
		/* write the output enable of interrupt2*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR2_OUTPUT_EN__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR2_OUTPUT_EN,
			v_output_enable_u8);
			com_rslt += stc_i2c_write_block(obj_i2c_data->client,
			BMI160_USER_INTR2_OUTPUT_EN__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
	return com_rslt;
}
/*!
 *  @brief This API is used to select
 *  the significant or any motion interrupt from the register 0x62 bit 1
 *  @param  v_intr_significant_motion_select_u8 :
 *  the value of significant or any motion interrupt selection
 *  value	   | Behaviour
 * ----------|-------------------
 *  0x00	   |  ANY_MOTION
 *  0x01	   |  SIGNIFICANT_MOTION
 *  @return results of bus communication function
 *  @retval 0 -> Success
 *  @retval -1 -> Error
 *
 *
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_select(
u8 *v_intr_significant_motion_select_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	/* check the p_bmi160 structure as NULL*/
		/* read the significant or any motion interrupt*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		*v_intr_significant_motion_select_u8 =
		BMI160_GET_BITSLICE(v_data_u8,
		BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT);
	return com_rslt;
}
/*!
 *  @brief This API is used to write, select
 *  the significant or any motion interrupt from the register 0x62 bit 1
 *  @param  v_intr_significant_motion_select_u8 :
 *  the value of significant or any motion interrupt selection
 *  value	  | Behaviour
 * ----------|-------------------
 *  0x00	  |  ANY_MOTION
 *  0x01	  |  SIGNIFICANT_MOTION
 *  @return results of bus communication function
 *  @retval 0 -> Success
 *  @retval -1 -> Error
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_select(
u8 v_intr_significant_motion_select_u8)
{
/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	if (v_intr_significant_motion_select_u8 <=
	BMI160_MAX_VALUE_SIGNIFICANT_MOTION) {
		/* write the significant or any motion interrupt*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT,
			v_intr_significant_motion_select_u8);
			com_rslt += stc_i2c_write_block(obj_i2c_data->client,
			BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
	} else {
	com_rslt = E_BMI160_OUT_OF_RANGE;
	}
return com_rslt;
}
 /*!
 *	@brief set the Low g interrupt mapped to interrupt1
 *	and interrupt2 from the register 0x55 and 0x57
 *	@brief interrupt1 bit 0 in the register 0x55
 *	@brief interrupt2 bit 0 in the register 0x57
 *
 *
 *	@param v_channel_u8: The value of low_g selection
 *   v_channel_u8  |   interrupt
 *  ---------------|---------------
 *       0         | BMI160_INTR1_MAP_LOW_G
 *       1         | BMI160_INTR2_MAP_LOW_G
 *
 *	@param v_intr_low_g_u8 : The value of low_g enable
 *	value    | interrupt enable
 * ----------|-------------------
 *  0x01     |  BMI160_ENABLE
 *  0x00     |  BMI160_DISABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g(
u8 v_channel_u8, u8 v_intr_low_g_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;

	switch (v_channel_u8) {
	/* write the low_g interrupt*/
	case BMI160_MAP_INTR1:
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_0_INTR1_LOW_G, v_intr_low_g_u8);
			com_rslt += stc_i2c_write_block(obj_i2c_data->client,
			BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
		break;
	case BMI160_MAP_INTR2:
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
			BMI160_USER_INTR_MAP_2_INTR2_LOW_G, v_intr_low_g_u8);
			com_rslt += stc_i2c_write_block(obj_i2c_data->client,
			BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
		break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
		break;
	}
return com_rslt;
} /*!
 *	@brief This API is used to read
 *	interrupt enable step detector interrupt from
 *	the register bit 0x52 bit 3
 *	@param v_step_intr_u8 : The value of step detector interrupt enable
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_detector_enable(
u8 *v_step_intr_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;

	/* read the step detector interrupt*/
	com_rslt = stc_i2c_read_block(obj_i2c_data->client,
	BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG,
	&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
	*v_step_intr_u8 = BMI160_GET_BITSLICE(v_data_u8,
	BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE);
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	interrupt enable step detector interrupt from
 *	the register bit 0x52 bit 3
 *	@param v_step_intr_u8 : The value of step detector interrupt enable
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 */
BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_detector_enable(
u8 v_step_intr_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_data_u8 = BMI160_INIT_VALUE;

	com_rslt = stc_i2c_read_block(obj_i2c_data->client,
	BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG,
	&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
	if (com_rslt == SUCCESS) {
		v_data_u8 = BMI160_SET_BITSLICE(v_data_u8,
		BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE,
		v_step_intr_u8);
		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
	}
	return com_rslt;
}
/*!
 *	@brief This API used to trigger the step detector
 *	interrupt
 *  @param  v_step_detector_u8 : The value of interrupt selection
 *  value    |  interrupt
 * ----------|-----------
 *   0       |  BMI160_MAP_INTR1
 *   1       |  BMI160_MAP_INTR2
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_map_step_detector_intr(
u8 v_step_detector_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_step_det_u8 = BMI160_INIT_VALUE;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 v_low_g_intr_u81_stat_u8 = BMI160_LOW_G_INTR_STAT;
	u8 v_low_g_intr_u82_stat_u8 = BMI160_LOW_G_INTR_STAT;
	u8 v_low_g_enable_u8 = BMI160_ENABLE_LOW_G;
	/* read the v_status_s8 of step detector interrupt*/
	com_rslt = bmi160_get_step_detector_enable(&v_step_det_u8);
	if (v_step_det_u8 != BMI160_STEP_DET_STAT_HIGH)
		com_rslt += bmi160_set_step_detector_enable(
		BMI160_STEP_DETECT_INTR_ENABLE);
	switch (v_step_detector_u8) {
	case BMI160_MAP_INTR1:
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_low_g_intr_u81_stat_u8;
		/* map the step detector interrupt
		to Low-g interrupt 1*/
		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
		/* Enable the Low-g interrupt*/
		com_rslt += stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_low_g_enable_u8;
		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);

		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	break;
	case BMI160_MAP_INTR2:
		/* map the step detector interrupt
		to Low-g interrupt 1*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_low_g_intr_u82_stat_u8;

		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
		/* Enable the Low-g interrupt*/
		com_rslt += stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_low_g_enable_u8;
		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;
	}
	return com_rslt;
}
/*!
 *	@brief This API used to trigger the  signification motion
 *	interrupt
 *  @param  v_significant_u8 : The value of interrupt selection
 *  value    |  interrupt
 * ----------|-----------
 *   0       |  BMI160_MAP_INTR1
 *   1       |  BMI160_MAP_INTR2
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
*/
BMI160_RETURN_FUNCTION_TYPE bmi160_map_significant_motion_intr(
u8 v_significant_u8)
{
	/* variable used for return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	u8 v_sig_motion_u8 = BMI160_INIT_VALUE;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 v_any_motion_intr1_stat_u8 = BMI160_ENABLE_ANY_MOTION_INTR1;
	u8 v_any_motion_intr2_stat_u8 = BMI160_ENABLE_ANY_MOTION_INTR2;
	u8 v_any_motion_axis_stat_u8 = BMI160_ENABLE_ANY_MOTION_AXIS;
	/* enable the significant motion interrupt */
	com_rslt = bmi160_get_intr_significant_motion_select(&v_sig_motion_u8);
	if (v_sig_motion_u8 != BMI160_SIG_MOTION_STAT_HIGH)
		com_rslt += bmi160_set_intr_significant_motion_select(
		BMI160_SIG_MOTION_INTR_ENABLE);
	switch (v_significant_u8) {
	case BMI160_MAP_INTR1:
		/* interrupt */
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_any_motion_intr1_stat_u8;
		/* map the signification interrupt to any-motion interrupt1*/
		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
		/* axis*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_ENABLE_0_ADDR,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_any_motion_axis_stat_u8;
		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_INTR_ENABLE_0_ADDR,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	break;

	case BMI160_MAP_INTR2:
		/* map the signification interrupt to any-motion interrupt2*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_any_motion_intr2_stat_u8;
		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
		/* axis*/
		com_rslt = stc_i2c_read_block(obj_i2c_data->client,
		BMI160_USER_INTR_ENABLE_0_ADDR,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_any_motion_axis_stat_u8;
		com_rslt += stc_i2c_write_block(obj_i2c_data->client,
		BMI160_USER_INTR_ENABLE_0_ADDR,
		&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
	break;

	}
	return com_rslt;
}

static int step_c_get_chip_type(struct i2c_client *client)
{
	int err = 0;
	u8 chip_id = 0;
	struct step_c_i2c_data *obj = obj_i2c_data;
	err = stc_i2c_read_block(client, BMI160_USER_CHIP_ID__REG, &chip_id, 1);
	bmi160_stc_delay(20);
	if (err < 0) {
		pr_err("read chip id failed.\n");
		return err;
	}
	switch (chip_id) {
	case SENSOR_CHIP_ID_BMI:
	case SENSOR_CHIP_ID_BMI_C2:
	case SENSOR_CHIP_ID_BMI_C3:
		obj->sensor_type = BMI160_STC_TYPE;
		strlcpy(obj->sensor_name, SENSOR_NAME, 48);
		break;
	default:
		obj->sensor_type = INVALID_TYPE;
		strlcpy(obj->sensor_name, UNKNOWN_SENSOR, 48);
		break;
	}
	if (obj->sensor_type == INVALID_TYPE) {
		pr_err("unknown sensor.\n");
		return ERROR;
	}
	pr_debug("read sensor chip id = %x ok.\n", (int)chip_id);
	return SUCCESS;
}

static int step_c_set_powermode(enum STC_POWERMODE_ENUM power_mode)
{
	int err = 0;
	u8 actual_power_mode = 0;
	struct step_c_i2c_data *obj = obj_i2c_data;

	if (power_mode == obj->power_mode)
		return err;
	mutex_lock(&obj->lock);
	if (power_mode == STC_SUSPEND_MODE)
		actual_power_mode = CMD_PMU_ACC_SUSPEND;
	else
		actual_power_mode = CMD_PMU_ACC_NORMAL;
	err = stc_i2c_write_block(obj_i2c_data->client,
		BMI160_CMD_COMMANDS__REG, &actual_power_mode, 1);
	bmi160_stc_delay(10);
	if (err < 0) {
		pr_err("set power mode failed.\n");
		mutex_unlock(&obj->lock);
		return err;
	}
	obj->power_mode = power_mode;
	mutex_unlock(&obj->lock);
	pr_debug("set power mode = %d ok.\n", (int)actual_power_mode);
	return err;
}

static int step_c_set_datarate(int datarate)
{
	return 0;
}


static int step_c_init_client(struct i2c_client *client)
{
	int err = 0;

	err = step_c_get_chip_type(client);
	if (err < 0)
		return err;
	err = step_c_set_datarate(BMI160_ACCEL_ODR_200HZ);
	if (err < 0)
		return err;
	return err;
}

static int step_c_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_i2c_data;
	if (file->private_data == NULL) {
		pr_err("file->private_data == NULL.\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int step_c_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long stc_c_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	return 0;
}

static const struct file_operations step_c_fops = {
	.owner = THIS_MODULE,
	.open = step_c_open,
	.release = step_c_release,
	.unlocked_ioctl = stc_c_unlocked_ioctl,
};

static struct miscdevice step_c_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "step_counter",
	.fops = &step_c_fops,
};
static int bmi160_step_c_open_report_data(int open)
{
	return 0;
}

static int bmi160_step_c_enable_nodata(int en)
{
	int err = 0;
	pr_debug("bmi160_step_c_enable_nodata\n");

	if (bmi160_set_step_counter_enable(en) < 0) {
		pr_err("set bmi160_STEP_COUNTER error");
		return -EINVAL;
	}
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	if (en == 1)
		err = step_c_set_powermode(STC_NORMAL_MODE);
	if (err)
		pr_err("set acc_op_mode failed");
	obj_i2c_data->stepcounter_enable = en;
	return err;
}

static int bmi160_step_c_set_delay(u64 ns)
{
	int err;
	int sample_delay = 0;
	int value = (int)ns / 1000 / 1000;

	pr_debug("bmi160_step_c_set_delay\n");
	if (value <= 5)
		sample_delay = BMI160_ACCEL_ODR_200HZ;
	else if (value <= 10)
		sample_delay = BMI160_ACCEL_ODR_100HZ;
	else if (value <= 20)
		sample_delay = BMI160_ACCEL_ODR_50HZ;
	else if (value <= 40)
		sample_delay = BMI160_ACCEL_ODR_25HZ;
	else
		sample_delay = BMI160_ACCEL_ODR_100HZ;
	pr_debug("sensor delay value = %d, sample delay = %d\n",
			value, sample_delay);
	err = step_c_set_datarate(sample_delay);
	if (err < 0)
		pr_err("set data rate error.\n");
	return err;
}

static int bmi160_step_c_enable_significant(int en)
{
	int err = 0;

	pr_debug("bmi160_step_c_enable_significant, en = %d\n", en);
	if (bmi160_set_intr_significant_motion_select(en) < 0) {
		pr_err("set bmi160_significant error");
		return -EINVAL;
	}
	if (en == 1) {
		err = step_c_set_powermode(STC_NORMAL_MODE);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
		err = bmi160_set_intr_enable_0
				(BMI160_ANY_MOTION_X_ENABLE, 1);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
		err += bmi160_set_intr_enable_0
				(BMI160_ANY_MOTION_Y_ENABLE, 1);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
		err += bmi160_set_intr_enable_0
				(BMI160_ANY_MOTION_Z_ENABLE, 1);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	}
	if (en == 0) {
		err = bmi160_set_intr_enable_0
				(BMI160_ANY_MOTION_X_ENABLE, 0);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
		err += bmi160_set_intr_enable_0
				(BMI160_ANY_MOTION_Y_ENABLE, 0);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
		err += bmi160_set_intr_enable_0
				(BMI160_ANY_MOTION_Z_ENABLE, 0);
		bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	}
	if (err)
		pr_err("set acc_config failed");
	obj_i2c_data->sigmotion_enable = en;
	return err;
}

static int bmi160_step_c_enable_step_detect(int en)
{
	int err = 0;

	pr_debug("bmi160_step_c_enable_step_detect, en = %d\n", en);
	if (bmi160_set_step_detector_enable(en) < 0) {
		pr_err("set bmi160_STEP_COUNTER error");
		return -EINVAL;
	}
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	#ifdef BMI160_ENABLE_INT1
	if (bmi160_set_intr_low_g(BMI160_MAP_INTR1, en) < 0) {
		pr_err("set bmi160_STEP_COUNTER_lowg error");
		return -EINVAL;
	}
	#endif
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	#ifdef BMI160_ENABLE_INT2
	if (bmi160_set_intr_low_g(BMI160_MAP_INTR2, en) < 0) {
		pr_err("set bmi160_STEP_COUNTER_lowg error");
		return -EINVAL;
	}
	#endif
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	if (en == 1)
		err = step_c_set_powermode(STC_NORMAL_MODE);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	if (err)
		pr_err("set acc_op_mode failed");
	obj_i2c_data->stepdet_enable = en;
	return err;
}

static int bmi160_stc_get_mode(struct i2c_client *client, u8 *mode)
{
	int comres = 0;
	u8 v_data_u8r = 0;
	comres = stc_i2c_read_block(client,
			BMI160_USER_ACC_PMU_STATUS__REG, &v_data_u8r, 1);
	*mode = BMI160_GET_BITSLICE(v_data_u8r,
			BMI160_USER_ACC_PMU_STATUS);
	return comres;
}
static int get_step_counter_enable(
	struct i2c_client *client, u8 *v_step_counter_u8)
{
	int com_rslt  = 0;
	u8 v_data_u8 = 0;
	com_rslt = stc_i2c_read_block(client,
			BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG,
			&v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
	*v_step_counter_u8 = BMI160_GET_BITSLICE(v_data_u8,
			BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE);
	return com_rslt;
}
static int bmi160_step_c_get_data(uint32_t *value, int *status)
{
	u16 data;
	int err;
	static u16 last_stc_value;
	u8 acc_mode;
	u8 stc_enable;
	struct step_c_i2c_data *obj = obj_i2c_data;

	err = bmi160_read_step_count(&data);
	if (0 == data) {
		err = bmi160_stc_get_mode(obj->client, &acc_mode);
		if (err < 0)
			pr_err("bmi160_acc_get_mode failed.\n");
		pr_debug("step_c_get_data acc mode = %d.\n", acc_mode);
		err = get_step_counter_enable(obj->client, &stc_enable);
		if (err < 0)
			pr_err("get_step_counter_enable failed.\n");
		pr_debug("step_c_get_data stc enable = %d.\n", stc_enable);
		if (acc_mode != 0 && stc_enable == 1)
			data = last_stc_value;
	}
	bmi160_stc_delay(30);
	if (err < 0) {
		pr_err("read step count fail.\n");
		return err;
	}

	if (data >= last_stc_value) {
		obj->pedo_data.last_step_counter_value += (
			data - last_stc_value);
		last_stc_value = data;
	} else {
		if (data > 0)
			last_stc_value = data;
	}
	*value = (int)obj->pedo_data.last_step_counter_value;
	*status = 1;
	pr_debug("step_c_get_data = %d.\n", (int)(*value));
	return err;

}

static int bmi160_stc_get_data_significant(uint32_t *value, int *status)
{
	pr_debug("bmi160_stc_get_data_significant\n");

	return 0;
}

static int bmi160_stc_get_data_step_d(uint32_t *value, int *status)
{
	pr_debug("bmi160_stc_get_data_step_d\n");

	return 0;
}

static int bmi160_map_interrupt(void)
{
	int err = 0;
	pr_debug("bmi160_map_interrupt\n");
#ifdef BMI160_ENABLE_INT1
	/*Set interrupt trige level way */
	err = bmi160_set_intr_edge_ctrl_stc(BMI160_MAP_INTR1, BMI_INT_LEVEL);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_set_intr_level_stc(BMI160_MAP_INTR1, 1);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_set_output_enable_stc(
	BMI160_INTR1_OUTPUT_ENABLE, ENABLE);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_map_significant_motion_intr(BMI160_MAP_INTR1);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_map_step_detector_intr(BMI160_MAP_INTR1);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	/*close step_detector in init function*/
	err += bmi160_set_step_detector_enable(0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	/*close log g in init function*/
	err += bmi160_set_intr_low_g(BMI160_MAP_INTR1, 0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	/*close significnt in init function*/
	err += bmi160_set_intr_significant_motion_select(0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_set_intr_enable_0
			(BMI160_ANY_MOTION_X_ENABLE, 0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_set_intr_enable_0
			(BMI160_ANY_MOTION_Y_ENABLE, 0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_set_intr_enable_0
			(BMI160_ANY_MOTION_Z_ENABLE, 0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
#endif

#ifdef BMI160_ENABLE_INT2
	/*Set interrupt trige level way */
	err = bmi160_set_intr_edge_ctrl_stc(BMI160_MAP_INTR2, BMI_INT_LEVEL);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_set_intr_level_stc(BMI160_MAP_INTR2, 1);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_set_output_enable_stc(
	BMI160_INTR2_OUTPUT_ENABLE, ENABLE);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_map_significant_motion_intr(BMI160_MAP_INTR2);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_map_step_detector_intr(BMI160_MAP_INTR2);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	/*close step_detector in init function*/
	err += bmi160_map_step_detector_intr(0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	/*close log g in init function*/
	err += bmi160_set_intr_low_g(BMI160_MAP_INTR2, 0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	/*close significnt in init function*/
	err += bmi160_set_intr_significant_motion_select(0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err = bmi160_set_intr_enable_0
		(BMI160_ANY_MOTION_X_ENABLE, 0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_set_intr_enable_0
		(BMI160_ANY_MOTION_Y_ENABLE, 0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
	err += bmi160_set_intr_enable_0
		(BMI160_ANY_MOTION_Z_ENABLE, 0);
	bmi160_stc_delay(BMI160_GEN_READ_WRITE_DELAY);
#endif
	return err;
}
static int step_c_i2c_probe(void)
{
	struct step_c_i2c_data *obj;
	struct step_c_control_path ctl = {0};
	struct step_c_data_path data = {0};
	int err = 0;

	pr_debug("step_c_i2c_probe\n");
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	obj_i2c_data = obj;
	if (NULL == bmi160_acc_i2c_client){
		pr_err("acc i2c client null\n");
		err = -EINVAL;
		goto exit;
	}
	obj->client = bmi160_acc_i2c_client;
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	obj->power_mode = STC_UNDEFINED_POWERMODE;
	obj->datarate = BMI160_ACCEL_ODR_RESERVED;
	mutex_init(&obj->lock);
	err = step_c_init_client(obj->client);
	if (err)
		goto exit_init_client_failed;
	err = misc_register(&step_c_device);
	if (err) {
		pr_err("misc device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}
	ctl.open_report_data = bmi160_step_c_open_report_data;
	ctl.enable_nodata = bmi160_step_c_enable_nodata;
	//ctl.set_delay = bmi160_step_c_set_delay;
	ctl.step_c_set_delay = bmi160_step_c_set_delay;
	ctl.step_d_set_delay = bmi160_step_c_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_counter_support_batch = false;
	ctl.enable_significant = bmi160_step_c_enable_significant;
	ctl.enable_step_detect = bmi160_step_c_enable_step_detect;

    ctl.step_d_flush = step_d_flush_report;//prize-chj-20190723 for fix cts testBatchAndFlush 
    ctl.step_c_flush = step_c_flush_report;//prize-chj-20190723 for fix cts testBatchAndFlush 

	err = step_c_register_control_path(&ctl);
	if (err) {
		pr_err("register step_counter control path err.\n");
		goto exit_create_attr_failed;
	}
	data.get_data = bmi160_step_c_get_data;
	data.vender_div = 1000;
	data.get_data_significant = bmi160_stc_get_data_significant;
	data.get_data_step_d = bmi160_stc_get_data_step_d;
	err = step_c_register_data_path(&data);
	if (err) {
		pr_err("step_c_register_data_path fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	err = bmi160_map_interrupt();
	if (err)
		pr_err("bmi160_map_interrupt fail = %d\n", err);
	step_c_init_flag = 0;
	pr_debug("%s: is ok.\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&step_c_device);
exit_misc_device_register_failed:
exit_init_client_failed:
	kfree(obj);
exit:
	step_c_init_flag = -1;
	pr_err("err = %d\n", err);
	return err;
}

static int bmi160_stc_remove(void)
{
	return 0;
}

static int bmi160_stc_local_init(void)
{
	pr_debug("bmi160_stc_local_init.\n");
	if (step_c_i2c_probe()) {
		pr_err("failed to register bmi160 step_c driver\n");
		return -ENODEV;
	}
	return 0;
}

static struct step_c_init_info bmi160_stc_init_info = {
	.name = "bmi160_step_counter",
	.init = bmi160_stc_local_init,
	.uninit = bmi160_stc_remove,
};

static int __init stc_init(void)
{
	pr_debug("stc_init\n");
	step_c_driver_add(&bmi160_stc_init_info);
	return 0;
}

static void __exit stc_exit(void)
{
	pr_debug();
}

module_init(stc_init);
module_exit(stc_exit);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("STEP COUNTER I2C Driver");
MODULE_AUTHOR("contact@bosch-sensortec.com>");
MODULE_VERSION(STC_DRIVER_VERSION);
