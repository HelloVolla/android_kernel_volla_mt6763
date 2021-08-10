/* gravityhub motion sensor driver
 *
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */


#define pr_fmt(fmt) "[gravhub] " fmt

#include <hwmsensor.h>
#include "gravityhub.h"
#include <fusion.h>
#include <SCP_sensorHub.h>
#include <linux/notifier.h>
#include "scp_helper.h"

static struct fusion_init_info gravityhub_init_info;

static int grav_get_data(int *x, int *y, int *z, int *scalar, int *status)
{
	int err = 0;
	struct data_unit_t data;
	uint64_t time_stamp = 0;

	err = sensor_get_data_from_hub(ID_GRAVITY, &data);
	if (err < 0) {
		pr_err("sensor_get_data_from_hub fail!!\n");
		return -1;
	}
	time_stamp				= data.time_stamp;
	*x = data.accelerometer_t.x;
	*y = data.accelerometer_t.y;
	*z = data.accelerometer_t.z;
	*status = data.accelerometer_t.status;
	return 0;
}
static int grav_open_report_data(int open)
{
	return 0;
}
static int grav_enable_nodata(int en)
{
	return sensor_enable_to_hub(ID_GRAVITY, en);
}
static int grav_set_delay(u64 delay)
{
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	unsigned int delayms = 0;

	delayms = delay / 1000 / 1000;
	return sensor_set_delay_to_hub(ID_GRAVITY, delayms);
#elif defined CONFIG_NANOHUB
	return 0;
#else
	return 0;
#endif
}
static int grav_batch(int flag,
	int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	grav_set_delay(samplingPeriodNs);
#endif
	return sensor_batch_to_hub(ID_GRAVITY,
		flag, samplingPeriodNs, maxBatchReportLatencyNs);
}

static int grav_flush(void)
{
	return sensor_flush_to_hub(ID_GRAVITY);
}
static int gravity_recv_data(struct data_unit_t *event, void *reserved)
{
	int err = 0;

	if (event->flush_action == DATA_ACTION)
		err = grav_data_report(event->accelerometer_t.x,
			event->accelerometer_t.y, event->accelerometer_t.z,
			event->accelerometer_t.status,
			(int64_t)event->time_stamp);
	else if (event->flush_action == FLUSH_ACTION)
		err = grav_flush_report();

	return err;
}
static int gravityhub_local_init(void)
{
	struct fusion_control_path ctl = {0};
	struct fusion_data_path data = {0};
	int err = 0;

	ctl.open_report_data = grav_open_report_data;
	ctl.enable_nodata = grav_enable_nodata;
	ctl.set_delay = grav_set_delay;
	ctl.batch = grav_batch;
	ctl.flush = grav_flush;
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	ctl.is_report_input_direct = true;
	ctl.is_support_batch = false;
#elif defined CONFIG_NANOHUB
	ctl.is_report_input_direct = true;
	ctl.is_support_batch = false;
#else
#endif
	err = fusion_register_control_path(&ctl, ID_GRAVITY);
	if (err) {
		pr_err("register gravity control path err\n");
		goto exit;
	}

	data.get_data = grav_get_data;
	data.vender_div = 1000;
	err = fusion_register_data_path(&data, ID_GRAVITY);
	if (err) {
		pr_err("register gravity data path err\n");
		goto exit;
	}
	err = scp_sensorHub_data_registration(ID_GRAVITY, gravity_recv_data);
	if (err < 0) {
		pr_err("SCP_sensorHub_data_registration failed\n");
		goto exit;
	}
	return 0;
exit:
	return -1;
}
static int gravityhub_local_uninit(void)
{
	return 0;
}

static struct fusion_init_info gravityhub_init_info = {
	.name = "gravity_hub",
	.init = gravityhub_local_init,
	.uninit = gravityhub_local_uninit,
};

static int __init gravityhub_init(void)
{
	fusion_driver_add(&gravityhub_init_info, ID_GRAVITY);
	return 0;
}

static void __exit gravityhub_exit(void)
{
	pr_debug("%s\n", __func__);
}

module_init(gravityhub_init);
module_exit(gravityhub_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ACTIVITYHUB driver");
MODULE_AUTHOR("hongxu.zhao@mediatek.com");
