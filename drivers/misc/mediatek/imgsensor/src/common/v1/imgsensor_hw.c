/*
 * Copyright (C) 2017 MediaTek Inc.
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

#include "imgsensor_sensor.h"

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/string.h>

#include "kd_camera_typedef.h"
#include "kd_camera_feature.h"


#include "imgsensor_hw.h"
//prize-modify-pengzhipeng-20190918-start
#ifdef MIPI_SWITCH
extern struct pinctrl_state *ppinctrl_cam_mipi_sel_h;
extern struct pinctrl_state *ppinctrl_cam_mipi_sel_l;
extern struct pinctrl		 *ppinctrl_cam;
extern struct pinctrl_state *ppinctrl_cam_mipi_en_h;
extern struct pinctrl_state *ppinctrl_cam_mipi_en_l;
#endif
int curr_sensor_id =0;
//prize-modify-pengzhipeng-20190918-end
enum IMGSENSOR_RETURN imgsensor_hw_release_all(struct IMGSENSOR_HW *phw)
{
	int i;

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (phw->pdev[i]->release != NULL)
			(phw->pdev[i]->release)(phw->pdev[i]->pinstance);
	}
	return IMGSENSOR_RETURN_SUCCESS;
}
enum IMGSENSOR_RETURN imgsensor_hw_init(struct IMGSENSOR_HW *phw)
{
	struct IMGSENSOR_HW_SENSOR_POWER      *psensor_pwr;
	struct IMGSENSOR_HW_CFG               *pcust_pwr_cfg;
	struct IMGSENSOR_HW_CUSTOM_POWER_INFO *ppwr_info;
	int i, j;
	char str_prop_name[LENGTH_FOR_SNPRINTF];
	struct device_node *of_node
		= of_find_compatible_node(NULL, NULL, "mediatek,camera_hw");

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (hw_open[i] != NULL)
			(hw_open[i])(&phw->pdev[i]);

		if (phw->pdev[i]->init != NULL)
			(phw->pdev[i]->init)(phw->pdev[i]->pinstance);
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		psensor_pwr = &phw->sensor_pwr[i];

		pcust_pwr_cfg = imgsensor_custom_config;
		while (pcust_pwr_cfg->sensor_idx != i)
			pcust_pwr_cfg++;

		if (pcust_pwr_cfg->sensor_idx == IMGSENSOR_SENSOR_IDX_NONE)
			continue;

		ppwr_info = pcust_pwr_cfg->pwr_info;
		while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE) {
			for (j = 0; j < IMGSENSOR_HW_ID_MAX_NUM; j++)
				if (ppwr_info->id == phw->pdev[j]->id)
					break;

			psensor_pwr->id[ppwr_info->pin] = j;
			ppwr_info++;
		}
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		memset(str_prop_name, 0, sizeof(str_prop_name));
		snprintf(str_prop_name,
					sizeof(str_prop_name),
					"cam%d_%s",
					i,
					"enable_sensor");
		if (of_property_read_string(
			of_node,
			str_prop_name,
			&phw->enable_sensor_by_index[i]) < 0) {
			pr_info("Property cust-sensor not defined\n");
			phw->enable_sensor_by_index[i] = NULL;
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN imgsensor_hw_power_sequence(
	struct IMGSENSOR_HW             *phw,
	enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
	enum   IMGSENSOR_HW_POWER_STATUS pwr_status,
	struct IMGSENSOR_HW_POWER_SEQ   *ppower_sequence,
	char *pcurr_idx)
{
	struct IMGSENSOR_HW_SENSOR_POWER *psensor_pwr =
	    &phw->sensor_pwr[sensor_idx];

	struct IMGSENSOR_HW_POWER_SEQ    *ppwr_seq = ppower_sequence;
	struct IMGSENSOR_HW_POWER_INFO   *ppwr_info;
	struct IMGSENSOR_HW_DEVICE       *pdev;
	int                               pin_cnt = 0;

	while (ppwr_seq < ppower_sequence + IMGSENSOR_HW_SENSOR_MAX_NUM &&
		ppwr_seq->name != NULL) {
		if (!strcmp(ppwr_seq->name, PLATFORM_POWER_SEQ_NAME)) {
			if (sensor_idx == ppwr_seq->_idx)
				break;
		} else {
			if (!strcmp(ppwr_seq->name, pcurr_idx))
				break;
		}
		ppwr_seq++;
	}
       curr_sensor_id = (int)sensor_idx;
	if (ppwr_seq->name == NULL)
		return IMGSENSOR_RETURN_ERROR;

	ppwr_info = ppwr_seq->pwr_info;

	while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE &&
		ppwr_info < ppwr_seq->pwr_info + IMGSENSOR_HW_POWER_INFO_MAX) {

		if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON &&
		   ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
			pdev = phw->pdev[psensor_pwr->id[ppwr_info->pin]];
		/*pr_debug(
		 *  "sensor_idx = %d, pin=%d, pin_state_on=%d, hw_id =%d\n",
		 *  sensor_idx,
		 *  ppwr_info->pin,
		 *  ppwr_info->pin_state_on,
		 * psensor_pwr->id[ppwr_info->pin]);
		 */

			if (pdev->set != NULL)
				pdev->set(
				    pdev->pinstance,
				    sensor_idx,
				    ppwr_info->pin,
				    ppwr_info->pin_state_on);

			mdelay(ppwr_info->pin_on_delay);
		}

		ppwr_info++;
		pin_cnt++;
	}

	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_OFF) {
		while (pin_cnt) {
			ppwr_info--;
			pin_cnt--;

			if (ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
				pdev =
				    phw->pdev[psensor_pwr->id[ppwr_info->pin]];
				mdelay(ppwr_info->pin_on_delay);

				if (pdev->set != NULL)
					pdev->set(
					    pdev->pinstance,
					    sensor_idx,
					    ppwr_info->pin,
					    ppwr_info->pin_state_off);
			}
		}
	}

	/* wait for power stable */
	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
		mdelay(5);
	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_hw_power(
	struct IMGSENSOR_HW     *phw,
	struct IMGSENSOR_SENSOR *psensor,
	char *curr_sensor_name,
	enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	enum IMGSENSOR_SENSOR_IDX sensor_idx = psensor->inst.sensor_idx;
	char str_index[LENGTH_FOR_SNPRINTF];

	pr_info(
		"sensor_idx %d, power %d curr_sensor_name %s, enable list %s\n",
		sensor_idx,
		pwr_status,
		curr_sensor_name,
		phw->enable_sensor_by_index[sensor_idx] == NULL
		? "NULL"
		: phw->enable_sensor_by_index[sensor_idx]);

	if (phw->enable_sensor_by_index[sensor_idx] &&
	!strstr(phw->enable_sensor_by_index[sensor_idx], curr_sensor_name))
		return IMGSENSOR_RETURN_ERROR;


	snprintf(str_index, sizeof(str_index), "%d", sensor_idx);
	//prize-modify-pengzhipeng-20190918-start
#ifdef MIPI_SWITCH
     if(pwr_status == IMGSENSOR_HW_POWER_STATUS_ON)
	{
		if (!IS_ERR(ppinctrl_cam_mipi_en_l))
		{
			pinctrl_select_state(ppinctrl_cam, ppinctrl_cam_mipi_en_l);
		}
		else
		{
			printk("ppinctrl_cam_mipi_sel_state h pinctrl err\n");
		}
		mdelay(5);	
	}
     else if(pwr_status == IMGSENSOR_HW_POWER_STATUS_OFF)
	{
		if (!IS_ERR(ppinctrl_cam_mipi_en_h))
		{
			pinctrl_select_state(ppinctrl_cam, ppinctrl_cam_mipi_en_h);
		}
		else
		{
			printk("ppinctrl_cam_mipi_sel_state l pinctrl err\n");
		}
	}

	if(pwr_status == IMGSENSOR_HW_POWER_STATUS_ON && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN)
	{
		if (!IS_ERR(ppinctrl_cam_mipi_sel_l))
		{
			pinctrl_select_state(ppinctrl_cam, ppinctrl_cam_mipi_sel_l);
		}
		else
		{
			printk("ppinctrl_cam_mipi_sel_state l pinctrl err\n");
		}
		mdelay(5);
	}
	else if(pwr_status == IMGSENSOR_HW_POWER_STATUS_ON && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN2)
	{
		if (!IS_ERR(ppinctrl_cam_mipi_sel_h))
		{
			pinctrl_select_state(ppinctrl_cam, ppinctrl_cam_mipi_sel_h);
		}
		else
		{
			printk("ppinctrl_cam_mipi_sel_state h pinctrl err\n");
		}
		mdelay(5);
	}
	else if(pwr_status == IMGSENSOR_HW_POWER_STATUS_OFF && sensor_idx == IMGSENSOR_SENSOR_IDX_MAIN)
	{
		if (!IS_ERR(ppinctrl_cam_mipi_sel_l))
		{
			pinctrl_select_state(ppinctrl_cam, ppinctrl_cam_mipi_sel_l);
		}
		else
		{
			printk("ppinctrl_cam_mipi_sel_state l pinctrl err\n");
		}
	}
#endif
//prize-modify-pengzhipeng-20190918-end
	imgsensor_hw_power_sequence(
	    phw,
	    sensor_idx,
	    pwr_status,
	    platform_power_sequence,
	    str_index);

	imgsensor_hw_power_sequence(
	    phw,
	    sensor_idx,
	    pwr_status,
	    sensor_power_sequence,
	    curr_sensor_name);

	return IMGSENSOR_RETURN_SUCCESS;
}

