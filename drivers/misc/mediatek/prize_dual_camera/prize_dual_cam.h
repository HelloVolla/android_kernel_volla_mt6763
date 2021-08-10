#ifndef __PRIZE_DUAL_CAM__
#define __PRIZE_DUAL_CAM__

#define CAMERA_DEBUG
#ifdef CAMERA_DEBUG

#define CAMERA_DBG(fmt,arg...) \
	do{\
		printk("<<SPC-DBG>>[%d]"fmt"\n", __LINE__, ##arg);\
    }while(0)
#else
#define CAMERA_DBG(fmt,arg...)
#endif

enum SENSOR_TYPE {
	SENSOR_TYPE_0310 = 0,
	SENSOR_TYPE_032A,
	SENSOR_TYPE_UNKNOWN,
};

struct sensor_info_t {
	enum SENSOR_TYPE sensor_type;
	unsigned int sensor_id;
	int (*open)(struct i2c_client *);
	void (*init)(struct i2c_client *);
	void (*stream_on)(struct i2c_client *);
	unsigned short (*get_shutter)(struct i2c_client *);
	unsigned int (*get_sensor_id)(struct i2c_client *,unsigned int *id);
	int (*set_power)(struct i2c_client *,unsigned int enable);
};

struct spc_data_t{
	char name[8];
	char is_enabled;
	enum SENSOR_TYPE sensor_type;
	unsigned int pdn_pin;
	struct sensor_info_t *ops;
	struct mutex ops_mutex;
};

#endif