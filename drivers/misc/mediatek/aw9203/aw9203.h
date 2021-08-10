#ifndef _AW9203_H_
#define _AW9203_H_

/*********************************************************
 *
 * marco
 *
 ********************************************************/
#define MAX_I2C_BUFFER_SIZE 65536

#define AW9203_REG_MAX              0xff

#define AW_AUTO_CALI

#ifdef AW_AUTO_CALI
#define CALI_NUM                    10
#define CALI_RAW_MIN                300
#define CALI_RAW_MAX                3700
#endif

enum aw9203_flags {
	AW9203_FLAG_NONR = 0,
	AW9203_FLAG_SKIP_INTERRUPTS = 1,
};

/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct aw9203 {
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;
	struct work_struct eint_work;
	struct mutex lock;
	int reset_gpio;
	int irq_gpio;
	int aw9203_irq_t;
	int irq_is_disable;
	unsigned char flags;
	unsigned char cali_flag;
	unsigned char cali_num;
	unsigned char cali_cnt;
	unsigned char cali_used;
	unsigned char old_cali_dir[6];
	unsigned int old_ofr_cfg[3];
	unsigned long rawdata_sum[6];
};

#endif
