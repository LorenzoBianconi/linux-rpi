/*
 * STMicroelectronics accelerometers driver
 *
 * Copyright 2012-2013 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>

#include <linux/iio/common/st_sensors.h>
#include "st_accel.h"

#define ST_ACCEL_NUMBER_DATA_CHANNELS		3

/* DEFAULT VALUE FOR SENSORS */
#define ST_ACCEL_DEFAULT_OUT_X_L_ADDR		0x28
#define ST_ACCEL_DEFAULT_OUT_Y_L_ADDR		0x2a
#define ST_ACCEL_DEFAULT_OUT_Z_L_ADDR		0x2c

/* FULLSCALE */
#define ST_ACCEL_FS_AVL_2G			2
#define ST_ACCEL_FS_AVL_4G			4
#define ST_ACCEL_FS_AVL_6G			6
#define ST_ACCEL_FS_AVL_8G			8
#define ST_ACCEL_FS_AVL_16G			16
#define ST_ACCEL_FS_AVL_100G			100
#define ST_ACCEL_FS_AVL_200G			200
#define ST_ACCEL_FS_AVL_400G			400

/* CUSTOM VALUES FOR SENSOR 1 */
#define ST_ACCEL_1_WAI_EXP			0x33
#define ST_ACCEL_1_ODR_ADDR			0x20
#define ST_ACCEL_1_ODR_MASK			0xf0
#define ST_ACCEL_1_ODR_AVL_1HZ_VAL		0x01
#define ST_ACCEL_1_ODR_AVL_10HZ_VAL		0x02
#define ST_ACCEL_1_ODR_AVL_25HZ_VAL		0x03
#define ST_ACCEL_1_ODR_AVL_50HZ_VAL		0x04
#define ST_ACCEL_1_ODR_AVL_100HZ_VAL		0x05
#define ST_ACCEL_1_ODR_AVL_200HZ_VAL		0x06
#define ST_ACCEL_1_ODR_AVL_400HZ_VAL		0x07
#define ST_ACCEL_1_ODR_AVL_1600HZ_VAL		0x08
#define ST_ACCEL_1_FS_ADDR			0x23
#define ST_ACCEL_1_FS_MASK			0x30
#define ST_ACCEL_1_FS_AVL_2_VAL			0x00
#define ST_ACCEL_1_FS_AVL_4_VAL			0x01
#define ST_ACCEL_1_FS_AVL_8_VAL			0x02
#define ST_ACCEL_1_FS_AVL_16_VAL		0x03
#define ST_ACCEL_1_FS_AVL_2_GAIN		IIO_G_TO_M_S_2(1000)
#define ST_ACCEL_1_FS_AVL_4_GAIN		IIO_G_TO_M_S_2(2000)
#define ST_ACCEL_1_FS_AVL_8_GAIN		IIO_G_TO_M_S_2(4000)
#define ST_ACCEL_1_FS_AVL_16_GAIN		IIO_G_TO_M_S_2(12000)
#define ST_ACCEL_1_BDU_ADDR			0x23
#define ST_ACCEL_1_BDU_MASK			0x80
#define ST_ACCEL_1_DRDY_IRQ_ADDR		0x22
#define ST_ACCEL_1_DRDY_IRQ_INT1_MASK		0x10
#define ST_ACCEL_1_DRDY_IRQ_INT2_MASK		0x08
#define ST_ACCEL_1_IHL_IRQ_ADDR			0x25
#define ST_ACCEL_1_IHL_IRQ_MASK			0x02
#define ST_ACCEL_1_MULTIREAD_BIT		true

/* CUSTOM VALUES FOR SENSOR 2 */
#define ST_ACCEL_2_WAI_EXP			0x32
#define ST_ACCEL_2_ODR_ADDR			0x20
#define ST_ACCEL_2_ODR_MASK			0x18
#define ST_ACCEL_2_ODR_AVL_50HZ_VAL		0x00
#define ST_ACCEL_2_ODR_AVL_100HZ_VAL		0x01
#define ST_ACCEL_2_ODR_AVL_400HZ_VAL		0x02
#define ST_ACCEL_2_ODR_AVL_1000HZ_VAL		0x03
#define ST_ACCEL_2_PW_ADDR			0x20
#define ST_ACCEL_2_PW_MASK			0xe0
#define ST_ACCEL_2_FS_ADDR			0x23
#define ST_ACCEL_2_FS_MASK			0x30
#define ST_ACCEL_2_FS_AVL_2_VAL			0X00
#define ST_ACCEL_2_FS_AVL_4_VAL			0X01
#define ST_ACCEL_2_FS_AVL_8_VAL			0x03
#define ST_ACCEL_2_FS_AVL_2_GAIN		IIO_G_TO_M_S_2(1000)
#define ST_ACCEL_2_FS_AVL_4_GAIN		IIO_G_TO_M_S_2(2000)
#define ST_ACCEL_2_FS_AVL_8_GAIN		IIO_G_TO_M_S_2(3900)
#define ST_ACCEL_2_BDU_ADDR			0x23
#define ST_ACCEL_2_BDU_MASK			0x80
#define ST_ACCEL_2_DRDY_IRQ_ADDR		0x22
#define ST_ACCEL_2_DRDY_IRQ_INT1_MASK		0x02
#define ST_ACCEL_2_DRDY_IRQ_INT2_MASK		0x10
#define ST_ACCEL_2_IHL_IRQ_ADDR			0x22
#define ST_ACCEL_2_IHL_IRQ_MASK			0x80
#define ST_ACCEL_2_OD_IRQ_ADDR			0x22
#define ST_ACCEL_2_OD_IRQ_MASK			0x40
#define ST_ACCEL_2_MULTIREAD_BIT		true

/* CUSTOM VALUES FOR SENSOR 3 */
#define ST_ACCEL_3_WAI_EXP			0x40
#define ST_ACCEL_3_ODR_ADDR			0x20
#define ST_ACCEL_3_ODR_MASK			0xf0
#define ST_ACCEL_3_ODR_AVL_3HZ_VAL		0x01
#define ST_ACCEL_3_ODR_AVL_6HZ_VAL		0x02
#define ST_ACCEL_3_ODR_AVL_12HZ_VAL		0x03
#define ST_ACCEL_3_ODR_AVL_25HZ_VAL		0x04
#define ST_ACCEL_3_ODR_AVL_50HZ_VAL		0x05
#define ST_ACCEL_3_ODR_AVL_100HZ_VAL		0x06
#define ST_ACCEL_3_ODR_AVL_200HZ_VAL		0x07
#define ST_ACCEL_3_ODR_AVL_400HZ_VAL		0x08
#define ST_ACCEL_3_ODR_AVL_800HZ_VAL		0x09
#define ST_ACCEL_3_ODR_AVL_1600HZ_VAL		0x0a
#define ST_ACCEL_3_FS_ADDR			0x24
#define ST_ACCEL_3_FS_MASK			0x38
#define ST_ACCEL_3_FS_AVL_2_VAL			0X00
#define ST_ACCEL_3_FS_AVL_4_VAL			0X01
#define ST_ACCEL_3_FS_AVL_6_VAL			0x02
#define ST_ACCEL_3_FS_AVL_8_VAL			0x03
#define ST_ACCEL_3_FS_AVL_16_VAL		0x04
#define ST_ACCEL_3_FS_AVL_2_GAIN		IIO_G_TO_M_S_2(61)
#define ST_ACCEL_3_FS_AVL_4_GAIN		IIO_G_TO_M_S_2(122)
#define ST_ACCEL_3_FS_AVL_6_GAIN		IIO_G_TO_M_S_2(183)
#define ST_ACCEL_3_FS_AVL_8_GAIN		IIO_G_TO_M_S_2(244)
#define ST_ACCEL_3_FS_AVL_16_GAIN		IIO_G_TO_M_S_2(732)
#define ST_ACCEL_3_BDU_ADDR			0x20
#define ST_ACCEL_3_BDU_MASK			0x08
#define ST_ACCEL_3_DRDY_IRQ_ADDR		0x23
#define ST_ACCEL_3_DRDY_IRQ_INT1_MASK		0x80
#define ST_ACCEL_3_DRDY_IRQ_INT2_MASK		0x00
#define ST_ACCEL_3_IHL_IRQ_ADDR			0x23
#define ST_ACCEL_3_IHL_IRQ_MASK			0x40
#define ST_ACCEL_3_IG1_EN_ADDR			0x23
#define ST_ACCEL_3_IG1_EN_MASK			0x08
#define ST_ACCEL_3_MULTIREAD_BIT		false

/* CUSTOM VALUES FOR SENSOR 4 */
#define ST_ACCEL_4_WAI_EXP			0x3a
#define ST_ACCEL_4_ODR_ADDR			0x20
#define ST_ACCEL_4_ODR_MASK			0x30 /* DF1 and DF0 */
#define ST_ACCEL_4_ODR_AVL_40HZ_VAL		0x00
#define ST_ACCEL_4_ODR_AVL_160HZ_VAL		0x01
#define ST_ACCEL_4_ODR_AVL_640HZ_VAL		0x02
#define ST_ACCEL_4_ODR_AVL_2560HZ_VAL		0x03
#define ST_ACCEL_4_PW_ADDR			0x20
#define ST_ACCEL_4_PW_MASK			0xc0
#define ST_ACCEL_4_FS_ADDR			0x21
#define ST_ACCEL_4_FS_MASK			0x80
#define ST_ACCEL_4_FS_AVL_2_VAL			0X00
#define ST_ACCEL_4_FS_AVL_6_VAL			0X01
#define ST_ACCEL_4_FS_AVL_2_GAIN		IIO_G_TO_M_S_2(1000)
#define ST_ACCEL_4_FS_AVL_6_GAIN		IIO_G_TO_M_S_2(3000)
#define ST_ACCEL_4_BDU_ADDR			0x21
#define ST_ACCEL_4_BDU_MASK			0x40
#define ST_ACCEL_4_DRDY_IRQ_ADDR		0x21
#define ST_ACCEL_4_DRDY_IRQ_INT1_MASK		0x04
#define ST_ACCEL_4_MULTIREAD_BIT		true

/* CUSTOM VALUES FOR SENSOR 5 */
#define ST_ACCEL_5_WAI_EXP			0x3b
#define ST_ACCEL_5_ODR_ADDR			0x20
#define ST_ACCEL_5_ODR_MASK			0x80
#define ST_ACCEL_5_ODR_AVL_100HZ_VAL		0x00
#define ST_ACCEL_5_ODR_AVL_400HZ_VAL		0x01
#define ST_ACCEL_5_PW_ADDR			0x20
#define ST_ACCEL_5_PW_MASK			0x40
#define ST_ACCEL_5_FS_ADDR			0x20
#define ST_ACCEL_5_FS_MASK			0x20
#define ST_ACCEL_5_FS_AVL_2_VAL			0X00
#define ST_ACCEL_5_FS_AVL_8_VAL			0X01
/* TODO: check these resulting gain settings, these are not in the datsheet */
#define ST_ACCEL_5_FS_AVL_2_GAIN		IIO_G_TO_M_S_2(18000)
#define ST_ACCEL_5_FS_AVL_8_GAIN		IIO_G_TO_M_S_2(72000)
#define ST_ACCEL_5_DRDY_IRQ_ADDR		0x22
#define ST_ACCEL_5_DRDY_IRQ_INT1_MASK		0x04
#define ST_ACCEL_5_DRDY_IRQ_INT2_MASK		0x20
#define ST_ACCEL_5_IHL_IRQ_ADDR			0x22
#define ST_ACCEL_5_IHL_IRQ_MASK			0x80
#define ST_ACCEL_5_OD_IRQ_ADDR			0x22
#define ST_ACCEL_5_OD_IRQ_MASK			0x40
#define ST_ACCEL_5_IG1_EN_ADDR			0x21
#define ST_ACCEL_5_IG1_EN_MASK			0x08
#define ST_ACCEL_5_MULTIREAD_BIT		false

/* CUSTOM VALUES FOR SENSOR 6 */
#define ST_ACCEL_6_WAI_EXP			0x32
#define ST_ACCEL_6_ODR_ADDR			0x20
#define ST_ACCEL_6_ODR_MASK			0x18
#define ST_ACCEL_6_ODR_AVL_50HZ_VAL		0x00
#define ST_ACCEL_6_ODR_AVL_100HZ_VAL		0x01
#define ST_ACCEL_6_ODR_AVL_400HZ_VAL		0x02
#define ST_ACCEL_6_ODR_AVL_1000HZ_VAL		0x03
#define ST_ACCEL_6_PW_ADDR			0x20
#define ST_ACCEL_6_PW_MASK			0x20
#define ST_ACCEL_6_FS_ADDR			0x23
#define ST_ACCEL_6_FS_MASK			0x30
#define ST_ACCEL_6_FS_AVL_100_VAL		0x00
#define ST_ACCEL_6_FS_AVL_200_VAL		0x01
#define ST_ACCEL_6_FS_AVL_400_VAL		0x03
#define ST_ACCEL_6_FS_AVL_100_GAIN		IIO_G_TO_M_S_2(49000)
#define ST_ACCEL_6_FS_AVL_200_GAIN		IIO_G_TO_M_S_2(98000)
#define ST_ACCEL_6_FS_AVL_400_GAIN		IIO_G_TO_M_S_2(195000)
#define ST_ACCEL_6_BDU_ADDR			0x23
#define ST_ACCEL_6_BDU_MASK			0x80
#define ST_ACCEL_6_DRDY_IRQ_ADDR		0x22
#define ST_ACCEL_6_DRDY_IRQ_INT1_MASK		0x02
#define ST_ACCEL_6_DRDY_IRQ_INT2_MASK		0x10
#define ST_ACCEL_6_IHL_IRQ_ADDR			0x22
#define ST_ACCEL_6_IHL_IRQ_MASK			0x80
#define ST_ACCEL_6_MULTIREAD_BIT		true

/* CUSTOM VALUES FOR SENSOR 7 */
#define ST_ACCEL_7_ODR_ADDR			0x20
#define ST_ACCEL_7_ODR_MASK			0x30
#define ST_ACCEL_7_ODR_AVL_280HZ_VAL		0x00
#define ST_ACCEL_7_ODR_AVL_560HZ_VAL		0x01
#define ST_ACCEL_7_ODR_AVL_1120HZ_VAL		0x02
#define ST_ACCEL_7_ODR_AVL_4480HZ_VAL		0x03
#define ST_ACCEL_7_PW_ADDR			0x20
#define ST_ACCEL_7_PW_MASK			0xc0
#define ST_ACCEL_7_FS_AVL_2_GAIN		IIO_G_TO_M_S_2(488)
#define ST_ACCEL_7_BDU_ADDR			0x21
#define ST_ACCEL_7_BDU_MASK			0x40
#define ST_ACCEL_7_DRDY_IRQ_ADDR		0x21
#define ST_ACCEL_7_DRDY_IRQ_INT1_MASK		0x04
#define ST_ACCEL_7_MULTIREAD_BIT		false

static const struct iio_chan_spec st_accel_8bit_channels[] = {
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_X, 1, IIO_MOD_X, 's', IIO_LE, 8, 8,
			ST_ACCEL_DEFAULT_OUT_X_L_ADDR+1),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_Y, 1, IIO_MOD_Y, 's', IIO_LE, 8, 8,
			ST_ACCEL_DEFAULT_OUT_Y_L_ADDR+1),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_Z, 1, IIO_MOD_Z, 's', IIO_LE, 8, 8,
			ST_ACCEL_DEFAULT_OUT_Z_L_ADDR+1),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct iio_chan_spec st_accel_12bit_channels[] = {
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_X, 1, IIO_MOD_X, 's', IIO_LE, 12, 16,
			ST_ACCEL_DEFAULT_OUT_X_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_Y, 1, IIO_MOD_Y, 's', IIO_LE, 12, 16,
			ST_ACCEL_DEFAULT_OUT_Y_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_Z, 1, IIO_MOD_Z, 's', IIO_LE, 12, 16,
			ST_ACCEL_DEFAULT_OUT_Z_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct iio_chan_spec st_accel_16bit_channels[] = {
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_X, 1, IIO_MOD_X, 's', IIO_LE, 16, 16,
			ST_ACCEL_DEFAULT_OUT_X_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_Y, 1, IIO_MOD_Y, 's', IIO_LE, 16, 16,
			ST_ACCEL_DEFAULT_OUT_Y_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL,
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			ST_SENSORS_SCAN_Z, 1, IIO_MOD_Z, 's', IIO_LE, 16, 16,
			ST_ACCEL_DEFAULT_OUT_Z_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct st_sensor_settings st_accel_sensors_settings[] = {
	{
		.wai = ST_ACCEL_1_WAI_EXP,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = LIS3DH_ACCEL_DEV_NAME,
			[1] = LSM303DLHC_ACCEL_DEV_NAME,
			[2] = LSM330D_ACCEL_DEV_NAME,
			[3] = LSM330DL_ACCEL_DEV_NAME,
			[4] = LSM330DLC_ACCEL_DEV_NAME,
			[5] = LSM303AGR_ACCEL_DEV_NAME,
			[6] = LIS2DH12_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_12bit_channels,
		.odr = {
			.addr = ST_ACCEL_1_ODR_ADDR,
			.mask = ST_ACCEL_1_ODR_MASK,
			.odr_avl = {
				{ 1, ST_ACCEL_1_ODR_AVL_1HZ_VAL, },
				{ 10, ST_ACCEL_1_ODR_AVL_10HZ_VAL, },
				{ 25, ST_ACCEL_1_ODR_AVL_25HZ_VAL, },
				{ 50, ST_ACCEL_1_ODR_AVL_50HZ_VAL, },
				{ 100, ST_ACCEL_1_ODR_AVL_100HZ_VAL, },
				{ 200, ST_ACCEL_1_ODR_AVL_200HZ_VAL, },
				{ 400, ST_ACCEL_1_ODR_AVL_400HZ_VAL, },
				{ 1600, ST_ACCEL_1_ODR_AVL_1600HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_1_ODR_ADDR,
			.mask = ST_ACCEL_1_ODR_MASK,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = ST_ACCEL_1_FS_ADDR,
			.mask = ST_ACCEL_1_FS_MASK,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.value = ST_ACCEL_1_FS_AVL_2_VAL,
					.gain = ST_ACCEL_1_FS_AVL_2_GAIN,
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_4G,
					.value = ST_ACCEL_1_FS_AVL_4_VAL,
					.gain = ST_ACCEL_1_FS_AVL_4_GAIN,
				},
				[2] = {
					.num = ST_ACCEL_FS_AVL_8G,
					.value = ST_ACCEL_1_FS_AVL_8_VAL,
					.gain = ST_ACCEL_1_FS_AVL_8_GAIN,
				},
				[3] = {
					.num = ST_ACCEL_FS_AVL_16G,
					.value = ST_ACCEL_1_FS_AVL_16_VAL,
					.gain = ST_ACCEL_1_FS_AVL_16_GAIN,
				},
			},
		},
		.bdu = {
			.addr = ST_ACCEL_1_BDU_ADDR,
			.mask = ST_ACCEL_1_BDU_MASK,
		},
		.drdy_irq = {
			.int1 = {
				.addr = 0x22,
				.mask = 0x10,
			},
			.addr_ihl = ST_ACCEL_1_IHL_IRQ_ADDR,
			.mask_ihl = ST_ACCEL_1_IHL_IRQ_MASK,
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x07,
			},
		},
		.sim = {
			.addr = 0x23,
			.value = BIT(0),
		},
		.multi_read_bit = ST_ACCEL_1_MULTIREAD_BIT,
		.bootime = 2,
	},
	{
		.wai = ST_ACCEL_2_WAI_EXP,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = LIS331DLH_ACCEL_DEV_NAME,
			[1] = LSM303DL_ACCEL_DEV_NAME,
			[2] = LSM303DLH_ACCEL_DEV_NAME,
			[3] = LSM303DLM_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_12bit_channels,
		.odr = {
			.addr = ST_ACCEL_2_ODR_ADDR,
			.mask = ST_ACCEL_2_ODR_MASK,
			.odr_avl = {
				{ 50, ST_ACCEL_2_ODR_AVL_50HZ_VAL, },
				{ 100, ST_ACCEL_2_ODR_AVL_100HZ_VAL, },
				{ 400, ST_ACCEL_2_ODR_AVL_400HZ_VAL, },
				{ 1000, ST_ACCEL_2_ODR_AVL_1000HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_2_PW_ADDR,
			.mask = ST_ACCEL_2_PW_MASK,
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = ST_ACCEL_2_FS_ADDR,
			.mask = ST_ACCEL_2_FS_MASK,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.value = ST_ACCEL_2_FS_AVL_2_VAL,
					.gain = ST_ACCEL_2_FS_AVL_2_GAIN,
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_4G,
					.value = ST_ACCEL_2_FS_AVL_4_VAL,
					.gain = ST_ACCEL_2_FS_AVL_4_GAIN,
				},
				[2] = {
					.num = ST_ACCEL_FS_AVL_8G,
					.value = ST_ACCEL_2_FS_AVL_8_VAL,
					.gain = ST_ACCEL_2_FS_AVL_8_GAIN,
				},
			},
		},
		.bdu = {
			.addr = ST_ACCEL_2_BDU_ADDR,
			.mask = ST_ACCEL_2_BDU_MASK,
		},
		.drdy_irq = {
			.int1 = {
				.addr = 0x22,
				.mask = 0x02,
			},
			.int2 = {
				.addr = 0x22,
				.mask = 0x10,
			},
			.addr_ihl = ST_ACCEL_2_IHL_IRQ_ADDR,
			.mask_ihl = ST_ACCEL_2_IHL_IRQ_MASK,
			.addr_od = ST_ACCEL_2_OD_IRQ_ADDR,
			.mask_od = ST_ACCEL_2_OD_IRQ_MASK,
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x07,
			},
		},
		.sim = {
			.addr = 0x23,
			.value = BIT(0),
		},
		.multi_read_bit = ST_ACCEL_2_MULTIREAD_BIT,
		.bootime = 2,
	},
	{
		.wai = ST_ACCEL_3_WAI_EXP,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = LSM330_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_16bit_channels,
		.odr = {
			.addr = ST_ACCEL_3_ODR_ADDR,
			.mask = ST_ACCEL_3_ODR_MASK,
			.odr_avl = {
				{ 3, ST_ACCEL_3_ODR_AVL_3HZ_VAL },
				{ 6, ST_ACCEL_3_ODR_AVL_6HZ_VAL, },
				{ 12, ST_ACCEL_3_ODR_AVL_12HZ_VAL, },
				{ 25, ST_ACCEL_3_ODR_AVL_25HZ_VAL, },
				{ 50, ST_ACCEL_3_ODR_AVL_50HZ_VAL, },
				{ 100, ST_ACCEL_3_ODR_AVL_100HZ_VAL, },
				{ 200, ST_ACCEL_3_ODR_AVL_200HZ_VAL, },
				{ 400, ST_ACCEL_3_ODR_AVL_400HZ_VAL, },
				{ 800, ST_ACCEL_3_ODR_AVL_800HZ_VAL, },
				{ 1600, ST_ACCEL_3_ODR_AVL_1600HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_3_ODR_ADDR,
			.mask = ST_ACCEL_3_ODR_MASK,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = ST_ACCEL_3_FS_ADDR,
			.mask = ST_ACCEL_3_FS_MASK,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.value = ST_ACCEL_3_FS_AVL_2_VAL,
					.gain = ST_ACCEL_3_FS_AVL_2_GAIN,
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_4G,
					.value = ST_ACCEL_3_FS_AVL_4_VAL,
					.gain = ST_ACCEL_3_FS_AVL_4_GAIN,
				},
				[2] = {
					.num = ST_ACCEL_FS_AVL_6G,
					.value = ST_ACCEL_3_FS_AVL_6_VAL,
					.gain = ST_ACCEL_3_FS_AVL_6_GAIN,
				},
				[3] = {
					.num = ST_ACCEL_FS_AVL_8G,
					.value = ST_ACCEL_3_FS_AVL_8_VAL,
					.gain = ST_ACCEL_3_FS_AVL_8_GAIN,
				},
				[4] = {
					.num = ST_ACCEL_FS_AVL_16G,
					.value = ST_ACCEL_3_FS_AVL_16_VAL,
					.gain = ST_ACCEL_3_FS_AVL_16_GAIN,
				},
			},
		},
		.bdu = {
			.addr = ST_ACCEL_3_BDU_ADDR,
			.mask = ST_ACCEL_3_BDU_MASK,
		},
		.drdy_irq = {
			.int1 = {
				.addr = 0x23,
				.mask = 0x80,
			},
			.addr_ihl = ST_ACCEL_3_IHL_IRQ_ADDR,
			.mask_ihl = ST_ACCEL_3_IHL_IRQ_MASK,
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x07,
			},
			.ig1 = {
				.en_addr = ST_ACCEL_3_IG1_EN_ADDR,
				.en_mask = ST_ACCEL_3_IG1_EN_MASK,
			},
		},
		.sim = {
			.addr = 0x24,
			.value = BIT(0),
		},
		.multi_read_bit = ST_ACCEL_3_MULTIREAD_BIT,
		.bootime = 2,
	},
	{
		.wai = ST_ACCEL_4_WAI_EXP,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = LIS3LV02DL_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_12bit_channels,
		.odr = {
			.addr = ST_ACCEL_4_ODR_ADDR,
			.mask = ST_ACCEL_4_ODR_MASK,
			.odr_avl = {
				{ 40, ST_ACCEL_4_ODR_AVL_40HZ_VAL },
				{ 160, ST_ACCEL_4_ODR_AVL_160HZ_VAL, },
				{ 640, ST_ACCEL_4_ODR_AVL_640HZ_VAL, },
				{ 2560, ST_ACCEL_4_ODR_AVL_2560HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_4_PW_ADDR,
			.mask = ST_ACCEL_4_PW_MASK,
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = ST_ACCEL_4_FS_ADDR,
			.mask = ST_ACCEL_4_FS_MASK,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.value = ST_ACCEL_4_FS_AVL_2_VAL,
					.gain = ST_ACCEL_4_FS_AVL_2_GAIN,
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_6G,
					.value = ST_ACCEL_4_FS_AVL_6_VAL,
					.gain = ST_ACCEL_4_FS_AVL_6_GAIN,
				},
			},
		},
		.bdu = {
			.addr = ST_ACCEL_4_BDU_ADDR,
			.mask = ST_ACCEL_4_BDU_MASK,
		},
		.drdy_irq = {
			.int1 = {
				.addr = 0x21,
				.mask = 0x04,
			},
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x07,
			},
		},
		.sim = {
			.addr = 0x21,
			.value = BIT(1),
		},
		.multi_read_bit = ST_ACCEL_4_MULTIREAD_BIT,
		.bootime = 2, /* guess */
	},
	{
		.wai = ST_ACCEL_5_WAI_EXP,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = LIS331DL_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_8bit_channels,
		.odr = {
			.addr = ST_ACCEL_5_ODR_ADDR,
			.mask = ST_ACCEL_5_ODR_MASK,
			.odr_avl = {
				{ 100, ST_ACCEL_5_ODR_AVL_100HZ_VAL },
				{ 400, ST_ACCEL_5_ODR_AVL_400HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_5_PW_ADDR,
			.mask = ST_ACCEL_5_PW_MASK,
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = ST_ACCEL_5_FS_ADDR,
			.mask = ST_ACCEL_5_FS_MASK,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.value = ST_ACCEL_5_FS_AVL_2_VAL,
					.gain = ST_ACCEL_5_FS_AVL_2_GAIN,
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_8G,
					.value = ST_ACCEL_5_FS_AVL_8_VAL,
					.gain = ST_ACCEL_5_FS_AVL_8_GAIN,
				},
			},
		},
		.drdy_irq = {
			.int1 = {
				.addr = 0x22,
				.mask = 0x04,
			},
			.int2 = {
				.addr = 0x22,
				.mask = 0x20,
			},
			.addr_ihl = ST_ACCEL_5_IHL_IRQ_ADDR,
			.mask_ihl = ST_ACCEL_5_IHL_IRQ_MASK,
			.addr_od = ST_ACCEL_5_OD_IRQ_ADDR,
			.mask_od = ST_ACCEL_5_OD_IRQ_MASK,
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x07,
			},
		},
		.sim = {
			.addr = 0x21,
			.value = BIT(7),
		},
		.multi_read_bit = ST_ACCEL_5_MULTIREAD_BIT,
		.bootime = 2, /* guess */
	},
	{
		.wai = ST_ACCEL_6_WAI_EXP,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = H3LIS331DL_DRIVER_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_12bit_channels,
		.odr = {
			.addr = ST_ACCEL_6_ODR_ADDR,
			.mask = ST_ACCEL_6_ODR_MASK,
			.odr_avl = {
				{ 50, ST_ACCEL_6_ODR_AVL_50HZ_VAL },
				{ 100, ST_ACCEL_6_ODR_AVL_100HZ_VAL, },
				{ 400, ST_ACCEL_6_ODR_AVL_400HZ_VAL, },
				{ 1000, ST_ACCEL_6_ODR_AVL_1000HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_6_PW_ADDR,
			.mask = ST_ACCEL_6_PW_MASK,
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = ST_ACCEL_6_FS_ADDR,
			.mask = ST_ACCEL_6_FS_MASK,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_100G,
					.value = ST_ACCEL_6_FS_AVL_100_VAL,
					.gain = ST_ACCEL_6_FS_AVL_100_GAIN,
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_200G,
					.value = ST_ACCEL_6_FS_AVL_200_VAL,
					.gain = ST_ACCEL_6_FS_AVL_200_GAIN,
				},
				[2] = {
					.num = ST_ACCEL_FS_AVL_400G,
					.value = ST_ACCEL_6_FS_AVL_400_VAL,
					.gain = ST_ACCEL_6_FS_AVL_400_GAIN,
				},
			},
		},
		.bdu = {
			.addr = ST_ACCEL_6_BDU_ADDR,
			.mask = ST_ACCEL_6_BDU_MASK,
		},
		.drdy_irq = {
			.int1 = {
				.addr = 0x22,
				.mask = 0x02,
			},
			.int2 = {
				.addr = 0x22,
				.mask = 0x10,
			},
			.addr_ihl = ST_ACCEL_6_IHL_IRQ_ADDR,
			.mask_ihl = ST_ACCEL_6_IHL_IRQ_MASK,
		},
		.sim = {
			.addr = 0x23,
			.value = BIT(0),
		},
		.multi_read_bit = ST_ACCEL_6_MULTIREAD_BIT,
		.bootime = 2,
	},
	{
		/* No WAI register present */
		.sensors_supported = {
			[0] = LIS3L02DQ_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_12bit_channels,
		.odr = {
			.addr = ST_ACCEL_7_ODR_ADDR,
			.mask = ST_ACCEL_7_ODR_MASK,
			.odr_avl = {
				{ 280, ST_ACCEL_7_ODR_AVL_280HZ_VAL, },
				{ 560, ST_ACCEL_7_ODR_AVL_560HZ_VAL, },
				{ 1120, ST_ACCEL_7_ODR_AVL_1120HZ_VAL, },
				{ 4480, ST_ACCEL_7_ODR_AVL_4480HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_7_PW_ADDR,
			.mask = ST_ACCEL_7_PW_MASK,
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.gain = ST_ACCEL_7_FS_AVL_2_GAIN,
				},
			},
		},
		/*
		 * The part has a BDU bit but if set the data is never
		 * updated so don't set it.
		 */
		.bdu = {
		},
		.drdy_irq = {
			.int1 = {
				.addr = 0x21,
				.mask = 0x04,
			},
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x07,
			},
		},
		.sim = {
			.addr = 0x21,
			.value = BIT(1),
		},
		.multi_read_bit = ST_ACCEL_7_MULTIREAD_BIT,
		.bootime = 2,
	},
	{
		.wai = 0x44,
		.wai_addr = ST_SENSORS_DEFAULT_WAI_ADDRESS,
		.sensors_supported = {
			[0] = LIS2DW12_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_12bit_channels,
		.odr = {
			.addr = 0x20,
			.mask = 0xf0,
			.odr_avl = {
				{ .hz = 1, .value = 0x01, },
				{ .hz = 12, .value = 0x02, },
				{ .hz = 25, .value = 0x03, },
				{ .hz = 50, .value = 0x04, },
				{ .hz = 100, .value = 0x05, },
				{ .hz = 200, .value = 0x06, },
			},
		},
		.pw = {
			.addr = 0x20,
			.mask = 0xf0,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.fs = {
			.addr = 0x25,
			.mask = 0x30,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.value = 0x00,
					.gain = IIO_G_TO_M_S_2(976),
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_4G,
					.value = 0x01,
					.gain = IIO_G_TO_M_S_2(1952),
				},
				[2] = {
					.num = ST_ACCEL_FS_AVL_8G,
					.value = 0x02,
					.gain = IIO_G_TO_M_S_2(3904),
				},
				[3] = {
					.num = ST_ACCEL_FS_AVL_16G,
					.value = 0x03,
					.gain = IIO_G_TO_M_S_2(7808),
				},
			},
		},
		.bdu = {
			.addr = 0x21,
			.mask = 0x08,
		},
		.drdy_irq = {
			.int1 = {
				.addr = 0x23,
				.mask = 0x01,
			},
			.int2 = {
				.addr = 0x24,
				.mask = 0x01,
			},
			.addr_ihl = 0x22,
			.mask_ihl = 0x08,
			.addr_od = 0x22,
			.mask_od = 0x20,
			.stat_drdy = {
				.addr = ST_SENSORS_DEFAULT_STAT_ADDR,
				.mask = 0x01,
			},
		},
		.sim = {
			.addr = 0x21,
			.value = BIT(0),
		},
		.multi_read_bit = false,
		.bootime = 2,
	},
};

static int st_accel_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int *val,
							int *val2, long mask)
{
	int err;
	struct st_sensor_data *adata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		err = st_sensors_read_info_raw(indio_dev, ch, val);
		if (err < 0)
			goto read_error;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = adata->current_fullscale->gain / 1000000;
		*val2 = adata->current_fullscale->gain % 1000000;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = adata->odr;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

read_error:
	return err;
}

static int st_accel_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE: {
		int gain;

		gain = val * 1000000 + val2;
		err = st_sensors_set_fullscale_by_gain(indio_dev, gain);
		break;
	}
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val2)
			return -EINVAL;
		mutex_lock(&indio_dev->mlock);
		err = st_sensors_set_odr(indio_dev, val);
		mutex_unlock(&indio_dev->mlock);
		return err;
	default:
		return -EINVAL;
	}

	return err;
}

static ST_SENSORS_DEV_ATTR_SAMP_FREQ_AVAIL();
static ST_SENSORS_DEV_ATTR_SCALE_AVAIL(in_accel_scale_available);

static struct attribute *st_accel_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_accel_attribute_group = {
	.attrs = st_accel_attributes,
};

static const struct iio_info accel_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_accel_attribute_group,
	.read_raw = &st_accel_read_raw,
	.write_raw = &st_accel_write_raw,
	.debugfs_reg_access = &st_sensors_debugfs_reg_access,
};

#ifdef CONFIG_IIO_TRIGGER
static const struct iio_trigger_ops st_accel_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = ST_ACCEL_TRIGGER_SET_STATE,
	.validate_device = st_sensors_validate_device,
};
#define ST_ACCEL_TRIGGER_OPS (&st_accel_trigger_ops)
#else
#define ST_ACCEL_TRIGGER_OPS NULL
#endif

int st_accel_common_probe(struct iio_dev *indio_dev)
{
	struct st_sensor_data *adata = iio_priv(indio_dev);
	int irq = adata->get_irq_data_ready(indio_dev);
	int err;

	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &accel_info;
	mutex_init(&adata->tb.buf_lock);

	err = st_sensors_power_enable(indio_dev);
	if (err)
		return err;

	err = st_sensors_check_device_support(indio_dev,
					ARRAY_SIZE(st_accel_sensors_settings),
					st_accel_sensors_settings);
	if (err < 0)
		goto st_accel_power_off;

	adata->num_data_channels = ST_ACCEL_NUMBER_DATA_CHANNELS;
	adata->multiread_bit = adata->sensor_settings->multi_read_bit;
	indio_dev->channels = adata->sensor_settings->ch;
	indio_dev->num_channels = ST_SENSORS_NUMBER_ALL_CHANNELS;

	adata->current_fullscale = (struct st_sensor_fullscale_avl *)
					&adata->sensor_settings->fs.fs_avl[0];
	adata->odr = adata->sensor_settings->odr.odr_avl[0].hz;

	if (!adata->dev->platform_data)
		adata->dev->platform_data =
			(struct st_sensors_platform_data *)&default_accel_pdata;

	err = st_sensors_init_sensor(indio_dev, adata->dev->platform_data);
	if (err < 0)
		goto st_accel_power_off;

	err = st_accel_allocate_ring(indio_dev);
	if (err < 0)
		goto st_accel_power_off;

	if (irq > 0) {
		err = st_sensors_allocate_trigger(indio_dev,
						 ST_ACCEL_TRIGGER_OPS);
		if (err < 0)
			goto st_accel_probe_trigger_error;
	}

	err = iio_device_register(indio_dev);
	if (err)
		goto st_accel_device_register_error;

	dev_info(&indio_dev->dev, "registered accelerometer %s\n",
		 indio_dev->name);

	return 0;

st_accel_device_register_error:
	if (irq > 0)
		st_sensors_deallocate_trigger(indio_dev);
st_accel_probe_trigger_error:
	st_accel_deallocate_ring(indio_dev);
st_accel_power_off:
	st_sensors_power_disable(indio_dev);

	return err;
}
EXPORT_SYMBOL(st_accel_common_probe);

void st_accel_common_remove(struct iio_dev *indio_dev)
{
	struct st_sensor_data *adata = iio_priv(indio_dev);

	st_sensors_power_disable(indio_dev);

	iio_device_unregister(indio_dev);
	if (adata->get_irq_data_ready(indio_dev) > 0)
		st_sensors_deallocate_trigger(indio_dev);

	st_accel_deallocate_ring(indio_dev);
}
EXPORT_SYMBOL(st_accel_common_remove);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics accelerometers driver");
MODULE_LICENSE("GPL v2");
