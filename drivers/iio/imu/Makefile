#
# Makefile for Inertial Measurement Units
#

# When adding new entries keep the list in alphabetical order
adis16400-y             := adis16400_core.o
adis16400-$(CONFIG_IIO_BUFFER) += adis16400_buffer.o
obj-$(CONFIG_ADIS16400) += adis16400.o
obj-$(CONFIG_ADIS16480) += adis16480.o

adis_lib-y += adis.o
adis_lib-$(CONFIG_IIO_ADIS_LIB_BUFFER) += adis_trigger.o
adis_lib-$(CONFIG_IIO_ADIS_LIB_BUFFER) += adis_buffer.o
obj-$(CONFIG_IIO_ADIS_LIB) += adis_lib.o

obj-y += inv_mpu6050/

obj-$(CONFIG_KMX61) += kmx61.o

st_lsm6dsx-y := st_lsm6dsx_core.o st_lsm6dsx_ring.o

obj-$(CONFIG_IIO_ST_LSM6DSX) += st_lsm6dsx.o
obj-$(CONFIG_IIO_ST_LSM6DSX_I2C) += st_lsm6dsx_i2c.o
obj-$(CONFIG_IIO_ST_LSM6DSX_SPI) += st_lsm6dsx_spi.o
