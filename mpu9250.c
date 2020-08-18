// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>

#include <linux/i2c.h>
#include <linux/regmap.h>

#include <linux/delay.h>
#include <linux/uaccess.h>

#include "mpu9250.h"

/*Se define un major number arbitrario (estoy inventando) */
#define MY_MAJOR_NUM 202

#define MPU9250_MAX_DATA_SIZE_16BITS	10 /* x,y,z for acc, gyro, mag + temp*/
#define MPU9250_ACC_DATA_START		0 
#define MPU9250_GYRO_DATA_START		3
#define MPU9250_MAG_DATA_START		6
#define MPU9250_TEMP_DATA_START		9

/* Definicion de estructura cdev que representa internamente un chardevice  */
static struct cdev my_dev;

/* Puntero a estructura que representa un dispositivo esclavo
 * conectado al bus*/

static struct i2c_client *slaveDevice;

static u8 Gscale = GFS_250DPS;
static u8 Ascale = AFS_2G;
static u8 Mscale = MFS_16BITS;
static u8 Mmode = M_100HZ;

static void readAccelData(int16_t * destination);
static void readGyroData(int16_t * destination);
static void readMagData(int16_t * destination);
static int16_t readTempData(void);

/*********************************************************************************
 * Definiciones de funciones sobre archivos y estructura correspondiente
 *********************************************************************************/


static int my_dev_open(struct inode *inode, struct file *file)  {
	pr_info("my_dev_open() fue invocada.\n");
	return 0;
}

static int my_dev_close(struct inode *inode, struct file *file)  {
	pr_info("my_dev_close() is called.\n");
	return 0;
}

static long my_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)  {
	pr_info("my_dev_ioctl() is called. cmd = %d, arg = %ld\n", cmd, arg);
	return 0;
}

static ssize_t my_dev_read(struct file *filep, char __user *buffer, size_t len, loff_t *offset)  {
	int16_t sensorData[MPU9250_MAX_DATA_SIZE_16BITS];
	memset(sensorData,0,MPU9250_MAX_DATA_SIZE_16BITS*2);
	readAccelData(&sensorData[MPU9250_ACC_DATA_START]);
	readGyroData(&sensorData[MPU9250_GYRO_DATA_START]);
	readMagData(&sensorData[MPU9250_MAG_DATA_START]);
	sensorData[MPU9250_TEMP_DATA_START] = readTempData();

	if((MPU9250_MAX_DATA_SIZE_16BITS*2) < len)
		len = MPU9250_MAX_DATA_SIZE_16BITS*2;

	if (copy_to_user(buffer,sensorData, len))
        	return -EFAULT;
	
	return len;
}

/* declaracion de una estructura del tipo file_operations */

static const struct file_operations my_dev_fops = {
	.owner = THIS_MODULE,
	.open = my_dev_open,
	.release = my_dev_close,
	.read = my_dev_read,
	.unlocked_ioctl = my_dev_ioctl,
};

/*----------------------------------------------------------------------*/
/**********************************************************************
 *  Driver MPU9250 - Low level
 **********************************************************************/
static int mpu9250_write_range(struct i2c_client *client, u8 addr, u8 *p, int cnt)
{
	struct i2c_msg msg;
	int ret;

	u8 buf[MESG_MAX_MSG_SIZE + 3];

	buf[0] = addr;
	memcpy(buf + 1, p, cnt);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = cnt + 1;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		pr_info("MPU9250 Error %d writing to register 0x%x\n", ret, addr);
	return ret < 0 ? ret : 0;
}

static void mpu9250_write(struct i2c_client *client, u8 addr, u8 val)
{
	mpu9250_write_range(client, addr, &val, 1);
}

static int mpu9250_read_range(struct i2c_client *client, u8 addr, u8 *p, int cnt)
{
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = cnt;
	msg[1].buf = p;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		pr_info("MPU9250 Error %d reading from register 0x%x\n", ret, addr);

	return ret;
}

static u8 mpu9250_read(struct i2c_client *client, u8 addr)
{
	int ret;
	u8 val;

	ret = mpu9250_read_range(client, addr, &val, 1);
	if (ret < 0)
		val = 0;

	return val;
}

/**********************************************************************
 *  Driver MPU9250 - high level
 **********************************************************************/

static int mpu9250_isAlive(struct i2c_client *client)
{
	int ret = -1;

	if (WHO_AM_I_RESPONSE == mpu9250_read(client, WHO_AM_I))
		ret = 0;

	return ret;	
}



/* Inicialización del modulo MPU9250 obtenido de:
   https://github.com/vedranMv/tm4c_mpu9250/blob/master/mpu9250/api_mpu9250.c
*/

/**
 * Configure MPU9250 accelerometer and gyroscope
 * Configures gyro for 1kHz sampling rate, 42Hz bandwidth and 200Hz output rate
 * and use full scale readings (+/- 250 dps). Configures accel for 1kHz sampling
 * rate, 200Hz output rate and full scale readings (+/- 16g). Finally, configure
 * interrupt pin to be active high, push-pull, held high until cleared and
 * cleared by reading ANY register. I2C bypass is disabled to allow for SPI.
 * Data-ready interrupts are only allowed.
 */
static int mpu9250_accel_gyro_init(struct i2c_client *client)
{
	//float gBias[3], aBias[3];
	u8 c;

	int ret = mpu9250_isAlive(client);

	if (ret < 0)
		return ret;

	//calibrateMPU9250(gBias, aBias);

	// wake up device
	// Clear sleep mode bit (6), enable all sensors
	mpu9250_write(client, PWR_MGMT_1, 0x00);
	msleep(100); // Wait for all registers to reset

	// Get stable time source
	// Auto select clock source to be PLL gyroscope reference if ready else
	mpu9250_write(client, PWR_MGMT_1, 0x01);
	msleep(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
	// respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion
	// update rates cannot be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
	// 8 kHz, or 1 kHz
	mpu9250_write(client, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	// Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above.
	mpu9250_write(client, SMPLRT_DIV, 0x00);

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
	// left-shifted into positions 4:3

	// get current GYRO_CONFIG register value
	c = mpu9250_read(client, GYRO_CONFIG);
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
	// GYRO_CONFIG
	c |= 0x03;
	// Write new GYRO_CONFIG value to register
	mpu9250_write(client, GYRO_CONFIG, c);

	// Set accelerometer full-scale range configuration
	// Get current ACCEL_CONFIG register value
	c = mpu9250_read(client, ACCEL_CONFIG);
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	// Write new ACCEL_CONFIG register value
	mpu9250_write(client, ACCEL_CONFIG, c);

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by
	// choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
	// 1.13 kHz
	// Get current ACCEL_CONFIG2 register value
	c = mpu9250_read(client, ACCEL_CONFIG2);
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	// Write new ACCEL_CONFIG2 register value
	mpu9250_write(client, ACCEL_CONFIG2, c);
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because
	// of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
	// until interrupt cleared, clear on read of any register, and DISABLE
	// I2C_BYPASS_EN -> otherwise communication with AK8963 doesn't work when
	//  using SPI
	mpu9250_write(client, INT_PIN_CFG, 0x30);
	// Enable data ready (bit 0) interrupt
	mpu9250_write(client, INT_ENABLE, 0x01);
	msleep(100);
	
	ret = 0;

	return ret;
}

/**
 * Initialize AK8963 magnetometer. 16bit data @ 100Hz
 * AK8963 is configured as a slave device of MPU9250 and not directly accessible.
 * Every transaction with AK8963 has to be carried through MPU9250 by configuring
 * its I2C slave communication registers. Result of the I2C transaction is also
 * stored internally in MPU and needs to be read as one would normally read
 * MPU registers.
 */
static int mpu9250_mag_init(struct i2c_client *client)
{
	int ret = mpu9250_isAlive(client);

	if (ret < 0)
		return ret;

	//  Initialization uses I2C channel number 4 for writing data

	//  Configure master I2C clock (400kHz) for MPU to talk to slaves
	mpu9250_write(client,  I2C_MST_CTRL, 0x5D);
	//  Enable I2C master
	mpu9250_write(client,  USER_CTRL, 0x20);

	//  Stop I2C slave number 4
	mpu9250_write(client,  I2C_SLV4_CTRL, 0x00);
	//  Set address for I2C4 slave to that of AK8963, writing mode (MSB=0)
	mpu9250_write(client,  I2C_SLV4_ADDR, AK8963_ADDRESS);
	//  Select which register is being updated
	mpu9250_write(client,  I2C_SLV4_REG, AK8963_CNTL);
	// Set value to write into the register:
	//      16-bit continuous measurements @ 100Hz
	mpu9250_write(client,  I2C_SLV4_DO, Mscale << 4 | Mmode);
	// Trigger write data to slave device 4 -> AK8963
	mpu9250_write(client,  I2C_SLV4_CTRL, 0x80);
	msleep(5);

	ret = 0;

	return ret;
}

/**
 * Read raw accelerometer data into a provided buffer
 * @param destination Buffer to save x, y, z acceleration data (min. size = 3)
 */
static void readAccelData(int16_t * destination)
{
	u8 rawData[6];  // x/y/z accel register data stored here
	// Read the six raw data registers into data array
	mpu9250_read_range(slaveDevice, ACCEL_XOUT_H, &rawData[0], 6);

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

/**
 * Read raw gyroscope data into a provided buffer
 * @param destination Buffer to save x, y, z gyroscope data (min. size = 3)
 */
static void readGyroData(int16_t * destination)
{
	u8 rawData[6];  // x/y/z gyro register data stored here
	// Read the six raw data registers sequentially into data array
	mpu9250_read_range(slaveDevice, GYRO_XOUT_H, &rawData[0], 6);

	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

/**
 * Read raw magnetometer data into a provided buffer
 * @param destination Buffer to save x, y, z magnetometer data (min. size = 3)
 */
static void readMagData(int16_t * destination)
{
	// x,y,z gyro register data, ST2 register stored here, must read ST2 at end
	// of data acquisition
	u8 rawData[7];
	u8 c;

	//  TODO: In case it's necessary to always have new data when calling this
	//  function, implement hanging function which reads Status1 register and
	//  checks for DRDY bit -> I noticed it doesn't always work (i.e. new data
	//  is available but bit is 0)

	//  Stop any ongoing I2C0 operations
	mpu9250_write(slaveDevice,  I2C_SLV0_CTRL, 0x00);
	// Set to read from slave address of AK8963
	mpu9250_write(slaveDevice,  I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);

	// Start reading from X_out, 7 bytes. This will read 6 data registers and
	//  one status register which will tell whether there was an overflow
	//  in measurements
	mpu9250_write(slaveDevice,  I2C_SLV0_REG, AK8963_XOUT_L);
	// Read 7 bytes from I2C slave 0
	mpu9250_write(slaveDevice,  I2C_SLV0_CTRL, 0x87);
	// Move 7 registers from MPU reg to here
	mpu9250_read_range(slaveDevice, EXT_SENS_DATA_00, rawData, 7);

	c = 0 & rawData[6]; // End data read by reading ST2 register
	// Check if magnetic sensor overflow is set, if not then report data
	if (!(c & 0x08)){
		// Turn the MSB and LSB into a signed 16-bit value
		destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
		// Data stored as little Endian
		destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
		destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	}
}

/**
 * Read data from internal temperature sensor
 * @return Temperature data
 */
static int16_t readTempData(void)
{
	u8 rawData[2]; // x/y/z gyro register data stored here
	// Read the two raw data registers sequentially into data array
	mpu9250_read_range(slaveDevice, TEMP_OUT_H, &rawData[0], 2);
	// Turn the MSB and LSB into a 16-bit value
	return ((int16_t)rawData[0] << 8) | rawData[1];
}

/**********************************************************************
 *  Sección de acceso a MPU9250 mediante I2C
 **********************************************************************/
/* Paso 1: Declarar IDs soportados por el driver (mediante of_device_id)*/

static const struct of_device_id mpu9250_of_match[] = {
	{ .compatible = "ale,mpu9250" },
	{ },
};

/* Paso 2: Invocar a MODULE_DEVICE_TABLE(of, my_of_match_table ) 
para exponer el dispositivo.*/

MODULE_DEVICE_TABLE(of, mpu9250_of_match);

/* Paso 3: 
+ Escribir las funciones probe() y remove(), teniendo en cuenta que el
comportamiento del driver se basa mucho en probe().
+ remove() debe deshacer todo lo que hizo probe().
+ Se utiliza probe() para registrar el device mediante el framework misc.
*/

static int mpu9250_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int ret;

	/* Obtenemos el primer identificador de dispositivo */
	dev_t dev = MKDEV(MY_MAJOR_NUM, 0);
	
	pr_info("Char driver init\n");

	/* Asignado de device numbers */

	ret = register_chrdev_region(dev, 1, "my_char_device");

	if (ret < 0){
		pr_info("Imposible asignar major number %d\n", MY_MAJOR_NUM);
		return ret;
	}

	/* Se inicializa la estructura cdev y se la agrega al espacio kernel */
	cdev_init(&my_dev, &my_dev_fops);
	ret= cdev_add(&my_dev, dev, 1);

	if (ret < 0){
		unregister_chrdev_region(dev, 1);
		pr_info("Unable to add cdev\n");
		return ret;
	}

	pr_info("Informacion de dispositivo conectado (struct i2c_client):\n");

	pr_info("\tDireccion: %#x\n",client->addr);
	pr_info("\tNombre: %s\n",client->name);

	pr_info("\tDriver: %s\n",(client->dev).driver->name);

	slaveDevice = client;	//guardamos este puntero para el uso de read y write

	pr_info("Checking device...\n");
		
	ret = mpu9250_isAlive(client);

	if (ret < 0){
		unregister_chrdev_region(dev, 1);
		pr_info("MPU9250 not response.\n");
		return ret;
	}

	ret = mpu9250_accel_gyro_init(client);

	if (ret < 0){
		unregister_chrdev_region(dev, 1);
		pr_info("Error during MPU9250 Accel and Gyro initialization.\n");
		return ret;
	}

	ret = mpu9250_mag_init(client);

	if (ret < 0){
		unregister_chrdev_region(dev, 1);
		pr_info("Error during MPU9250 Magnetometer initialization.\n");
		return ret;
	}
	pr_info("\tMPU9250 is alive and initialized.\n");
	
	return(0);
}

static int mpu9250_i2c_remove(struct i2c_client *client)
{
	pr_info("MPU9250 unloaded\n");
	cdev_del(&my_dev);
	unregister_chrdev_region(MKDEV(MY_MAJOR_NUM, 0), 1);
	return(0);
}

/* Paso 4: 
+ Declarar y llenar la estructura i2c_driver. Los campos .probe y .remove
deben ser asignados con las correspondientes funciones.
+ En la subestructura .driver el campo .owner debe ser seteado a
THIS_MODULE, también llenar el campo .name
+ En la subestructura .driver el campo .of_match_table se setea con el
array of_device_id.
*/

enum mpu9250_device_type {
	MPU9250,
};

static const struct i2c_device_id mpu9250_i2c_id[] = {
	{ "mpu9250", MPU9250 },
	{ }
};

static struct i2c_driver mpu9250_i2c_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name	= "mpu9250_i2c",
		.of_match_table = mpu9250_of_match,
	},
	.probe		= mpu9250_i2c_probe,
	.remove		= mpu9250_i2c_remove,
	.id_table	= mpu9250_i2c_id,
};

/* Paso 5: 
+ Utilizar la macro module_i2c_driver() para exponer el driver al kernel,
pasando como argumento la estructura i2c_driver declarada antes.
*/

module_i2c_driver(mpu9250_i2c_driver);


/**********************************************************************
 * Seccion sobre Informacion del modulo
 **********************************************************************/
MODULE_AUTHOR("Alejandro Permingeat <apermingeat@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Este modulo interactua con la IMPU MPU9250");
MODULE_INFO(mse_imd, "Esto no es para simples mortales");
