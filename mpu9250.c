// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>

#include <linux/i2c.h>
#include <linux/regmap.h>
#include "mpu9250.h"

/*Se define un major number arbitrario (estoy inventando) */
#define MY_MAJOR_NUM 202

/* Definicion de estructura cdev que representa internamente un chardevice  */
static struct cdev my_dev;

/* Puntero a estructura que representa un dispositivo esclavo
 * conectado al bus*/

static struct i2c_client *slaveDevice;

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

/* declaracion de una estructura del tipo file_operations */

static const struct file_operations my_dev_fops = {
	.owner = THIS_MODULE,
	.open = my_dev_open,
	.release = my_dev_close,
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

	pr_info("\tMPU9250 is alive\n");
	
	return(0);
}

static int mpu9250_i2c_remove(struct i2c_client *client)
{
	pr_info("Fin del mundo\n");
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
MODULE_DESCRIPTION("Este modulo interactua con el system call ioctl");
MODULE_INFO(mse_imd, "Esto no es para simples mortales");
