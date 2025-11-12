#include <linux/module.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/kmod.h>	
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include "mpu6050_drv.h"


// static int16_t gyro_16_data[3];
// static int16_t accer_16_data[3];
static int16_t mpu6050data[12];

struct mpu6050_dev{
	int major;
	struct class *mpu6050_class;
    struct device_node *nd;
	struct i2c_client *mpu6050_client;
};


struct mpu6050_dev mpu6050cdev = {
    .major = 0,
    .mpu6050_class = NULL,
    .nd = NULL,
	.mpu6050_client = NULL,
};
//3.实现相应的write等函数

static int mpu6050_read_regs(struct mpu6050_dev *dev,u8 reg,void *val,int len)
{
	struct i2c_client *client = mpu6050cdev.mpu6050_client;
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;  //从机地址
	msg[0].flags = 0;           //写标志
	msg[0].buf = &reg;          //写数据地址，这里是要访问的寄存器地址
	msg[0].len = 1;				//写数据的长度

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;           //读标志
	msg[1].buf = val;
	msg[1].len = len;
	return i2c_transfer(client -> adapter, msg, 2);

}
static int mpu6050_write_regs(struct mpu6050_dev *dev, u8 reg, u8 *buf, u8 len)
{
	u8 b[256];
	struct i2c_msg msg;
	struct i2c_client *client = mpu6050cdev.mpu6050_client;
	
	b[0] = reg;					/* 寄存器首地址 */
	memcpy(&b[1],buf,len);		/* 将要写入的数据拷贝到数组b里面 */
		
	msg.addr = client->addr;	/* mpu6050i2c地址 */
	msg.flags = 0;				/* 标记为写数据 */
	msg.buf = b;				/* 要写入的数据缓冲区 */
	msg.len = len + 1;			/* 要写入的数据长度 */

	return i2c_transfer(client->adapter, &msg, 1);
}

static ssize_t mpu6050_write (struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
		
		return 0;
}


static int16_t MPU6050_Byte_to_HalfWord(uint8_t DataL, uint8_t DataH)
{
    int16_t Data;
    
    Data = (DataH << 8) | DataL;  //DataL放到低8位， DataH放到高8位
    

	
    return Data;                                //返回转换后的数据
}

static ssize_t mpu6050_read (struct file *file, char __user *buf, size_t size, loff_t *offset)
{
	int err;
	u8 accerdata[6];
	u8 gyrodata[6];
	
	mpu6050_read_regs(&mpu6050cdev,MPU6050_RA_ACCEL_XOUT_H,accerdata,6);
	mpu6050data[0] = MPU6050_Byte_to_HalfWord(accerdata[1],accerdata[0]);
	mpu6050data[1] = MPU6050_Byte_to_HalfWord(accerdata[3],accerdata[2]);
	mpu6050data[2] = MPU6050_Byte_to_HalfWord(accerdata[5],accerdata[4]);
	mpu6050_read_regs(&mpu6050cdev,MPU6050_RA_GYRO_XOUT_H,gyrodata,6);
	mpu6050data[3] = MPU6050_Byte_to_HalfWord(gyrodata[1],gyrodata[0]);
	mpu6050data[4] = MPU6050_Byte_to_HalfWord(gyrodata[3],gyrodata[2]);
	mpu6050data[5] = MPU6050_Byte_to_HalfWord(gyrodata[5],gyrodata[4]);
	
	err = copy_to_user(buf,mpu6050data,12);
	return 0;

}
static int mpu6050_open (struct inode *node, struct file *file)
{
		printk("%s %s line %d\n",__FILE__,__FUNCTION__,__LINE__);
		return 0;
}

static int mpu6050_release (struct inode *node, struct file *file)
{
		printk("%s %s line %d\n",__FILE__,__FUNCTION__,__LINE__);
		return 0;
}


//2.定义自己的file_operations结构体
static struct file_operations mpu6050_fops = {
		.owner   = THIS_MODULE,
		.open    = mpu6050_open,
		.write   = mpu6050_write,
		.read    = mpu6050_read,
		.release = mpu6050_release,

};
static int cdevmpu_init(struct i2c_client *client){

	int err;   
	int ret;
	u8 reg_buf;
	printk("the i2c dev init\r\n");
	mpu6050cdev.major = register_chrdev(0,CDEV_NAME,&mpu6050_fops);  //注册设备文件结构体   声明设备名称
                                 
        if (mpu6050cdev.major < 0||mpu6050cdev.major == 0){
				printk("register field");
				return -EIO;
		}
	
	mpu6050cdev.mpu6050_class = class_create(THIS_MODULE, "mpu6050_class");//创建设备类
	err = PTR_ERR(mpu6050cdev.mpu6050_class);
	if (IS_ERR(mpu6050cdev.mpu6050_class)){
			unregister_chrdev(mpu6050cdev.major,CDEV_NAME);
		 	printk("%s %s line %d\n",__FILE__,__FUNCTION__,__LINE__);
			return -1;
			}
	device_create(mpu6050cdev.mpu6050_class,NULL,MKDEV(mpu6050cdev.major, 0),NULL,CDEV_NAME);
	printk("the i2c dev create\r\n");

	//初始化mpu
    reg_buf = 0x00;
	ret = mpu6050_write_regs(&mpu6050cdev,MPU6050_PWR_MGMT_1, &reg_buf, 1);
	printk("the i2c send\r\n");
	ret = mpu6050_write_regs(&mpu6050cdev,MPU6050_PWR_MGMT_2, &reg_buf, 1);
	reg_buf = 0x09;
	ret = mpu6050_write_regs(&mpu6050cdev,MPU6050_SMPLRT_DIV, &reg_buf, 1);//采样率分频寄存器，
    reg_buf = 0x06;
	ret = mpu6050_write_regs(&mpu6050cdev,MPU6050_CONFIG, &reg_buf, 1);//配置寄存器低通滤波
    reg_buf = 0x18;
	ret = mpu6050_write_regs(&mpu6050cdev,MPU6050_GYRO_CONFIG, &reg_buf, 1);//陀螺仪配置寄存器，选择满量程 ±2000°/s
    reg_buf = 0x18;
	ret = mpu6050_write_regs(&mpu6050cdev,MPU6050_ACCEL_CONFIG, &reg_buf, 1);//加速度计配置寄存器，选择满量程 ±16g

	return 0;
}

static const struct of_device_id mpu6050_of_match_table[] = {
	{ .compatible = "topeet,mpu6050"},
	{/* sentinel */},
};
static const struct i2c_device_id mpu6050_ids[] = {
	{ "topeet,mpu6050",0},
	{},
};
static int mpu6050_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    //搭建字符设备驱动框架 使其生成字符设备驱动  /dev/xxx 便于用户操作
	int ret;
	mpu6050cdev.mpu6050_client = client;
	ret = cdevmpu_init(client);
	return ret;
}
static int mpu6050_remove(struct i2c_client *client)
{

    unregister_chrdev(mpu6050cdev.major,CDEV_NAME);	//卸载已注册设备（解注册）
	device_destroy(mpu6050cdev.mpu6050_class,MKDEV(mpu6050cdev.major, 0));
	class_destroy(mpu6050cdev.mpu6050_class);
  												//销毁设备
                                				//销毁设备类
	return 0;
}
static struct i2c_driver mpu6050_driver = {   

	.driver = {
		.name = "topeet,mpu6050",
		.of_match_table = mpu6050_of_match_table,
		.owner = THIS_MODULE,
	},
	.probe = mpu6050_probe,
	.remove	= mpu6050_remove,
	.id_table = mpu6050_ids,
};


//4.定义入口函数，用来注册驱动程序
static int __init mpu6050_init(void)
{   
	return i2c_add_driver(&mpu6050_driver);  //1.注册i2c驱动
}
//出口函数
static void __exit mpu6050_exit(void)
{	
	i2c_del_driver(&mpu6050_driver);          //end -- 删除i2c驱动
	printk("%s %s line %d\n",__FILE__,__FUNCTION__,__LINE__);
		
}

module_init(mpu6050_init);
module_exit(mpu6050_exit);

MODULE_LICENSE("GPL");





