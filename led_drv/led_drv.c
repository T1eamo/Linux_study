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
//1.确定主设备号         0代表系统随机分配
#define LED_NAME "led"
static int major = 0;
static struct class *led_class;
//定义寄存器
#define BUS_IOC_GPIO2C_IOMUX_SEL_H_BASE 0xFD5F8054   //复用寄存器
#define GPIO_SWPORT_DDR_H_BASE          0xFEC3000C   //方向寄存器
#define GPIO_SWPORT_DR_H_BASE           0xFEC30004   //数据寄存器

#define LEDOFF 0/*关闭*/
#define LEDON  1/*打开*/

//地址映射后的虚拟地址指针

static  void __iomem *BUS_IOC_GPIO2C_IOMUX_SEL_H;
static  void __iomem *GPIO_SWPORT_DDR_H;
static  void __iomem *GPIO_SWPORT_DR_H;



//static struct class *led_class;


//3.实现相应的write等函数


static ssize_t led_write (struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
		
		unsigned int ret;
		unsigned char databuf[1];
		ret =copy_from_user(databuf,buf,size);
		if(ret < 0){
			printk("open error");
			return -EFAULT;
		}
		printk("%s %s line %d\n",__FILE__,__FUNCTION__,__LINE__);
		if(databuf[0] == LEDON){
			writel(0x00100010,GPIO_SWPORT_DR_H);
		}else if(databuf[0] == LEDOFF){
			writel(0x00100000,GPIO_SWPORT_DR_H);
		}
		return 0;
}
static int led_open (struct inode *node, struct file *file)
{
		printk("%s %s line %d\n",__FILE__,__FUNCTION__,__LINE__);
		return 0;
}

static int led_release (struct inode *node, struct file *file)
{
		printk("%s %s line %d\n",__FILE__,__FUNCTION__,__LINE__);
		return 0;
}




//2.定义自己的file_operations结构体
static struct file_operations led_drv = {
		.owner   = THIS_MODULE,
		.open    = led_open,
		.write   = led_write,
		.release = led_release,

};
//4.定义入口函数，用来注册驱动程序
static int __init led_init(void)
{   
	u32 val;
	int err;
	//进行地址映射    
	BUS_IOC_GPIO2C_IOMUX_SEL_H = ioremap(BUS_IOC_GPIO2C_IOMUX_SEL_H_BASE,4); //返回类型为 void __iomem*
	GPIO_SWPORT_DDR_H = ioremap(GPIO_SWPORT_DDR_H_BASE,4);
	GPIO_SWPORT_DR_H = ioremap(GPIO_SWPORT_DR_H_BASE,4);

	val =  readl(BUS_IOC_GPIO2C_IOMUX_SEL_H);
	val &= 0xFFF0;
	writel(val,BUS_IOC_GPIO2C_IOMUX_SEL_H);

	val = readl(GPIO_SWPORT_DDR_H);//H是CD组gpio，L是AB组，1/2/3/4是不同的基地址
	val |= 1 <<4;
	val |= 1 <<20;
	writel(val,GPIO_SWPORT_DDR_H);


	major = register_chrdev(0,LED_NAME,&led_drv);  //注册设备文件结构体   声明设备名称
                                 //创建设备类
        if (major < 0||major == 0){
				printk("register field");
				return -EIO;
		}
	
	led_class = class_create(THIS_MODULE, "led_class");
	err = PTR_ERR(led_class);
	if (IS_ERR(led_class)){
			unregister_chrdev(major,"led");
		 	printk("%s %s line %d\n",__FILE__,__FUNCTION__,__LINE__);
			return -1;
			}
	device_create(led_class,NULL,MKDEV(major, 0),NULL,"led");
	return 0;
}
//出口函数
static void __exit led_exit(void)
{		
	//取消地址映射
	iounmap(BUS_IOC_GPIO2C_IOMUX_SEL_H);
	iounmap(GPIO_SWPORT_DDR_H);
	iounmap(GPIO_SWPORT_DR_H);
	device_destroy(led_class,MKDEV(major, 0));
	class_destroy(led_class);
  												//销毁设备
                                				//销毁设备类
	unregister_chrdev(major,LED_NAME);	//卸载已注册设备（解注册）
	printk("%s %s line %d\n",__FILE__,__FUNCTION__,__LINE__);
		
}

module_init(led_init);
module_exit(led_exit);

MODULE_LICENSE("GPL");





