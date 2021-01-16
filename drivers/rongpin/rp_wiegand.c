/*wiegand 协议接收驱动代码*/
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#define WIEGAND_MAJOR 243        
//delay 33ms to convert data
#define TIMER_DELAY HZ/8
#define DEVICE_NAME "wiegand" 
 
bool TIMEER_FLAG = false;   
bool RF_OPEN_FLAG = false;
static struct class *cdev_class; 
//dev_t dev = 0;

#define  D0_GPIO_NO  32*8+7-8  //gpio2_a7
#define  D1_GPIO_NO  32*8+1-8  //gpio2_b0

struct wiegand_dev
{
    char wiegand[26];    //Wiegand-26
    int hid;
    int pid;
    int count;    //Global Counter
    struct cdev cdev;
    struct semaphore sem;
    struct completion receive_completion;
    struct timer_list wiegand_timer;
    struct work_struct     pen_event_work;
    struct workqueue_struct *ts_workqueue;

    int gpio_d0;
    int gpio_d1;
    int d0_irq;
    int d1_irq;
    
};

static struct wiegand_dev *rf_card;


//应该除了初始化外，只能有这一个地方能够将rf_card->count置为0，其余地方置为0皆不合法
static char convert_data(void)
{
    int i,even,odd;
        int cardno ; 

    //偶校验
        even = 0;
        for(i = 1; i < 13;i++)
        {
         if(rf_card->wiegand[i] == 1)    
          even = (~even) & 0x01; 
        }
         if(even != rf_card->wiegand[0])
          {
        rf_card->count = 0;
        goto error;
          }    
    //奇校验  
        odd = 1;
        for(i = 13; i< 25;i++)
        {
        if(rf_card->wiegand[i] == 1)             
            odd = (~odd)& 0x01;          
         }
         if(odd != rf_card->wiegand[25])
          {
        rf_card->count = 0;
           goto error;
          }

    //奇偶校验通过
    rf_card->hid = 0;
    for(i = 1 ;i<=8;i++)//hid转换
        rf_card->hid =  rf_card->hid << 1 |rf_card->wiegand[i];    

    rf_card->pid = 0;
    for(i = 9 ;i<=24;i++)//pid转换
        rf_card->pid =  rf_card->pid << 1 |rf_card->wiegand[i];    
      
        cardno = rf_card->hid << 16 | rf_card->pid;
        rf_card->count = 0;
   
        printk("%s,%s,%d,cardno=(%x,%d)\n",__FILE__,__func__,__LINE__,cardno,cardno);
    return 0;
    
error:
    printk("Parity Efficacy Error!\n");
    return -1;
}


//static void wiegand_pen_irq_work(struct work_struct *work)


static void wiegand_do_timer(unsigned long arg)
{
    //wait_for_completion(&(rf_card->receive_completion));//等待维根数据传送完毕才进行转换


        printk("%s,%s,%d,rf_card->count=%d arg=%lu\n",__FILE__,__func__,__LINE__,rf_card->count,arg);
    
    disable_irq(rf_card->d0_irq);
    disable_irq(rf_card->d1_irq);//防止wieg_data在转换期间发生变化
    
    convert_data();
    up(&rf_card->sem);
    
    rf_card->count =0 ;
    //TIMEER_FLAG = false;
    enable_irq(rf_card->d0_irq);
    enable_irq(rf_card->d1_irq);
    TIMEER_FLAG = false;
}

static void enable_irqs(void)
{    
    enable_irq(rf_card->d0_irq);
    enable_irq(rf_card->d1_irq);
    printk("%s:the irq is setup!\n",__func__);
}

static irqreturn_t wiegand_irq0(int irq, void *dev_id)
{
//    printk("%s:the irq0 !\n",__func__);
    disable_irq_nosync(rf_card->d0_irq);
    //udelay(5);

    //(gpio_get_value(rf_card->gpio_d0) == 0)
    {
           rf_card->wiegand[rf_card->count] = 0;
         rf_card->count ++;
        //udelay(100);
    } 
    //else 
    //    goto end;
    
    enable_irq(rf_card->d0_irq);
    /*
    if(rf_card->count == 1)
        init_completion(&(rf_card->receive_completion));
    else if(rf_card->count == 26)
        complete(&(rf_card->receive_completion));//只能在此处complete    
    */
       if(TIMEER_FLAG == false)
       {
        rf_card->wiegand_timer.expires = jiffies + TIMER_DELAY;
        add_timer(&rf_card->wiegand_timer);
        TIMEER_FLAG = true;
    }
    return IRQ_HANDLED;
//end:
//    enable_irq(rf_card->d0_irq);
//    return IRQ_HANDLED;

}

static irqreturn_t wiegand_irq1(int irq, void *dev_id)
{

//    printk("%s:the irq1 !\n",__func__);
    disable_irq_nosync(rf_card->d1_irq);//必须用这个函数
   
//    udelay(5);

    //if(gpio_get_value(rf_card->gpio_d1) == 0 )
    {
           rf_card->wiegand[rf_card->count] = 1;
        rf_card->count ++;
        //udelay(100);
    }
    //else
    //    goto end;
    
    enable_irq(rf_card->d1_irq);

    //rf_card->count经过上面“++”之后都会大1
    /*if(rf_card->count == 1)
        init_completion(&(rf_card->receive_completion));
    else if(rf_card->count == 26)
        complete(&(rf_card->receive_completion));//只能在此处complete*/
     if(TIMEER_FLAG== false)
       {
        rf_card->wiegand_timer.expires = jiffies + TIMER_DELAY;
        add_timer(&rf_card->wiegand_timer);
        TIMEER_FLAG = true;
    }
//    return IRQ_HANDLED;
//end:
//    enable_irq(rf_card->d1_irq);
    return IRQ_HANDLED;
}

static ssize_t rfcd_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{        
        int max_count ;  
        char cardno[12];

      //  unsigned long p = *ppos ;  

    if ( down_interruptible(&rf_card->sem) ) {
                printk("lengzhg sem error\n");
        return -1 ;
        }
        max_count  = sizeof(cardno);

        if ( size >max_count )  { 
             return 0;  
        }
        /*if ( size >(max_count-p) )  { 
             size = max_count-p;
        }*/

        sprintf(cardno,"%d",rf_card->hid << 16 | rf_card->pid);

        printk(KERN_INFO"cardno:%s\n",cardno);

    if ( copy_to_user(buf,cardno,size)){
             return -EFAULT;
        }
        //*ppos += size;

    return size;
}

static ssize_t rfcd_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    return 0;
}

static int request_irqs(void)
{
    int ret;

        rf_card->d0_irq = gpio_to_irq(rf_card->gpio_d0);
        rf_card->d1_irq = gpio_to_irq(rf_card->gpio_d1);

    printk("%s:d0_irq=%d,d1_irq=%d\n",__func__, rf_card->d0_irq , rf_card->d1_irq );
    
    ret = request_irq(rf_card->d0_irq,wiegand_irq0,IRQF_SHARED | IRQF_TRIGGER_FALLING,"wiegand_data0",rf_card);

    if(ret)
    {
        printk("%s:request rf_card->d1_irq):%d,ret:%d failed!\n",__func__,rf_card->d0_irq,ret);
        return -1;   
    }
    ret = request_irq(rf_card->d1_irq,wiegand_irq1,IRQF_SHARED | IRQF_TRIGGER_FALLING,"wiegand_data1",rf_card);

    if(ret) 
    {
        printk("%s:request rf_card->d1_irq:%d,ret:%d failed!\n",__func__,rf_card->d1_irq,ret);
        return -1;   
    }

    printk(KERN_INFO"%s:request irqs success!\n",__func__);
    return 0;
}

static int rfcd_open(struct inode *inode, struct file *filp)
{
       if (RF_OPEN_FLAG)  { 
           return -EBUSY;//Don't open again!

     }        
    RF_OPEN_FLAG = true;
    TIMEER_FLAG = false;

    //setup_timer(&rf_card->wiegand_timer,wiegand_do_timer,0);
        printk("lengzhg %s: wiegand open\n",__func__);

    memset(rf_card->wiegand,0x00,26);
    rf_card->count = 0;

//    enable_irqs();

    printk("lengzhg %s: the device have been open\n",__func__);
    return 0;
}

static void free_irqs(void)
{
    free_irq(rf_card->d0_irq,rf_card);
    free_irq(rf_card->d1_irq,rf_card);
}

int rfcd_release(struct inode *inode, struct file *filp)
{
//    disable_irq(rf_card->d0_irq);
//    disable_irq(rf_card->d1_irq);

    RF_OPEN_FLAG = false;

//    del_timer_sync(&rf_card->wiegand_timer);
    return 0; 
}

static struct file_operations rfcd_fops = 
{
    .owner = THIS_MODULE,
    .read = rfcd_read,
    .write = rfcd_write,
    .open = rfcd_open,
    .release = rfcd_release,
};

static int __init wiegand_init(void)
{
    int err,result;
    dev_t devno = MKDEV(WIEGAND_MAJOR, 1);
        int ret ; 

    //if(WIEGAND_MAJOR)
    if(0)
        result = register_chrdev_region(devno,1,DEVICE_NAME);   
    else
        result = alloc_chrdev_region(&devno,0,1,DEVICE_NAME);
    
    if(result < 0)
    {
        printk("lengzhg %s:register_chrdev_region error\n",__func__);
        return result;
    }

    rf_card = kmalloc(sizeof(struct wiegand_dev),GFP_KERNEL);
    if (!rf_card)
    {
        result = -ENOMEM;
        goto fail_malloc;
    }

    memset(rf_card,0,sizeof(struct wiegand_dev));

    rf_card->count = 0;

    cdev_init(&(rf_card->cdev), &rfcd_fops);
     
    rf_card->cdev.owner = THIS_MODULE;

    err = cdev_add(&rf_card->cdev, devno, 1);

    if(err)
    {
        printk("lengzhg adding err\r\n");
        unregister_chrdev_region(devno,1);
        kfree(rf_card);
        free_irqs();
        return err;
    }
 
    cdev_class = class_create(THIS_MODULE, DEVICE_NAME);//动态创建设备结点
    if(IS_ERR(cdev_class))
    { 
        printk("ERR:cannot create a cdev_class\n");
        unregister_chrdev_region(devno,1);
        return -1;
    }
    device_create(cdev_class,NULL, devno, 0, DEVICE_NAME);
 
    init_completion(&(rf_card->receive_completion));
    sema_init(&rf_card->sem,0);

    rf_card->gpio_d0 = D0_GPIO_NO;
    rf_card->gpio_d1 = D1_GPIO_NO;    

      ret = gpio_request(rf_card->gpio_d0, "wiegand_d0");
        if ( ret ) {
        printk("request gpio_d1 error\n");
        goto fail_malloc;
        }
      ret = gpio_request(rf_card->gpio_d1, "wiegand_d1");
        if ( ret ) {
           printk("request gpio_d1 error\n");
         goto fail_malloc;
        }
    result = request_irqs();

    if(result < 0)
    {
        printk("%s:request_irqs error\n",__func__);
        return result;
    }
    setup_timer(&rf_card->wiegand_timer,wiegand_do_timer,0);

    //disable_irq(rf_card->d0_irq);
    //disable_irq(rf_card->d1_irq);
//    INIT_WORK(&rf_card->pen_event_work, wiegand_pen_irq_work);
    
    printk (KERN_INFO "%s initialized\n",DEVICE_NAME);
    return 0;

fail_malloc: 
    unregister_chrdev_region(devno,1);
    
    return result;
}

static void __exit wiegand_exit(void)
{

    cdev_del(&rf_card->cdev);
    free_irqs();
    kfree(rf_card);
    unregister_chrdev_region(MKDEV(WIEGAND_MAJOR,0),1);
    printk (KERN_INFO"%s removed\n",DEVICE_NAME);
}

module_init(wiegand_init);
module_exit(wiegand_exit);

MODULE_AUTHOR("naro123@163.com");
MODULE_LICENSE("GPL");
