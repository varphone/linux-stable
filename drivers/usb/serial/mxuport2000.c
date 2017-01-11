/*	
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/ioctl.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
#include <asm/uaccess.h>
#else
#include <linux/uaccess.h>
#endif
#include <linux/usb.h>
#include <linux/circ_buf.h>

#define KERNEL_2_6 1

#include "mxuport2000.h"

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
#include "usb-serial.h"
#else
#include <linux/usb/serial.h>
#endif

MODULE_AUTHOR("Eric Lo");
MODULE_DESCRIPTION("MOXA UPort 2000 Series Device Driver");
MODULE_LICENSE("GPL");

static struct usb_driver mxu2_driver = {
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15))
	.owner		= THIS_MODULE,
#endif
	.name		= "mxuport2c",
	.probe		= usb_serial_probe,
	.disconnect	= usb_serial_disconnect,
	.id_table	= mxu2_id_table_combined,
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.no_dynamic_id	= 1,
#endif
};

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13))
static struct usb_serial_driver mxu2_2210_device = {
	.driver                 = {
	        .owner		= THIS_MODULE,
		.name		= "moxa2210",
         },
        .description            = DRIVER_DESC_2210,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.usb_driver		= &mxu2_driver,
#endif
#else
static struct usb_serial_device_type mxu2_2210_device = {
	.owner                  = THIS_MODULE,
        .name                   = "MOXA USB Serial Adapter",
        .short_name             = "MOXA 2210",
#endif
        .id_table               = mxu2_2210_id_table,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))      
        .num_interrupt_in       = 1,
        .num_bulk_in            = 4,
        .num_bulk_out           = 4,
#endif        
        .num_ports              = 4,
	.attach			= mxu2_startup,
#ifdef  ASYNCB_FIRST_KERNEL
	.disconnect		= mxu2_disconnect,
	.release		= mxu2_release,
#else
	.shutdown		= mxu2_shutdown,
#endif
	.open			= mxu2_open,
	.close			= mxu2_close,
	.write			= mxu2_write,
	.write_room		= mxu2_write_room,
	.chars_in_buffer	= mxu2_chars_in_buffer,
	.throttle		= mxu2_throttle,
	.unthrottle		= mxu2_unthrottle,
	.calc_num_ports		= mxu2_calc_num_ports,
	.ioctl			= mxu2_ioctl,
	.set_termios		= mxu2_set_termios,
	.tiocmget               = mxu2_tiocmget,
        .tiocmset               = mxu2_tiocmset,
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))	
	.get_icount		= mxu2_get_icount,
#endif			
	.break_ctl		= mxu2_break,
	.read_int_callback      = mxu2_interrupt_callback,
	.read_bulk_callback	= mxu2_bulk_in_callback, 
	.write_bulk_callback	= mxu2_bulk_out_callback,
};

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13))
static struct usb_serial_driver mxu2_2250_device = {
	.driver                 = {
		.owner		= THIS_MODULE,
		.name		= "moxa2250",
         },
        .description            = DRIVER_DESC_2250,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.usb_driver		= &mxu2_driver,
#endif
#else
static struct usb_serial_device_type mxu2_2250_device = {
	.owner                  = THIS_MODULE,
        .name                   = "MOXA USB Serial Adapter",
        .short_name             = "MOXA 2250",
#endif
        .id_table               = mxu2_2250_id_table,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))      
        .num_interrupt_in       = 1,
        .num_bulk_in            = 4,
        .num_bulk_out           = 4,
#endif        
        .num_ports              = 4,
	.attach			= mxu2_startup,
#ifdef  ASYNCB_FIRST_KERNEL
	.disconnect		= mxu2_disconnect,
	.release		= mxu2_release,
#else
	.shutdown		= mxu2_shutdown,
#endif
	.open			= mxu2_open,
	.close			= mxu2_close,
	.write			= mxu2_write,
	.write_room		= mxu2_write_room,
	.chars_in_buffer	= mxu2_chars_in_buffer,
	.throttle		= mxu2_throttle,
	.unthrottle		= mxu2_unthrottle,
	.calc_num_ports		= mxu2_calc_num_ports,
	.ioctl			= mxu2_ioctl,
	.set_termios		= mxu2_set_termios,
	.tiocmget               = mxu2_tiocmget,
        .tiocmset               = mxu2_tiocmset,
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))	
	.get_icount		= mxu2_get_icount,
#endif			
	.break_ctl		= mxu2_break,
	.read_int_callback      = mxu2_interrupt_callback,
	.read_bulk_callback	= mxu2_bulk_in_callback, 
	.write_bulk_callback	= mxu2_bulk_out_callback,
};

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13))
static struct usb_serial_driver mxu2_2410_device = {
	.driver                 = {
		.owner		= THIS_MODULE,
		.name		= "moxa2410",
         },
        .description            = DRIVER_DESC_2410,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.usb_driver		= &mxu2_driver,
#endif
#else
static struct usb_serial_device_type mxu2_2410_device = {
	.owner                  = THIS_MODULE,
        .name                   = "MOXA USB Serial Adapter",
        .short_name             = "MOXA 2410",
#endif
        .id_table               = mxu2_2410_id_table,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))         
        .num_interrupt_in       = 1,
        .num_bulk_in            = 4,
        .num_bulk_out           = 4,
#endif        
        .num_ports              = 4,
	.attach			= mxu2_startup,
#ifdef  ASYNCB_FIRST_KERNEL
	.disconnect		= mxu2_disconnect,
	.release		= mxu2_release,
#else
	.shutdown		= mxu2_shutdown,
#endif
	.open			= mxu2_open,
	.close			= mxu2_close,
	.write			= mxu2_write,
	.write_room		= mxu2_write_room,
	.chars_in_buffer	= mxu2_chars_in_buffer,
	.throttle		= mxu2_throttle,
	.unthrottle		= mxu2_unthrottle,
	.calc_num_ports		= mxu2_calc_num_ports,
	.ioctl			= mxu2_ioctl,
	.set_termios		= mxu2_set_termios,
	.tiocmget               = mxu2_tiocmget,
        .tiocmset               = mxu2_tiocmset,
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))	
	.get_icount		= mxu2_get_icount,
#endif			
	.break_ctl		= mxu2_break,
	.read_int_callback      = mxu2_interrupt_callback,
	.read_bulk_callback	= mxu2_bulk_in_callback, 
	.write_bulk_callback	= mxu2_bulk_out_callback,
};

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13))
static struct usb_serial_driver mxu2_2450_device = {
	.driver                 = {
		.owner		= THIS_MODULE,
		.name		= "moxa2450",
         },
        .description            = DRIVER_DESC_2450,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.usb_driver		= &mxu2_driver,
#endif
#else
static struct usb_serial_device_type mxu2_2450_device = {
	.owner                  = THIS_MODULE,
        .name                   = "MOXA USB Serial Adapter",
        .short_name             = "MOXA 2450",
#endif
        .id_table               = mxu2_2450_id_table,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))               
        .num_interrupt_in       = 1,
        .num_bulk_in            = 4,
        .num_bulk_out           = 4,
#endif        
        .num_ports              = 4,
	.attach			= mxu2_startup,
#ifdef  ASYNCB_FIRST_KERNEL
	.disconnect		= mxu2_disconnect,
	.release		= mxu2_release,
#else
	.shutdown		= mxu2_shutdown,
#endif
	.open			= mxu2_open,
	.close			= mxu2_close,
	.write			= mxu2_write,
	.write_room		= mxu2_write_room,
	.chars_in_buffer	= mxu2_chars_in_buffer,
	.throttle		= mxu2_throttle,
	.unthrottle		= mxu2_unthrottle,
	.calc_num_ports		= mxu2_calc_num_ports,
	.ioctl			= mxu2_ioctl,
	.set_termios		= mxu2_set_termios,
	.tiocmget               = mxu2_tiocmget,
        .tiocmset               = mxu2_tiocmset,
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))	
	.get_icount		= mxu2_get_icount,
#endif			
	.break_ctl		= mxu2_break,
	.read_int_callback      = mxu2_interrupt_callback,
	.read_bulk_callback	= mxu2_bulk_in_callback, 
	.write_bulk_callback	= mxu2_bulk_out_callback,
};

#if(LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
static struct usb_serial_driver * const serial_drivers[] = {
	&mxu2_2410_device, &mxu2_2450_device, &mxu2_2210_device,
	&mxu2_2250_device, NULL
};
#endif

module_init(mxu2_init);
module_exit(mxu2_exit);

//-----------------------------------------------------------------------------
#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
int __init mxu2_init(void)
{
	int retval;

	retval = usb_serial_register(&mxu2_2410_device);
	if(retval)
		goto failed_2410_device_register;

	retval = usb_serial_register(&mxu2_2450_device);
	if(retval)
		goto failed_2450_device_register;

	retval = usb_serial_register(&mxu2_2210_device);
	if(retval)
		goto failed_2210_device_register;

	retval = usb_serial_register(&mxu2_2250_device);
	if(retval)
		goto failed_2250_device_register;

	retval = usb_register(&mxu2_driver);
	if (retval)
		goto failed_usb_register;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,27))      		
	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
	       DRIVER_DESC "\n");
#else	       
	info(DRIVER_DESC " " DRIVER_VERSION);
#endif
	return 0;

failed_usb_register:
	usb_serial_deregister(&mxu2_2250_device);
failed_2250_device_register:
	usb_serial_deregister(&mxu2_2210_device);
failed_2210_device_register:
	usb_serial_deregister(&mxu2_2450_device);
failed_2450_device_register:
	usb_serial_deregister(&mxu2_2410_device);
failed_2410_device_register:

	return retval;
}


void __exit mxu2_exit (void)
{
	usb_deregister(&mxu2_driver);
	usb_serial_deregister(&mxu2_2450_device);
	usb_serial_deregister(&mxu2_2410_device);
	usb_serial_deregister(&mxu2_2250_device);
	usb_serial_deregister(&mxu2_2210_device);
}
#else
int __init mxu2_init(void)
{
	int ret;
	
	ret = usb_serial_register_drivers(&mxu2_driver, serial_drivers);
	
	if(ret == 0)
		printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
	       DRIVER_DESC "\n");
		   
	return ret;
}


void __exit mxu2_exit (void)
{
	usb_serial_deregister_drivers(&mxu2_driver, serial_drivers);
}
#endif


static int mxu2_startup (struct usb_serial *serial)
{
	struct mxu2_serial *mx_serial;
	struct mxu2_port *mx_port;
	struct usb_device *dev;
	__u16 Data;
	int i, status;
	
	if(!serial) {
		dev_err(&dev->dev, "%s - Invalid Handler\n", __FUNCTION__);
		return -1;
	}

	dev = serial->dev;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,16))
	mx_serial = kzalloc(sizeof(struct mxu2_serial), GFP_KERNEL);
#else
	mx_serial = kmalloc(sizeof(struct mxu2_serial), GFP_KERNEL);
#endif
	if (mx_serial == NULL){
		dev_err(&dev->dev, "%s - out of memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	memset(mx_serial, 0, sizeof(struct mxu2_serial));
	mx_serial->serial = serial;
	mx_serial->status_polling_started = FALSE;
	usb_set_serial_data(serial,mx_serial);
	mx_serial->model_type = mxu2_calc_num_ports(serial);
	mx_get_op_mode(serial);
	mx_serial->shadow_gpio = 0x00;
	sema_init(&mx_serial->open_close_sem,1);
	mx_serial->port_intf_status = MX_4PORT_DEFAULT_INTF_RS232;
	if(mx_serial->model_type == MX_2_PORTS)
		mx_serial->port_intf_status = MX_2PORT_DEFAULT_INTF_RS232;

	for (i = 0; i < serial->num_ports; ++i){
		mx_port = kmalloc(sizeof(struct mxu2_port), GFP_KERNEL);

		if (mx_port == NULL){
			dev_err(&dev->dev,"%s - out of memory\n",__FUNCTION__);
			status = -ENOMEM;
			goto free_mxports;
		}

		memset(mx_port, 0, sizeof(struct mxu2_port));
		spin_lock_init(&mx_port->mxp_slock);
		usb_set_serial_port_data(serial->port[i],mx_port);
		mx_port->port = serial->port[i];
		mx_port->mxp_serial = mx_serial;
		mx_port->port_num = i+1;
		mx_port->flags = ASYNC_LOW_LATENCY;
		mx_port->closing_wait = MX_DEFAULT_CLOSING_WAIT;
		mx_port->close_delay = MX_DEFAULT_CLOSE_DELAY;
		mx_port->interface = MX_RS232;
		mx_port->write_buf = mx_buf_alloc();

		if(mx_port->write_buf == NULL){
			dev_err(&dev->dev,"%s - out of memory\n",__FUNCTION__);
			status = -ENOMEM;
			goto free_mxports;
		}
		
		mx_set_reg_offset(mx_port);

		status = mx_init_startup_regs(mx_port);
		if(status < 0)
			goto free_mxports;

		mx_port->control_urb = usb_alloc_urb(0,GFP_ATOMIC);

		if(mx_port->control_urb == NULL){
			status = -ENOMEM;
			goto free_mxports;
		}

		mx_port->ctrl_buf = kmalloc(MX_CTRL_BUF_SIZE,GFP_KERNEL);

		if(mx_port->ctrl_buf == NULL){
			status = -ENOMEM;
			goto free_mxports;
		}
	}

	Data = MX_ZLP_REG_EN_ALL_SP;
	status = mx_set_reg_sync(serial->port[0],ZLP_REG5,Data);
	if(status < 0) 
		goto free_mxports;

	/* setting configuration feature to one */
	status = usb_control_msg (serial->dev, usb_sndctrlpipe(serial->dev, 0), (__u8)0x03, 0x00,0x01,0x00, 0x00, 0x00, 5*HZ);
	
	if(status < 0) 
		goto free_mxports;

	return 0;

free_mxports:

	for(--i;i<=0;--i){
		mx_port = usb_get_serial_port_data(serial->port[i]);
		mx_buf_free(mx_port->write_buf);
		kfree(mx_port);
		usb_set_serial_port_data(serial->port[i],NULL);
	}

	kfree(mx_serial);
	usb_set_serial_data(serial,NULL);

	return status;
}

#ifdef  ASYNCB_FIRST_KERNEL
static void mxu2_disconnect (struct usb_serial *serial)
{
	struct mxu2_port *mx_port;
	int i;

	for (i=0; i < serial->num_ports; ++i){
		mx_port = usb_get_serial_port_data(serial->port[i]);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10))
		usb_kill_urb(mx_port->read_urb);
#else
		usb_unlink_urb(mxport->read_urb);
#endif
	}

}

static void mxu2_release (struct usb_serial *serial)
{
	struct mxu2_serial *mx_serial = usb_get_serial_data(serial);
	struct mxu2_port *mx_port;
	int i;

	for (i=0; i < serial->num_ports; ++i){
		mx_port = usb_get_serial_port_data(serial->port[i]);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10))
		usb_kill_urb(mx_port->control_urb);
#endif
		if(mx_port->ctrl_buf)
			kfree(mx_port->ctrl_buf);

		if(mx_port->control_urb)
			usb_free_urb(mx_port->control_urb);

		if(mx_port){
			mx_buf_free(mx_port->write_buf);
			kfree(mx_port);
			usb_set_serial_port_data(serial->port[i],NULL);
		}
	}

	kfree(mx_serial);
	usb_set_serial_data(serial,NULL);
}

#else

static void mxu2_shutdown (struct usb_serial *serial)
{
	struct mxu2_serial *mx_serial = usb_get_serial_data(serial);
	struct mxu2_port *mx_port;
	int i;

	for (i=0; i < serial->num_ports; ++i){
		mx_port = usb_get_serial_port_data(serial->port[i]);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10))
		usb_kill_urb(mx_port->control_urb);
#endif
		if(mx_port->ctrl_buf)
			kfree(mx_port->ctrl_buf);

		if(mx_port->control_urb)
			usb_free_urb(mx_port->control_urb);

		if(mx_port){
			mx_buf_free(mx_port->write_buf);
			kfree(mx_port);
			usb_set_serial_port_data(serial->port[i],NULL);
		}
	}

	kfree(mx_serial);
	usb_set_serial_data(serial,NULL);
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu2_open (struct usb_serial_port *port, struct file *filp)
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
static int mxu2_open (struct tty_struct *tty, struct usb_serial_port *port)
#else
static int mxu2_open (struct tty_struct *tty, struct usb_serial_port *port, struct file *filp)
#endif
{
	struct usb_serial *serial = port->serial;
	struct mxu2_serial *mx_serial = usb_get_serial_data(serial);
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
	struct termios tmp_termios; 
#else
	struct ktermios tmp_termios;
#endif
	int status;

	if((serial == NULL) || (mx_port == NULL) || (mx_serial == NULL))
		return -ENODEV;
	
	if(down_interruptible(&mx_serial->open_close_sem))
		return -ERESTARTSYS;

     	usb_clear_halt(serial->dev, port->write_urb->pipe);
     	usb_clear_halt(serial->dev, port->read_urb->pipe);
	
	status = mx_init_open_regs(mx_port);
	if(status < 0)
		return status;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	if (port->tty)
		port->tty->low_latency = MX_DEFAULT_LOW_LATENCY;
#else
	if (port->port.tty)
		port->port.tty->low_latency = MX_DEFAULT_LOW_LATENCY;
#endif		

	if((mx_serial->NoOfOpenPorts == 0) && (!mx_serial->status_polling_started)){
		mx_serial->status_polling_started = TRUE;
                mx_serial->interrupt_in_buffer = serial->port[0]->interrupt_in_buffer;
		mx_serial->interrupt_in_endpoint = serial->port[0]->interrupt_in_endpointAddress;
                mx_serial->interrupt_read_urb = serial->port[0]->interrupt_in_urb;
		mx_serial->interrupt_read_urb->interval = MX_INT_READ_URB_INTERVAL;

	        usb_fill_int_urb(mx_serial->interrupt_read_urb,
                        serial->dev,
                        usb_rcvintpipe(serial->dev,mx_serial->interrupt_in_endpoint),
                        mx_serial->interrupt_in_buffer,
                        mx_serial->interrupt_read_urb->transfer_buffer_length,
		        mxu2_interrupt_callback, mx_serial,
                        mx_serial->interrupt_read_urb->interval);

		status = usb_submit_urb(mx_serial->interrupt_read_urb,GFP_KERNEL);
		if(status)
			dev_err(&port->dev, "%s - Error %d submitting interrupt urb\n", __FUNCTION__, status);
	}

	mx_port->read_urb = port->read_urb;
	mx_port->read_urb_state = MX_READ_URB_RUNNING;

	if((mx_serial->model_type == MX_2_PORTS) && (mx_port->port_num == MX_PORT2)){
	        usb_fill_bulk_urb(mx_port->read_urb,serial->dev,
		        usb_rcvbulkpipe(serial->dev, port->bulk_in_endpointAddress+2),
			port->bulk_in_buffer,
	                mx_port->read_urb->transfer_buffer_length,
	                mxu2_bulk_in_callback,mx_port);
	}
        else{
		usb_fill_bulk_urb(mx_port->read_urb,
			serial->dev,
			usb_rcvbulkpipe(serial->dev, port->bulk_in_endpointAddress),
			port->bulk_in_buffer,
			mx_port->read_urb->transfer_buffer_length,
			mxu2_bulk_in_callback,mx_port);
	}

	status = usb_submit_urb (mx_port->read_urb,GFP_KERNEL);

	if (status)
		dev_err(&port->dev, "%s - Error %d submitting read urb\n", __FUNCTION__, status);

        init_waitqueue_head(&mx_port->wait_chase);
        memset(&(mx_port->icount), 0x00, sizeof(mx_port->icount));
        mx_port->shadowMCR = MX_MCR_MASTER_IE;
        mx_port->openPending = FALSE;
        mx_port->open = TRUE; 
	mx_buf_clear(mx_port->write_buf);
	mx_port->write_urb_in_use = MX_WRITE_URB_FREE;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
        if (port->tty) 
                mxu2_set_termios(port, &tmp_termios);
#else
	mx_port->port->port.tty = tty;
	if (port->port.tty) 
                mxu2_set_termios(tty,port, &tmp_termios);
#endif                

	mx_port->icount.tx = 0;
	mx_port->icount.rx = 0;

	mx_serial->NoOfOpenPorts++;
	
	up(&mx_serial->open_close_sem);

        return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu2_close (struct usb_serial_port *port, struct file * filp)
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
static void mxu2_close (struct usb_serial_port *port)
#else
static void mxu2_close (struct tty_struct *tty, struct usb_serial_port *port, struct file * filp)
#endif
{
	struct usb_serial *serial = port->serial;
	struct mxu2_serial *mx_serial = usb_get_serial_data(serial);
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
	__u16 Data;
	int do_up, delay;

	if ((serial == NULL) || (mx_serial == NULL) || (mx_port == NULL))
		return;

	do_up = !down_interruptible(&mx_serial->open_close_sem);
	
	if (serial->dev){
		mx_block_until_tx_empty(mx_port);
		usb_unlink_urb (port->write_urb);
		usb_unlink_urb (port->read_urb);
		usb_unlink_urb (mx_serial->interrupt_read_urb);
		usb_unlink_urb (mx_port->control_urb);
	}

	mx_port->write_urb_in_use = MX_WRITE_URB_FREE;
	
	Data = 0x00;
	mx_set_Uart_Reg(port,MODEM_CONTROL_REGISTER,Data);
	Data = 0x00;
	mx_set_Uart_Reg(port,INTERRUPT_ENABLE_REGISTER,Data);

	mx_port->open         = FALSE;
	mx_port->openPending  = FALSE;
	mx_serial->NoOfOpenPorts--;

	if(mx_serial->NoOfOpenPorts == 0)
		mx_serial->status_polling_started = FALSE;
	
	if(mx_port->close_delay){
		delay = mx_port->close_delay * HZ;
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(delay);
		set_current_state(TASK_RUNNING);
	}

	if(do_up)
		up(&mx_serial->open_close_sem);
}   


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
static int mxu2_write (struct usb_serial_port *port, int from_user, const unsigned char *data, int count)
#else
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu2_write (struct usb_serial_port *port, const unsigned char *data, int count)
#else
static int mxu2_write (struct tty_struct *tty1, struct usb_serial_port *port, const unsigned char *data, int count)
#endif
#endif
{
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))	
	struct tty_struct *tty = port->tty;
#else
	struct tty_struct *tty = port->port.tty;
#endif	
	int transfer_size = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
	unsigned char *data_buf;
#endif

	if((mx_port == NULL) || !mx_port->open)
		return -ENODEV;
	
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
	if(from_user){
		data_buf = kmalloc(count, GFP_ATOMIC);

		if(copy_from_user(data_buf, data, count))
			return -EFAULT;
	}
#endif
	if(count){
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
		if(from_user){
			transfer_size = mx_buf_put(mx_port, data_buf, count);
			kfree(data_buf);
		}
		else
			transfer_size = mx_buf_put(mx_port, data, count);
#else
		transfer_size = mx_buf_put(mx_port, data, count);
#endif
	}

	mx_port->sw_tx_stopped = MX_SW_TX_RUNNING;

	if(mx_port->sw_function & MX_IN_SW_ENABLE){
		if(tty->stopped)
			mx_port->sw_tx_stopped = MX_SW_TX_STOPPED;
		else
			mx_port->sw_tx_stopped = MX_SW_TX_RUNNING;
	}
			
	mx_send(mx_port);

	return transfer_size;	
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))	
static int mxu2_write_room (struct usb_serial_port *port)
{
#else
static int mxu2_write_room (struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
#endif		
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
	int room = 0;
	unsigned long flags;

        if (mx_port == NULL)
                return -ENODEV;
	
	spin_lock_irqsave(&mx_port->mxp_slock, flags);
	room = mx_buf_space_avail(mx_port->write_buf);
	spin_unlock_irqrestore(&mx_port->mxp_slock, flags);

        return room;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu2_chars_in_buffer (struct usb_serial_port *port)
{
#else
static int mxu2_chars_in_buffer (struct tty_struct *tty)
{
#endif	

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct mxu2_port *mx_port;
	int chars = 0;
	unsigned long flags;
	
	if(port == NULL)
		return -ENODEV; 
		
	mx_port = usb_get_serial_port_data(port);
#else
	struct usb_serial_port *port;
	struct mxu2_port *mx_port;
	int chars = 0;
	unsigned long flags;
	
	if(tty == NULL)
		return -ENODEV;
		
	port = tty->driver_data;
	
	if(port == NULL)
		return -ENODEV; 
		
	mx_port = usb_get_serial_port_data(port);
#endif

	

        if (mx_port == NULL)
                return -ENODEV;
	
	spin_lock_irqsave(&mx_port->mxp_slock, flags);
	chars = mx_buf_data_avail(mx_port->write_buf);
	spin_unlock_irqrestore(&mx_port->mxp_slock, flags);
	
    	return chars;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu2_throttle (struct usb_serial_port *port)
{
#else
static void mxu2_throttle (struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
#endif	
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = port->tty;
#endif	
	unsigned char stop_char = STOP_CHAR(tty);
	unsigned long flags;

	if ((mx_port == NULL) || !mx_port->open || !tty)
		return;

	if (I_IXOFF(tty)){
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
		mxu2_write (port,0, &stop_char, 1);
#else
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))

		mxu2_write (port, &stop_char, 1);
#else

		mxu2_write (NULL, port, &stop_char, 1);
#endif
#endif
	}

	if (I_IXOFF(tty) || C_CRTSCTS(tty)){
		spin_lock_irqsave(&mx_port->mxp_slock,flags);
		if(mx_port->read_urb_state == MX_READ_URB_RUNNING)
			mx_port->read_urb_state = MX_READ_URB_STOPPING;
		spin_unlock_irqrestore(&mx_port->mxp_slock,flags);
	}
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu2_unthrottle (struct usb_serial_port *port)
{
#else
static void mxu2_unthrottle (struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
#endif	
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))	 
	struct tty_struct *tty = port->tty;
#endif	
	struct urb *urb;
	unsigned long flags;
	unsigned char start_char = START_CHAR(tty);

	if ((mx_port == NULL) || !mx_port->open || !tty)
		return;

	if (I_IXOFF(tty)){
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
		mxu2_write (port, 0, &start_char, 1);
#else
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))

		mxu2_write (port, &start_char, 1);
#else

		mxu2_write (NULL, port, &start_char, 1);
#endif
#endif
	}

	if (I_IXOFF(tty) || C_CRTSCTS(tty)){
		spin_lock_irqsave(&mx_port->mxp_slock,flags);
		if (mx_port->read_urb_state == MX_READ_URB_STOPPED) {
			mx_port->read_urb_state = MX_READ_URB_RUNNING;
			urb = mx_port->port->read_urb;
			urb->complete = mxu2_bulk_in_callback;
			urb->context = mx_port;
			urb->dev = mx_port->port->serial->dev;
			usb_submit_urb(urb, GFP_ATOMIC);
			spin_unlock_irqrestore(&mx_port->mxp_slock,flags);
		}
		else{
			mx_port->read_urb_state = MX_READ_URB_RUNNING;
			spin_unlock_irqrestore(&mx_port->mxp_slock,flags);
		}
	}
}


static int mxu2_calc_num_ports(struct usb_serial *serial)
{
	if(usb_match_id(serial->interface, mxu2_2210_id_table))
		return MX_2_PORTS;
	
	if(usb_match_id(serial->interface, mxu2_2250_id_table))
		return MX_2_PORTS;
	
	if(usb_match_id(serial->interface, mxu2_2410_id_table))
		return MX_4_PORTS;
	
	if(usb_match_id(serial->interface, mxu2_2450_id_table))
		return MX_4_PORTS;

	return -ENODEV;
}

#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))
static int mxu2_get_icount(struct tty_struct *tty,
		struct serial_icounter_struct *icount)
{
	struct usb_serial_port *port = tty->driver_data;
	struct mxu2_port *mxport = usb_get_serial_port_data(port);
	struct async_icount cnow = mxport->icount;

	dbg("%s - (%d) TIOCGICOUNT RX=%d, TX=%d",
		__func__, port->number,
		cnow.rx, cnow.tx);

	icount->cts = cnow.cts;
	icount->dsr = cnow.dsr;
	icount->rng = cnow.rng;
	icount->dcd = cnow.dcd;
	icount->rx = cnow.rx;
	icount->tx = cnow.tx;
	icount->frame = cnow.frame;
	icount->overrun = cnow.overrun;
	icount->parity = cnow.parity;
	icount->brk = cnow.brk;
	icount->buf_overrun = cnow.buf_overrun;

	return 0;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu2_ioctl (struct usb_serial_port *port, struct file *file, unsigned int cmd, unsigned long arg)
{
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int mxu2_ioctl (struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
#else
static int mxu2_ioctl (struct tty_struct *tty, unsigned int cmd, unsigned long arg)
#endif
{
	struct usb_serial_port *port = tty->driver_data;
#endif	
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
	struct mxu2_serial *mx_serial = mx_port->mxp_serial;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = port->tty;
#endif	
	struct async_icount cnow;
	struct async_icount cprev;
	struct serial_icounter_struct icount;

	if ((mx_port == NULL) || (tty == NULL))
		return -ENODEV;

	switch (cmd){ 
		case TIOCSERGETLSR:
			return mx_get_lsr_info(mx_port, (unsigned int *) arg);
		case TIOCMBIS:
		case TIOCMBIC:
		case TIOCMSET:
			return mx_set_modem_info(mx_port, cmd, (unsigned int *) arg);
		case TIOCMGET: 
			return mx_get_modem_info(mx_port, (unsigned int *) arg);
		case TIOCGSERIAL:
			return mx_get_serial_info(mx_port, (struct serial_struct __user *) arg);
		case TIOCSSERIAL:
			return mx_set_serial_info(mx_port, (struct serial_struct __user *) arg);
			break;
		case TIOCMIWAIT:
			cprev = mx_port->icount;
			while (1) {

				if (signal_pending(current))
					return -ERESTARTSYS;

				cnow = mx_port->icount;
				if ((cnow.rng == cprev.rng) &&
				    (cnow.dsr == cprev.dsr) &&
				    (cnow.dcd == cprev.dcd) &&
				    (cnow.cts == cprev.cts))
					return -EIO;

				if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
				    ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
				    ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
				    ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) 
					return 0;

				cprev = cnow;
			}
			break;

		case TIOCGICOUNT:
			cnow = mx_port->icount;
			icount.cts = cnow.cts;
			icount.dsr = cnow.dsr;
			icount.rng = cnow.rng;
			icount.dcd = cnow.dcd;
			icount.rx = cnow.rx;
			icount.tx = cnow.tx;
			icount.frame = cnow.frame;
			icount.overrun = cnow.overrun;
			icount.parity = cnow.parity;
			icount.brk = cnow.brk;
			icount.buf_overrun = cnow.buf_overrun;

			if (copy_to_user((void *)arg, &icount, sizeof(icount)))
				return -EFAULT;

			return 0;
		case MX_IOCTL_UPORT2000_SET_INTERFACE:

			return mx_set_interface(mx_port, (unsigned char) arg);

		case MX_IOCTL_UPORT2000_GET_INTERFACE:
			if (copy_to_user((void *)arg, &mx_port->interface, sizeof(mx_port->interface)))
				return -EFAULT;

			return 0;
		case MX_IOCTL_UPORT2000_LOCATE_DEVICE:

			return mx_set_beeper_led(mx_port,(unsigned char) arg);
		case MX_IOCTL_UPORT2000_GET_PRODUCT_ID:

			if (copy_to_user((void *)arg, &mx_serial->product_name, sizeof(mx_serial->product_name)))
				return -EFAULT;

			return 0;
		default:
			break;
	}

	return -ENOIOCTLCMD;
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
static void mxu2_set_termios (struct usb_serial_port *port, struct termios *old_termios)
#else
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu2_set_termios (struct usb_serial_port *port, struct ktermios *old_termios)
#else
static void mxu2_set_termios (struct tty_struct *tty, struct usb_serial_port *port, struct ktermios *old_termios)
#endif
#endif
{
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = port->tty;
#endif	
	tcflag_t cflag,iflag;

	if ((mx_port == NULL) || !tty || !tty->termios || !mx_port->open)
		return;

	cflag = tty->termios->c_cflag;
	iflag = tty->termios->c_iflag;

	if (old_termios && (cflag == old_termios->c_cflag) && (iflag == old_termios->c_iflag)){
			return;
	}

	mx_change_port_settings(mx_port, old_termios);

	if(mx_port->read_urb->status != -EINPROGRESS){
		mx_port->read_urb->dev = serial->dev;
	 	usb_submit_urb(mx_port->read_urb, GFP_ATOMIC);
	}
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu2_tiocmget(struct usb_serial_port *port, struct file *file)
{
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int mxu2_tiocmget(struct tty_struct *tty, struct file *file)
#else
static int mxu2_tiocmget(struct tty_struct *tty)
#endif
{
	struct usb_serial_port *port = tty->driver_data;
#endif	
        unsigned int result;
        __u16 msr,mcr;
	int status;

	status = mx_get_Uart_Reg(port,MODEM_STATUS_REGISTER,&msr);
	if(status < 0)
		return status;

	status = mx_get_Uart_Reg(port,MODEM_CONTROL_REGISTER,&mcr);
	if(status < 0)
		return status;

        result = ((mcr & MX_MCR_DTR)	  ? TIOCM_DTR  : 0)
               | ((mcr & MX_MCR_RTS)	  ? TIOCM_RTS  : 0)
	       | ((mcr & MX_MCR_LOOPBACK) ? TIOCM_LOOP : 0)
	       | ((msr & MX_MSR_CTS)	  ? TIOCM_CTS  : 0)
	       | ((msr & MX_MSR_CD)	  ? TIOCM_CAR  : 0)
	       | ((msr & MX_MSR_RI)	  ? TIOCM_RI   : 0)
	       | ((msr & MX_MSR_DSR)	  ? TIOCM_DSR  : 0);

        return result;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu2_tiocmset(struct usb_serial_port *port, struct file *file,
        unsigned int set, unsigned int clear)
{
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int mxu2_tiocmset(struct tty_struct *tty, struct file *file,
        unsigned int set, unsigned int clear)
#else
static int mxu2_tiocmset(struct tty_struct *tty,
        unsigned int set, unsigned int clear)
#endif		
{
	struct usb_serial_port *port = tty->driver_data;
#endif	
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
	unsigned int status, mcr;

	if (mx_port == NULL)
                return -ENODEV;

	mcr = mx_port->shadowMCR;

        if (clear & TIOCM_RTS)
                mcr &= ~MX_MCR_RTS;
        if (clear & TIOCM_DTR)
                mcr &= ~MX_MCR_DTR;
        if (clear & TIOCM_LOOP)
                mcr &= ~MX_MCR_LOOPBACK;
	
        if (set & TIOCM_RTS)
                mcr |= MX_MCR_RTS;
        if (set & TIOCM_DTR)
                mcr |= MX_MCR_DTR;
        if (set & TIOCM_LOOP)
                mcr |= MX_MCR_LOOPBACK;

	mx_port->shadowMCR = mcr;

        status = mx_set_Uart_Reg(port,MODEM_CONTROL_REGISTER,mcr);
        if(status < 0)
		return status;

        return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu2_break (struct usb_serial_port *port, int break_state)
{
#else
static void mxu2_break (struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
#endif	
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;
        unsigned char data;
	
	if ((serial == NULL) || (mx_port == NULL))
		return;

	if (serial->dev)
		mx_block_until_chase_response(mx_port);

        if(break_state == -1)
                data = mx_port->shadowLCR | MX_LCR_SET_BREAK;
        else
                data = mx_port->shadowLCR & ~MX_LCR_SET_BREAK;

        mx_port->shadowLCR  = data;
	mx_set_Uart_Reg(port,LINE_CONTROL_REGISTER,mx_port->shadowLCR);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,6))
	wait_ms(100);
#endif

	return;
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void mxu2_interrupt_callback (struct urb *urb,struct pt_regs *regs)
#else
static void mxu2_interrupt_callback (struct urb *urb)
#endif
{
	struct mxu2_serial *mx_serial = (struct mxu2_serial *)urb->context;
	struct mxu2_port *mx_port;
	struct usb_serial *serial = mx_serial->serial;
	unsigned char *data;
	int result,length,i;
	__u16 Data,wval;
	__u8 sp[5];
	
	if(!urb || (mx_serial == NULL) || (serial == NULL))
		return;

	switch (urb->status){
		case 0:
			break;
		case -ECONNRESET:
		case -ENOENT:
		case -ESHUTDOWN:
			dbg("%s - urb shutting down with status: %d", __FUNCTION__, urb->status);
			return;
		default:
			dbg("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
			goto exit;
	}

	length = urb->actual_length;
	data = urb->transfer_buffer;

	if(length > 5)
		return;

	if(mx_serial->model_type == MX_4_PORTS){
		sp[0]=(__u8)data[0];	
		sp[1]=(__u8)data[1];	
		sp[2]=(__u8)data[2];	
		sp[3]=(__u8)data[3];
	}
	else{
		sp[0]=(__u8)data[0];	
		sp[1]=(__u8)data[2];	
	}

	for(i=0;i<serial->num_ports;i++){
		mx_port = usb_get_serial_port_data(serial->port[i]);
		if(mx_serial->model_type == MX_2_PORTS)
			wval = ((__u16)(mx_port->port_num+1))<<8;
		else
			wval = ((__u16)mx_port->port_num)<<8;

		if(mx_port->open){
			if(sp[i] & 0x01){
				dbg("SP%d No Interrupt !!!\n",i);
			}
			else{
				switch(sp[i] & 0x0f){
					case MX_IIR_RLS: 
						mx_port->MsrLsr=MX_HANDLE_LSR;
						mx_get_reg(mx_port,wval,LINE_STATUS_REGISTER,&Data);
						break;
					case MX_IIR_MS:  
						mx_port->MsrLsr=MX_HANDLE_MSR;	
						mx_get_reg(mx_port,wval, MODEM_STATUS_REGISTER, &Data);	
						break;
				}
			}
			
		}
	}

exit:
	if( mx_serial->status_polling_started == FALSE )
		return;

	result = usb_submit_urb (urb, GFP_ATOMIC);

	if (result)
		dev_err(&urb->dev->dev, "%s - Error %d submitting interrupt urb\n", __FUNCTION__, result);

	return;
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void mxu2_bulk_in_callback (struct urb *urb, struct pt_regs *regs)
#else
static void mxu2_bulk_in_callback (struct urb *urb)
#endif
{
	struct mxu2_port *mx_port = (struct mxu2_port *)urb->context;
	struct usb_serial_port *port = (struct usb_serial_port *)mx_port->port;
	struct usb_serial *serial = port->serial;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))	
	struct tty_struct *tty = port->tty;
#else
	struct tty_struct *tty = port->port.tty;
#endif	
	unsigned long flags;
	int status = 0;
	
	/* Check port is valid or not */
	if(mx_port == NULL)
		return;

	if(!urb || (mx_port == NULL) || (port == NULL) || (serial == NULL))
		return;

        switch (urb->status){
                case 0:
                        break;
                case -ECONNRESET:
                case -ENOENT:
                case -ESHUTDOWN:
                        dbg("%s - bulk in urb shutting down with status: %d", __FUNCTION__, urb->status);
			return;
                default:
			dbg("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
			return;
        }
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	if (port->tty && urb->actual_length){
#else
	if (port->port.tty && urb->actual_length){
#endif		
		if(mx_port->open){
			mx_recv(mx_port, urb->transfer_buffer, urb->actual_length);

			if(mx_port->sw_function & MX_IN_SW_ENABLE){
				if(!tty->stopped && mx_port->sw_tx_stopped){
					mx_port->sw_tx_stopped = 0;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,9))
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
					mxu2_write (port, NULL, 0);
#else
					mxu2_write (NULL, port, NULL, 0);
#endif
#else

					mxu2_write (port, 0, NULL, 0);
#endif
				}
			}
		}
		else
			dbg("%s - port closed, dropping data\n",__FUNCTION__);
		
		spin_lock_irqsave(&mx_port->mxp_slock,flags);
		mx_port->icount.rx += urb->actual_length;
		spin_unlock_irqrestore(&mx_port->mxp_slock,flags);
	}

	spin_lock_irqsave(&mx_port->mxp_slock,flags);
	if(mx_port->read_urb_state == MX_READ_URB_RUNNING){
		mx_port->read_urb->dev = serial->dev;
		status = usb_submit_urb(mx_port->read_urb, GFP_ATOMIC);
	}
	else if(mx_port->read_urb_state == MX_READ_URB_STOPPING){
		mx_port->read_urb_state = MX_READ_URB_STOPPED;
	}
	spin_unlock_irqrestore(&mx_port->mxp_slock,flags);

	if (status)
		dev_err(&port->dev, "%s - Error %d submitting bulk in urb\n", __FUNCTION__, status);
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void mxu2_bulk_out_callback (struct urb *urb, struct pt_regs *regs)
#else
static void mxu2_bulk_out_callback (struct urb *urb)
#endif
{
	struct mxu2_port *mx_port = (struct mxu2_port *)urb->context ;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = mx_port->port->tty;
#else
	struct tty_struct *tty = mx_port->port->port.tty;
#endif	

	/* Check port is valid or not */
	if(mx_port == NULL)
		return;

	mx_port->write_urb_in_use = MX_WRITE_URB_FREE;

	if(!urb || (mx_port == NULL) || !tty)
		return;

        switch (urb->status){
                case 0:
                        break;
                case -ECONNRESET:
                case -ENOENT:
                case -ESHUTDOWN:
                        dbg("%s - bulk out urb shutting down with status: %d", __FUNCTION__, urb->status);
			return;
                default:
                        dbg("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
			return;
        }

	if (tty){
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))
		schedule_work(&mx_port->port->work);
#else
		tty_wakeup(tty);
#endif
	}
	
	mx_send(mx_port);
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void mxu2_control_callback(struct urb *urb, struct pt_regs *regs)
#else
static void mxu2_control_callback(struct urb *urb)
#endif
{
	struct mxu2_port *mx_port = (struct mxu2_port *)urb->context;
	unsigned char *data;
	__u8 regval = 0;

	if(!urb || (mx_port == NULL))
                return;

        switch (urb->status){
                case 0:
                        break;
                case -ECONNRESET:
                case -ENOENT:
                case -ESHUTDOWN:
                        dbg("%s - urb shutting down with status: %d", __FUNCTION__, urb->status);                        return;
                default:
                        dbg("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
		return;
        }

	data=urb->transfer_buffer;
	regval=(__u8)data[0];

	if(mx_port->MsrLsr == MX_HANDLE_MSR)
		mx_handle_newMsr(mx_port,regval);
	else if(mx_port->MsrLsr == MX_HANDLE_LSR)
		mx_handle_newLsr(mx_port,regval);

	return;
}

//-----------------------------------------------------------------------------

#if defined(_SCREEN_INFO_H) || (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15))
static void mx_recv(struct mxu2_port *mx_port, unsigned char *data, int length)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = mx_port->port->tty;
#else
	struct tty_struct *tty = mx_port->port->port.tty;
#endif	
	int cnt;

	if(!tty)
		return;

	do{
	    	if (mx_port->read_urb_state == MX_READ_URB_STOPPING){
	       	    	dbg("%s - dropping data, %d bytes lost\n", __FUNCTION__, length);			    
            		break;
        	}
		
		cnt = tty_buffer_request_room(tty, length);

	        if(cnt < length){
			dbg("%s - dropping data, %d bytes lost\n", __FUNCTION__, length);			    
			break;            
		}

		cnt = length;

		tty_insert_flip_string(tty, data, cnt);

		length -= cnt;

		tty_flip_buffer_push(tty);

	}while(length > 0);

}
#else
static void mx_recv(struct mxu2_port *mx_port, unsigned char *data, int length)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = mx_port->port->tty;
#else
	struct tty_struct *tty = mx_port->port->port.tty;
#endif	
	unsigned char *ch;
	char *fp;
	int i, cnt;

	if(!tty)
		return;	

	ch = tty->flip.char_buf;
	fp = tty->flip.flag_buf;

	do {
	    	if (mx_port->read_urb_state == MX_READ_URB_STOPPING){
        	    	dbg("%s - dropping data, %d bytes lost\n", __FUNCTION__, length);			    
            		break;
        	}
        
	        cnt = tty->ldisc.receive_room(tty);
        
	        if(cnt < length){
		            dbg("%s - dropping data, %d bytes lost\n", __FUNCTION__, length);			    
		            break;            
		}
        
	        cnt = length;                    
        
	        for(i=0; i<cnt; i++){
	            *ch++=data[i];
	            *fp++=TTY_NORMAL;
	            tty->flip.count++;
	        }

		length -= cnt;

        	tty->ldisc.receive_buf(tty, 
                	               tty->flip.char_buf, 
                        	       tty->flip.flag_buf,
                               		cnt);
		if(tty->ldisc.receive_room(tty) < MX_THROTTLED_THRESHOLD){
			if(!test_and_set_bit(TTY_THROTTLED, &tty->flags) &&
			   tty->driver->throttle)
				tty->driver->throttle(tty);
		}
		
	} while (length > 0);
}
#endif


static void mx_send(struct mxu2_port *mx_port)
{
	struct mxu2_serial *mx_serial = mx_port->mxp_serial;
	struct usb_serial_port *port = mx_port->port;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = port->tty;
#else
	struct tty_struct *tty = port->port.tty;
#endif	
	int count, status;
	unsigned long flags;

	if((mx_port == NULL) || (mx_serial == NULL) || (port == NULL) || (tty == NULL))
		return;

	spin_lock_irqsave(&mx_port->mxp_slock, flags);

	if(mx_port->sw_tx_stopped){
		spin_unlock_irqrestore(&mx_port->mxp_slock, flags);
		return;
	}

	if(mx_port->write_urb_in_use){
		spin_unlock_irqrestore(&mx_port->mxp_slock, flags);
		return;
	}
	count = mx_buf_get(mx_port->write_buf,
				port->write_urb->transfer_buffer,
				port->bulk_out_size);

	if(count == 0){
		spin_unlock_irqrestore(&mx_port->mxp_slock, flags);
		return;
	}
	
	mx_port->write_urb_in_use = MX_WRITE_URB_BUSY;
	spin_unlock_irqrestore(&mx_port->mxp_slock, flags);
	
	if((mx_serial->model_type == MX_2_PORTS) && (mx_port->port_num == MX_PORT2)){
		usb_fill_bulk_urb(port->write_urb,
		                mx_serial->serial->dev,
		                usb_sndbulkpipe(mx_serial->serial->dev,
				(port->bulk_out_endpointAddress)+2),
				port->write_urb->transfer_buffer,
				count,
				mxu2_bulk_out_callback,
				mx_port);
	}
        else{
		usb_fill_bulk_urb(port->write_urb,
				mx_serial->serial->dev,
				usb_sndbulkpipe(mx_serial->serial->dev,
				port->bulk_out_endpointAddress),
				port->write_urb->transfer_buffer, 
				count,
				mxu2_bulk_out_callback,
				mx_port);
	}


	status = usb_submit_urb (port->write_urb,GFP_ATOMIC);

	if (status){
		dev_err(&port->dev, "%s - Error %d submitting write urb\n", __FUNCTION__, status);
		mx_port->write_urb_in_use = MX_WRITE_URB_FREE;
	}
	else{
		spin_lock_irqsave(&mx_port->mxp_slock,flags);
	 	mx_port->icount.tx += count;
		spin_unlock_irqrestore(&mx_port->mxp_slock,flags);
	}
}


static int mx_handle_newMsr(struct mxu2_port *mx_port,__u8 newMsr)
{
	struct async_icount *icount;

	if(mx_port == NULL)
		return -ENODEV;

	icount = &mx_port->icount;

                if (newMsr & MX_MSR_DELTA_CTS) 
                        icount->cts++;
                if (newMsr & MX_MSR_DELTA_DSR)
                        icount->dsr++;
                if (newMsr & MX_MSR_DELTA_CD)
                        icount->dcd++;
                if (newMsr & MX_MSR_DELTA_RI)
                        icount->rng++;

	return 0;
}


static int mx_handle_newLsr(struct mxu2_port *mx_port,__u8 newLsr)
{
        struct async_icount *icount;

        if (newLsr & MX_LSR_BI)
                newLsr &= (__u8)(MX_LSR_OE | MX_LSR_BI);

        icount = &mx_port->icount;

        if (newLsr & MX_LSR_BI) 
                icount->brk++;
	if (newLsr & MX_LSR_OE)
                icount->overrun++;
        if (newLsr & MX_LSR_PE)
                icount->parity++;
        if (newLsr & MX_LSR_FE)
                icount->frame++;

	return 0;
}


static int mx_get_reg(struct mxu2_port *mcs,__u16 Wval, __u16 reg, __u16 * val)
{
        struct usb_ctrlrequest *dr = NULL;
        struct usb_device *dev = mcs->port->serial->dev;
        unsigned char *buffer = NULL;
        int ret = 0;

	if(dev == NULL)
		return -ENODEV;

        buffer = mcs->ctrl_buf;

        dr = (void *)(buffer + 2);
        dr->bRequestType = MX_RD_RTYPE;
        dr->bRequest = MX_RDREQ;
        dr->wValue = cpu_to_le16(Wval);
        dr->wIndex = cpu_to_le16(reg);
        dr->wLength = cpu_to_le16(2);

        usb_fill_control_urb(mcs->control_urb,dev,usb_rcvctrlpipe(dev,0),(unsigned char *)dr,buffer,2,mxu2_control_callback,mcs);

        mcs->control_urb->transfer_buffer_length = 2;

        ret = usb_submit_urb(mcs->control_urb,GFP_ATOMIC);

        return ret;
}


static void mx_block_until_chase_response(struct mxu2_port *mx_port)
{
	int timeout = 1*HZ, wait = 10, count;

	while (1){

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
		count = mxu2_chars_in_buffer(mx_port->port);
#else

		count = mxu2_chars_in_buffer(mx_port->port->port.tty);
#endif
		
		if(count <= 0){
			return;
		}

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(timeout);
		set_current_state(TASK_RUNNING);
		wait--;

		if (wait == 0)
			return;
	}
}


static void mx_block_until_tx_empty (struct mxu2_port *mx_port)
{
	int timeout, wait = 20, count;
	
	timeout = (mx_port->closing_wait * HZ)/wait;

	while (1){

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
		count = mxu2_chars_in_buffer(mx_port->port);
#else

		count = mxu2_chars_in_buffer(mx_port->port->port.tty);
#endif

		if(count <= 0)
			return;

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(timeout);
		set_current_state(TASK_RUNNING);
		wait--;

		if (wait == 0)
			return;
	}
}	


static int mx_get_lsr_info(struct mxu2_port *mx_port, unsigned int *value)
{
	unsigned int result = 0;
	int count;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
        count = mxu2_chars_in_buffer(mx_port->port);
#else

        count = mxu2_chars_in_buffer(mx_port->port->port.tty);
#endif

        if(count == 0)
		result = TIOCSER_TEMT;

	if (copy_to_user(value, &result, sizeof(int)))
		return -EFAULT;

	return 0;
}


static int mx_get_modem_info(struct mxu2_port *mx_port, unsigned int *value)
{
	unsigned int mcr = mx_port->shadowMCR, result = 0;
	__u16 msr;
	int status;

	status = mx_get_Uart_Reg(mx_port->port,MODEM_STATUS_REGISTER,&msr);
	if(status < 0)
		return status;

	result = ((mcr & MX_MCR_DTR) ? TIOCM_DTR : 0)	  /* 0x002 */
	       | ((mcr & MX_MCR_RTS) ? TIOCM_RTS : 0)   /* 0x004 */
	       | ((msr & MX_MSR_CTS) ? TIOCM_CTS : 0)   /* 0x020 */
	       | ((msr & MX_MSR_CD)  ? TIOCM_CAR : 0)   /* 0x040 */
	       | ((msr & MX_MSR_RI)  ? TIOCM_RI  : 0)   /* 0x080 */
	       | ((msr & MX_MSR_DSR) ? TIOCM_DSR : 0);  /* 0x100 */

	if (copy_to_user(value, &result, sizeof(int)))
		return -EFAULT;

	return 0;
}


static int mx_set_modem_info(struct mxu2_port *mx_port, unsigned int cmd, unsigned int *value)
{
	struct usb_serial_port *port = mx_port->port;
	unsigned int mcr, arg;
	__u16 Data;
	int status;

	if ((mx_port == NULL) || (port == NULL))
		return -ENODEV;

	mcr = mx_port->shadowMCR;

	if (copy_from_user(&arg, value, sizeof(int)))
		return -EFAULT;

	switch (cmd) {
		case TIOCMBIS:
			if (arg & TIOCM_RTS)
				mcr |= MX_MCR_RTS;
			if (arg & TIOCM_DTR)
				mcr |= MX_MCR_RTS;
			if (arg & TIOCM_LOOP)
				mcr |= MX_MCR_LOOPBACK;
			break;
		case TIOCMBIC:
			if (arg & TIOCM_RTS)
				mcr &= ~MX_MCR_RTS;
			if (arg & TIOCM_DTR)
				mcr &= ~MX_MCR_RTS;
			if (arg & TIOCM_LOOP)
				mcr &= ~MX_MCR_LOOPBACK;
			break;
		case TIOCMSET:
			mcr &=  ~(MX_MCR_RTS | MX_MCR_DTR | MX_MCR_LOOPBACK);
			mcr |= ((arg & TIOCM_RTS) ? MX_MCR_RTS : 0);
			mcr |= ((arg & TIOCM_DTR) ? MX_MCR_DTR : 0);
			mcr |= ((arg & TIOCM_LOOP) ? MX_MCR_LOOPBACK : 0);
			break;
	}

	mx_port->shadowMCR = mcr;

	Data = mx_port->shadowMCR;
	status = mx_set_Uart_Reg(port,MODEM_CONTROL_REGISTER,Data);
	if(status < 0)
		return status;

	return 0;
}


static int mx_get_serial_info(struct mxu2_port *mx_port, struct serial_struct * ret_arg)
{
	struct serial_struct tmp;

	if (mx_port == NULL)
		return -ENODEV;

	if (!ret_arg)
		return -EFAULT;

	memset(&tmp, 0, sizeof(tmp));

	tmp.type		= PORT_16550A;
	tmp.line		= mx_port->port->serial->minor;
	tmp.port		= mx_port->port->number;
	tmp.irq			= 0;
	tmp.flags		= mx_port->flags | ASYNC_LOW_LATENCY;
        tmp.xmit_fifo_size      = MX_WRITE_BUF_SIZE;
	tmp.baud_base		= 921600;
	tmp.close_delay		= mx_port->close_delay;
	tmp.closing_wait	= mx_port->closing_wait;

	if (copy_to_user(ret_arg, &tmp, sizeof(*ret_arg)))
		return -EFAULT;

	return 0;
}


static int mx_set_serial_info(struct mxu2_port *mx_port, struct serial_struct * new_arg)
{
	struct serial_struct new_serial;
	unsigned int baud=0;

	if(copy_from_user(&new_serial, new_arg, sizeof(new_serial)))
		return -EFAULT;

	if(new_serial.flags & MX_SPD_MASK){
		mx_port->flags = new_serial.flags;

		if(new_serial.flags & ASYNC_SPD_HI)
			baud = 57600;
		if(new_serial.flags & ASYNC_SPD_VHI)
			baud = 115200;
		if(new_serial.flags & ASYNC_SPD_SHI)
			baud = 230400;
		if(new_serial.flags & ASYNC_SPD_WARP)
			baud = 460800;

		if(new_serial.flags & MX_SPD_MASK)
			mx_send_cmd_write_baud_rate (mx_port, baud);
	}
	
	mx_port->close_delay = new_serial.close_delay;
	mx_port->closing_wait = new_serial.closing_wait;

	return	0;
}


static int mx_send_cmd_write_baud_rate (struct mxu2_port *mx_port, int baudRate)
{
	struct usb_serial_port *port = mx_port->port;
	unsigned char number ;
	int divisor = 0, status;
	__u16 Data;

	if ((mx_port == NULL) || (port == NULL))
		return -ENODEV;

	number = mx_port->port_num - 1;

	mx_calc_baud_rate_divisor(baudRate, &divisor);
	status = mx_get_reg_sync(port,mx_port->SpRegOffset,&Data);
	if(status < 0)		
		return status;

	Data = (Data & MX_SPR_EXCLUDE_CLK_MASK)| MX_SPR_147456;
	status = mx_set_reg_sync(port,mx_port->SpRegOffset,Data);
	if(status < 0)		
		return status;

        Data = mx_port->shadowLCR | MX_LCR_DLAB;
        mx_port->shadowLCR  = Data;
	status = mx_set_Uart_Reg(port,LINE_CONTROL_REGISTER,Data);
	if(status < 0)		
		return status;

	Data = LOW8 (divisor);
	status = mx_set_Uart_Reg(port,DIVISOR_LATCH_LSB,Data);
	if(status < 0)		
		return status;
	
	Data = HIGH8 (divisor);
	status = mx_set_Uart_Reg(port,DIVISOR_LATCH_MSB,Data);
	if(status < 0)		
		return status;
	
        Data = mx_port->shadowLCR & ~MX_LCR_DLAB;
        mx_port->shadowLCR = Data;
	status = mx_set_Uart_Reg(port,LINE_CONTROL_REGISTER,Data);
	
	return status;
}


static void mx_calc_baud_rate_divisor (int baudRate, int *divisor)
{
	*divisor = 921600/baudRate;
}


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
static void mx_change_port_settings (struct mxu2_port *mx_port, struct termios *old_termios)
#else
static void mx_change_port_settings (struct mxu2_port *mx_port, struct ktermios *old_termios)
#endif
{
	struct usb_serial_port *port = (struct usb_serial_port *)mx_port->port;
	struct usb_serial *serial = port->serial;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = port->tty;
#else
	struct tty_struct *tty = port->port.tty;
#endif	
	unsigned int baud, old_baud;
	tcflag_t cflag,iflag;
	__u16 Data;
	__u8 lData, lParity = 0, lStop;

	if((mx_port == NULL) || (port == NULL) || (serial == NULL))
		return ;

	if (!mx_port->open && !mx_port->openPending)
		return;

	if (!tty || !tty->termios)
		return;

	cflag = tty->termios->c_cflag;
	iflag = tty->termios->c_iflag;

	switch (cflag & CSIZE){
		case CS5:
			lData = MX_LCR_BITS_5; 
			break;
		case CS6:
			lData = MX_LCR_BITS_6; 
			break;
		case CS7:
			lData = MX_LCR_BITS_7; 
			break;
		case CS8:
		default:
			lData = MX_LCR_BITS_8;
			break;
	}

	if (cflag & PARENB){
		if (cflag & PARODD)
			lParity = MX_LCR_PAR_ODD;
		else
			lParity = MX_LCR_PAR_EVEN;
	} 
	
	if(cflag & CMSPAR)
		lParity = lParity | 0x20;

	if (cflag & CSTOPB)
		lStop = MX_LCR_STOP_2;
	else
		lStop = MX_LCR_STOP_1;

	mx_port->shadowLCR &= ~(MX_LCR_BITS_MASK | MX_LCR_STOP_MASK | MX_LCR_PAR_MASK);
	mx_port->shadowLCR |= (lData | lParity | lStop);

	Data = mx_port->shadowLCR;
	mx_set_Uart_Reg(port,LINE_CONTROL_REGISTER,Data);
	
	// Follows needs to be check??? If there are some problem, you can uncomment these
	//Data = 0x0b;
	//mx_port->shadowMCR = Data;
	//mx_set_Uart_Reg(port,MODEM_CONTROL_REGISTER,Data);

	mx_port->shadowMCR = MX_MCR_MASTER_IE;

	if (!(cflag & CBAUD)) {
		mx_port->set_B0 = true;
		mx_port->shadowMCR &= ~(MX_MCR_DTR | MX_MCR_RTS);
	}
	else {	
		mx_port->shadowMCR |= (MX_MCR_DTR | MX_MCR_RTS);
		mx_port->set_B0 = false;	
	}
	
	if ((cflag & CRTSCTS) && (cflag & CBAUD)) 
		mx_port->shadowMCR |= (MX_MCR_XON_ANY);
	else {
		mx_port->shadowMCR |= MX_MCR_RTS;
		mx_port->shadowMCR &= ~(MX_MCR_XON_ANY);
	}

	Data = mx_port->shadowMCR;

	mx_set_Uart_Reg(port,MODEM_CONTROL_REGISTER,Data);
	
	if (iflag & IXON){
		mx_port->sw_function |= MX_IN_SW_ENABLE;
	}
	else{
		mx_port->sw_function &= ~MX_IN_SW_ENABLE;
		tty->stopped = 0;
		mx_port->sw_tx_stopped = MX_SW_TX_RUNNING;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,9))

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
		mxu2_write (port, NULL, 0);
#else

		mxu2_write (NULL ,port, NULL, 0);
#endif
#else

		mxu2_write (port, 0, NULL, 0);
#endif
	}
	
	baud = tty_get_baud_rate(tty);
	old_baud = tty_termios_baud_rate(old_termios);

	if (!baud)
		baud = 9600;

	if(baud != old_baud)
		mx_send_cmd_write_baud_rate (mx_port, baud);

	return;
}



static int mx_set_reg_sync(struct usb_serial_port *port, __u16 reg, __u16  val)
{
        struct usb_device *dev = port->serial->dev;

	if(dev == NULL)
		return -ENODEV;

	val = val & 0x00ff;

        return usb_control_msg(dev, usb_sndctrlpipe(dev, 0), MX_WRREQ,
                        MX_WR_RTYPE, val, reg, NULL, 0,MX_WDR_TIMEOUT);
}


static int mx_get_reg_sync(struct usb_serial_port *port, __u16 reg, __u16 * val)
{
        struct usb_device *dev = port->serial->dev;
        int ret = 0;

	if(dev == NULL)
		return -ENODEV;

        ret = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), MX_RDREQ,
                        MX_RD_RTYPE, 0, reg, val, VENDOR_READ_LENGTH,MX_WDR_TIMEOUT);
	*val = (*val) & 0x00ff;

        return ret;
}


static int mx_set_Uart_Reg(struct usb_serial_port *port, __u16 reg, __u16  val)
{
	struct mxu2_serial *mx_serial = usb_get_serial_data(port->serial);
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
        struct usb_device *dev = port->serial->dev;

	if((dev == NULL) || (mx_port == NULL))
		return -ENODEV;

	val = val & 0x00ff;
	
	if(mx_serial->model_type == MX_4_PORTS){
	        val |= ((__u16)mx_port->port_num)<<8;
	}
	else{
		if( mx_port->port_num == 1)
		        val |= ((__u16)mx_port->port_num)<<8;
		else
		        val |= ((__u16)(mx_port->port_num+1))<<8;
	}

        return usb_control_msg(dev, usb_sndctrlpipe(dev, 0), MX_WRREQ,
                        MX_WR_RTYPE, val, reg, NULL, 0,MX_WDR_TIMEOUT);
}


static int mx_get_Uart_Reg(struct usb_serial_port *port, __u16 reg, __u16 * val)
{
	struct mxu2_serial *mx_serial = usb_get_serial_data(port->serial);
	struct mxu2_port *mx_port = usb_get_serial_port_data(port);
        struct usb_device *dev = port->serial->dev;
        __u16 Wval;
        int ret = 0;

	if((dev == NULL) || (mx_port == NULL))
		return -ENODEV;

	if(mx_serial->model_type ==MX_4_PORTS){
        	Wval=((__u16)mx_port->port_num)<<8;
	}
	else{
		if(mx_port->port_num == 1)
	        	Wval=((__u16)mx_port->port_num)<<8;
		else
	        	Wval=((__u16)(mx_port->port_num+1))<<8;
	}

	ret = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), MX_RDREQ,
                        MX_RD_RTYPE, Wval, reg, val,VENDOR_READ_LENGTH,MX_WDR_TIMEOUT);

	*val = (*val) & 0x00ff;

        return ret;
}


static int mx_set_reg_offset(struct mxu2_port *mx_port)
{
	struct mxu2_serial *mx_serial = mx_port->mxp_serial;

	switch(mx_serial->model_type){
		case MX_2_PORTS:
			if(mx_port->port_num == MX_PORT1){
				mx_port->SpRegOffset = 0x00;
				mx_port->ControlRegOffset = 0x01;
				mx_port->DcrRegOffset = 0x04;
			}
			else if(mx_port->port_num == MX_PORT2){
				mx_port->SpRegOffset = 0x0a;
				mx_port->ControlRegOffset = 0x0b;
				mx_port->DcrRegOffset = 0x19;
			}
			else
				return -ENODEV;

			break;
		case MX_4_PORTS:
			if(mx_port->port_num == MX_PORT1){
				mx_port->SpRegOffset = 0x00;
				mx_port->ControlRegOffset = 0x01;
				mx_port->DcrRegOffset = 0x04 ;
			}
			else if(mx_port->port_num == MX_PORT2){
				mx_port->SpRegOffset = 0x08;
				mx_port->ControlRegOffset = 0x09;
				mx_port->DcrRegOffset = 0x16;
			}
			else if(mx_port->port_num == MX_PORT3){
				mx_port->SpRegOffset = 0x0a;
				mx_port->ControlRegOffset = 0x0b;
				mx_port->DcrRegOffset = 0x19;
			}
			else if(mx_port->port_num == MX_PORT4){
				mx_port->SpRegOffset = 0x0c;
				mx_port->ControlRegOffset = 0x0d;
				mx_port->DcrRegOffset = 0x1c ;
			}
			else
				return -ENODEV;
		
			break;
		default:
			return -ENODEV;
	}

	return mx_serial->model_type;
}


static int mx_init_startup_regs(struct mxu2_port *mx_port)
{
	struct mxu2_serial *mx_serial = mx_port->mxp_serial;
	__u16 Data;
	int status;

	if((mx_serial == NULL)||(mx_port == NULL))
		return -ENODEV;
	
	Data = (MX_CONR_DRV_DONE | MX_CONR_CTS_SIGNAL_ENABLE |
		MX_CONR_AUTO_HW | MX_CONR_RX_DISABLE);
	status = mx_set_reg_sync(mx_port->port,mx_port->ControlRegOffset,Data);
	if(status < 0) 
		return status; 
	
	Data = MX_DCR0_USB_SUSPEND_TX | MX_DCR0_RTS_EMPTY_LOW |
	       MX_DCR0_SET_GPIO_OUT;
	status = mx_set_reg_sync(mx_port->port,(__u16)(mx_port->DcrRegOffset+0),Data);
	if(status < 0) 
		return status; 

	if(mx_serial->product_name != MOXA_DEVICE_ID_2450){
		status = mx_set_beeper_led(mx_port,MX_DEVICE_ON);
		if(status < 0)
			return status;
	}
	
	Data = MX_DCR1_GPIO_CURRENT_8 | MX_DCR1_TX_CURRENT_8;
	status = mx_set_reg_sync(mx_port->port,(__u16)(mx_port->DcrRegOffset+1),Data);
	if(status < 0) 
		return status; 
	
	Data = MX_DCR2_EWU_RI | MX_DCR2_RWM_RESUME;
	status = mx_set_reg_sync(mx_port->port,(__u16)(mx_port->DcrRegOffset+2),Data);
	if(status < 0) 
		return status; 
	
	Data = 0x00;
	status = mx_set_reg_sync(mx_port->port,CLK_START_VALUE_REGISTER,Data);
	if(status < 0) 
		return status; 
	
	Data = 0x20;
	status = mx_set_reg_sync(mx_port->port,CLK_MULTI_REGISTER,Data);
	if(status < 0) 
		return status; 
	
	if(mx_serial->model_type == MX_2_PORTS ){
		Data = 0xff;
		status = mx_set_reg_sync(mx_port->port,
				(__u16)(ZLP_REG1+((__u16)mx_port->port_num)),Data);
		if(status < 0) 
			return status; 
	}
	else{
		Data = 0xff;
	        status = mx_set_reg_sync(mx_port->port,
	                 (__u16)(ZLP_REG1+((__u16)mx_port->port_num)-0x1),Data);
		if(status < 0)
			return status; 
	}	

	return status;
}


static int mx_init_open_regs(struct mxu2_port *mx_port)
{
	struct mxu2_serial *mx_serial = mx_port->mxp_serial;
	int status;
	__u16 Data;

	Data = (MX_SPR_RESET_UART | MX_SPR_RESET_BULK_IN_FIFO |
		MX_SPR_RESET_BULK_OUT_FIFO);
	status = mx_set_reg_sync(mx_port->port,mx_port->SpRegOffset,Data);
	if(status < 0)
		return status;
	
	Data = 0x00;
	status = mx_set_reg_sync(mx_port->port,mx_port->SpRegOffset,Data);
	if(status < 0)
		return status;
	
	Data = 0x00;
	status = mx_set_Uart_Reg(mx_port->port,INTERRUPT_ENABLE_REGISTER,Data);
	if(status < 0)
		return status;

	Data = 0x00;
	status = mx_set_Uart_Reg(mx_port->port,FIFO_CONTROL_REGISTER,Data);
	if(status < 0)
		return status;

	Data = (MX_FCR_FIFO_ENABLE | MX_FCR_FLUSH_RHR | MX_FCR_FLUSH_THR |
		MX_FCR_DMA | MX_FCR_RHR_TRI_14);  
	status = mx_set_Uart_Reg(mx_port->port,FIFO_CONTROL_REGISTER,Data);
	if(status < 0)
		return status;

	Data = (MX_LCR_BITS_8 | MX_LCR_STOP_1 | MX_LCR_PAR_NONE);
	status = mx_set_Uart_Reg(mx_port->port,LINE_CONTROL_REGISTER,Data);
	if(status < 0)
		return status;
	mx_port->shadowLCR = Data;

	Data = MX_MCR_DTR | MX_MCR_RTS | MX_MCR_MASTER_IE;
	status = mx_set_Uart_Reg(mx_port->port,MODEM_CONTROL_REGISTER,Data);
	if(status < 0)
		return status;
	mx_port->shadowMCR = Data;

	Data = MX_IER_LS | MX_IER_MS;
        status = mx_set_Uart_Reg(mx_port->port,INTERRUPT_ENABLE_REGISTER,Data);
	if(status < 0)
		return status;

        status = mx_get_reg_sync(mx_port->port,mx_port->ControlRegOffset,&Data);
	if(status < 0)
		return status;

	if(mx_serial->op_mode != MX_1_OP_MODE){
		status = mx_set_interface(mx_port, mx_port->interface);
		if(status < 0)
			return status;
	}

	Data = Data & ~MX_CONR_RX_DISABLE;
        status = mx_set_reg_sync(mx_port->port,mx_port->ControlRegOffset,Data);

	return status;
}


static void mx_get_op_mode(struct usb_serial *serial)
{
	struct mxu2_serial *mx_serial = usb_get_serial_data(serial);

	if(usb_match_id(serial->interface, mxu2_2210_id_table)){
		mx_serial->op_mode = MX_1_OP_MODE;
		mx_serial->product_name = MOXA_DEVICE_ID_2210;
	}
	
	if(usb_match_id(serial->interface, mxu2_2250_id_table)){
		mx_serial->op_mode = MX_3_OP_MODES;
		mx_serial->product_name = MOXA_DEVICE_ID_2250;
	}
	
	if(usb_match_id(serial->interface, mxu2_2410_id_table)){
		mx_serial->op_mode = MX_1_OP_MODE;
		mx_serial->product_name = MOXA_DEVICE_ID_2410;
	}	

	if(usb_match_id(serial->interface, mxu2_2450_id_table)){
		mx_serial->op_mode = MX_3_OP_MODES;
		mx_serial->product_name = MOXA_DEVICE_ID_2450;
	}
}


static int mx_set_interface(struct mxu2_port *mx_port, unsigned char sval)
{
	struct mxu2_serial *mx_serial = mx_port->mxp_serial;
	unsigned char val,index,shift_offset;
	__u16 Data;
	int status;
	
	index = mx_port->port_num - 1;
	shift_offset = index*2;
	val = mx_serial->port_intf_status;

	if(mx_serial->op_mode == MX_1_OP_MODE){
		return -EPERM;
	}

	if(mx_serial->op_mode == MX_3_OP_MODES){
		val &= ~(0x03 << index*2);

		switch(sval){
			case MX_RS232:
				Data = MX_RS485_DISABLE;
				val |= (0x03 << shift_offset);
				break;
			case MX_RS422:
				Data = MX_RS485_DISABLE;
				val |= (0x01 << shift_offset);
				break;
			case MX_RS485_2W:
				Data = MX_RS485_ENABLE;
				break;
			case MX_RS485_4W:
				val |= (0x02 << shift_offset);
				Data = MX_RS485_ENABLE;
				break;
			default:
				return -EPERM;
		}
		
		mx_serial->port_intf_status = val;
		mx_port->interface = sval;
		status = mx_set_Uart_Reg(mx_port->port,SCRATCH_PAD_REGISTER,Data);
		if(status < 0)
			return status;

		mx_gpio_out(mx_port,MX_CMD_CFG_P0,0x00);
		mx_gpio_out(mx_port,MX_CMD_OUT_P0,val);
	}
	
	return 0;
}


static int mx_set_beeper_led(struct mxu2_port *mx_port,unsigned char sval)
{
	struct mxu2_serial *mx_serial = mx_port->mxp_serial;
	unsigned char val;

	if(mx_serial->op_mode == MX_1_OP_MODE){	//UPort 2210/2410
		switch (sval){
			case MX_DEVICE_ON:
				gpio_write_pin(mx_port,MX_GPIO_1,MX_PULL_HIGH);
				gpio_write_pin(mx_port,MX_GPIO_0,MX_PULL_LOW);
				break;
			case MX_DEVICE_OFF:
				gpio_write_pin(mx_port,MX_GPIO_1,MX_PULL_LOW);
				gpio_write_pin(mx_port,MX_GPIO_0,MX_PULL_HIGH);
				break;
			default:
				return -EPERM;
		}

		return 0;
	}

	if(mx_serial->product_name == MOXA_DEVICE_ID_2250){ //UPort 2250
		switch (sval){
			case MX_DEVICE_ON:
				mx_serial->port_intf_status |= MX_2_PORT_ON;
				break;
			case MX_DEVICE_OFF:
				mx_serial->port_intf_status &= ~MX_2_PORT_ON;
				break;
			default:
				return -EPERM;
		}

		val = mx_serial->port_intf_status;
		mx_gpio_out(mx_port,MX_CMD_CFG_P0,0x00);
		mx_gpio_out(mx_port,MX_CMD_OUT_P0,val);
	}

	if(mx_serial->product_name == MOXA_DEVICE_ID_2450){ //UPort 2450
		switch (sval){
			case MX_DEVICE_ON:
				val = MX_4_PORT_ON;
				break;
			case MX_DEVICE_OFF:
				val = MX_4_PORT_OFF;
				break;
			default:
				return -EPERM;
		}

		mx_gpio_out(mx_port,MX_CMD_CFG_P1,0x00);
		mx_gpio_out(mx_port,MX_CMD_OUT_P1,val);
	}

	return 0;
}


static int mx_gpio_out(struct mxu2_port *mx_port,unsigned char cmd,
			   unsigned char val)
{
	unsigned char start_byte;
	int i;
	
	start_byte = 0x44;
	for(i = 0;i < 10;i++){
		if(i2c_start(mx_port,start_byte))
			break;
	}

	if(i == 10)
		return -EIO;

	if(!i2c_putb(mx_port,cmd))
		return -EIO;
		
	if(!i2c_putb(mx_port,val))
		return -EIO;

	i2c_stop(mx_port);
	
	return 0;
}

//-----------------------------------------------------------------------------


static struct circ_buf *mx_buf_alloc(void)
{
	struct circ_buf *cb;
	
	cb = (struct circ_buf *)kmalloc(sizeof(struct circ_buf), GFP_KERNEL);
	if(cb == NULL)
		return NULL;
	
	cb->buf = kmalloc(MX_WRITE_BUF_SIZE, GFP_KERNEL);
	if(cb->buf == NULL){
		kfree(cb);
		return NULL;
	}

	mx_buf_clear(cb);

	return cb;
}


static void mx_buf_free(struct circ_buf *cb)
{
	kfree(cb->buf);
	kfree(cb);
}


static void mx_buf_clear(struct circ_buf *cb)
{
	cb->head = cb->tail = 0;
}


static int mx_buf_data_avail(struct circ_buf *cb)
{
	return CIRC_CNT(cb->head, cb->tail, MX_WRITE_BUF_SIZE);
}


static int mx_buf_space_avail(struct circ_buf *cb)
{
	return CIRC_SPACE(cb->head, cb->tail, MX_WRITE_BUF_SIZE);
}

static int mx_buf_put(struct mxu2_port *mx_port, const char *buf, int count)
{
	int c, ret=0;
	struct circ_buf *cb;
	unsigned long flags;

	spin_lock_irqsave(&mx_port->mxp_slock, flags);

	cb = mx_port->write_buf;
	
	while(1){
		c = CIRC_SPACE_TO_END(cb->head, cb->tail, MX_WRITE_BUF_SIZE);
	
		if(count < c)
			c = count;

		if(c <= 0)
			break;

		memcpy(cb->buf + cb->head, buf, c);

		cb->head = (cb->head + c) & (MX_WRITE_BUF_SIZE-1);
		buf += c;
		count -= c;
		ret += c;
	}

	spin_unlock_irqrestore(&mx_port->mxp_slock, flags);

	return ret;
}


static int mx_buf_get(struct circ_buf *cb, char *buf, int count)
{
	int c, ret = 0;
	while(1){
		c = CIRC_CNT_TO_END(cb->head, cb->tail, MX_WRITE_BUF_SIZE);
		if(count < c)
			c = count;
		if(c <= 0)
			break;
		memcpy(buf, cb->buf + cb->tail, c);
		cb->tail = (cb->tail + c) & (MX_WRITE_BUF_SIZE-1);
		buf += c;
		count -= c;
		ret += c;
	}

	return ret;
}


static int gpio_write_pin(struct mxu2_port *mx_port, unsigned char gpio_port,
			unsigned char wval)
{
	struct mxu2_serial *mx_serial = mx_port->mxp_serial;
	int status;
	__u16 Data;
	
	Data = mx_serial->shadow_gpio;

	if(wval)
		Data |= gpio_port;
	else
		Data &= ~gpio_port;

	status = mx_set_reg_sync(mx_port->port,GPIO_REGISTER,Data);
	
	mx_serial->shadow_gpio = Data;
	
	return status; 
}


static int gpio_read_pin(struct mxu2_port *mx_port, unsigned char gpio_port)
{
	struct mxu2_serial *mx_serial = mx_port->mxp_serial;
	int status;
	__u16 Data;
	unsigned char ret;

	status = mx_get_reg_sync(mx_port->port,GPIO_REGISTER,&Data);
	if(status < 0) 
		return status; 

	mx_serial->shadow_gpio = Data;	

	if((unsigned char)Data & gpio_port)
		ret = 0;
	else
		ret = 1;
	
	return ret;
}


static int i2c_clk_hi(struct mxu2_port *mx_port)
{
	return gpio_write_pin(mx_port,MX_SCL,MX_PULL_HIGH);
}


static int i2c_clk_low(struct mxu2_port *mx_port)
{
	return gpio_write_pin(mx_port,MX_SCL,MX_PULL_LOW);
}


static int i2c_data_hi(struct mxu2_port *mx_port)
{
	return gpio_write_pin(mx_port,MX_SDA,MX_PULL_HIGH);
}


static int i2c_data_low(struct mxu2_port *mx_port)
{
	return gpio_write_pin(mx_port,MX_SDA,MX_PULL_LOW);
}


static int i2c_start(struct mxu2_port *mx_port, unsigned char val)
{
	int i,ret;

	i2c_clk_hi(mx_port);
	i2c_data_hi(mx_port);
	i2c_data_low(mx_port);

	for(i = 7;i >= 0;i--){
		i2c_clk_low(mx_port);

		if(val & (0x01 << i))
			i2c_data_hi(mx_port);
		else
			i2c_data_low(mx_port);

		i2c_clk_hi(mx_port);
	}

	i2c_clk_low(mx_port);
	i2c_data_low(mx_port);
	i2c_clk_hi(mx_port);
	ret = gpio_read_pin(mx_port,MX_SDA);
	i2c_clk_low(mx_port);
	i2c_data_hi(mx_port);

	return ret;
}


static void i2c_stop(struct mxu2_port *mx_port)
{
	i2c_data_hi(mx_port);
	i2c_data_low(mx_port);
	i2c_clk_hi(mx_port);
	i2c_data_hi(mx_port);
	i2c_clk_low(mx_port);
	i2c_clk_hi(mx_port);
}


static int i2c_putb(struct mxu2_port *mx_port, unsigned char val)
{
	int i,ret;
	
	i2c_data_hi(mx_port);

	for(i = 7;i >= 0;i--){
		if(val & (0x01 << i))
			i2c_data_hi(mx_port);
		else
			i2c_data_low(mx_port);

		i2c_clk_hi(mx_port);
		i2c_clk_low(mx_port);
	}
	i2c_data_low(mx_port);
	i2c_clk_hi(mx_port);
	ret = gpio_read_pin(mx_port,MX_SDA);
	i2c_clk_low(mx_port);
	i2c_data_hi(mx_port);
	i2c_data_hi(mx_port);

	return ret;	
}
