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

//#define MX_DEBUG	1

#ifdef MX_DEBUG
static int debug = 1;
#define DPRINTK(fmt, args...) printk( "%s: " fmt, __FUNCTION__ , ## args)
#else
static int debug = 0;
#define DPRINTK(fmt, args...)
#endif

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef LOW8
#define LOW8(val) ((unsigned char)(val & 0xff))
#endif

#ifndef HIGH8
#define HIGH8(val) ((unsigned char)((val & 0xff00) >> 8))
#endif

#define RAID_REG1	 		0x30
#define RAID_REG2 			0x31

#define ZLP_REG1  			0x3A
#define ZLP_REG2  			0x3B
#define ZLP_REG3  			0x3C
#define ZLP_REG4  			0x3D
#define ZLP_REG5  			0x3E

#ifndef SERIAL_MAGIC
#define SERIAL_MAGIC			0x6702
#endif

#define PORT_MAGIC			0x7301

#define USB_VENDOR_ID_MXUPORT		0x110a
#define MOXA_DEVICE_ID_2410		0x2410
#define MOXA_DEVICE_ID_2450		0x2450
#define MOXA_DEVICE_ID_2210		0x2210
#define MOXA_DEVICE_ID_2250		0x2250

#define MX_IIR_RLS      		0x06
#define MX_IIR_RDA      		0x04
#define MX_IIR_CTI      		0x0c
#define MX_IIR_THR      		0x02
#define MX_IIR_MS       		0x00

#define MX_LSR_DR       		0x0001
#define MX_LSR_OE       		0x0002
#define MX_LSR_PE       		0x0004
#define MX_LSR_FE       		0x0008
#define MX_LSR_BI       		0x0010
#define MX_LSR_THRE     		0x0020
#define MX_LSR_TEMT     		0x0040
#define MX_LSR_FIFOERR  		0x0080

#define MX_LCR_BITS_5			0x00
#define MX_LCR_BITS_6			0x01
#define MX_LCR_BITS_7			0x02
#define MX_LCR_BITS_8			0x03
#define MX_LCR_BITS_MASK		0x03

#define MX_LCR_STOP_1			0x00
#define MX_LCR_STOP_1_5			0x04
#define MX_LCR_STOP_2			0x04
#define MX_LCR_STOP_MASK		0x04

#define MX_LCR_PAR_NONE			0x00
#define MX_LCR_PAR_ODD			0x08
#define MX_LCR_PAR_EVEN			0x18
#define MX_LCR_PAR_MARK			0x28
#define MX_LCR_PAR_SPACE		0x38
#define MX_LCR_PAR_MASK			0x38

#define MX_LCR_SET_BREAK		0x40
#define MX_LCR_DL_ENABLE		0x80

#define MX_MCR_DTR			0x01
#define MX_MCR_RTS			0x02
#define MX_MCR_OUT1			0x04
#define MX_MCR_MASTER_IE		0x08
#define MX_MCR_LOOPBACK			0x10 
#define MX_MCR_XON_ANY			0x20
#define MX_MCR_IR_ENABLE		0x40
#define MX_MCR_BRG_DIV_4		0x80

#define MX_MSR_DELTA_CTS		0x01
#define MX_MSR_DELTA_DSR		0x02
#define MX_MSR_DELTA_RI			0x04
#define MX_MSR_DELTA_CD			0x08
#define MX_MSR_CTS			0x10
#define MX_MSR_DSR			0x20
#define MX_MSR_RI			0x40
#define MX_MSR_CD			0x80

#define MX_FCR_FIFO_ENABLE		0x01
#define MX_FCR_FLUSH_RHR		0x02
#define MX_FCR_FLUSH_THR		0x04
#define MX_FCR_DMA			0x08
#define MX_FCR_RHR_TRI_14		0x0c

#define MX_IER_THR			0x02
#define MX_IER_LS			0x04
#define MX_IER_MS			0x08

#define RECEIVE_BUFFER_REGISTER    	((__u16)(0x00))
#define TRANSMIT_HOLDING_REGISTER  	((__u16)(0x00))
#define INTERRUPT_ENABLE_REGISTER  	((__u16)(0x01))
#define INTERRUPT_IDENT_REGISTER   	((__u16)(0x02))
#define FIFO_CONTROL_REGISTER      	((__u16)(0x02))
#define LINE_CONTROL_REGISTER      	((__u16)(0x03))
#define MODEM_CONTROL_REGISTER     	((__u16)(0x04))
#define LINE_STATUS_REGISTER       	((__u16)(0x05))
#define MODEM_STATUS_REGISTER      	((__u16)(0x06))
#define SCRATCH_PAD_REGISTER       	((__u16)(0x07))
#define DIVISOR_LATCH_LSB          	((__u16)(0x00))
#define DIVISOR_LATCH_MSB          	((__u16)(0x01))

#define SP_REGISTER_BASE           	((__u16)(0x08))
#define CONTROL_REGISTER_BASE      	((__u16)(0x09))
#define DCR_REGISTER_BASE          	((__u16)(0x16))

#define SP1_REGISTER               	((__u16)(0x00))
#define CONTROL1_REGISTER          	((__u16)(0x01))
#define CLK_MULTI_REGISTER         	((__u16)(0x02))
#define CLK_START_VALUE_REGISTER   	((__u16)(0x03))
#define DCR1_REGISTER              	((__u16)(0x04))
#define GPIO_REGISTER              	((__u16)(0x07))

#define CLOCK_SELECT_REG1          	((__u16)(0x13))
#define CLOCK_SELECT_REG2          	((__u16)(0x14))

#define MX_LCR_DLAB       	   	((__u16)(0x0080))

#define MX_SPR_BULK_LOOP		0x01
#define MX_SPR_IGNORE_UART_ERROR_DATA	0x02
#define MX_SPR_RESET_BULK_OUT_FIFO	0x04
#define MX_SPR_RESET_BULK_IN_FIFO	0x08
#define MX_SPR_RESET_UART		0x80
#define MX_SPR_147456			0x50
#define MX_SPR_EXCLUDE_CLK_MASK		0x8f

#define MX_CONR_AUTO_HW			0x01
#define MX_CONR_CTS_SIGNAL_ENABLE	0x04
#define MX_CONR_DRV_DONE		0x08
#define	MX_CONR_NEGATE			0x10
#define MX_CONR_RX_DISABLE		0x20

#define MX_DCR0_NORMAL_TX		0x00
#define MX_DCR0_USB_SUSPEND_TX		0x01
#define MX_DCR0_RTS_ACTIVE_HIGH		0x10
#define MX_DCR0_RTS_EMPTY_LOW		0x20
#define MX_DCR0_RTS_EMPTY_HIGH		0x30
#define MX_DCR0_SET_GPIO_OUT		0x0C
#define MX_DCR0_SET_GPIO_IN		0x00

#define MX_DCR1_GPIO_CURRENT_8		0x01
#define MX_DCR1_TX_CURRENT_8		0x04

#define MX_DCR2_EWU_RI			0x04
#define MX_DCR2_RWM_RESUME		0x20

#define MX_ZLP_REG_EN_ALL_SP		0x0f

#define DRIVER_VERSION "1.0.5"
#define DRIVER_DESC_2210 "MOXA 2210 USB Serial Driver"
#define DRIVER_DESC_2250 "MOXA 2250 USB Serial Driver"
#define DRIVER_DESC_2410 "MOXA 2410 USB Serial Driver"
#define DRIVER_DESC_2450 "MOXA 2450 USB Serial Driver"
#define DRIVER_DESC "MOXA USB Serial Driver"

#define MX_WDR_TIMEOUT 			(5*HZ)
#define MX_WRITE_BUF_SIZE		1024
#define MX_CTRL_BUF_SIZE		16
#define MX_INT_READ_URB_INTERVAL	16
#define MX_WRITE_URB_FREE		0
#define MX_WRITE_URB_BUSY		1
#define MX_SW_TX_RUNNING		0
#define MX_SW_TX_STOPPED		1
#define MX_HANDLE_LSR			1
#define MX_HANDLE_MSR			0

#define MX_RD_RTYPE 			0xC0
#define MX_WR_RTYPE 			0x40
#define MX_RDREQ    			0x0D
#define MX_WRREQ    			0x0E
#define VENDOR_READ_LENGTH      	0x01

#define MX_2_PORTS			2
#define MX_4_PORTS			4
#define MX_PORT1			1
#define MX_PORT2			2
#define MX_PORT3			3
#define MX_PORT4			4
#define MX_READ_URB_RUNNING		0
#define MX_READ_URB_STOPPING		1
#define MX_READ_URB_STOPPED		2

#define MX_IN_SW_ENABLE			0x01
#define MX_OUT SW_ENABLE		0x02
#define MX_IN_XANY			0x04

#define MX_SPD_MASK			(ASYNC_SPD_HI | ASYNC_SPD_VHI | \
					 ASYNC_SPD_SHI | ASYNC_SPD_WARP)
#define MX_DEFAULT_LOW_LATENCY		1
#define MX_DEFAULT_CLOSING_WAIT		5	
#define MX_DEFAULT_CLOSE_DELAY		0
#define MX_1_OP_MODE			1
#define MX_3_OP_MODES			2
#define MX_RS485_DISABLE		0x00
#define MX_RS485_ENABLE			0xC0
#define MX_RS232			1
#define MX_RS422			2
#define MX_RS485_2W			4
#define MX_RS485_4W			8
#define MX_2PORT_DEFAULT_INTF_RS232	0x0F
#define MX_4PORT_DEFAULT_INTF_RS232	0xFF
#define MX_PULL_LOW			0
#define MX_PULL_HIGH			1
#define MX_GPIO_0			0x01
#define MX_GPIO_1			0x02
#define MX_SDA				MX_GPIO_0
#define MX_SCL				MX_GPIO_1
#define MX_CMD_CFG_P0			0x06
#define MX_CMD_CFG_P1			0x07
#define MX_CMD_OUT_P0			0x02
#define MX_CMD_OUT_P1			0x03
#define MX_DEVICE_OFF			0x00
#define MX_DEVICE_ON			0x01
#define MX_2_PORT_OFF			0x00
#define MX_2_PORT_ON			0x10
#define MX_4_PORT_OFF			0x01
#define MX_4_PORT_ON			0x02
#define MX_THROTTLED_THRESHOLD		512

#define MX_IOCTL_CODE			404
#define MX_IOCTL_UPORT2000_SET_INTERFACE	(MX_IOCTL_CODE + 21)
#define MX_IOCTL_UPORT2000_GET_INTERFACE	(MX_IOCTL_CODE + 22)
#define MX_IOCTL_UPORT2000_LOCATE_DEVICE	(MX_IOCTL_CODE + 23)
#define MX_IOCTL_UPORT2000_GET_PRODUCT_ID	(MX_IOCTL_CODE + 24)

static struct usb_device_id mxu2_2210_id_table [] = {
	{ USB_DEVICE(USB_VENDOR_ID_MXUPORT,MOXA_DEVICE_ID_2210) },
	{ }
};

static struct usb_device_id mxu2_2250_id_table [] = {
	{ USB_DEVICE(USB_VENDOR_ID_MXUPORT,MOXA_DEVICE_ID_2250) },
	{ }
};

static struct usb_device_id mxu2_2410_id_table [] = {
	{ USB_DEVICE(USB_VENDOR_ID_MXUPORT,MOXA_DEVICE_ID_2410) },
	{ }
};

static struct usb_device_id mxu2_2450_id_table [] = {
	{ USB_DEVICE(USB_VENDOR_ID_MXUPORT,MOXA_DEVICE_ID_2450) },
	{ }
};

static __devinitdata struct usb_device_id mxu2_id_table_combined [] = {
	{ USB_DEVICE(USB_VENDOR_ID_MXUPORT,MOXA_DEVICE_ID_2210) },
	{ USB_DEVICE(USB_VENDOR_ID_MXUPORT,MOXA_DEVICE_ID_2250) },
	{ USB_DEVICE(USB_VENDOR_ID_MXUPORT,MOXA_DEVICE_ID_2410) },
	{ USB_DEVICE(USB_VENDOR_ID_MXUPORT,MOXA_DEVICE_ID_2450) },
	{ }
};

MODULE_DEVICE_TABLE (usb, mxu2_id_table_combined);
MODULE_DEVICE_TABLE (usb, mxu2_2210_id_table);
MODULE_DEVICE_TABLE (usb, mxu2_2250_id_table);
MODULE_DEVICE_TABLE (usb, mxu2_2410_id_table);
MODULE_DEVICE_TABLE (usb, mxu2_2450_id_table);

MODULE_DESCRIPTION( DRIVER_DESC_2210 );
MODULE_DESCRIPTION( DRIVER_DESC_2250 );
MODULE_DESCRIPTION( DRIVER_DESC_2410 );
MODULE_DESCRIPTION( DRIVER_DESC_2450 );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

MODULE_PARM_DESC(debug, "Debug enabled or not");

struct mxu2_port 
{
	int			port_num;
	struct urb   		*read_urb;
	__u8			shadowLCR;
	__u8			shadowMCR;
	char			open;
	char			openPending;
	char			closePending;
	wait_queue_head_t	wait_chase;
	wait_queue_head_t	delta_msr_wait;
	struct async_icount	icount;
	struct usb_serial_port	*port;
	struct mxu2_serial	*mxp_serial;
	__u8 			SpRegOffset;
	__u8 			ControlRegOffset;
	__u8 			DcrRegOffset;
	__u8 			ClkSelectRegOffset;
	struct urb 		*control_urb;
        unsigned char		*ctrl_buf;
	int  			MsrLsr;
	unsigned char		read_urb_state;
	unsigned char		write_urb_in_use;
	struct circ_buf		*write_buf;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
        struct termios 		tmp_termios;
#else
        struct ktermios 	tmp_termios;
#endif
	spinlock_t 		mxp_slock;
	unsigned char		sw_tx_stopped;
	unsigned char		sw_function;
	int			flags;
	int			set_B0;
	int			close_delay;
	int			closing_wait;
	unsigned char		interface;
};


struct mxu2_serial 
{
	__u8			interrupt_in_endpoint;
	unsigned char  		*interrupt_in_buffer;
	struct urb 		*interrupt_read_urb;
	struct usb_serial	*serial;
	int			model_type;
	unsigned int		NoOfOpenPorts;
	unsigned char		status_polling_started;
	__u8 			bulk_fifo_status;
	struct semaphore	open_close_sem;
	unsigned char		op_mode;
	unsigned char		port_intf_status;
	__u16			shadow_gpio;
	unsigned int		product_name;
	
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int  mxu2_open(struct usb_serial_port *port, struct file *filp);

#elif (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
static int  mxu2_open(struct tty_struct *tty,
			struct usb_serial_port *port);

#else
static int  mxu2_open(struct tty_struct *tty, struct usb_serial_port *port,
			struct file *filp);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static void mxu2_close(struct usb_serial_port *port, struct file *filp);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30)
static void mxu2_close(struct usb_serial_port *port);
#else
static void mxu2_close(struct tty_struct *tty, struct usb_serial_port *port, struct file *filp);
#endif

static int  mxu2_startup(struct usb_serial *serial);

#ifdef	ASYNCB_FIRST_KERNEL
static void mxu2_disconnect(struct usb_serial *serial);
static void mxu2_release(struct usb_serial *serial);
#else
static void mxu2_shutdown(struct usb_serial *serial);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
static int mxu2_write(struct usb_serial_port *port, int from_user, const unsigned char *data, int count);
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int mxu2_write(struct usb_serial_port *port, const unsigned char *data, int count);
#else
static int mxu2_write(struct tty_struct *tty1, struct usb_serial_port *port, const unsigned char *data, int count);
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int  mxu2_write_room(struct usb_serial_port *port);
static int  mxu2_chars_in_buffer(struct usb_serial_port *port);
static void mxu2_throttle(struct usb_serial_port *port);
static void mxu2_unthrottle(struct usb_serial_port *port);
static int  mxu2_ioctl(struct usb_serial_port *port, struct file *file, unsigned int cmd, unsigned long arg);
#else
static int  mxu2_write_room(struct tty_struct *tty);
static int  mxu2_chars_in_buffer(struct tty_struct *tty);
static void mxu2_throttle(struct tty_struct *tty);
static void mxu2_unthrottle(struct tty_struct *tty);
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int  mxu2_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg);
#else
static int  mxu2_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg);
#endif
#endif

static int mxu2_calc_num_ports(struct usb_serial *serial);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void mxu2_set_termios	(struct usb_serial_port *port, struct termios *old_termios);
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static void mxu2_set_termios	(struct usb_serial_port *port, struct ktermios *old_termios);
#else
static void mxu2_set_termios	(struct tty_struct *tty1, struct usb_serial_port *port, struct ktermios *old_termios);
#endif
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int mxu2_tiocmget(struct usb_serial_port *port, struct file *file);
static int mxu2_tiocmset(struct usb_serial_port *port, struct file *file, unsigned int set, unsigned int clear);
static void mxu2_break(struct usb_serial_port *port, int break_state);
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int mxu2_tiocmget(struct tty_struct *tty, struct file *file);
static int mxu2_tiocmset(struct tty_struct *tty, struct file *file, unsigned int set, unsigned int clear);
#else
static int mxu2_tiocmget(struct tty_struct *tty);
static int mxu2_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear);
#endif
static void mxu2_break(struct tty_struct *tty, int break_state);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static void mxu2_interrupt_callback(struct urb *urb, struct pt_regs *regs);
static void mxu2_bulk_in_callback(struct urb *urb, struct pt_regs *regs);
static void mxu2_bulk_out_callback(struct urb *urb, struct pt_regs *regs);
static void mxu2_control_callback(struct urb *urb, struct pt_regs *regs);
#else
static void mxu2_interrupt_callback(struct urb *urb);
static void mxu2_bulk_in_callback(struct urb *urb);
static void mxu2_bulk_out_callback(struct urb *urb);
static void mxu2_control_callback(struct urb *urb);
#endif

#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))
static int mxu2_get_icount(struct tty_struct *tty,
		struct serial_icounter_struct *icount);
#endif		

//-----------------------------------------------------------------------------

static void mx_calc_baud_rate_divisor(int baudRate, int *divisor);
static void mx_recv(struct mxu2_port *mx_port, unsigned char *data, int len);
static void mx_send(struct mxu2_port *mx_port);
static void mx_block_until_chase_response(struct mxu2_port *mx_port);
static void mx_block_until_tx_empty(struct mxu2_port *mx_port);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void mx_change_port_settings(struct mxu2_port *mx_port, struct termios *old_termios);
#else
static void mx_change_port_settings(struct mxu2_port *mx_port, struct ktermios *old_termios);
#endif
static void mx_get_op_mode(struct usb_serial *serial);
static int mx_send_cmd_write_baud_rate(struct mxu2_port *mx_port, int baudRate);
static int mx_handle_newMsr(struct mxu2_port *port,__u8 newMsr);
static int mx_handle_newLsr(struct mxu2_port *port,__u8 newLsr);
static int mx_set_modem_info(struct mxu2_port *mx_port, unsigned int cmd, unsigned int *value);
static int mx_get_modem_info(struct mxu2_port *mx_port, unsigned int *value);
static int mx_get_serial_info(struct mxu2_port *mx_port, struct serial_struct * ret_arg);
static int mx_set_serial_info(struct mxu2_port *mx_port, struct serial_struct * new_arg);
static int mx_get_lsr_info(struct mxu2_port *mx_port, unsigned int *value);
static int mx_set_reg_offset(struct mxu2_port *mx_port);
static int mx_get_reg(struct mxu2_port *mcs,__u16 Wval, __u16 reg, __u16 *val);
static int mx_set_reg_sync(struct usb_serial_port *port, __u16 reg, __u16 val);
static int mx_get_reg_sync(struct usb_serial_port *port, __u16 reg, __u16 *val);
static int mx_set_Uart_Reg(struct usb_serial_port *port, __u16 reg, __u16 val);
static int mx_get_Uart_Reg(struct usb_serial_port *port, __u16 reg, __u16 *val);
static int mx_init_startup_regs(struct mxu2_port *mx_port);
static int mx_init_open_regs(struct mxu2_port *mx_port);
static int mx_set_interface(struct mxu2_port *mx_port, unsigned char sval);
static int mx_set_beeper_led(struct mxu2_port *mx_port, unsigned char sval);
static int mx_gpio_out(struct mxu2_port *mx_port, unsigned char cmd, unsigned char val);

//-----------------------------------------------------------------------------

static struct circ_buf *mx_buf_alloc(void);
static void mx_buf_free(struct circ_buf *cb);
static void mx_buf_clear(struct circ_buf *cb);
static int mx_buf_data_avail(struct circ_buf *cb);
static int mx_buf_space_avail(struct circ_buf *cb);
static int mx_buf_get(struct circ_buf *cb, char *buf, int count);
static int mx_buf_put(struct mxu2_port *mx_port, const char *buf, int count);

//-----------------------------------------------------------------------------

static int gpio_write_pin(struct mxu2_port *mx_port, unsigned char gpio_port, unsigned char wval);
static int gpio_read_pin(struct mxu2_port *mx_port, unsigned char gpio_port);

//-----------------------------------------------------------------------------

static int i2c_clk_hi(struct mxu2_port *mx_port);
static int i2c_clk_low(struct mxu2_port *mx_port);
static int i2c_data_hi(struct mxu2_port *mx_port);
static int i2c_data_low(struct mxu2_port *mx_port);
static int i2c_start(struct mxu2_port *mx_port, unsigned char val);
static void i2c_stop(struct mxu2_port *mx_port);
static int i2c_putb(struct mxu2_port *mx_port, unsigned char val);

//-----------------------------------------------------------------------------

int __init mxu2_init(void);
void __exit mxu2_exit(void);
