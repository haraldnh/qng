/*
 * Driver for the ComScire Quantum Noise Generator.
 * Version 0.7 for the Linux 2.6 series kernel.
 *
 * Copyright (C) 1999,2007 by Harald Nordgård-Hansen (hhansen@pvv.org)
 * Copying and other stuff permitted in accordance with the GPL.
 * NO WARRANTY, if it breaks, you get to keep both pieces.
 *
 * This driver was written using lp.c as an example.  Thanks to the
 * authors of that and other easily-read code in the kernel.
 *
 * Also thanks to Scott Wilber of ComScire for excellent help in
 * understanding how the device works.
 */

/*
 * While the driver is somewhat specific with respect to which bits it
 * reads and writes on the port, it should in theory work with any
 * parallel port through the parport abstractions.
 *
 * When the module is loaded, it can be configured using the parameter parport:
 *
 *	# insmod qng.o parport=1
 *	# insmod qng.o parport=auto (default)
 *
 * Please note that if you autoprobe for qng devices before setting
 * up your printers, and this driver probes for qng on a port that
 * has a printer connected, some small amount of output might occur.
 * This is due to the need for writing a couple of bytes to the data-
 * port as part of the probing sequence.
 * If the port is already claimed (shareable) by the lp driver (as should
 * usually happen), this driver will be denied exclusive access to
 * the port, and will (in theory) just stay away.
 *
 * With respect to interrupts, since the driver keeps the port claimed
 * at all times when interrupts are enabled, it is impossible to turn
 * off interrupts on a port once they are enabled.  To disable
 * interrupts on the device, the driver must be unloaded, the interrupts
 * turned off and the driver reloaded again.  If the driver is running
 * in polled mode, it will check for enabled interrupts on each call to
 * open, and will turn on these if this happens.
 *
 * Please also note that reading large amounts of data with polling will
 * kill any resemblance of performance on your machine, as the driver
 * has to busywait on the device in kernel space.  That hurts.
 */

#include <linux/module.h>
#include <linux/init.h>

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/system.h>

#include <linux/parport.h>

#define QNG_NO 1 /* The number of qng devices.  Note that only one
		    is defined in Devices.txt.  ComScire also only
		    recommend using only a single device, and using a faster
		    device instead of multiple slow ones if needed. */

#define QNG_DEBUG 0

#define QNG_MAJOR 77 /* from devices.txt */

/* Theory of operations, from ComScire:
 *
 * 1) The device takes its power from the Data register of the
 * parallel port This register must first be set high (FFh) to supply
 * power.  Wait about .25 seconds for the hardware to settle.  This
 * needs to be done only once after turning on the computer.
 *
 * 2) The random bits and a status/interrupt bit are all bits in the
 * Status register of the parallel port.  The random bits are
 * connected to bits 3, 4, 5 and 7 of the Status register.  Bit 6 of
 * the Status register is the status/interrupt line.  When this line
 * goes high, it signals that a new 4-bit random nibble is valid.
 * With our drivers, this line is used as an interrupt to the computer
 * which responds by reading the Status register and seperating the
 * appropriate 4 bits to send on to be used as random bits.  The line
 * may also be used in a polling mode by watching for a low-to-high
 * transition if interrupts are a problem.  If polling mode is used,
 * make sure that the random bits are read only once for each
 * transition of the status bit.
 *
 * For autodetecting, the device is turned off (by writing 0x00 to the
 * data port.  The rate of interrupts/transitions on the interface is
 * then counted.  The same procedure is then repeated with the power on.
 * If the rate shows that with power on the port receives more than
 * 2000 interrupts/second, and with power off less than this, we can
 * be fairly sure that a QNG device is attached to the port.
 *
 * If the device runs with interrupts on, we use an internal ring buffer
 * in order to speed up read requests.  In the case of polling, this is
 * not really useful (we would have to create a kernel thread to handle
 * this, which is sort of an overkill), and then the read call will
 * wait while the device is polled until enough data have been read.
 * (If interrupt driven read needs more data than is available in the
 * ring buffer, read will wait until the interrupt routine have supplied
 * enough data.)
 */

/* defines for bit positions of flags */
#define QNG_EXIST 0
#define QNG_BUSY 1
#define QNG_HAS_INTS 2
#define QNG_COUNT_INTS 3
#define QNG_PORT_CLAIMED 4

/* defines for port-device mappings. */
#define QNG_PARPORT_UNSPEC -4
#define QNG_PARPORT_AUTO -3
#define QNG_PARPORT_OFF -2
#define QNG_PARPORT_NONE -1

/* size of buffer used by interrupt routine */
#define QNG_BUFFER_SIZE 4096

typedef struct _qng_struct {
	struct pardevice * dev;
	volatile unsigned long flags;
	volatile int halfstate;
	volatile int head;
	volatile int tail;
	volatile int ints;
	int bufsize;
	unsigned char * volatile buffer;
} qng_struct;

static qng_struct qng_data[QNG_NO] =
{
	[0 ... QNG_NO-1] = {NULL, 0, 0, 0, 0, 0, 0, NULL}
};

/* --- low-level port access ----------------------------------- */

#define r_str(x) (parport_read_status(qng_data[x].dev->port))
#define w_dtr(x,y) parport_write_data(qng_data[x].dev->port, (y))

/* If we're running with interrupts, then fetch each nibble here,
   and stuff it into a ring buffer.  This should increase the speed
   of small reads enormously. */
static void qng_interrupt(int irq, void *dev)
{
	unsigned char nibble;
	int n;
	qng_struct *data = (qng_struct *) dev;

	if(test_bit(QNG_COUNT_INTS, &(data->flags))) {
		(data->ints)++;
		return;
	}

	if(data->bufsize == 0) return;

#if 0 /* no need, this cannot happen it seems. */
	if(! test_and_set_bit(QNG_HAS_INTS,&(data->flags))) {
#if QNG_DEBUG >= 1
		printk(KERN_INFO "qng: detected irq %d on %s\n",
		       irq, data->dev->port->name)
#endif
			;
	}
#endif

	n = parport_read_status(data->dev->port);
	nibble = ((n & 0x80)>>4) + ((n & 0x38)>>3);
	if(data->halfstate == 0) {
		data->halfstate = 1;
		data->buffer[data->head] = nibble;
	} else {
		data->halfstate = 0;
		data->buffer[data->head] |= (nibble << 4);
		data->head++;
		if(data->head == data->tail) data->head--;
		else if(data->head >= data->bufsize) data->head = 0;
	}
}

/* The maximum amount of time to wait for a bit is 10ms */
#define NIBBLE_POLL_DELAY (HZ+99/100)
static int qng_read_nibble(int nr)
{
     int i,j,n=0;

     /* Wait for nibble start (to only read a nibble once). */
     for(i=j=jiffies; j-i < NIBBLE_POLL_DELAY && (r_str(nr) & 0x40);) {
	     j = jiffies;
     }
     if(j-i >= NIBBLE_POLL_DELAY) {
	     /* This shouldn't happen... */
	     printk(KERN_ERR "qng: timed out in readnibble.\n");
	     return(-1);
     }
     /* Wait for nibble ready. */
     for(i=j=jiffies; j-i < NIBBLE_POLL_DELAY && !((n=r_str(nr)) & 0x40);) {
	     j = jiffies;
     }
     if(j-i >= NIBBLE_POLL_DELAY) {
	     printk(KERN_ERR "qng: timed out in readnibble.\n");
	     return(-1);
     }
     return(((n & 0x80)>>4) + ((n & 0x38)>>3));
}

static ssize_t qng_read_polled(unsigned int minor, char *buf, size_t length)
{
	ssize_t count;
	int t;
	unsigned char c;
	char *tmp = buf;

	for(count = 0; count < length; count++) {
		t = (qng_read_nibble(minor)<<4) + qng_read_nibble(minor);
		if(t == -1) return -EIO;
		c = t & 0xff;
		if(__put_user(c, tmp)) return -EFAULT;
		tmp++;
		/* This kills some speed on the device, as we will
		   miss some nibbles, but the machine becomes _much_
		   more responsive for other use... */
		cond_resched();
	}
	return count;
}

/* The maximum amount of time to wait for the interrupt routine to
   generate more data. */
#define INTERRUPT_WAIT (10*HZ+99)/100
static ssize_t qng_read_intr(unsigned int minor, char *buf, size_t length)
{
	ssize_t count;
	int i, j;
	char *tmp = buf;

	count = 0;
	j = jiffies;
	while(count < length) {
		i = qng_data[minor].head - qng_data[minor].tail;
		if(i < 0) i += qng_data[minor].bufsize;
		if(i > 0) {
			if(__put_user(qng_data[minor].buffer[qng_data[minor].tail], tmp))
				return -EFAULT;
			tmp++;
			count++;
			qng_data[minor].tail++;
			if(qng_data[minor].tail >= qng_data[minor].bufsize)
				qng_data[minor].tail = 0;
			j = jiffies;
		} else {
			/* Let's wait for the interrupt routine to
			   receive more data... */
			if(jiffies - j >= INTERRUPT_WAIT) {
				printk(KERN_ERR
				       "qng: timed out waiting for ints.\n");
#if QNG_DEBUG >= 1
				printk(KERN_INFO
				       "qng: turning off interrupts on %s.\n",
				       qng_data[minor].dev->port->name);
#endif
				clear_bit(QNG_HAS_INTS,
					  &(qng_data[minor].flags));
				return -EIO;
			}
			cond_resched();
		}
	}
	return count;
}

static ssize_t qng_read(struct file * file, char * buf,
			size_t length, loff_t * ppos)
{
	unsigned int minor=MINOR(file->f_dentry->d_inode->i_rdev);

	if(test_bit(QNG_HAS_INTS,&(qng_data[minor].flags)) &&
	   qng_data[minor].dev->port->irq == PARPORT_IRQ_NONE)
		clear_bit(QNG_HAS_INTS, &(qng_data[minor].flags));

	if(test_bit(QNG_HAS_INTS,&(qng_data[minor].flags)) &&
		qng_data[minor].bufsize > 0)
		return(qng_read_intr(minor,buf,length));
	else
		return(qng_read_polled(minor,buf,length));
}

static int qng_open(struct inode * inode, struct file * file)
{
	unsigned int minor = MINOR(inode->i_rdev);

	if (minor >= QNG_NO)
		return -ENXIO;
	if (!test_bit(QNG_EXIST,&(qng_data[minor].flags)))
		return -ENXIO;
	if (test_and_set_bit(QNG_BUSY, &(qng_data[minor].flags)))
		/* only one accessor at any moment */
		return -EBUSY;

	if(!test_and_set_bit(QNG_PORT_CLAIMED, &(qng_data[minor].flags)))
		parport_claim(qng_data[minor].dev);
	if(!test_bit(QNG_HAS_INTS, &(qng_data[minor].flags)) &&
	   qng_data[minor].dev->port->irq != PARPORT_IRQ_NONE) {
		set_bit(QNG_HAS_INTS, &(qng_data[minor].flags));
		printk(KERN_INFO "qng: detected interrupts on %s\n",
		       qng_data[minor].dev->port->name);
		qng_data[minor].dev->port->ops->enable_irq
			(qng_data[minor].dev->port);
	}
#if QNG_DEBUG >= 3
	printk(KERN_INFO "qng: Open with minor at %d, using %s\n", minor,
	       (test_bit(QNG_HAS_INTS,&(qng_data[minor].flags))?
		       "interrupts":"polling"));
#endif
	return 0;
}

static int qng_release(struct inode * inode, struct file * file)
{
	unsigned int minor = MINOR(inode->i_rdev);

	if(!test_bit(QNG_HAS_INTS, &(qng_data[minor].flags)) &&
	   test_and_clear_bit(QNG_PORT_CLAIMED, &(qng_data[minor].flags)))
		parport_release(qng_data[minor].dev);

	clear_bit(QNG_BUSY, &(qng_data[minor].flags));
	return 0;
}


static struct file_operations qng_fops = {
	.read    = qng_read,
	.open    = qng_open,
	.release = qng_release
};

/* --- initialisation code ------------------------------------- */

static int parport_nr[QNG_NO] = { [0 ... QNG_NO-1] = QNG_PARPORT_UNSPEC };
static char *parport[QNG_NO] = { NULL,  };

MODULE_AUTHOR("Harald Nordgård-Hansen <hhansen@pvv.org>");
MODULE_DESCRIPTION("ComScire Quantum Noise Generator (/dev/qng) driver");
MODULE_LICENSE("GPLv2");

int qng_count_transitions(int nr, int time) { /* time in msec */
	if(test_bit(QNG_HAS_INTS, &(qng_data[nr].flags))) {
		/* When ints are available, this is mostly done in
		   the interrupt servicing routine... */
		qng_data[nr].ints = 0;
#if QNG_DEBUG >= 3
		printk(KERN_INFO "qng: starting count %d, %ld.\n",
		       qng_data[nr].ints, jiffies);
#endif
		set_bit(QNG_COUNT_INTS, &(qng_data[nr].flags));
		current->state = TASK_INTERRUPTIBLE;
		schedule_timeout((time*HZ+999)/1000);
		clear_bit(QNG_COUNT_INTS, &(qng_data[nr].flags));
#if QNG_DEBUG >= 3
		printk(KERN_INFO "qng: stopping count %d, %ld.\n",
		       qng_data[nr].ints, jiffies);
#endif
		return qng_data[nr].ints;
	} else {
		/* ... doing this by polling is manual, though. */
		int i=0;
		int count = 0;
		int timeout;

		timeout = jiffies + ((time*HZ+999)/1000);
#if QNG_DEBUG >= 3
		printk(KERN_INFO "qng: starting polled %d, %ld.\n",
		       count, jiffies);
#endif
		while(jiffies < timeout) {
			if(i == 0 && (r_str(nr) & 0x40)) {
				i = 1;
				count++;
			} else if(i == 1 && !(r_str(nr) & 0x40))
				i = 0;
		}
#if QNG_DEBUG >= 3
		printk(KERN_INFO "qng: stopping polled %d, %ld.\n",
		       count, jiffies);
#endif
		return count;
	}
}

/* Try to check that the port actually has a qng device attached. */
/* Please note that testing this with polling can cause long delays
   in the kernel.  (At the moment ~50msec, could possibly be lower?).
   This is a _bad_ thing, but I don't know how to do it better... */
int qng_validate(int nr) {
	int i,j;

	w_dtr(nr,0);
	current->state = TASK_INTERRUPTIBLE;
	schedule_timeout(HZ/4);
	i = qng_count_transitions(nr,50);
	w_dtr(nr,0xff);
	current->state = TASK_INTERRUPTIBLE;
	schedule_timeout(HZ/4);
	j = qng_count_transitions(nr,50);
	if(i > 100 || j < 100) {
#if QNG_DEBUG >= 2
		printk(KERN_INFO "qng: no qng device on %s.\n",
		       qng_data[nr].dev->port->name);
#endif
		return 1;
	}
	/* At this point the device is detected, and up and running. */
	return 0;
}


int qng_register(int nr, struct parport *port)
{
	static int did_version = 0;

	qng_data[nr].dev = parport_register_device(port, "qng", 
						   NULL, NULL,
						   qng_interrupt,
						   PARPORT_FLAG_EXCL,
						   (void *)&qng_data[nr]);
	if (qng_data[nr].dev == NULL)
		return 1;
	if(did_version++ == 0)
		printk(KERN_INFO "qng.c v0.7");
	set_bit(QNG_PORT_CLAIMED, &(qng_data[nr].flags));
	parport_claim(qng_data[nr].dev);
	if (port->irq != PARPORT_IRQ_NONE) {
		set_bit(QNG_HAS_INTS, &(qng_data[nr].flags));
		port->ops->enable_irq(port);
	}
	if (qng_validate(nr)) {
		clear_bit(QNG_HAS_INTS, &(qng_data[nr].flags));
		if(test_and_clear_bit(QNG_PORT_CLAIMED, &(qng_data[nr].flags)))
			parport_release(qng_data[nr].dev);
		parport_unregister_device(qng_data[nr].dev);
		qng_data[nr].dev = NULL;
		return 1;
	}
	set_bit(QNG_EXIST, &(qng_data[nr].flags));
	if(!test_bit(QNG_HAS_INTS, &(qng_data[nr].flags)) &&
	   test_and_clear_bit(QNG_PORT_CLAIMED, &(qng_data[nr].flags)))
		parport_release(qng_data[nr].dev);
	printk(KERN_INFO "qng: using %s (%s).\n", port->name,
	       (port->irq == PARPORT_IRQ_NONE) ?
	       "with polling" : "interrupt-driven");

	return 0;
}

int qng_init(void)
{
	unsigned int count = 0;
	unsigned int i;
	struct parport *port;

	switch (parport_nr[0])
	{
	case QNG_PARPORT_OFF:
		return 0;

	case QNG_PARPORT_UNSPEC:
	case QNG_PARPORT_AUTO:
		for (port = parport_enumerate(); port; port = port->next) {
			if (!qng_register(count, port))
				if(++count == QNG_NO)
					break;
		}
		break;

	default:
		for (i = 0; i < QNG_NO; i++) {
			for (port = parport_enumerate(); port;
			     port = port->next) {
				if (port->number == parport_nr[i]) {
					if (!qng_register(i, port))
						count++;
					break;
				}
			}
		}
		break;
	}

	if (count) {
		if (register_chrdev(QNG_MAJOR, "qng", &qng_fops)) {
			printk(KERN_ERR "qng: unable to get major %d\n",
			       QNG_MAJOR);
			for(i = 0; i < QNG_NO; i++)
				if(qng_data[i].dev) {
					if(test_and_clear_bit(QNG_PORT_CLAIMED, &(qng_data[i].flags)))
						parport_release(qng_data[i].dev);
					parport_unregister_device(
						qng_data[i].dev);
				}
			return -EIO;
		}
	} else {
#if QNG_DEBUG >= 1
		printk(KERN_INFO "qng: driver loaded but no devices found\n");
#endif
		return 0;
	}
	/* Do final setup for each device... */
	for (i = 0; i < QNG_NO; i++) {
		if(test_bit(QNG_EXIST,&(qng_data[i].flags))) {
			qng_data[i].buffer =
				(char *) kmalloc(QNG_BUFFER_SIZE, GFP_KERNEL);
			if (!qng_data[i].buffer) {
				printk(KERN_ERR "qng: unable to allocate "
				       "internal buffer.  Will not make "
				       "use of interrupts.\n");
				qng_data[i].bufsize = 0;
			} else {
				qng_data[i].bufsize = QNG_BUFFER_SIZE;
			}
		}
	}
#if QNG_DEBUG >= 1
	printk(KERN_INFO "qng: driver ready.\n");
#endif
	return 0;
}

int init_module(void)
{
	if (parport[0]) {
		/* The user gave some parameters.  Let's take a look. */
		if (!strcmp(parport[0], "auto"))
			parport_nr[0] = QNG_PARPORT_AUTO;
		else {
			int n;
			for (n = 0; n < QNG_NO && parport[n]; n++) {
				if (!strcmp(parport[n], "none"))
					parport_nr[n] = QNG_PARPORT_NONE;
				else {
					char *ep;
					unsigned long r = simple_strtoul(parport[n], &ep, 0);
					if (ep != parport[n])
						parport_nr[n] = r;
					else {
						printk(KERN_ERR "qng: bad port specifier '%s'\n", parport[n]);
						return -ENODEV;
					}
				}
			}
		}
	}
	return qng_init();
}

void cleanup_module(void)
{
	int i;

	unregister_chrdev(QNG_MAJOR, "qng");
	for(i = 0; i < QNG_NO; i++)
		if(qng_data[i].dev) {
			if(test_and_clear_bit(QNG_PORT_CLAIMED,
					      &(qng_data[i].flags)))
				parport_release(qng_data[i].dev);
			parport_unregister_device(qng_data[i].dev);
		}
}

/*
 * Local variables:
 *  c-indent-level: 8
 *  c-basic-offset: 8
 *  tab-width: 8
 *  indent-tabs-mode: t
 * End:
 */
