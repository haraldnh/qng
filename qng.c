/*
 * Driver for the ComScire Quantum Noise Generator.
 * Version 0.8 for modern kernels.
 *
 * Copyright (C) 1999,2013 by Harald Nordgård-Hansen (hhansen@pvv.org)
 * Copying and other stuff permitted in accordance with the GPL.
 * NO WARRANTY, if it breaks, you get to keep both pieces.
 *
 * Thanks to Scott Wilber of ComScire for excellent help in
 * understanding how the device works.
 */

/*
 * While the driver is somewhat specific with respect to which bits it
 * reads and writes on the port, it should in theory work with any
 * parallel port through the parport abstractions.
 *
 * When the module is loaded, it will probe available parallell ports
 * for qng devices.  Please note that if this probe for qng devices
 * happens before setting up any printers, and this driver probes for
 * qng on a port that has a printer connected, some small amount of
 * output might occur.  This is due to the need for writing a couple
 * of bytes to the dataport as part of the probing sequence.  If the
 * port is already claimed (shareable) by the lp driver (as should
 * usually happen), this driver will be denied exclusive access to the
 * port, and will (in theory) just stay away.
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
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/slab.h>

#include <asm/irq.h>
#include <asm/uaccess.h>

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
#define QNG_HAS_INTS 1
#define QNG_COUNT_INTS 2
#define QNG_PORT_CLAIMED 3

/* size of buffer used by interrupt routine */
#define QNG_BUFFER_SIZE 262144

typedef struct _qng_struct {
	struct parport *port;
	struct pardevice *pdev;
	wait_queue_head_t wq;
	int use_count;
	struct semaphore lock;
	volatile unsigned long flags;
	volatile int halfstate;
	volatile int head;
	volatile int tail;
	volatile int ints;
	int bufsize;
	unsigned char * volatile buffer;
} qng_struct;

static qng_struct qng_data[QNG_NO];
static struct class *devclass = NULL;

/* --- low-level port access ----------------------------------- */

#define r_str(x) (parport_read_status(qng_data[x].port))
#define w_dtr(x,y) parport_write_data(qng_data[x].port, (y))

/* If we're running with interrupts, then fetch each nibble here,
   and stuff it into a ring buffer.  This should increase the speed
   of small reads enormously. */
static void qng_interrupt(void *dev)
{
	unsigned char nibble;
	int n;
	qng_struct *data = (qng_struct *) dev;

	if(test_bit(QNG_COUNT_INTS, &(data->flags))) {
		(data->ints)++;
		return;
	}

	if(data->bufsize == 0) return;

	/* Bail out if queueu is already full */
	if( ((data->head + 1) == data->tail) ||
	    (((data->head + 1) >= data->bufsize) && (data->tail == 0)) )
		return;

	n = parport_read_status(data->port);
	nibble = ((n & 0x80)>>4) + ((n & 0x38)>>3);
	if(data->halfstate == 0) {
		data->halfstate = 1;
		data->buffer[data->head] = nibble;
	} else {
		data->halfstate = 0;
		data->buffer[data->head] |= (nibble << 4);
		data->head++;
		if(data->head >= data->bufsize)
			data->head = 0;
		wake_up_interruptible(&data->wq);
	}
}

/* The maximum amount of time to wait for a bit is 10ms */
#define NIBBLE_POLL_DELAY (HZ+99/100)
static int qng_read_nibble(int nr)
{
	long unsigned int i,j;
	int n=0;

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
		if (down_interruptible(&qng_data[minor].lock))
			return -ERESTARTSYS;
		t = (qng_read_nibble(minor)<<4) + qng_read_nibble(minor);
		up(&qng_data[minor].lock);
		if(t == -1) return -EIO;
		c = t & 0xff;
		if(__put_user(c, tmp)) return -EFAULT;
		tmp++;
		/* This kills some speed on the device, as we will
		   miss some nibbles, but the machine becomes _much_
		   more responsive for other use... */
		schedule();
	}
	return count;
}

/* The maximum amount of time to wait for the interrupt routine to
   generate more data.  With multiple readers, this can be quite large
   witout being broken, so set it quite high. */
#define INTERRUPT_WAIT 4*HZ
static ssize_t qng_read_intr(struct file *file, char *buf, size_t length)
{
	ssize_t count = 0;
	int i;
	char *tmp = buf;
	unsigned int minor = MINOR(file->f_dentry->d_inode->i_rdev);
	qng_struct *data = &qng_data[minor];

	/* First satisfy as much as possible from the buffer */
	if (down_interruptible(&data->lock))
		return -ERESTARTSYS;
	if (data->head != data->tail) {
		if (data->head > data->tail) {
			count = min(length, (size_t)(data->head - data->tail));
			if (copy_to_user(tmp, &data->buffer[data->tail],
					 count)) {
				up(&data->lock);
				return -EFAULT;
			}
			data->tail += count;
			tmp += count;
		} else {
			count = min(length,
				    (size_t)(data->bufsize - data->tail));
			if (copy_to_user(tmp, &data->buffer[data->tail],
					 count)) {
				up(&data->lock);
				return -EFAULT;
			}
			data->tail += count;
			tmp += count;
			if (data->tail >= data->bufsize)
				data->tail = 0;
			if (count < length) {
				i = min((int)(length - count), data->head);
				if (copy_to_user(tmp, data->buffer, i)) {
					up(&data->lock);
					return -EFAULT;
				}
				data->tail = i;
				count += i;
				tmp += i;
			}
		}
	}
	if (count == length) {
		up(&data->lock);
		return count;
	}
	if (file->f_flags & O_NONBLOCK) {
		up(&data->lock);
		if (count) return count;
		else return -EAGAIN;
	}

	/* We need to wait for data to become available */
	while (count < length) {
		while ((count < length) && (data->tail != data->head)) {
			if (__put_user(data->buffer[data->tail], tmp)) {
				up(&data->lock);
				return -EFAULT;
			}
			tmp++;
			count++;
			data->tail++;
			if (data->tail >= data->bufsize)
				data->tail = 0;
		}
		if (count == length) break;

		up(&data->lock);
		i = wait_event_interruptible_timeout(data->wq,
						     data->tail != data->head,
						     INTERRUPT_WAIT);
		if (i < 0) {
			return -ERESTARTSYS;
		}
		if (i == 0) {
			printk(KERN_ERR "qng: timed out waiting for ints.\n");
#if QNG_DEBUG >= 1
			printk(KERN_INFO "qng: turning off interrupts on %s.\n",
			       data->port->name);
#endif
			clear_bit(QNG_HAS_INTS, &data->flags);
			return -EIO;
		}
		if (down_interruptible(&data->lock))
			return -ERESTARTSYS;
	}

	up(&data->lock);
	return count;
}

static ssize_t qng_read(struct file * file, char * buf,
			size_t length, loff_t * ppos)
{
	unsigned int minor=MINOR(file->f_dentry->d_inode->i_rdev);

	if(test_bit(QNG_HAS_INTS,&(qng_data[minor].flags)) &&
	   qng_data[minor].port->irq == PARPORT_IRQ_NONE)
		clear_bit(QNG_HAS_INTS, &(qng_data[minor].flags));

	if(test_bit(QNG_HAS_INTS,&(qng_data[minor].flags)) &&
		qng_data[minor].bufsize > 0)
		return(qng_read_intr(file,buf,length));
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

	/* Put a (high) limit on number of concurrent users */
	if (qng_data[minor].use_count >= 50)
		return -EBUSY;

	if ((qng_data[minor].use_count == 0) &&
	    (!test_and_set_bit(QNG_PORT_CLAIMED, &(qng_data[minor].flags))))
		parport_claim(qng_data[minor].pdev);
	qng_data[minor].use_count++;
	if(!test_bit(QNG_HAS_INTS, &(qng_data[minor].flags)) &&
	   qng_data[minor].port->irq != PARPORT_IRQ_NONE) {
		set_bit(QNG_HAS_INTS, &(qng_data[minor].flags));
		printk(KERN_INFO "qng: detected interrupts on %s\n",
		       qng_data[minor].port->name);
		qng_data[minor].port->ops->enable_irq(qng_data[minor].port);
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

	qng_data[minor].use_count--;
	if ((qng_data[minor].use_count == 0) &&
	    !test_bit(QNG_HAS_INTS, &(qng_data[minor].flags)) &&
	    test_and_clear_bit(QNG_PORT_CLAIMED, &(qng_data[minor].flags)))
		parport_release(qng_data[minor].pdev);

	return 0;
}


static struct file_operations qng_fops = {
	.read    = qng_read,
	.open    = qng_open,
	.release = qng_release
};

/* --- initialisation code ------------------------------------- */

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


static void qng_attach(struct parport *port)
{
	int i;
	dev_t devt;

	for (i = 0; i < QNG_NO; i++ )
		if(qng_data[i].port == NULL)
			break;
	if (i == QNG_NO) {
		printk(KERN_WARNING "qng: No space for more devices.");
		return;
	}

	qng_data[i].pdev = parport_register_device(port, KBUILD_MODNAME, 
						   NULL, NULL, qng_interrupt,
						   PARPORT_FLAG_EXCL,
						   &qng_data[i]);
	if (qng_data[i].pdev == NULL)
		return;
	qng_data[i].port = port;
	set_bit(QNG_PORT_CLAIMED, &(qng_data[i].flags));
	parport_claim(qng_data[i].pdev);
	if (port->irq != PARPORT_IRQ_NONE) {
		set_bit(QNG_HAS_INTS, &(qng_data[i].flags));
		port->ops->enable_irq(port);
	}
	if (qng_validate(i)) {
		clear_bit(QNG_HAS_INTS, &(qng_data[i].flags));
		if(test_and_clear_bit(QNG_PORT_CLAIMED, &(qng_data[i].flags)))
			parport_release(qng_data[i].pdev);
		parport_unregister_device(qng_data[i].pdev);
		qng_data[i].pdev = NULL;
		qng_data[i].port = NULL;
		return;
	}
	set_bit(QNG_EXIST, &(qng_data[i].flags));
	if(!test_bit(QNG_HAS_INTS, &(qng_data[i].flags)) &&
	   test_and_clear_bit(QNG_PORT_CLAIMED, &(qng_data[i].flags)))
		parport_release(qng_data[i].pdev);
	printk(KERN_INFO "qng: using %s (%s).\n", port->name,
	       (port->irq == PARPORT_IRQ_NONE) ?
	       "with polling" : "interrupt-driven");

	/* Do final setup for device... */
	init_waitqueue_head(&qng_data[i].wq);
	sema_init(&qng_data[i].lock, 1);
	qng_data[i].buffer = (char *) kmalloc(QNG_BUFFER_SIZE, GFP_KERNEL);
	if (!qng_data[i].buffer) {
		printk(KERN_ERR "qng: unable to allocate "
		       "internal buffer.  Will not make "
		       "use of interrupts.\n");
		qng_data[i].bufsize = 0;
	} else {
		qng_data[i].bufsize = QNG_BUFFER_SIZE;
	}
	qng_data[i].head = qng_data[i].tail = qng_data[i].halfstate = 0;
	devt = MKDEV(QNG_MAJOR, i);
	device_create(devclass, NULL, devt, NULL, "qng%d", i);
}

static void qng_detach(struct parport *port)
{
	int i;
	dev_t devt;

	for (i = 0; i < QNG_NO; i++) {
		if(qng_data[i].port == port) {
			devt = MKDEV(QNG_MAJOR, i);
			device_destroy(devclass, devt);
			/* Power down */
			parport_write_data(port, 0);
			parport_release(qng_data[i].pdev);
			parport_unregister_device(qng_data[i].pdev);
			if(qng_data[i].buffer != NULL)
				kfree(qng_data[i].buffer);
			qng_data[i].port = NULL;
			qng_data[i].pdev = NULL;
			qng_data[i].flags = 0;
			qng_data[i].bufsize = 0;
			qng_data[i].buffer = NULL;
			break;
		}
	}
}


static struct parport_driver qng_driver = {
	.name = KBUILD_MODNAME,
	.attach = qng_attach,
	.detach = qng_detach,
};

static int __init qng_init_module(void)
{
	int ret;

	printk(KERN_INFO "qng.c v0.8");

	ret = register_chrdev(QNG_MAJOR, "qng", &qng_fops);
	
	if (ret)
		printk(KERN_ERR "qng: unable to get major %d\n", QNG_MAJOR);

	if (ret == 0) {
		devclass = class_create(THIS_MODULE, "qng");
		ret = parport_register_driver(&qng_driver);
		if (ret) {
			printk(KERN_ERR "qng: Unable to register for parport");
			unregister_chrdev(QNG_MAJOR, "qng");
		}
	}
	return ret;
}

static void __exit qng_cleanup_module(void)
{
	unregister_chrdev(QNG_MAJOR, "qng");
	parport_unregister_driver(&qng_driver);
	class_destroy(devclass);
}

module_init(qng_init_module);
module_exit(qng_cleanup_module);

MODULE_AUTHOR("Harald Nordgård-Hansen <hhansen@pvv.org>");
MODULE_DESCRIPTION("ComScire Quantum Noise Generator (/dev/qng) driver");
MODULE_LICENSE("GPL v2");

/*
 * Local variables:
 *  c-indent-level: 8
 *  c-basic-offset: 8
 *  tab-width: 8
 *  indent-tabs-mode: t
 * End:
 */
