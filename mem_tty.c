/*
 * Mem TTY driver
 *
 * Copyright (C) 2003 marco corvi <marco_corvi@geocities.com>
 * 
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 * This driver has been adapted from the tiny_tty driver by
 * Greg Kroah-Hartman (greg@kroah.com)
 *
 * This driver shows how to create a minimal tty driver.  i
 * It relies on a memory buffer acting as backing hardware, 
 * it uses a timer to emulate delay in receiving the data.
 */

#include <asm/uaccess.h>
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>

#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "marco corvi <marco_corvi@geocities.com>"
#define DRIVER_DESC "Memory TTY driver"

/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");


#define MEM_TTY_MAJOR	240	/* experimental range */
#define MEM_TTY_MINORS	255	/* use the whole major up */

#define MEM_HW_SIZE    128


struct mem_tty_serial {
  struct tty_struct *tty;  /* pointer to the tty for this device */
  struct file * filp;
  int    open_count;       /* number of times this port has been opened */
  struct semaphore sem;    /* locks this structure */

  struct timer_list	*timer;
  char * hw_buffer;
  int    head;
  int    tail;
  int    throttling;

  wait_queue_head_t   wait;    /* wait_queue */
  int mcr;                     /* modem control register */
  int msr;                     /* modem status register */
  struct serial_struct serial; /* serial_struct data */
  struct async_icount  icount; /* intr. count */
};

static int			mem_tty_refcount;
static struct tty_driver	mem_tty_driver;
static struct tty_struct	*mem_tty[MEM_TTY_MINORS];
static struct termios		*mem_tty_termios[MEM_TTY_MINORS];
static struct termios		*mem_tty_termios_locked[MEM_TTY_MINORS];
static struct mem_tty_serial	*mem_tty_table[MEM_TTY_MINORS];	/* initially all NULL */


// #define DELAY_TIME		HZ /* 1 second per character */
#define DELAY_TIME 2 /* 2 jiffies */

static void mem_tty_timer (unsigned long data)
{
  struct mem_tty_serial *mem = (struct mem_tty_serial *)data;
  struct tty_struct *tty;

  if (!mem)
    return;

  printk(KERN_ALERT "mem_tty_timer head %d tail %d\n", mem->head, mem->tail );
  if ( mem->tail == mem->head ) 
    return;

  tty = mem->tty;

  if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
    tty_flip_buffer_push(tty);
  }

  tty_insert_flip_char(tty, mem->hw_buffer[mem->tail], 0);
  tty_flip_buffer_push(tty);
  mem->tail = ( mem->tail + 1) % MEM_HW_SIZE;
  // tty_schedule_flip (tty); // This is not necessary

  // printk(KERN_ALERT "mem_tty_timer ok %c\n", mem_tty_data_char);
  
  if ( mem->tail != mem->head ) {  
    /* resubmit the timer again */
    mem->timer->expires = jiffies + DELAY_TIME;
    add_timer (mem->timer);
  }
}

static int mem_tty_ioctl (struct tty_struct *tty, struct file * file,
                          unsigned int cmd, unsigned long arg )
{
  struct mem_tty_serial *mem = tty->driver_data;
  struct serial_struct          tmp_serial;
  struct serial_icounter_struct tmp_icount;
  unsigned int value;

  if ( ! mem ) 
    return -ENODEV;
  down (&mem->sem);
  if (!mem->open_count) { /* port was not opened */
    up (&mem->sem);
    return -EINVAL;
  }

  switch ( cmd ) {
    case TIOCMGET: 
      value = ( (mem->mcr & UART_MCR_DTR) ? TIOCM_DTR : 0 )
            | ( (mem->mcr & UART_MCR_RTS) ? TIOCM_RTS : 0 )
            | ( (mem->msr & UART_MSR_CTS) ? TIOCM_CTS : 0 )
            | ( (mem->msr & UART_MSR_DCD) ? TIOCM_CAR : 0 )
            | ( (mem->msr & UART_MSR_RI)  ? TIOCM_RI  : 0 )
            | ( (mem->msr & UART_MSR_DSR) ? TIOCM_DSR : 0 );
      if ( copy_to_user( (unsigned int *)arg, &value, sizeof(unsigned int)))
        return -EFAULT;
      return 0;
    case TIOCMBIS:
      if ( copy_from_user( &value, (unsigned int *)arg, sizeof(unsigned int)))
        return -EFAULT;
      if ( value & TIOCM_RTS ) mem->mcr |= UART_MCR_RTS;
      if ( value & TIOCM_DTR ) mem->mcr |= UART_MCR_DTR;
      if ( value & TIOCM_LOOP ) mem->mcr |= UART_MCR_LOOP;
      return 0;
    case TIOCMBIC:
      if ( copy_from_user( &value, (unsigned int *)arg, sizeof(unsigned int)))
        return -EFAULT;
      if ( value & TIOCM_RTS ) mem->mcr &= ~UART_MCR_RTS;
      if ( value & TIOCM_DTR ) mem->mcr &= ~UART_MCR_DTR;
      if ( value & TIOCM_LOOP ) mem->mcr &= ~UART_MCR_LOOP;
      return 0;
    case TIOCMSET:
      if ( copy_from_user( &value, (unsigned int *)arg, sizeof(unsigned int)))
        return -EFAULT;
      mem->mcr &= ~( UART_MCR_RTS | UART_MCR_DTR | UART_MCR_LOOP);
      if ( value & TIOCM_RTS ) mem->mcr |= UART_MCR_RTS;
      if ( value & TIOCM_DTR ) mem->mcr |= UART_MCR_DTR;
      if ( value & TIOCM_LOOP ) mem->mcr |= UART_MCR_LOOP;
      return 0;
    case TIOCGSERIAL:
      memset( &tmp_serial, 0, sizeof(struct serial_struct) );
      // copy all the fields except for reserved and not-used
      tmp_serial.type = mem->serial.type;
      tmp_serial.line = mem->serial.line;
      tmp_serial.port = mem->serial.port;
      tmp_serial.irq  = mem->serial.irq ;
      tmp_serial.flags = mem->serial.flags;
      tmp_serial.xmit_fifo_size = mem->serial.xmit_fifo_size;
      tmp_serial.baud_base = mem->serial.baud_base;
      tmp_serial.close_delay = mem->serial.close_delay;
      tmp_serial.closing_wait = mem->serial.closing_wait;
      tmp_serial.custom_divisor = mem->serial.custom_divisor;
      tmp_serial.hub6 = mem->serial.hub6;
      tmp_serial.io_type = mem->serial.io_type;
      tmp_serial.port_high = mem->serial.port_high;
      tmp_serial.iomem_reg_shift = mem->serial.iomem_reg_shift;
      tmp_serial.iomem_base = mem->serial.iomem_base;
      if ( copy_to_user( (struct serial_struct *)arg, &tmp_serial,
                         sizeof(struct serial_struct) ) )
        return -EFAULT;
      return 0;
    case TIOCSSERIAL:
      if ( copy_from_user( &mem->serial, (struct serial_struct *)arg,
                           sizeof(struct serial_struct)))
        return -EFAULT;
      return 0;
    case TIOCMIWAIT:
      if ( copy_from_user( &value, (unsigned int *)arg, sizeof(unsigned int)))
        return -EFAULT;
      {
        DECLARE_WAITQUEUE(wait, current);
        struct async_icount ic_now;
        struct async_icount ic_prev = mem->icount;
        while (1) {
          add_wait_queue( &mem->wait, &wait);
          set_current_state( TASK_INTERRUPTIBLE );
          schedule();
          remove_wait_queue( &mem->wait, &wait);

          if ( signal_pending( current ) )
            return -ERESTARTSYS;
          ic_now = mem->icount;
          if ( ic_now.rng == ic_prev.rng   /* ring */
            && ic_now.dsr == ic_prev.dsr   /* data set ready */
            && ic_now.dcd == ic_prev.dcd   /* data carrier detect */
            && ic_now.cts == ic_prev.cts ) /* clear to send */
            return -EIO;
          if ( ( (value & TIOCM_RNG ) && ( ic_now.rng != ic_prev.rng) )
            || ( (value & TIOCM_DSR ) && ( ic_now.dsr != ic_prev.dsr) )
            || ( (value & TIOCM_CD  ) && ( ic_now.dcd != ic_prev.dcd) )
            || ( (value & TIOCM_CTS ) && ( ic_now.cts != ic_prev.cts) ) )
            return 0;
        }
      }
      break;
    case TIOCGICOUNT:
      tmp_icount.cts = mem->icount.cts;
      tmp_icount.dsr = mem->icount.dsr;
      tmp_icount.rng = mem->icount.rng;
      tmp_icount.dcd = mem->icount.dcd;
      tmp_icount.rx = mem->icount.rx;
      tmp_icount.tx = mem->icount.tx;
      tmp_icount.frame = mem->icount.frame;
      tmp_icount.overrun = mem->icount.overrun;
      tmp_icount.parity = mem->icount.parity;
      tmp_icount.brk = mem->icount.brk;
      tmp_icount.buf_overrun = mem->icount.buf_overrun;
      if ( copy_to_user( ( struct serial_icounter_struct *) arg, &tmp_icount,
                           sizeof( struct serial_icounter_struct ) ) )
        return -EFAULT;
      return 0;
  }
  return -ENOIOCTLCMD;
}

static int mem_tty_open (struct tty_struct *tty, struct file * filp)
{
  struct mem_tty_serial *mem;
  struct timer_list *timer;

  MOD_INC_USE_COUNT;

  /* initialize the pointer in case something fails */
  tty->driver_data = NULL;

  /* get the serial object associated with this tty pointer */
  // mem = mem_tty_table[minor(tty->device)];
  mem = mem_tty_table[MINOR(tty->device)];

  if (mem == NULL) {
    /* first time accessing this device, let's create it */
    mem = kmalloc (sizeof (*mem), GFP_KERNEL);
    if (!mem) {
      MOD_DEC_USE_COUNT;
      return -ENOMEM;
    }
    // create hardware buffer
    mem->hw_buffer = kmalloc( MEM_HW_SIZE, GFP_KERNEL);
    if ( ! mem->hw_buffer ) {
      kfree( mem );
      MOD_DEC_USE_COUNT;
      return -ENOMEM;
    }

    init_MUTEX (&mem->sem);
    mem->open_count = 0;
    mem->timer = NULL;

    // connection state information
    init_waitqueue_head( & mem->wait );


    // mem_tty_table[minor(tty->device)] = mem;
    mem_tty_table[MINOR(tty->device)] = mem;
  }

  down (&mem->sem);
  /* save our structure within the tty structure */
  tty->driver_data = mem;
  mem->tty = tty;
  mem->filp = filp;

  ++mem->open_count;
  if (mem->open_count == 1) {
    /* this is the first time this port is opened */
    /* do any hardware initialization needed here */
    mem->head = 0;
    mem->tail = 0;
    mem->throttling = 0;

    /* reset connection state information */
    mem->mcr = TIOCM_DTR | TIOCM_RTS ;
    mem->msr = TIOCM_CTS | TIOCM_CAR | TIOCM_RI | TIOCM_DSR ;
    memset( & mem->serial, 0, sizeof( struct serial_struct ) );
    mem->serial.type = PORT_UNKNOWN;
    mem->serial.port = SERIAL_IO_PORT;
    mem->serial.line = 1;
    mem->serial.closing_wait = ASYNC_CLOSING_WAIT_INF;
    mem->serial.irq   = 0;
    mem->serial.flags = 0x0;
    mem->serial.xmit_fifo_size = MEM_HW_SIZE;   
    mem->serial.baud_base = 9600;
    mem->serial.close_delay = 0*HZ;
    mem->serial.custom_divisor = 0;
    mem->serial.hub6 = 1;
    mem->serial.io_type = SERIAL_IO_HUB6;
    memset( & mem->icount, 0, sizeof(struct async_icount) );

    /* create our timer and submit it */
    if (!mem->timer) {
      timer = kmalloc (sizeof (*timer), GFP_KERNEL);
      if (!timer) {
        --mem->open_count;
	up (&mem->sem);
        MOD_DEC_USE_COUNT;
	return -ENOMEM;
      }
      mem->timer = timer;
    }
    init_timer (mem->timer);
    mem->timer->data = (unsigned long )mem;
    mem->timer->function = mem_tty_timer;
    
    // mem->timer->expires = jiffies + DELAY_TIME;
    // add_timer (mem->timer);
  }

  up (&mem->sem);
  return 0;
}

static void do_close (struct mem_tty_serial *mem)
{
  down (&mem->sem);

  if (!mem->open_count) { /* port was never opened */
    return;
  }

  --mem->open_count;
  if (mem->open_count <= 0) {
    /* The port is being closed by the last user. */
    /* Do any hardware specific stuff here */

    /* shut down our timer */
    /* N.B. The timer memory is not kfree'd */
    del_timer (mem->timer);
  }

  MOD_DEC_USE_COUNT;
  up (&mem->sem);
}

static void mem_tty_close (struct tty_struct *tty, struct file * filp)
{
  struct mem_tty_serial *mem = tty->driver_data;

  if ( !mem ) {
    return;
  }
  do_close (mem);
}	

static int mem_tty_write (struct tty_struct * tty, int from_user, const unsigned char *buf, int count)
{
    struct mem_tty_serial *mem = tty->driver_data;
    int retval = -EINVAL;
    int c, c1;
    unsigned char * tmp_buf = NULL;

    if ( ! mem ) 
      return -ENODEV;

    down (&mem->sem);
    if (!mem->open_count) { /* port was not opened */
      up (&mem->sem);
      return -EINVAL;
    }

    if ( mem->throttling ) {
      up (&mem->sem);
      return -EAGAIN;
    }


    if ( from_user ) {
      if ( mem->head >= mem->tail ) {
        if ( mem->head == MEM_HW_SIZE-1 && mem->tail == 0 ) {
          up(&mem->sem);
          return -EAGAIN;
        }
        if ( count > MEM_HW_SIZE - mem->head)
          count = MEM_HW_SIZE - mem->head;
      } else {
        if ( mem->head == mem->tail-1 ) {
          up(&mem->sem);
          return -EAGAIN;
        }
        if ( count > mem->tail-1 - mem->head )
          count = mem->tail-1 - mem->head;
      }
      retval = count;

      /* now send the data out the harware port */
      tmp_buf = kmalloc( count, GFP_KERNEL );
      if ( tmp_buf == NULL ) {
        up(&mem->sem);
        return -ENOMEM;
      }
      memset( tmp_buf, 0, count);

      copy_from_user( tmp_buf, buf, count);
      c1 = mem->head; // current writing head
      for (c=0; c<count; c++) {
        mem->hw_buffer[c1] = tmp_buf[c];
        c1 = (c1+1) % MEM_HW_SIZE;
      }

      if ( mem->head == mem->tail ) { // queue read()
        printk(KERN_ALERT "mem_tty_write submit timer \n");
        mem->timer->expires = jiffies + DELAY_TIME;
        add_timer (mem->timer);
      }
      mem->head = c1; // next position to write
        
      kfree( tmp_buf );
    } else {
      // nothing much to do when the data come from the kernel
      printk(KERN_ALERT "mem_tty_write from_kernel count %d %x\n",
        count, buf[0] );

      retval = count;
    }
    
    up(&mem->sem);
        
    // printk(KERN_ALERT "mem_tty_write from_user %d count %d return %d\n",
    //	 from_user, count, retval);
    return retval;
}

static void mem_tty_throttle(struct tty_struct *tty)
{
  struct mem_tty_serial *mem = tty->driver_data;

  if (!mem) 
    return;
  down (&mem->sem);
  if (!mem->open_count) { /* port was not opened */
    up (&mem->sem);
    return;
  }

  del_timer (mem->timer);
  mem->throttling = 1;

  up (&mem->sem);
}

static void mem_tty_unthrottle(struct tty_struct *tty)
{
  struct mem_tty_serial *mem = tty->driver_data;

  if (!mem) 
    return
  down (&mem->sem);
  if (!mem->open_count) { /* port was not opened */
    up (&mem->sem);
    return;
  }
  mem->throttling = 0;
  if ( mem->head == mem->tail ) { // queue read()
    printk(KERN_ALERT "mem_tty_unthrottle submit timer \n");
    mem->timer->expires = jiffies + DELAY_TIME;
    add_timer (mem->timer);
  }
  up (&mem->sem);
}

  

static void mem_tty_flush_chars (struct tty_struct *tty) 
{
  // struct mem_tty_serial *mem = tty->driver_data;
  return;
}

static int mem_tty_chars_in_buffer(struct tty_struct *tty)
{
  struct mem_tty_serial *mem = tty->driver_data;
  int chars;

  if (!mem) 
    return -ENODEV;
  down (&mem->sem);
  if (!mem->open_count) { /* port was not opened */
    up (&mem->sem);
    return -EINVAL;
  }

  if ( mem->head >= mem->tail ) {
    chars = mem->head - mem->tail;
  } else {
    chars = MEM_HW_SIZE - mem->tail + mem->head;
  }
  up (&mem->sem);
  if ( chars > 0 )
    printk(KERN_ALERT "mem_tty_chars_in_buffer chars %d\n", chars);

  return chars;
}

static int mem_tty_write_room (struct tty_struct *tty) 
{
  struct mem_tty_serial *mem = tty->driver_data;
  int room;

  if (!mem) 
    return -ENODEV;
  down (&mem->sem);
  if (!mem->open_count) { /* port was not opened */
    up (&mem->sem);
    return -EINVAL;
  }

  /* calculate how much room is left in the device */
  if ( mem->head >= mem->tail ) {
    room = MEM_HW_SIZE - mem->head + mem->tail - 1;
  } else {
    room = mem->tail-1 - mem->head;
  }
  up (&mem->sem);
  if ( room > 0 )
    printk(KERN_ALERT "mem_tty_write_room room %d\n", room);

  return room;
}

static struct tty_driver mem_tty_driver = {
	magic:			TTY_DRIVER_MAGIC,
	driver_name:		"mem_tty",
#ifndef CONFIG_DEVFS_FS
	name:			"ttty",
#else
	name:			"tts/ttty%d",
#endif
	major:			MEM_TTY_MAJOR,
	num:			MEM_TTY_MINORS,
	type:			TTY_DRIVER_TYPE_SERIAL,
	subtype:		SERIAL_TYPE_NORMAL,
	flags:			TTY_DRIVER_REAL_RAW,
	
	refcount:		&mem_tty_refcount,
	table:			mem_tty,
	termios:		mem_tty_termios,
	termios_locked:		mem_tty_termios_locked,

	open:			mem_tty_open,
	close:			mem_tty_close,
	write:			mem_tty_write,
	write_room:		mem_tty_write_room,
        ioctl:                  mem_tty_ioctl,
	throttle:               mem_tty_throttle,
	unthrottle:             mem_tty_unthrottle,
	flush_chars:            mem_tty_flush_chars,
	chars_in_buffer:        mem_tty_chars_in_buffer,

};

static int __init mem_tty_init(void)
{
	/* register the tty driver */
	mem_tty_driver.init_termios          = tty_std_termios;
	mem_tty_driver.init_termios.c_lflag &= ~( ECHO );
	mem_tty_driver.init_termios.c_cflag  = 
		B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	mem_tty_driver.init_termios.c_oflag &= ~( OPOST );
	mem_tty_driver.init_termios.c_cc[VMIN] = 1;
	mem_tty_driver.init_termios.c_cc[VTIME] = 0;

	if (tty_register_driver (&mem_tty_driver)) {
		printk (KERN_ERR "failed to register mem tty driver\n");
		return -1;
	}

	printk (KERN_INFO DRIVER_DESC " " DRIVER_VERSION "\n");

	return 0;
}

static void __exit mem_tty_exit(void)
{
  struct mem_tty_serial *mem;
  int i;

  tty_unregister_driver(&mem_tty_driver);

  /* shut down all of the timers and free the memory */
  for (i = 0; i < MEM_TTY_MINORS; ++i) {
    mem = mem_tty_table[i];
    if (mem) {
      /* close the port */
      while (mem->open_count)
        do_close (mem);
      /* shut down our timer and free the memory */
      del_timer (mem->timer);
      kfree (mem->timer);

      kfree( mem->hw_buffer );
      kfree (mem);
      mem_tty_table[i] = NULL;
    }
  }
}

module_init(mem_tty_init);
module_exit(mem_tty_exit);
