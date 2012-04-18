/*
 * METEC braille line driver
 *
 * Copyright (C) 2011 E3C Tecnologia Ltda
 * Author: Dimitri Eberhardt Prado <dprado@e3c.com.br>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/i2c/twl.h>
#include <linux/kthread.h>
#include <linux/input.h>

#include <mach/gpio.h>
#include <plat/mux.h>
#include <asm/mach-types.h>

#define METEC_CLK      159
#define METEC_STROBE   161
#define METEC_DIN      162
#define METEC_DOUT     158
#define METEC_SHUTDOWN 156
#define VOLTAGE 0xFF
#define DEVICE_NAME "braille"

#define NUMBER_OF_LINES 2

#undef METEC_DEBUG

#if NUMBER_OF_LINES > 2
#error Invalid line length
#endif

static int major = 0;
static int dev_open = false;

struct task_struct* read_thread;
struct input_dev *input_dev;

static void write_pattern(char* patt);
static uint32_t cursor_map[] = {
    16,
    17,
    18,
    19,
    12,
    13,
    14,
    15,
    8,
    9,
    10,
    11,
    4,
    5,
    6,
    7,
    0,
    1,
    2,
    3,
    36,
    37,
    38,
    39,
    32,
    33,
    34,
    35,
    28,
    29,
    30,
    31,
    24,
    25,
    26,
    27,
    20,
    21,
    22,
    23
};

static uint32_t key_map[] = {
    KEY_Q,
    KEY_R,
    KEY_S,
    KEY_T,
    KEY_M,
    KEY_N,
    KEY_O,
    KEY_P,
    KEY_I,
    KEY_J,
    KEY_K,
    KEY_L,
    KEY_E,
    KEY_F,
    KEY_G,
    KEY_H,
    KEY_A,
    KEY_B,
    KEY_C,
    KEY_D,
    KEY_EQUAL,
    KEY_MINUS,
    KEY_LEFTBRACE,
    KEY_RIGHTBRACE,
    KEY_6,
    KEY_7,
    KEY_8,
    KEY_9,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_5,
    KEY_Y,
    KEY_Z,
    KEY_0,
    KEY_1,
    KEY_U,
    KEY_V,
    KEY_W,
    KEY_X
};


static int metec_open(struct inode *inode, struct file *filp)
{
    if(dev_open)
        return -EBUSY;
    dev_open = true;
    return 0;
}

static int metec_release(struct inode *inode, struct file *filp)
{
    dev_open = false;
    return 0;
}

void build_pattern(char* buf, char* patt)
{
#ifdef METEC_DEBUG
    int i = 0;
    printk("IN: ");
    for(i = 0; i < NUMBER_OF_LINES * 20; i++)
    {
        printk(" %02x", buf[i]);
    }
    printk("\n");
#else
    int i = 0;
#endif
    for(i = 0; i < NUMBER_OF_LINES; i++)
    {
        patt[0 + (24*(NUMBER_OF_LINES-i-1))] = 0;
        patt[1 + (24*(NUMBER_OF_LINES-i-1))] = (buf[19 + (20*i)] & 0x40) << 1 | (buf[18 + (20*i)] & 0x40) >> 1 | (buf[17 + (20*i)] & 0x40) >> 3 | (buf[16 + (20*i)] & 0x40) >> 5;
        patt[2 + (24*(NUMBER_OF_LINES-i-1))] = (buf[15 + (20*i)] & 0x40) << 1| (buf[14 + (20*i)] & 0x40) >> 1| (buf[13 + (20*i)] & 0x40) >> 3| (buf[12 + (20*i)] & 0x40) >> 5;
        patt[3 + (24*(NUMBER_OF_LINES-i-1))] = (buf[11 + (20*i)] & 0x40) << 1| (buf[10 + (20*i)] & 0x40) >> 1| (buf[9 + (20*i)] & 0x40) >> 3| (buf[8 + (20*i)] & 0x40) >> 5;
        patt[4 + (24*(NUMBER_OF_LINES-i-1))] = (buf[7 + (20*i)] & 0x40) << 1| (buf[6 + (20*i)] & 0x40) >> 1| (buf[5 + (20*i)] & 0x40) >> 3| (buf[4 + (20*i)] & 0x40) >> 5;
        patt[5 + (24*(NUMBER_OF_LINES-i-1))] = (buf[3 + (20*i)] & 0x40) << 1| (buf[2 + (20*i)] & 0x40) >> 1| (buf[1 + (20*i)] & 0x40) >> 3| (buf[0 + (20*i)] & 0x40) >> 5;

        patt[1 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[19 + (20*i)] & 0x80) >> 1| (buf[18 + (20*i)] & 0x80) >> 3 | (buf[17 + (20*i)] & 0x80) >> 5 | (buf[16 + (20*i)] & 0x80) >> 7;
        patt[2 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[15 + (20*i)] & 0x80) >> 1| (buf[14 + (20*i)] & 0x80) >> 3| (buf[13 + (20*i)] & 0x80) >> 5| (buf[12 + (20*i)] & 0x80) >> 7;
        patt[3 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[11 + (20*i)] & 0x80) >> 1| (buf[10 + (20*i)] & 0x80) >> 3| (buf[9 + (20*i)] & 0x80) >> 5| (buf[8 + (20*i)] & 0x80) >> 7;
        patt[4 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[7 + (20*i)] & 0x80) >> 1| (buf[6 + (20*i)] & 0x80) >> 3| (buf[5 + (20*i)] & 0x80) >> 5| (buf[4 + (20*i)] & 0x80) >> 7;
        patt[5 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[3 + (20*i)] & 0x80) >> 1| (buf[2 + (20*i)] & 0x80) >> 3| (buf[1 + (20*i)] & 0x80) >> 5| (buf[0 + (20*i)] & 0x80) >> 7;

        patt[6 + (24*(NUMBER_OF_LINES-i-1))] = 0;

        patt[7 + (24*(NUMBER_OF_LINES-i-1))] = (buf[19 + (20*i)] & 0x4) << 5 | (buf[18 + (20*i)] & 0x4) << 3| (buf[17 + (20*i)] & 0x4) << 1| (buf[16 + (20*i)] & 0x4) >> 1;
        patt[8 + (24*(NUMBER_OF_LINES-i-1))] = (buf[15 + (20*i)] & 0x4) << 5| (buf[14 + (20*i)] & 0x4) << 3| (buf[13 + (20*i)] & 0x4) << 1| (buf[12 + (20*i)] & 0x4) >> 1;
        patt[9 + (24*(NUMBER_OF_LINES-i-1))] = (buf[11 + (20*i)] & 0x4) << 5| (buf[10 + (20*i)] & 0x4) << 3| (buf[9 + (20*i)] & 0x4) << 1| (buf[8 + (20*i)] & 0x4) >> 1;
        patt[10 + (24*(NUMBER_OF_LINES-i-1))] = (buf[7 + (20*i)] & 0x4) << 5| (buf[6 + (20*i)] & 0x4) << 3| (buf[5 + (20*i)] & 0x4) << 1| (buf[4 + (20*i)] & 0x4) >> 1;
        patt[11 + (24*(NUMBER_OF_LINES-i-1))] = (buf[3 + (20*i)] & 0x4) << 5| (buf[2 + (20*i)] & 0x4) << 3| (buf[1 + (20*i)] & 0x4) << 1| (buf[0 + (20*i)] & 0x4) >> 1;

        patt[7 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[19 + (20*i)] & 0x20) << 1 | (buf[18 + (20*i)] & 0x20) >> 1| (buf[17 + (20*i)] & 0x20) >> 3| (buf[16 + (20*i)] & 0x20) >> 5;
        patt[8 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[15 + (20*i)] & 0x20) << 1| (buf[14 + (20*i)] & 0x20) >> 1| (buf[13 + (20*i)] & 0x20) >> 3| (buf[12 + (20*i)] & 0x20) >> 5;
        patt[9 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[11 + (20*i)] & 0x20) << 1| (buf[10 + (20*i)] & 0x20) >> 1| (buf[9 + (20*i)] & 0x20) >> 3| (buf[8 + (20*i)] & 0x20) >> 5;
        patt[10 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[7 + (20*i)] & 0x20) << 1| (buf[6 + (20*i)] & 0x20) >> 1| (buf[5 + (20*i)] & 0x20) >> 3| (buf[4 + (20*i)] & 0x20) >> 5;
        patt[11 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[3 + (20*i)] & 0x20) << 1| (buf[2 + (20*i)] & 0x20) >> 1| (buf[1 + (20*i)] & 0x20) >> 3| (buf[0 + (20*i)] & 0x20) >> 5;

        patt[12 + (24*(NUMBER_OF_LINES-i-1))] = 0;

        patt[13 + (24*(NUMBER_OF_LINES-i-1))] = (buf[19 + (20*i)] & 0x2) << 6 | (buf[18 + (20*i)] & 0x2) << 4 | (buf[17 + (20*i)] & 0x2) << 2 | (buf[16 + (20*i)] & 0x2);
        patt[14 + (24*(NUMBER_OF_LINES-i-1))] = (buf[15 + (20*i)] & 0x2) << 6| (buf[14 + (20*i)] & 0x2) << 4| (buf[13 + (20*i)] & 0x2) << 2| (buf[12 + (20*i)] & 0x2);
        patt[15 + (24*(NUMBER_OF_LINES-i-1))] = (buf[11 + (20*i)] & 0x2) << 6| (buf[10 + (20*i)] & 0x2) << 4| (buf[9 + (20*i)] & 0x2) << 2| (buf[8 + (20*i)] & 0x2);
        patt[16 + (24*(NUMBER_OF_LINES-i-1))] = (buf[7 + (20*i)] & 0x2) << 6| (buf[6 + (20*i)] & 0x2) << 4| (buf[5 + (20*i)] & 0x2) << 2| (buf[4 + (20*i)] & 0x2);
        patt[17 + (24*(NUMBER_OF_LINES-i-1))] = (buf[3 + (20*i)] & 0x2) << 6| (buf[2 + (20*i)] & 0x2) << 4| (buf[1 + (20*i)] & 0x2) << 2| (buf[0 + (20*i)] & 0x2);

        patt[13 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[19 + (20*i)] & 0x10) << 2 | (buf[18 + (20*i)] & 0x10) | (buf[17 + (20*i)] & 0x10) >> 2 | (buf[16 + (20*i)] & 0x10) >> 4;
        patt[14 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[15 + (20*i)] & 0x10) << 2 | (buf[14 + (20*i)] & 0x10) | (buf[13 + (20*i)] & 0x10) >> 2| (buf[12 + (20*i)] & 0x10) >> 4;
        patt[15 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[11 + (20*i)] & 0x10) << 2 | (buf[10 + (20*i)] & 0x10) | (buf[9 + (20*i)] & 0x10) >> 2| (buf[8 + (20*i)] & 0x10) >> 4;
        patt[16 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[7 + (20*i)] & 0x10) << 2 | (buf[6 + (20*i)] & 0x10) | (buf[5 + (20*i)] & 0x10) >> 2| (buf[4 + (20*i)] & 0x10) >> 4;
        patt[17 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[3 + (20*i)] & 0x10) << 2 | (buf[2 + (20*i)] & 0x10) | (buf[1 + (20*i)] & 0x10) >> 2| (buf[0 + (20*i)] & 0x10) >> 4;

        patt[18 + (24*(NUMBER_OF_LINES-i-1))] = VOLTAGE;

        patt[19 + (24*(NUMBER_OF_LINES-i-1))] = (buf[19 + (20*i)] & 0x01) << 7| (buf[18 + (20*i)] & 0x01) << 5| (buf[17 + (20*i)] & 0x01) << 3| (buf[16 + (20*i)] & 0x01) << 1;
        patt[20 + (24*(NUMBER_OF_LINES-i-1))] = (buf[15 + (20*i)] & 0x01) << 7| (buf[14 + (20*i)] & 0x01) << 5| (buf[13 + (20*i)] & 0x01) << 3| (buf[12 + (20*i)] & 0x01) << 1;
        patt[21 + (24*(NUMBER_OF_LINES-i-1))] = (buf[11 + (20*i)] & 0x01) << 7| (buf[10 + (20*i)] & 0x01) << 5| (buf[9 + (20*i)] & 0x01) << 3| (buf[8 + (20*i)] & 0x01) << 1;
        patt[22 + (24*(NUMBER_OF_LINES-i-1))] = (buf[7 + (20*i)] & 0x01) << 7| (buf[6 + (20*i)] & 0x01) << 5| (buf[5 + (20*i)] & 0x01) << 3| (buf[4 + (20*i)] & 0x01) << 1;
        patt[23 + (24*(NUMBER_OF_LINES-i-1))] = (buf[3 + (20*i)] & 0x01) << 7| (buf[2 + (20*i)] & 0x01) << 5| (buf[1 + (20*i)] & 0x01) << 3| (buf[0 + (20*i)] & 0x01) << 1;

        patt[19 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[19 + (20*i)] & 0x08) << 3| (buf[18 + (20*i)] & 0x08) << 1| (buf[17 + (20*i)] & 0x08) >> 1| (buf[16 + (20*i)] & 0x08) >> 3;
        patt[20 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[15 + (20*i)] & 0x08) << 3| (buf[14 + (20*i)] & 0x08) << 1| (buf[13 + (20*i)] & 0x08) >> 1| (buf[12 + (20*i)] & 0x08) >> 3;
        patt[21 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[11 + (20*i)] & 0x08) << 3| (buf[10 + (20*i)] & 0x08) << 1| (buf[9 + (20*i)] & 0x08) >> 1| (buf[8 + (20*i)] & 0x08) >> 3;
        patt[22 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[7 + (20*i)] & 0x08) << 3| (buf[6 + (20*i)] & 0x08) << 1| (buf[5 + (20*i)] & 0x08) >> 1| (buf[4 + (20*i)] & 0x08) >> 3;
        patt[23 + (24*(NUMBER_OF_LINES-i-1))] |= (buf[3 + (20*i)] & 0x08) << 3| (buf[2 + (20*i)] & 0x08) << 1| (buf[1 + (20*i)] & 0x08) >> 1| (buf[0 + (20*i)] & 0x08) >> 3;
    }
#ifdef METEC_DEBUG
    printk("OUT: ");
    for(i = 0; i < NUMBER_OF_LINES * 24; i++)
    {
        printk(" %02x", patt[i]);
    }
    printk("\n");
#endif

}

ssize_t metec_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    char patt[NUMBER_OF_LINES * 24] = {0};
    char buffer[NUMBER_OF_LINES * 20] = {0};
    if(!dev_open)
        return -EBUSY;
    memcpy(buffer, buf, (count>(NUMBER_OF_LINES * 20)?NUMBER_OF_LINES * 20:count));
    build_pattern(buffer, patt);
    write_pattern(patt);
    return (count<=(NUMBER_OF_LINES * 20)?count:(NUMBER_OF_LINES * 20));
}

struct file_operations metec_fops = {
  write: metec_write,
  open: metec_open,
  release: metec_release
};

spinlock_t metec_lock;;

static void write_pattern(char* patt)
{
    int i = 0;
    int j = 0;
    unsigned long flags;

    spin_lock_irqsave(&metec_lock, flags);

    gpio_set_value(METEC_STROBE, 1);
    udelay(10);
    for(i = 0; i < NUMBER_OF_LINES * 24; i++)
    {
        for(j = 7; j >= 0; j--)
        {
            gpio_set_value(METEC_CLK, 0);
            gpio_set_value(METEC_DIN, (patt[i]&(1<<j))?1:0);
            udelay(10);
            gpio_set_value(METEC_CLK, 1);
            udelay(10);
        }
    }
    gpio_set_value(METEC_STROBE, 0);
    spin_unlock_irqrestore(&metec_lock, flags);
}

static int read_keys(void* data)
{
    uint64_t state = 0;
    uint64_t front_keys = 0;
    uint64_t last_front_keys = 0;
    uint64_t cursor_keys = 0;
    uint64_t last_cursor_keys = 0;
    uint32_t i = 0;
    uint32_t j = 0;
    unsigned long flags;

    set_current_state(TASK_INTERRUPTIBLE);
    while(!kthread_should_stop())
    {
        spin_lock_irqsave(&metec_lock, flags);
        gpio_set_value(METEC_STROBE, 0);
        udelay(10);
        for(i = 0; i < NUMBER_OF_LINES*20; i++)
        {
            state = 0;
            for(j = 0; j < 2; j++)
            {
                udelay(10);
                gpio_set_value(METEC_CLK, 0);
                udelay(10);
                state <<= 1;
                state |= gpio_get_value(METEC_DOUT);
                gpio_set_value(METEC_CLK, 1);
            }
            if(state & 1)
            {
                cursor_keys |= (uint64_t)1 << i;
            }
            else
            {
                cursor_keys &= ~ ((uint64_t)1<< i);
            }
            if(state & 2)
            {
                front_keys |= (uint64_t)1 << i;
            }
            else
            {
                front_keys &= ~ ((uint64_t)1<< i);
            }
        }
        udelay(10);
        gpio_set_value(METEC_CLK, 0);
        gpio_set_value(METEC_STROBE, 1);

        spin_unlock_irqrestore(&metec_lock, flags);
        /* set unused bits */
        cursor_keys |= 0xffffff0000000000LL;
        front_keys |= 0xffffff0000000000LL;

        if(front_keys != last_front_keys)
        {
            uint32_t k = 0;
    #ifdef METEC_DEBUG
            printk("METEC frontkeys changed from %llx to %llx\n", last_front_keys, front_keys);
    #endif
            uint64_t diff_map = last_front_keys ^ front_keys;
            for(k = 0; k < NUMBER_OF_LINES*20; k++)
            {
                if(diff_map & ((uint64_t)1 << k))
                {
                    input_report_key(input_dev, key_map[k], (~front_keys & ((uint64_t)1 << k)?1:0));
                }
            }

            last_front_keys = front_keys;
        }

        if(cursor_keys != last_cursor_keys)
        {
            uint32_t k = 0;
    #ifdef METEC_DEBUG
            printk("METEC cursorkeys changed from %llx to %llx\n", last_cursor_keys, cursor_keys);
    #endif
            /* lets report every difference, only on cursor keys */
            uint64_t diff_map = last_cursor_keys ^ cursor_keys;
            for(k = 0; k < NUMBER_OF_LINES*20; k++)
            {
                if(diff_map & ((uint64_t)1 << k))
                {
                    // report only key press
                    if(!(cursor_keys & ((uint64_t)1 << k)))
                    {
                        input_event(input_dev, EV_MSC, MSC_RAW, cursor_map[k]);
                    }
                }
            }
            last_cursor_keys = cursor_keys;
        }
        msleep(50);
    }
    return 0;
}

static int metec_probe(struct platform_device *pdev)
{
    int err = 0;
    int i = 0;
    char patt[48] = { 0 };

    if((err = gpio_request(METEC_STROBE, "metec STROBE")) < 0)
    {
        printk("Error requesting RS %d\n", err);
        return err;
    }
    if((err = gpio_request(METEC_CLK, "metec CLOCK")) < 0)
    {
        printk("Error requesting RW %d\n", err);
        return err;
    }
    if((err = gpio_request(METEC_DIN, "metec DATAIN")) < 0)
    {
        printk("Error requesting E %d\n", err);
        return err;
    }
    if((err = gpio_request(METEC_DOUT, "metec DATAOUT")) < 0)
    {
        printk("Error requesting DB4 %d\n", err);
        return err;
    }
    if((err = gpio_request(METEC_SHUTDOWN, "metec SHUTDOWN")) < 0)
    {
        printk("Error requesting DB5 %d\n", err);
        return err;
    }
    gpio_direction_output(METEC_STROBE, 0);
    gpio_direction_output(METEC_CLK, 0);
    gpio_direction_output(METEC_DIN, 0);
    gpio_direction_output(METEC_SHUTDOWN, 0);
    if((err = gpio_direction_input(METEC_DOUT)) < 0)
    {
        printk("Error setting dout as INPUT %d\n", err);
        return err;
    }

    gpio_set_value(METEC_SHUTDOWN, 1);
    gpio_set_value(METEC_STROBE, 0);
    gpio_set_value(METEC_CLK, 0);
    mdelay(1);

    write_pattern(patt);

    input_dev = input_allocate_device();
    if (!input_dev)
    {
        err = -ENOMEM;
        goto err_free_mem;
    }

    input_dev->name     = "metec-keys";
    input_dev->id.bustype   = BUS_HOST;
    input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP) | BIT_MASK(EV_MSC);
    input_dev->mscbit[0] = BIT(MSC_RAW);
    for(; i < 20*NUMBER_OF_LINES; i++)
    {
        set_bit(key_map[i],  input_dev->keybit);
    }

    err = input_register_device(input_dev);
    if (err)
        goto err_free_mem;


    read_thread = kthread_run(read_keys, 0, "metec_keys");

	return 0;

err_free_mem:
    printk("Error registering device\n");
    input_free_device(input_dev);
    return err;
}

static int metec_remove(struct platform_device *pdev)
{
    unregister_chrdev(major, DEVICE_NAME);
    gpio_set_value(METEC_SHUTDOWN, 0);
    kthread_stop(read_thread);
    input_unregister_device(input_dev);
	return 0;
}

static int metec_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    unsigned long flags;

    spin_lock_irqsave(&metec_lock, flags);
    gpio_set_value(METEC_SHUTDOWN, 0);
    spin_unlock_irqrestore(&metec_lock, flags);
	return 0;
}

static int metec_resume(struct platform_device *pdev)
{
    unsigned long flags;

    spin_lock_irqsave(&metec_lock, flags);
    gpio_set_value(METEC_SHUTDOWN, 1);
    spin_unlock_irqrestore(&metec_lock, flags);
	return 0;
}

struct platform_driver metec_driver = {
	.probe		= metec_probe,
	.remove		= metec_remove,
	.suspend	= metec_suspend,
	.resume		= metec_resume,
	.driver		= {
		.name	= "metec",
		.owner	= THIS_MODULE,
	},
};

static struct class* metec_class;
static struct device* metec_device;

static int __init metec_drv_init(void)
{
    major = register_chrdev(0, DEVICE_NAME, &metec_fops);
    printk("Metec braille line registered major %d\n", major);

    metec_class = class_create(THIS_MODULE, "braille");
    metec_device = device_create(metec_class, NULL, MKDEV(major, 0), NULL, "braille");

	return platform_driver_register(&metec_driver);
}

static void __exit metec_drv_exit(void)
{
	platform_driver_unregister(&metec_driver);
}

module_init(metec_drv_init);
module_exit(metec_drv_exit);
