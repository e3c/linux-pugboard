/*
 * LCD panel support 40x2 LCD  
 *
 * Copyright (C) 201 E3C Tecnologia Ltda 
 * Author: Dimitri Eberhardt Prado <dprado@e3c.com.br> 
 *
 * Derived from drivers/video/omap/metec-2430sdp.c
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

#define LINE_LENGTH 20

#undef METEC_DEBUG

static int major = 0;
static int dev_open = false;

struct task_struct* read_thread;
struct input_dev *input_dev;

static void write_pattern(char* patt);

static const unsigned int keymap[20] = 
{
    KEY_F17,
    KEY_F18,
    KEY_F19,
    KEY_F20,
    KEY_F13,
    KEY_F14,
    KEY_F15,
    KEY_F16,
    KEY_F9,
    KEY_F10,
    KEY_F11,
    KEY_F12,
    KEY_F5,
    KEY_F6,
    KEY_F7,
    KEY_F8,
    KEY_F1,
    KEY_F2,
    KEY_F3,
    KEY_F4,
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
#if LINE_LENGTH < 20
#error Invalid line length
#endif
#ifdef METEC_DEBUG
    int i = 0;
    printk("IN: ");
    for(i = 0; i < 20; i++)
    {
        printk(" %02x", buf[i]);
    }
    printk("\n");
#endif

    patt[0] = 0;
    patt[1] = (buf[19] & 0x40) << 1 | (buf[18] & 0x40) >> 1 | (buf[17] & 0x40) >> 3 | (buf[16] & 0x40) >> 5;
    patt[2] = (buf[15] & 0x40) << 1| (buf[14] & 0x40) >> 1| (buf[13] & 0x40) >> 3| (buf[12] & 0x40) >> 5;
    patt[3] = (buf[11] & 0x40) << 1| (buf[10] & 0x40) >> 1| (buf[9] & 0x40) >> 3| (buf[8] & 0x40) >> 5;
    patt[4] = (buf[7] & 0x40) << 1| (buf[6] & 0x40) >> 1| (buf[5] & 0x40) >> 3| (buf[4] & 0x40) >> 5;
    patt[5] = (buf[3] & 0x40) << 1| (buf[2] & 0x40) >> 1| (buf[1] & 0x40) >> 3| (buf[0] & 0x40) >> 5;

    patt[1] |= (buf[19] & 0x80) >> 1| (buf[18] & 0x80) >> 3 | (buf[17] & 0x80) >> 5 | (buf[16] & 0x80) >> 7;
    patt[2] |= (buf[15] & 0x80) >> 1| (buf[14] & 0x80) >> 3| (buf[13] & 0x80) >> 5| (buf[12] & 0x80) >> 7;
    patt[3] |= (buf[11] & 0x80) >> 1| (buf[10] & 0x80) >> 3| (buf[9] & 0x80) >> 5| (buf[8] & 0x80) >> 7;
    patt[4] |= (buf[7] & 0x80) >> 1| (buf[6] & 0x80) >> 3| (buf[5] & 0x80) >> 5| (buf[4] & 0x80) >> 7;
    patt[5] |= (buf[3] & 0x80) >> 1| (buf[2] & 0x80) >> 3| (buf[1] & 0x80) >> 5| (buf[0] & 0x80) >> 7;

    patt[6] = 0;

    patt[7] = (buf[19] & 0x4) << 5 | (buf[18] & 0x4) << 3| (buf[17] & 0x4) << 1| (buf[16] & 0x4) >> 1;
    patt[8] = (buf[15] & 0x4) << 5| (buf[14] & 0x4) << 3| (buf[13] & 0x4) << 1| (buf[12] & 0x4) >> 1;
    patt[9] = (buf[11] & 0x4) << 5| (buf[10] & 0x4) << 3| (buf[9] & 0x4) << 1| (buf[8] & 0x4) >> 1;
    patt[10] = (buf[7] & 0x4) << 5| (buf[6] & 0x4) << 3| (buf[5] & 0x4) << 1| (buf[4] & 0x4) >> 1;
    patt[11] = (buf[3] & 0x4) << 5| (buf[2] & 0x4) << 3| (buf[1] & 0x4) << 1| (buf[0] & 0x4) >> 1;

    patt[7] |= (buf[19] & 0x20) << 1 | (buf[18] & 0x20) >> 1| (buf[17] & 0x20) >> 3| (buf[16] & 0x20) >> 5;
    patt[8] |= (buf[15] & 0x20) << 1| (buf[14] & 0x20) >> 1| (buf[13] & 0x20) >> 3| (buf[12] & 0x20) >> 5;
    patt[9] |= (buf[11] & 0x20) << 1| (buf[10] & 0x20) >> 1| (buf[9] & 0x20) >> 3| (buf[8] & 0x20) >> 5;
    patt[10] |= (buf[7] & 0x20) << 1| (buf[6] & 0x20) >> 1| (buf[5] & 0x20) >> 3| (buf[4] & 0x20) >> 5;
    patt[11] |= (buf[3] & 0x20) << 1| (buf[2] & 0x20) >> 1| (buf[1] & 0x20) >> 3| (buf[0] & 0x20) >> 5;

    patt[12] = 0;

    patt[13] = (buf[19] & 0x2) << 6 | (buf[18] & 0x2) << 4 | (buf[17] & 0x2) << 2 | (buf[16] & 0x2);
    patt[14] = (buf[15] & 0x2) << 6| (buf[14] & 0x2) << 4| (buf[13] & 0x2) << 2| (buf[12] & 0x2);
    patt[15] = (buf[11] & 0x2) << 6| (buf[10] & 0x2) << 4| (buf[9] & 0x2) << 2| (buf[8] & 0x2);
    patt[16] = (buf[7] & 0x2) << 6| (buf[6] & 0x2) << 4| (buf[5] & 0x2) << 2| (buf[4] & 0x2);
    patt[17] = (buf[3] & 0x2) << 6| (buf[2] & 0x2) << 4| (buf[1] & 0x2) << 2| (buf[0] & 0x2);

    patt[13] |= (buf[19] & 0x10) << 2 | (buf[18] & 0x10) | (buf[17] & 0x10) >> 2 | (buf[16] & 0x10) >> 4;
    patt[14] |= (buf[15] & 0x10) << 2 | (buf[14] & 0x10) | (buf[13] & 0x10) >> 2| (buf[12] & 0x10) >> 4;
    patt[15] |= (buf[11] & 0x10) << 2 | (buf[10] & 0x10) | (buf[9] & 0x10) >> 2| (buf[8] & 0x10) >> 4;
    patt[16] |= (buf[7] & 0x10) << 2 | (buf[6] & 0x10) | (buf[5] & 0x10) >> 2| (buf[4] & 0x10) >> 4;
    patt[17] |= (buf[3] & 0x10) << 2 | (buf[2] & 0x10) | (buf[1] & 0x10) >> 2| (buf[0] & 0x10) >> 4;

    patt[18] = VOLTAGE;

    patt[19] = (buf[19] & 0x01) << 7| (buf[18] & 0x01) << 5| (buf[17] & 0x01) << 3| (buf[16] & 0x01) << 1;
    patt[20] = (buf[15] & 0x01) << 7| (buf[14] & 0x01) << 5| (buf[13] & 0x01) << 3| (buf[12] & 0x01) << 1;
    patt[21] = (buf[11] & 0x01) << 7| (buf[10] & 0x01) << 5| (buf[9] & 0x01) << 3| (buf[8] & 0x01) << 1;
    patt[22] = (buf[7] & 0x01) << 7| (buf[6] & 0x01) << 5| (buf[5] & 0x01) << 3| (buf[4] & 0x01) << 1;
    patt[23] = (buf[3] & 0x01) << 7| (buf[2] & 0x01) << 5| (buf[1] & 0x01) << 3| (buf[0] & 0x01) << 1;

    patt[19] |= (buf[19] & 0x08) << 3| (buf[18] & 0x08) << 1| (buf[17] & 0x08) >> 1| (buf[16] & 0x08) >> 3;
    patt[20] |= (buf[15] & 0x08) << 3| (buf[14] & 0x08) << 1| (buf[13] & 0x08) >> 1| (buf[12] & 0x08) >> 3;
    patt[21] |= (buf[11] & 0x08) << 3| (buf[10] & 0x08) << 1| (buf[9] & 0x08) >> 1| (buf[8] & 0x08) >> 3;
    patt[22] |= (buf[7] & 0x08) << 3| (buf[6] & 0x08) << 1| (buf[5] & 0x08) >> 1| (buf[4] & 0x08) >> 3;
    patt[23] |= (buf[3] & 0x08) << 3| (buf[2] & 0x08) << 1| (buf[1] & 0x08) >> 1| (buf[0] & 0x08) >> 3;

#ifdef METEC_DEBUG
    printk("OUT: ");
    for(i = 0; i < 24; i++)
    {
        printk(" %02x", patt[i]);
    }
    printk("\n");
#endif

}

ssize_t metec_write(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    if(!dev_open)
        return -EBUSY;
    char patt[LINE_LENGTH+4] = {0}; 
    char buffer[LINE_LENGTH] = {0};
    memcpy(buffer, buf, (count>LINE_LENGTH?LINE_LENGTH:count));
    build_pattern(buffer, patt);  
    write_pattern(patt);
    return (count<=LINE_LENGTH?count:LINE_LENGTH); 
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
    for(i = 0; i < 24; i++)
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
    uint32_t state = 0;
    uint32_t front_keys = 0;
    uint32_t last_front_keys = 0;
    uint32_t cursor_keys = 0;
    uint32_t last_cursor_keys = 0;
    uint32_t i = 0;
    uint32_t j = 0;
    unsigned long flags;
    
    set_current_state(TASK_INTERRUPTIBLE);
    while(!kthread_should_stop())
    {
        spin_lock_irqsave(&metec_lock, flags);
        gpio_set_value(METEC_STROBE, 0);
        udelay(10);
        for(i = 0; i < 20; i++)
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
                cursor_keys |= 1 << i;
            }
            else
            {
                cursor_keys &= ~ (1<< i);
            }
            if(state & 2) 
            {
                front_keys |= 1 << i;
            }
            else
            {
                front_keys &= ~ (1<< i);
            }
        }
        udelay(10);
        gpio_set_value(METEC_CLK, 0);
        gpio_set_value(METEC_STROBE, 1);
        
        spin_unlock_irqrestore(&metec_lock, flags);

        if(front_keys != last_front_keys)
        {
    #ifdef METEC_DEBUG
            printk("METEC frontkeys changed from %x to %x\n", last_front_keys, front_keys);
    #endif
            last_front_keys = front_keys;
        }

        if(cursor_keys != last_cursor_keys)
        {
            uint32_t k = 0;
    #ifdef METEC_DEBUG
            printk("METEC cursorkeys changed from %x to %x\n", last_cursor_keys, cursor_keys);
    #endif
            /* lets report every difference, only on cursor keys */
            uint32_t diff_map = last_cursor_keys ^ cursor_keys;
            for(k = 0; k < 20; k++)
            {
                if(diff_map & (1 << k))
                {
                    input_event(input_dev, EV_MSC, MSC_SCAN, k);
                    input_report_key(input_dev, keymap[k], (cursor_keys & (1 << k))?0:1);
                    input_sync(input_dev);
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
    unsigned short *keycodes;

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
    
    char patt[] = { "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00" };
    write_pattern(patt);

    keycodes = kzalloc(20 * sizeof(*keycodes), GFP_KERNEL);
    input_dev = input_allocate_device();

    input_dev->name     = "metec-keys";
    input_dev->id.bustype   = BUS_HOST;
    input_dev->evbit[0] = BIT_MASK(EV_KEY); 
    input_dev->keycode  = keycodes;
    input_dev->keycodesize  = sizeof(unsigned int);
    input_dev->keycodemax   = ARRAY_SIZE(keymap);

    memcpy(input_dev->keycode, keymap, sizeof(keymap));

    input_set_capability(input_dev, EV_MSC, MSC_SCAN);

    err = input_register_device(input_dev);
    if (err)
        goto err_free_mem;

    read_thread = kthread_run(read_keys, 0, "metec_keys");    

	return 0;

err_free_mem:
    input_free_device(input_dev);
    kfree(keycodes);
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
	return 0;
}

static int metec_resume(struct platform_device *pdev)
{
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

static int __init metec_drv_init(void)
{
    major = register_chrdev(0, DEVICE_NAME, &metec_fops);
    printk("Metec braille line registered major %d\n", major);
	return platform_driver_register(&metec_driver);
}

static void __exit metec_drv_exit(void)
{
	platform_driver_unregister(&metec_driver);
}

module_init(metec_drv_init);
module_exit(metec_drv_exit);
