/*
 * LCD panel support 40x2 LCD  
 *
 * Copyright (C) 201 E3C Tecnologia Ltda 
 * Author: Dimitri Eberhardt Prado <dprado@e3c.com.br> 
 *
 * Derived from drivers/video/omap/lcd-2430sdp.c
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

#include <mach/gpio.h>
#include <plat/mux.h>
#include <asm/mach-types.h>

#define LCD_RS  144
#define LCD_RW  146
#define LCD_E   143
#define LCD_DB4 145
#define LCD_DB5 14
#define LCD_DB6 23
#define LCD_DB7 22
#define DEVICE_NAME "lcd"

#undef LCD_DEBUG

spinlock_t lcd_lock;;
static void put_char(char cmd);
static void put_cmd(char cmd);

#define CHAR_MAP_SIZE 3 
static char* char_map[CHAR_MAP_SIZE][2] = { 
        {"\x0a", ""},
        {"\x0d", ""},
        {"\xc3\x87", "\xc7"} /* Ç */  
        }; 

static int major = 0;
static int dev_open = false;

static int cur_line = 0;
static int cur_col = 0;
static char second_line[40];

static int lcd_open(struct inode *inode, struct file *filp)
{
    if(dev_open)
        return -EBUSY;
    dev_open = true;
    return 0;
}

static int lcd_release(struct inode *inode, struct file *filp)
{
    dev_open = false;
    return 0;
}

static void scroll()
{
    int i = 0;
#ifdef LCD_DEBUG
    printk("scroll 2nd line %s\n", second_line);
#endif
    put_cmd(0x80);
    cur_line= 0;
    for(i = 0; i < 40; i++)
    {
        put_char(second_line[i]);
    } 
    cur_col = 0;
}

static void new_line()
{
    int i = 0;
#ifdef LCD_DEBUG
    printk("newline\n");
#endif
    if(cur_line == 1)
    {
        scroll();
    }
    put_cmd(0xC0);
    cur_line = 1;
    for(i = 0; i < 40; i++)
    {
        put_char(0x20);
    }
    put_cmd(0xC0);
    cur_col = 0;
}

static void carriage_return()
{
#ifdef LCD_DEBUG
    printk("carriage return\n");
#endif
    if(cur_line == 0)
    {
        put_cmd(0x80);
    }
    else
    {
        put_cmd(0xC0);
    }
    cur_col = 0;
}

static int translate(char* in)
{
    int i = 0;
    int j = 0;
#ifdef LCD_DEBUG
    printk("translating %x\n", in[0]);
#endif
    for(i = 0; i < CHAR_MAP_SIZE; i++)
    {
        if(memcmp(char_map[i][0], in, strlen(char_map[i][0]))==0)
        {
#ifdef LCD_DEBUG
            printk("found match on index %d %x == in %x\n", i, char_map[i][0][0], in[0]);
#endif

            /* special case for \f (form feed) */
            if(in[0] == 0x0c)
            {
                new_line();
                new_line();
            }
            /* special case for \n */
            if(in[0] == 0x0a)
            {
                new_line();
            }
            /* special case for \r */
            if(in[0] == 0x0d)
            {
                carriage_return();   
            }

            for(j = 0; j < strlen(char_map[i][1]); j++)
            {
                put_char(char_map[i][1][j]);
            }
            return strlen(char_map[i][0]);
        }
    }
    put_char(in[0]);
    return 1;
}

ssize_t lcd_write(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    int i = 0;
    while((i < 80) && (i < count))
    {
        int res = translate(&buf[i]);
#ifdef LCD_DEBUG
        printk("translate returned %d\n", res);
#endif
        i += res;
    } 
    return count; 
}

struct file_operations lcd_fops = {
  write: lcd_write,
  open: lcd_open,
  release: lcd_release
};


static void lcd_init_write(unsigned char cmd) 
{ 
    gpio_set_value(LCD_RS, 0);
    gpio_set_value(LCD_RW, 0);
    mdelay(1);

    gpio_set_value(LCD_DB7, ((cmd & 0x80)?1:0));
    gpio_set_value(LCD_DB6, ((cmd & 0x40)?1:0));
    gpio_set_value(LCD_DB5, ((cmd & 0x20)?1:0));
    gpio_set_value(LCD_DB4, ((cmd & 0x10)?1:0));

    gpio_set_value(LCD_E, 1);
    mdelay(1);
    gpio_set_value(LCD_E, 0);
} 

static void write_byte(char cmd, char rs)
{
    spin_lock(&lcd_lock);

    gpio_set_value(LCD_RS, rs);
    gpio_set_value(LCD_RW, 0);
    mdelay(1);

    gpio_set_value(LCD_DB7, (cmd & 0x80)?1:0);
    gpio_set_value(LCD_DB6, (cmd & 0x40)?1:0);
    gpio_set_value(LCD_DB5, (cmd & 0x20)?1:0);
    gpio_set_value(LCD_DB4, (cmd & 0x10)?1:0);

    gpio_set_value(LCD_E, 1);
    mdelay(1);
    gpio_set_value(LCD_E, 0);
    mdelay(1);

    gpio_set_value(LCD_DB7, (cmd & 0x08)?1:0);
    gpio_set_value(LCD_DB6, (cmd & 0x04)?1:0);
    gpio_set_value(LCD_DB5, (cmd & 0x02)?1:0);
    gpio_set_value(LCD_DB4, (cmd & 0x1)?1:0);
    gpio_set_value(LCD_E, 1);
    mdelay(1);
    gpio_set_value(LCD_E, 0);
    
    spin_unlock(&lcd_lock);
}

static void put_char(char cmd)
{
#ifdef LCD_DEBUG
    printk("put %x\n", cmd);
#endif
    write_byte(cmd, 1);
    if(cur_line == 1)
    {
        printk("2nd col %d char %x\n", cur_col, cmd);
        second_line[cur_col] = cmd;
    }
    cur_col++;
}

static void put_cmd(char cmd)
{
#ifdef LCD_DEBUG
    printk("cmd %x\n", cmd);
#endif
    write_byte(cmd, 0);
}

static int lcd_40x2_probe(struct platform_device *pdev)
{
    int err = 0;
    if((err = gpio_request(LCD_RS, "lcd RS")) < 0)
    {
        printk("Error requesting RS %d\n", err);
        return err;
    }
    if((err = gpio_request(LCD_RW, "lcd RW")) < 0)
    {
        printk("Error requesting RW %d\n", err);
        return err;
    }
    if((err = gpio_request(LCD_E, "lcd E")) < 0)
    {
        printk("Error requesting E %d\n", err);
        return err;
    }
    if((err = gpio_request(LCD_DB4, "lcd DB4")) < 0)
    {
        printk("Error requesting DB4 %d\n", err);
        return err;
    }
    if((err = gpio_request(LCD_DB5, "lcd DB5")) < 0)
    {
        printk("Error requesting DB5 %d\n", err);
        return err;
    }
    if((err = gpio_request(LCD_DB6, "lcd DB6")) < 0)
    {
        printk("Error requesting DB6 %d\n", err);
        return err;
    }
    if((err = gpio_request(LCD_DB7, "lcd DB7")) < 0)
    {
        printk("Error requesting DB7 %d\n", err);
        return err;
    }
    mdelay(1);
    gpio_direction_output(LCD_RS, 0);
    gpio_direction_output(LCD_RW, 0);
    gpio_direction_output(LCD_E, 0);
    gpio_direction_output(LCD_DB4, 0);
    gpio_direction_output(LCD_DB5, 0);
    gpio_direction_output(LCD_DB6, 0);
    gpio_direction_output(LCD_DB7, 0);
    mdelay(50);

    /* magic init sequence for 4bit comm. */
    lcd_init_write(0x30);
    mdelay(10);
    lcd_init_write(0x30);
    mdelay(5);
    lcd_init_write(0x30);
    mdelay(5);
    lcd_init_write(0x20);

    /* 4bit 2lines 5x7 font */
    put_cmd(0x28);

  /* turn off display */
    put_cmd(0x08);
    /* clear display */
    put_cmd(0x01);
    mdelay(5);
    /* increment display */
    put_cmd(0x06);
    /* turn on display, cursor */
    put_cmd(0x0E);
    /* go home */
    put_cmd(0x02);

	return 0;
}

static int lcd_40x2_remove(struct platform_device *pdev)
{
    unregister_chrdev(major, DEVICE_NAME);
	return 0;
}

static int lcd_40x2_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    /* turn off display */
    put_cmd(0x08);
	return 0;
}

static int lcd_40x2_resume(struct platform_device *pdev)
{
    /* turn on display, cursor */
    put_cmd(0x0E);
	return 0;
}

struct platform_driver lcd_40x2_driver = {
	.probe		= lcd_40x2_probe,
	.remove		= lcd_40x2_remove,
	.suspend	= lcd_40x2_suspend,
	.resume		= lcd_40x2_resume,
	.driver		= {
		.name	= "lcd_40x2",
		.owner	= THIS_MODULE,
	},
};

static int __init lcd_40x2_drv_init(void)
{
    memset(second_line, 0x20, sizeof(second_line));
    major = register_chrdev(0, DEVICE_NAME, &lcd_fops);
    printk("LCD 40x2 registered major %d\n", major);
	return platform_driver_register(&lcd_40x2_driver);
}

static void __exit lcd_40x2_drv_exit(void)
{
	platform_driver_unregister(&lcd_40x2_driver);
}

module_init(lcd_40x2_drv_init);
module_exit(lcd_40x2_drv_exit);
