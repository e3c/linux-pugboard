/*
 * LCD panel support I2C LCD  
 *
 * Copyright (C) 2012 E3C Tecnologia Ltda 
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
#include <linux/i2c.h>
#include <linux/i2c/twl.h>

#include <mach/gpio.h>
#include <plat/mux.h>
#include <asm/mach-types.h>

#include <linux/list.h>

#define DEVICE_NAME "lcdpug"

#undef LCD_DEBUG

static spinlock_t lcd_lock;;
static void put_char(char cmd);
static void put_cmd(char cmd);

struct i2c_client* lcd_client = NULL;

static struct char_map_list{
    char* key;
    char* map;;
    struct list_head list;
} char_map;

static int major = 0;
static int dev_open = false;

static int cur_line = 0;
static int cur_col = 0;
static char second_line[40];


static ssize_t char_map_show(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    struct char_map_list *tmp;
    struct list_head *pos;
    int count=0;
    list_for_each(pos, &char_map.list)
    {
        tmp = list_entry(pos, struct char_map_list, list);
        count += snprintf(buf+count, PAGE_SIZE - count, "%s => %x\n", tmp->key, tmp->map[0]);
    }
    return count; 
}

static ssize_t char_map_remove(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t n)
{
    struct char_map_list *tmp;
    struct list_head *pos;

    char* read_key = buf;
    unsigned int read_hex;
    unsigned char key[7];
    int i = 0;

    for(i = 0; i < (strlen(read_key)>14?14:strlen(read_key)); i+=2)
    {
        printk(KERN_EMERG "%c %d\n", read_key[i],i);
        if(read_key[i] == '\n') break; 
        if (sscanf(read_key+i, "%x", &read_hex) != 1) {
            printk(KERN_EMERG "lcdpug: wrong key format, must be HH[HH[HH[HH[HH[HH]]]]]\n");
            return -EINVAL;
        }
        key[i/2] = (unsigned char)read_hex;
    }

    list_for_each(pos, &char_map.list)
    {
        tmp = list_entry(pos, struct char_map_list, list);
        if(strcmp(tmp->key, key) == 0)
        {
            kfree(tmp->key);
            kfree(tmp->map);
            list_del(tmp);
            kfree(tmp);
            printk(KERN_EMERG "removed key %s\n", read_key);
            return n;
        }

    }
    printk(KERN_EMERG "key %s not found\n", read_key);
    return n; 
}

static ssize_t char_map_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t n)
{
    char* read_key = buf;
    char* read_map = strchr(buf, ',') + 1;
    if(read_map == 1)
    {
            printk(KERN_EMERG "lcdpug: wrong key/map format, missing comma\n");
            return -EINVAL;
    }
    unsigned char* key = (char*)kmalloc((strlen(read_key) - strlen(read_map))/2, GFP_KERNEL);
    unsigned char* map = (char*)kmalloc(strlen(read_map+1)/2 + 1, GFP_KERNEL);
    printk(KERN_EMERG "Malloced map %p\n", map);
    printk(KERN_EMERG "Malloced key %p\n", key);
    unsigned int read_hex;

    int i = 0;
    for(i = 0 ; i < strlen(read_map); i+=2)
    {
        printk(KERN_EMERG "%d\n", i);
        if (sscanf(read_map+i, "%x", &read_hex) != 1) {
            printk(KERN_EMERG "lcdpug: wrong map format, must be HH[HH[HH[HH[HH[HH]]]]]\n");
            return -EINVAL;
        }
        printk(KERN_EMERG "read %d %x\n", i, read_hex);
        map[i/2] = (unsigned char)read_hex;
    }
    map[i/2] = 0;

    for(i = 0; i < strlen(read_key) - strlen(read_map) - 1; i+=2)
    {
        printk(KERN_EMERG "%d\n", i);
        if (sscanf(read_key+i, "%x", &read_hex) != 1) {
            printk(KERN_EMERG "lcdpug: wrong key format, must be HH[HH[HH[HH[HH[HH]]]]]\n");
            return -EINVAL;
        }
        printk(KERN_EMERG "read %d %x\n", i, read_hex);
        key[i/2] = (unsigned char)read_hex;
    }
    key[i/2] = 0;

    struct char_map_list* tmp_item = (struct char_map_list*)kmalloc(sizeof(struct char_map_list), GFP_KERNEL);
    printk(KERN_EMERG "Malloced item %p\n", tmp_item);
    tmp_item->key = key;
    tmp_item->map = map;
    printk(KERN_EMERG "Adding to list\n");
    list_add(&tmp_item->list, &char_map.list);
    return n;
}

DEVICE_ATTR(cmap_add, 0222, NULL, char_map_store);
DEVICE_ATTR(cmap_del, 0222, NULL, char_map_remove);
DEVICE_ATTR(cmap_show, 0444, char_map_show, NULL);

static int lcd_open(struct inode *inode, struct file *filp)
{
    printk("lcd open\n");
    if(dev_open)
        return -EBUSY;
    dev_open = true;
    return 0;
}

static int lcd_release(struct inode *inode, struct file *filp)
{
    printk("lcd close\n");
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
#if 0
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
#endif
    return 1;
}

static ssize_t lcd_write(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    int i = 0;
    printk("lcd write\n");
#if 0
    while((i < 80) && (i < count))
    {
        int res = translate(&buf[i]);
#ifdef LCD_DEBUG
        printk("translate returned %d\n", res);
#endif
        i += res;
    } 
#endif
    return count; 
}

static struct file_operations lcd_fops = {
  write: lcd_write,
  open: lcd_open,
  release: lcd_release
};


static void lcd_init_write(unsigned char cmd) 
{ 
#if 0
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
#endif
} 

static void write_byte(char cmd, char rs)
{
#if 0
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
#endif
}

static void put_char(char cmd)
{
#ifdef LCD_DEBUG
    printk("put %x\n", cmd);
#endif
#if 0
    write_byte(cmd, 1);
    if(cur_line == 1)
    {
        second_line[cur_col] = cmd;
    }
    cur_col++;
#endif
}

static void put_cmd(char cmd)
{
#ifdef LCD_DEBUG
    printk("cmd %x\n", cmd);
#endif
    write_byte(cmd, 0);
}

static int lcd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
    int err = 0;
    printk("LCD probe!!!!!!!!!!!!!!!!!!!!!!!!!! Client is %x\n", client);
    lcd_client = client;

#if 0
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

    /* temporary hack to add ã and ç to CGRAM */
    put_cmd(0x40);
    put_char(0x00);
    put_char(0x00);
    put_char(0x00);
    put_char(0x00);
    put_char(0x00);
    put_char(0x00);
    put_char(0x00);
    put_char(0x00);

    /* ç */
    put_char(0x00);
    put_char(0x00);
    put_char(0x0E);
    put_char(0x10);
    put_char(0x11);
    put_char(0x0E);
    put_char(0x04);
    put_char(0x0C);

    /* ã */
    put_char(0x0D);
    put_char(0x12);
    put_char(0x00);
    put_char(0x0E);
    put_char(0x01);
    put_char(0x0F);
    put_char(0x11);
    put_char(0x0F);

    /* á */
    put_char(0x02);
    put_char(0x04);
    put_char(0x00);
    put_char(0x0E);
    put_char(0x01);
    put_char(0x0F);
    put_char(0x11);
    put_char(0x0F);

    /* é */
    put_char(0x02);
    put_char(0x04);
    put_char(0x00);
    put_char(0x0E);
    put_char(0x11);
    put_char(0x1F);
    put_char(0x10);
    put_char(0x0E);

    /* ê */
    put_char(0x04);
    put_char(0x0A);
    put_char(0x00);
    put_char(0x0E);
    put_char(0x11);
    put_char(0x1F);
    put_char(0x10);
    put_char(0x0E);

    /* ó */
    put_char(0x00);
    put_char(0x02);
    put_char(0x04);
    put_char(0x00);
    put_char(0x0E);
    put_char(0x11);
    put_char(0x11);
    put_char(0x0E);

    /* go home */
    put_cmd(0x02);
#endif
    int res = i2c_master_send(client, "\x00\x38", 2);
    printk("master_send returned %d\n", res);
    mdelay(10);
    res = i2c_master_send(client, "\x00\x39", 2);
    printk("master_send returned %d\n", res);
    mdelay(10);
    res = i2c_master_send(client, "\x00\x14\x78\x5e\x6d\x0c\x01\x06", 8);
    printk("master_send returned %d\n", res);
    mdelay(10);
	return 0;
}

static int lcd_i2c_remove(struct i2c_client* client)
{
    unregister_chrdev(major, DEVICE_NAME);
    lcd_client = NULL;
	return 0;
}

static int lcd_i2c_suspend(struct i2c_client* client, pm_message_t mesg)
{
    /* turn off display */
#if 0
    put_cmd(0x08);
#endif
	return 0;
}

static int lcd_i2c_resume(struct i2c_client* client)
{
    /* turn on display, cursor */
#if 0
    put_cmd(0x0E);
#endif
	return 0;
}

static const struct i2c_device_id lcd_id[] = {
    { "lcdpug", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, lcd_id);

static struct i2c_driver lcd_i2c_driver = {
	.probe		= lcd_i2c_probe,
	.remove		= lcd_i2c_remove,
	.suspend	= lcd_i2c_suspend,
	.resume		= lcd_i2c_resume,
    .id_table   = lcd_id,
	.driver		= {
		.name	= "lcdpug",
	},
};

static struct class* lcdpug_class;
static struct device* lcdpug_device;

static int __init lcd_i2c_drv_init(void)
{
    memset(second_line, 0x20, sizeof(second_line));
    major = register_chrdev(0, DEVICE_NAME, &lcd_fops);
    printk("LCD I2C ST7036 registered major %d\n", major);

    lcdpug_class = class_create(THIS_MODULE, "lcdpug");
    lcdpug_device = device_create(lcdpug_class, NULL, MKDEV(major, 0), NULL, "lcdpug");

    INIT_LIST_HEAD( &char_map.list);
    device_create_file(lcdpug_device, &dev_attr_cmap_add);
    device_create_file(lcdpug_device, &dev_attr_cmap_del);
    device_create_file(lcdpug_device, &dev_attr_cmap_show);

	int res = i2c_add_driver(&lcd_i2c_driver);
    printk("LCD I2C add driver returned %x\n", res);
    return 0;
}

static void __exit lcd_i2c_drv_exit(void)
{
	i2c_del_driver(&lcd_i2c_driver);
    device_destroy(lcdpug_class, MKDEV(major, 0));
}

module_init(lcd_i2c_drv_init);
module_exit(lcd_i2c_drv_exit);
