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

struct i2c_client* lcd_client = NULL;

static struct char_map_list{
    char* key;
    char* map;
    char alternate;
    struct list_head list;
} char_map;

static int major = 0;
static int dev_open = false;

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
        count += snprintf(buf+count, PAGE_SIZE - count, "%s => %x%x%x%x%x%x%x%x,%x\n", tmp->key, tmp->map[7], tmp->map[6], tmp->map[5], tmp->map[4], tmp->map[3], tmp->map[2], tmp->map[1], tmp->map[0], tmp->alternate);
    }
    return count; 
}

static ssize_t char_map_remove(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t n)
{
    struct char_map_list *tmp;
    struct list_head *q, *pos;

    list_for_each_safe(pos, q, &char_map.list)
    {
        tmp = list_entry(pos, struct char_map_list, list);
        if(strncmp(tmp->key, buf, n) == 0)
        {
            kfree(tmp->key);
            kfree(tmp->map);
            list_del(pos);
            kfree(tmp);
            printk(KERN_EMERG "removed key %s\n", buf);
            return n;
        }

    }
    printk(KERN_EMERG "key %s not found\n", buf);
    return n; 
}

static ssize_t char_map_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t n)
{
    const char* read_key = buf;
    char* read_map = strchr(buf, ',') + 1;
    unsigned char* key = (char*)kmalloc((strlen(read_key) - strlen(read_map))/2, GFP_KERNEL);
    unsigned char* map = (char*)kmalloc(9, GFP_KERNEL);
    struct char_map_list *tmp;
    struct list_head *q, *pos;
    struct char_map_list* tmp_item = (struct char_map_list*)kmalloc(sizeof(struct char_map_list), GFP_KERNEL);
    unsigned long long hex = 0;
    unsigned int alt;
    int i = 0;

    if((int)read_map == 1)
    {
            printk(KERN_EMERG "lcdpug: wrong key/map format, missing comma\n");
            return -EINVAL;
    }
    printk(KERN_EMERG "Malloced map %p\n", map);
    printk(KERN_EMERG "Malloced key %p\n", key);
    if (sscanf(read_map+i, "%llx,%x", &hex,&alt) != 2) {
        if(sscanf(read_map+i, ",%x", &alt) != 1){
            printk(KERN_EMERG "lcdpug: wrong map format, must be 8*HH\n");
            return -EINVAL;
        }
    }
    if(hex)
    {
        printk(KERN_EMERG "read %d %llx\n", i, hex);
        memcpy(map, &hex, 8);
        map[8] = 0;
    }
    else
    {
        printk(KERN_EMERG "no map supplied, using simple translantion rule\n");
        memset(map, 0, 9);
    }

    strncpy(key, read_key, read_map - read_key - 1);

    /* remove old key */
    list_for_each_safe(pos, q, &char_map.list)
    {
        tmp = list_entry(pos, struct char_map_list, list);
        if(strncmp(tmp->key, key, strlen(key)) == 0)
        {
            kfree(tmp->key);
            kfree(tmp->map);
            list_del(pos);
            kfree(tmp);
            printk(KERN_EMERG "removed previous key %s\n", key);
            break;
        }
    }

    printk(KERN_EMERG "Malloced item %p\n", tmp_item);
    tmp_item->key = key;
    tmp_item->map = map;
    tmp_item->alternate = alt;
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
    int res = 0;
    char tmp[80];
    printk("lcd write\n");
    while((i < 39) && (i < count - 1))
    {
#if 0
        int res = translate(&buf[i]);
#ifdef LCD_DEBUG
        printk("translate returned %d\n", res);
#endif
        i += res;
#endif
        tmp[i*2] = 0xc0;
        tmp[(i*2)+1] = buf[i];     
        i++;
    } 
    tmp[i*2] = 0x40;
    tmp[(i*2)+1] = buf[i];
    res = i2c_master_send(lcd_client, tmp, (i*2) + 2);
    printk(KERN_EMERG "write res %d\n", res);
    return count; 
}

static struct file_operations lcd_fops = {
  write: lcd_write,
  open: lcd_open,
  release: lcd_release
};

static int lcd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
    int res = i2c_master_send(client, "\x00\x38", 2);
    mdelay(10);
    res = i2c_master_send(client, "\x00\x39", 2);
    mdelay(10);
    res = i2c_master_send(client, "\x80\x14\x80\x78\x80\x5e\x80\x6d\x80\x0c\x80\x01\x00\x06", 14);
    mdelay(10);
    lcd_client = client;
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
    int res = 0;

    major = register_chrdev(0, DEVICE_NAME, &lcd_fops);
    printk("LCD I2C ST7036 registered major %d\n", major);

    lcdpug_class = class_create(THIS_MODULE, "lcdpug");
    lcdpug_device = device_create(lcdpug_class, NULL, MKDEV(major, 0), NULL, "lcdpug");

    INIT_LIST_HEAD( &char_map.list);
    res = device_create_file(lcdpug_device, &dev_attr_cmap_add);
    res = device_create_file(lcdpug_device, &dev_attr_cmap_del);
    res = device_create_file(lcdpug_device, &dev_attr_cmap_show);

	res = i2c_add_driver(&lcd_i2c_driver);
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
