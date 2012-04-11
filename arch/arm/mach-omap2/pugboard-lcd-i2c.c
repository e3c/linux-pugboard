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

#define LCD_DEBUG(...) 

struct i2c_client* lcd_client = NULL;

static struct char_map_list{
    char* key;
    unsigned long long  map;
    unsigned char alternate;
    struct list_head list;
} char_map;

static struct char_map_list* cgram_map[16] = {0}; 

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
        count += snprintf(buf+count, PAGE_SIZE - count, "%s => %llx,%x\n", tmp->key, tmp->map, tmp->alternate);
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
            list_del(pos);
            kfree(tmp);
            /* clear cgram settings */
            memset(cgram_map, 0, sizeof(cgram_map));
            printk("removed key %s\n", buf);
            return n;
        }

    }
    printk("key %s not found\n", buf);
    return n; 
}

static ssize_t char_map_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf, size_t n)
{
    const char* read_key = buf;
    struct char_map_list *tmp;
    char* read_map = strchr(buf, ',') + 1;
    unsigned char* key;
    struct list_head *q, *pos;
    unsigned int alt;
    int i = 0;
    struct char_map_list* tmp_item = (struct char_map_list*)kmalloc(sizeof(struct char_map_list), GFP_KERNEL);

    key = (char*)kmalloc(read_map - read_key, GFP_KERNEL);
    memset(key, 0, read_map - read_key);

    if((int)read_map == 1)
    {
            printk("lcdpug: wrong key/map format, missing comma\n");
            return -EINVAL;
    }
    LCD_DEBUG("Malloced key %p\n", key);
    tmp_item->map = 0;
    if (sscanf(read_map+i, "%llx,%x", &tmp_item->map, &alt) != 2) {
        if(sscanf(read_map+i, ",%x", &alt) != 1){
            printk("lcdpug: wrong map format, must be 8*HH\n");
            return -EINVAL;
        }
    }
    strncpy(key, read_key, read_map - read_key - 1);

    /* remove old key */
    list_for_each_safe(pos, q, &char_map.list)
    {
        tmp = list_entry(pos, struct char_map_list, list);
        if(strncmp(tmp->key, key, strlen(key)) == 0)
        {
            kfree(tmp->key);
            list_del(pos);
            kfree(tmp);
            /* clear cgram settings */
            memset(cgram_map, 0, sizeof(cgram_map));
            LCD_DEBUG("removed previous key %s\n", key);
            break;
        }
    }

    LCD_DEBUG("Malloced item %p\n", tmp_item);
    tmp_item->key = key;
    tmp_item->alternate = (unsigned char)alt;
    LCD_DEBUG("Adding to list\n");
    list_add(&tmp_item->list, &char_map.list);
    return n;
}

DEVICE_ATTR(cmap_add, 0222, NULL, char_map_store);
DEVICE_ATTR(cmap_del, 0222, NULL, char_map_remove);
DEVICE_ATTR(cmap_show, 0444, char_map_show, NULL);

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

static void translate(char* out_buffer, int* out_idx, char* in_buffer, int* in_idx)
{
    struct char_map_list *tmp;
    struct list_head *pos;
    int i = 0;
    /* let´s short cut and check cgram entries first */
    for(i = 0; i < 16; i++)
    {
        if(cgram_map[i])
        {
            if(strncmp(cgram_map[i]->key,&(in_buffer[*in_idx]), strlen(cgram_map[i]->key))==0)
            {
                LCD_DEBUG("got match on cgram %d\n", i);
                out_buffer[(*out_idx)++] = i; 
                *in_idx += strlen(cgram_map[i]->key);
                return;
            }
        }
    }

    list_for_each(pos, &char_map.list)
    {
        LCD_DEBUG("A\n");
        tmp = list_entry(pos, struct char_map_list, list);
        if(strncmp(tmp->key,&(in_buffer[*in_idx]), strlen(tmp->key))==0)
        {
            LCD_DEBUG("B\n");
            if(!tmp->map)
            {
                LCD_DEBUG("D\n");
                /* direct translation */
                out_buffer[(*out_idx)++] = tmp->alternate;
                *in_idx += strlen(tmp->key);
                return;
            }
            else
            {
                LCD_DEBUG("C\n");
                /* first we find an empty cgram location */
                for(i = 0; i < 16; i++)
                {
                    if(!cgram_map[i])
                    {
                        unsigned char cgram_cmd[18] = {0};
                        int j = 0;
                        int res = 0;

                        cgram_map[i] = tmp;
                        LCD_DEBUG(KERN_EMERG "adding match %x on cgram %d\n", cgram_map[i], i);
                        out_buffer[(*out_idx)++] = i;
                        *in_idx += strlen(tmp->key);

                        cgram_cmd[0] = 0x80;
                        cgram_cmd[1] = 0x40 | i; 
                        LCD_DEBUG("map 80 %x 40\n", cgram_cmd[1]);
                        cgram_cmd[2] = 0x40;
                        for(j = 0; j < 8; j++)
                        {
                            cgram_cmd[j + 3] = ((unsigned char*)&tmp->map)[j] & 0x1F;
                            LCD_DEBUG("map %x\n", cgram_cmd[j +3]);
                        }
                        res = i2c_master_send(lcd_client, cgram_cmd, 11);
                        if(res != 11)
                        {
                            udelay(10);
                            LCD_DEBUG("retrying send\n");
                            res = i2c_master_send(lcd_client, cgram_cmd, 11);
                        }
                        LCD_DEBUG("write cgram res %d\n", res);
                        return;
                    }
                }
                LCD_DEBUG("D\n");
                /* if we get here, all cgram is in use. Use alternate */
                out_buffer[(*out_idx)++] = tmp->alternate;
                *in_idx += strlen(tmp->key);
                return;
            }
        }
    }
    LCD_DEBUG("E\n");
    out_buffer[(*out_idx)++] = in_buffer[(*in_idx)++];
}

static char write_buffer[40];
static int write_buffer_idx = 0;

static ssize_t lcd_write(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    int i = 0;
    int res = 0;
    char buffer[23];

    if(write_buffer_idx == 0)
    {
        /* clear cgram settings */
        memset(cgram_map, 0, sizeof(cgram_map));
        memset(write_buffer, 0x20, 40);
    }

    LCD_DEBUG("write_buffer_idx: %d count: %d write_buffer:[%s]\n", write_buffer_idx, count, write_buffer);
    while((write_buffer_idx < 40) && (i < count))
    {
        if(buf[i] == '\n')
        {
            LCD_DEBUG("got flush! write_buffer_idx: %d i: %d write_buffer: [%s]\n", write_buffer_idx, i, write_buffer);
             break;
        }
        translate(write_buffer, &write_buffer_idx, buf, &i);
        //write_buffer[write_buffer_idx++] = buf[i++];
    }
    if((buf[i] == '\n') || (write_buffer_idx >= 40))
    {
        /* got flush let´s print*/
        buffer[0] = 0x80;
        buffer[1] = 0x80; /* position on 0x0 */
        buffer[2] = 0x40;
        for( i = 0; i < 20; i++)
        {
            buffer[i + 3] = write_buffer[i];
            LCD_DEBUG("ddram %x\n", buffer[i + 3]);
        }
        res = i2c_master_send(lcd_client, buffer, 23);
        if(res != 23)
        {
            udelay(10);
            LCD_DEBUG("retrying send\n");
            res = i2c_master_send(lcd_client, buffer, 23);
        }
        LCD_DEBUG("write res1 %d\n", res);

        buffer[0] = 0x80;
        buffer[1] = 0xc0; /* position on 1x0 */
        buffer[2] = 0x40;
        for( i = 0; i < 20; i++)
        {
            buffer[i + 3] = write_buffer[i+20];
        }
        res = i2c_master_send(lcd_client, buffer, 23);
        if(res != 23)
        {
            udelay(10);
            LCD_DEBUG("retrying send\n");
            res = i2c_master_send(lcd_client, buffer, 23);
        }
        LCD_DEBUG("write res2 %d\n", res);

        write_buffer_idx = 0;
    }
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
    res = i2c_master_send(client, "\x00\x14\x78\x5e\x6d\x0c\x01\x06", 8);
    mdelay(10);
    res = i2c_master_send(client, "\x00\x38", 2);
    lcd_client = client;
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
    int res = 0;

    memset(cgram_map, 0, sizeof(cgram_map));

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
