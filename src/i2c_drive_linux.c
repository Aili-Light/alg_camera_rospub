#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "i2c_drive_linux.h"

int i2c_transformer(uint8_t i2c_bus_index, uint8_t addr, uint8_t *w_buf, uint32_t w_len, uint8_t *r_buf, uint32_t r_len)
{
    if (w_buf == NULL || w_len == 0 || (r_len > 0 && r_buf == NULL))
    {
        printf("Error: Invalid input parameters\n");
        return -1;
    }
    int ret;
    struct i2c_rdwr_ioctl_data work_queue;
    unsigned int idx;
    int fd;
    char i2c_bus_path[NAME_MAX];
    memset(i2c_bus_path,0,sizeof(i2c_bus_path));
    snprintf(i2c_bus_path,12,"/dev/i2c-%d",i2c_bus_index);

    fd = open(i2c_bus_path, O_RDWR);
    if (fd < 0)
    {
        printf("Error on opening the device file \n");
        return -1;
    }

    addr = addr >> 1;
    // printf("path:%s addr:0x%02x\n",i2c_bus_path,addr);

    if (r_len > 0)
    {
        work_queue.nmsgs = 2;
    }
    else
    {
        work_queue.nmsgs = 1;
    }
    work_queue.msgs = (struct i2c_msg *)malloc(work_queue.nmsgs * sizeof(struct i2c_msg));
    if (!work_queue.msgs)
    {
        printf("Memory alloc error\n");
        close(fd);
        return -1;
    }

    ioctl(fd, I2C_TIMEOUT, 2); // 设置超时
    ioctl(fd, I2C_RETRIES, 1); // 设置重试次数

    // 设置写操作
    work_queue.msgs[0].len = w_len; // 写入w_len字节的寄存器地址
    work_queue.msgs[0].addr = addr;
    work_queue.msgs[0].flags = 0; // 写操作
    work_queue.msgs[0].buf = w_buf;
    if (r_len > 0)
    {
        // 设置读操作
        work_queue.msgs[1].len = r_len;      // 读取r_len字节的数据
        work_queue.msgs[1].flags = I2C_M_RD; // 读操作
        work_queue.msgs[1].addr = addr;
        work_queue.msgs[1].buf = r_buf; // 将读取的数据存储在buff中
    }

    ret = ioctl(fd, I2C_RDWR, (unsigned long)&work_queue);
    if (ret < 0)
    {
        printf("Error during I2C_RDWR ioctl with error code: %d\n", ret);
        free(work_queue.msgs);
        close(fd);
        return -1;
    }

    free(work_queue.msgs);
    close(fd);
    return 0;
}

int i2c_write_0808(uint8_t i2c_bus_index, uint8_t addr, uint8_t reg, uint8_t dat)
{
    uint8_t reg_buf = reg;
    return i2c_transformer(i2c_bus_index, addr, &reg_buf, 1, NULL, 0);
}

int i2c_read_0808(uint8_t i2c_bus_index, uint8_t addr, uint8_t reg, uint8_t *dat)
{
    uint8_t reg_buf = reg;
    return i2c_transformer(i2c_bus_index, addr, &reg_buf, 1, dat, 1);
}

int i2c_write_1608(uint8_t i2c_bus_index, uint8_t addr, uint16_t reg, uint8_t dat)
{
    printf("i2c_write_1608: 0x%02x,0x%04x,0x%02x \n", (addr << 1), reg, dat);
    uint8_t reg_buf[3];
    reg_buf[0] = (reg >> 8) & 0xff;
    reg_buf[1] = (reg) & 0xff;
    reg_buf[2] = dat;
    return i2c_transformer(i2c_bus_index, addr, reg_buf, 3, NULL, 0);
}

int i2c_read_1608(uint8_t i2c_bus_index, uint8_t addr, uint16_t reg, uint8_t *dat)
{
    uint8_t reg_buf[2];
    reg_buf[0] = (reg >> 8) & 0xff;
    reg_buf[1] = (reg) & 0xff;
    return i2c_transformer(i2c_bus_index, addr, reg_buf, 2, dat, 1);
}



int i2c_read_08XX(uint8_t i2c_bus_index, uint8_t addr, uint8_t reg, uint8_t *dat, uint32_t r_len)
{
    uint8_t reg_buf = reg;
    return i2c_transformer(i2c_bus_index, addr, &reg_buf, 1, dat, r_len);
}


int i2c_read_16XX(uint8_t i2c_bus_index, uint8_t addr, uint16_t reg, uint8_t *dat, uint32_t r_len)
{
    uint8_t reg_buf[2];
    reg_buf[0] = (reg >> 8) & 0xff;
    reg_buf[1] = (reg) & 0xff;
    return i2c_transformer(i2c_bus_index, addr, reg_buf, 2, dat, r_len);
}

int i2c_xfer_transformer(uint8_t i2c_bus_index, uint8_t addr, uint16_t reg, uint8_t *dat, uint32_t r_len , size_t wri_size)
{
    uint8_t reg_buf[2];
    reg_buf[0] = (reg >> 8) & 0xff;
    reg_buf[1] = (reg) & 0xff;
    return i2c_transformer(i2c_bus_index, addr, reg_buf, wri_size, dat, r_len);
}

int hal_i2c_linux_transformer(struct hal_i2c *bus, uint8_t addr, uint8_t *w_buf, uint32_t w_len, uint8_t *r_buf, uint32_t r_len)
{
    if (w_buf == NULL || w_len == 0 || (r_len > 0 && r_buf == NULL))
    {
        printf("Error: Invalid input parameters\n");
        return -1;
    }
    int ret;
    struct i2c_rdwr_ioctl_data work_queue;
    unsigned int idx;
    int fd;
    if (bus->bus_handle == NULL) {
        // 第一次调用时打开设备
        fd = open(bus->device_path, O_RDWR);
        if (fd < 0)
        {
            printf("Error on opening the device file \n");
            return -1;
        }
        bus->bus_handle = (void*)(intptr_t)fd;
    } else {
        fd = (int)(intptr_t)bus->bus_handle;
    }
    // printf("fd:%d addr:0x%02x w:%d r:%d \n",fd, addr,w_len,r_len);

    if (r_len > 0)
    {
        work_queue.nmsgs = 2;
    }
    else
    {
        work_queue.nmsgs = 1;
    }
    work_queue.msgs = (struct i2c_msg *)malloc(work_queue.nmsgs * sizeof(struct i2c_msg));
    if (!work_queue.msgs)
    {
        printf("Memory alloc error\n");
        close(fd);
        return -1;
    }

    ioctl(fd, I2C_TIMEOUT, 2); // 设置超时
    ioctl(fd, I2C_RETRIES, 1); // 设置重试次数

    // 设置写操作
    work_queue.msgs[0].len = w_len; // 写入w_len字节的寄存器地址
    work_queue.msgs[0].addr = addr;
    work_queue.msgs[0].flags = 0; // 写操作
    work_queue.msgs[0].buf = w_buf;
    if (r_len > 0)
    {
        // 设置读操作
        work_queue.msgs[1].len = r_len;      // 读取r_len字节的数据
        work_queue.msgs[1].flags = I2C_M_RD; // 读操作
        work_queue.msgs[1].addr = addr;
        work_queue.msgs[1].buf = r_buf; // 将读取的数据存储在buff中
    }

    ret = ioctl(fd, I2C_RDWR, (unsigned long)&work_queue);
    if (ret < 0)
    {
        printf("Error during I2C_RDWR ioctl with error code: %d\n", ret);
        free(work_queue.msgs);
        return -1;
    }

    free(work_queue.msgs);
    return 0;
}

int hal_i2c_linux_init(struct hal_i2c *bus, const char *device_path)
{
    int fd = open(device_path, O_RDWR);
    if (fd < 0) 
    {
        printf("hal_i2c_linux_init open %s faild:%d \n",device_path,fd);
        return -1;
    }
    bus->bus_handle = (void*)(intptr_t)fd;
    bus->transfer = hal_i2c_linux_transformer;
    return 0;
}

