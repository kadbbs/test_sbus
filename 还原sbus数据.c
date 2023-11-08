/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-10-26     RT-Thread    first version
 */

/*
平台：stm32f407
系统：rt-thread
实现：串口2输入sbus数据，还原16通道数据存储在CH[16]数组内



*/
 
#include <rtthread.h>

#include <rtdevice.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#define STACK_SIZE 1024



rt_dev_t u2_dev;//uart2 RT=PA3,TX=PA2
//struct serial_configure u2_configs=RT_SERIAL_CONFIG_DEFAULT;


//配置串口信息为8位data,2位停止，偶校验，波特率100000。
struct serial_configure u2_configs={\

        100000, /* 100000bits/s */  \
        DATA_BITS_9,      /* 8 databits */     \
        STOP_BITS_2,      /* 2 stopbit */      \
        PARITY_EVEN,      /* PARITY_EVEN  */     \
        BIT_ORDER_LSB,    /* LSB first sent */ \
        NRZ_NORMAL,       /* Normal mode */    \
        RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
        0

};

ALIGN(RT_ALIGN_SIZE)
static char thread_stack[STACK_SIZE];
static struct rt_thread rt_sbus;

struct rt_semaphore sem;
rt_int16_t CH[16];
/*
 *
 *
 * CH[x]为通道数数据
 *
 *
 *
 * */
void Sbus_Data_Count(uint8_t *buf)
{
    CH[ 0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
    CH[ 1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
    CH[ 2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )  | (int16_t)buf[ 6] << 10 ) & 0x07FF;
    CH[ 3] = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
    CH[ 4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
    CH[ 5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[10] <<  9 ) & 0x07FF;
    CH[ 6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
    CH[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;

    CH[ 8] = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;
    CH[ 9] = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;
    CH[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 )  | (int16_t)buf[17] << 10 ) & 0x07FF;
    CH[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;
    CH[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;
    CH[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 )  | (int16_t)buf[21] <<  9 ) & 0x07FF;
    CH[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6 )) & 0x07FF;
    CH[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3 )) & 0x07FF;
}

rt_err_t rx_callback(rt_device_t dev,rt_size_t size){

    rt_sem_release(&sem);


    return RT_EOK;
}





void read_sbus(void * arg){
    char buff[25];
    char data;
    rt_size_t len;
    while(1){

//        while(len=rt_device_read(u2_dev,0,buff,128)>0){
//
//            rt_sem_take(&sem,RT_WAITING_FOREVER);
//
//        }


        if((len=rt_device_read(u2_dev,0,buff,sizeof(buff)))==25){
            //rt_device_write(u2_dev, 0, &data, 1);
            if(buff[0]==0x0f & buff[24]==0x00){
                Sbus_Data_Count(buff);
            }



        }else{
            rt_sem_take(&sem,RT_WAITING_FOREVER);
        }

    }


}



int main(void)
{



    rt_err_t ret=0;
    u2_dev = rt_device_find("uart2");

    if(u2_dev == RT_NULL){
        LOG_E("u2_dev find is error!!!\n");
        return -EINVAL;
    }
    ret=rt_device_open(u2_dev, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_INT_RX);
    if(ret<0){
        LOG_E("rt_device_open[uart2] failed ...\n");
        return ret;
    }

    rt_device_control(u2_dev,RT_DEVICE_CTRL_CONFIG,&u2_configs);


    rt_device_set_rx_indicate(u2_dev, rx_callback);

    //rt_device_write(u2_dev, 0, "test\r\n", 6);

    rt_sem_init(&sem,"rx_sem",0,RT_IPC_FLAG_FIFO);




    rt_thread_init(&rt_sbus,"get_sbus", read_sbus, RT_NULL, &thread_stack[0],sizeof(thread_stack), 1, 5);

    rt_thread_startup(&rt_sbus);






    return RT_EOK;
}
