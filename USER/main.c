#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "OLED_I2C.h"
#include "wifi.h"
#include "max30102.h"
#include "algorithm.h"
#include "stmflash.h"
//任务优先级
#define START_TASK_PRIO 1
//任务堆栈大小
#define START_STK_SIZE 128
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define TASK1_TASK_PRIO 2
//任务堆栈大小
#define TASK1_STK_SIZE 50
//任务句柄
TaskHandle_t Task1Task_Handler;
//任务函数
void task1_task(void *pvParameters);

//任务优先级
#define TASK2_TASK_PRIO 3
//任务堆栈大小
#define TASK2_STK_SIZE 50
//任务句柄
TaskHandle_t Task2Task_Handler;
//任务函数
void task2_task(void *pvParameters);

#define FLASH_SAVE_ADDR 0X080024B0 //设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)

char Data_buff[128]; //数据缓冲区
char Data_len[5];    //数据长度缓冲区
char Data_id[5];     //数据发送者的id缓冲区
char buf[20];        //缓存数组
//心率数据缓存
int i;
uint32_t aun_ir_buffer[500];  // 红外LED传感器数据缓冲区
int32_t n_ir_buffer_length;   // 红外传感器数据长度
uint32_t aun_red_buffer[500]; // 红色LED传感器数据缓冲区
int32_t n_sp02;
int8_t ch_spo2_valid; // SP02计算有效性指示器
int32_t n_heart_rate; // 心率值
int8_t ch_hr_valid;   // 心率计算有效性指示器
uint8_t uch_dummy;
uint32_t un_min, un_max, un_prev_data;
int32_t n_brightness;
float f_temp;
u8 temp_num = 0;
u8 temp[6];
u8 str[100];
u8 dis_hr = 0, dis_spo2 = 0;
#define MAX_BRIGHTNESS 255
//蜂鸣器
void beeps1()
{
    BEEP = 0;
    delay_ms(100);
    BEEP = 1;
}
void beeps2()
{
    BEEP = 0;
    delay_ms(500);
    BEEP = 1;
}
void xinlv_init()
{
    un_min = 0x3FFFF;         // 初始化最小值为最大可能值
    un_max = 0;               // 初始化最大值为最小可能值
    n_ir_buffer_length = 500; // 设置缓冲区长度为500，存储5秒的样本（假设采样率为100sps）

    // 读取前500个样本，并确定信号范围
    for (i = 0; i < n_ir_buffer_length; i++)
    {
        while (MAX30102_INT == 1)
            ; // 等待中断引脚断言

        max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp);                                                        // 从MAX30102的FIFO读取数据
        aun_red_buffer[i] = (long)((long)((long)temp[0] & 0x03) << 16) | (long)temp[1] << 8 | (long)temp[2]; // 组合数据得到实际的红光值
        aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03) << 16) | (long)temp[4] << 8 | (long)temp[5];  // 组合数据得到实际的IR值

        if (un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i]; // 更新信号最小值
        if (un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i]; // 更新信号最大值
    }
    un_prev_data = aun_red_buffer[i]; // 保存前一个数据点

    // 在采集前500个样本后计算心率和血氧饱和度
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
}

void get_xinlv()
{
    i = 0;
    un_min = 0x3FFFF; // 重新初始化最小值
    un_max = 0;       // 重新初始化最大值

    // 丢弃前100个样本，并将最后400个样本移到顶部
    for (i = 100; i < 500; i++)
    {
        aun_red_buffer[i - 100] = aun_red_buffer[i]; // 移动红光缓冲区
        aun_ir_buffer[i - 100] = aun_ir_buffer[i];   // 移动IR缓冲区

        // 更新信号最小值和最大值
        if (un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i];
        if (un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i];
    }

    // 采集100个样本后再计算心率
    for (i = 400; i < 500; i++)
    {
        un_prev_data = aun_red_buffer[i - 1]; // 保存前一个数据点
        while (MAX30102_INT == 1)
            ;                                                                                                // 等待中断引脚断言
        max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp);                                                        // 从MAX30102的FIFO读取数据
        aun_red_buffer[i] = (long)((long)((long)temp[0] & 0x03) << 16) | (long)temp[1] << 8 | (long)temp[2]; // 组合数据得到实际的红光值
        aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03) << 16) | (long)temp[4] << 8 | (long)temp[5];  // 组合数据得到实际的IR值

        if (aun_red_buffer[i] > un_prev_data)
        {
            f_temp = aun_red_buffer[i] - un_prev_data; // 计算当前样本与前一个样本的差值
            f_temp /= (un_max - un_min);               // 归一化差值
            f_temp *= MAX_BRIGHTNESS;                  // 根据最大亮度调整
            n_brightness -= (int)f_temp;               // 调整亮度
            if (n_brightness < 0)
                n_brightness = 0; // 确保亮度不小于0
        }
        else
        {
            f_temp = un_prev_data - aun_red_buffer[i]; // 计算当前样本与前一个样本的差值
            f_temp /= (un_max - un_min);               // 归一化差值
            f_temp *= MAX_BRIGHTNESS;                  // 根据最大亮度调整
            n_brightness += (int)f_temp;               // 调整亮度
            if (n_brightness > MAX_BRIGHTNESS)
                n_brightness = MAX_BRIGHTNESS; // 确保亮度不大于最大值
        }

        if (ch_hr_valid == 1 && n_heart_rate < 120)
        {                          // 如果心率有效且小于120
            dis_hr = n_heart_rate; // 保存心率
            dis_spo2 = n_sp02;     // 保存血氧饱和度
        }
        else
        {
            dis_hr = 0;   // 无效时心率设为0
            dis_spo2 = 0; // 无效时血氧饱和度设为0
        }
    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); // 计算心率和血氧饱和度
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //设置系统中断优先级分组4
    delay_init();                                   //延时函数初始化
    uart_init(115200);                              //初始化串口
    LED_Init();
    beeps1(); //初始化LED
    delay_ms(100);
    OLED_Init();  // OLED初始化
    OLED_Clear(); //清屏
    OLED_ShowCH(0, 0, (u8 *)"Init.....");
    Usart2_DMAInit(115200); //串口2 开启DMA 和 空闲中断  wifi
    WiFi_ResetIO_Init();    //初始化WiFi的复位IO
    while (WiFi_InitServer_AP())
    {                  //循环，初始化，建立服务器，直到成功
        delay_ms(200); //延时
    }
    WiFi_RxCounter = 0;                       // WiFi接收数据量变量清零
    memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE); //清空WiFi接收缓冲区
    OLED_Clear();                             //清屏

    max30102_init(); //心率血氧初始化
    xinlv_init();    //心率初始数据采集
    sprintf(buf, "心率:--- ");
    OLED_ShowCH(0, 0, (u8 *)buf);
    sprintf(buf, "血氧:--- ");
    OLED_ShowCH(0, 2, (u8 *)buf);

    //创建开始任务
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
    vTaskStartScheduler();                           //开启任务调度
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //进入临界区
    //创建TASK1任务
    xTaskCreate((TaskFunction_t)task1_task,
                (const char *)"task1_task",
                (uint16_t)TASK1_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TASK1_TASK_PRIO,
                (TaskHandle_t *)&Task1Task_Handler);
    //创建TASK2任务
    xTaskCreate((TaskFunction_t)task2_task,
                (const char *)"task2_task",
                (uint16_t)TASK2_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TASK2_TASK_PRIO,
                (TaskHandle_t *)&Task2Task_Handler);
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

// task1任务函数
void task1_task(void *pvParameters)
{
    u8 task1_num = 0;

    while (1)
    {
        get_xinlv(); //获取心率血氧
                     //		sprintf(buf,"%03d %03d",dis_hr,dis_spo2);
                     //		STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)buf,7);//FLASH保存

        LEDC = !LEDC;
        WiFi_Get_LinkSta(); //检测有无客户端连接或是断开

        if (WiFi_Get_Data(Data_buff, Data_len, Data_id))
        { //接收数据

            if (strstr((char *)Data_buff, "tcpL"))
            {                                                                                //收起请求帧
                sprintf(buf, "s %d %d ", dis_hr, dis_spo2);                                  //构建发送数值
                WiFi_SendData(Char_to_Hex(Data_id, strlen(Data_id)), buf, strlen(buf), 500); //发送心率和血氧
            }
        }
        vTaskDelay(100);
    }
}

// task2任务函数
void task2_task(void *pvParameters)
{

    while (1)
    {

        if (dis_hr != 0 && dis_spo2 != 0) //非0判断
        {
            if (dis_hr > 99)
            {
                dis_hr = 99;
            }
            sprintf(buf, "心率:%3d ", dis_hr); //**HR:%3d SpO2:%3d
            OLED_ShowCH(0, 0, (u8 *)buf);
            sprintf(buf, "血氧:%3d ", dis_spo2); //**HR:%3d SpO2:%3d
            OLED_ShowCH(0, 2, (u8 *)buf);

            if (dis_spo2 < 80)
            {             //血氧过低
                beeps1(); //蜂鸣器短鸣
            }
            if (dis_hr<60 | dis_hr> 120)
            {             //心率过低  心率过高
                beeps2(); //蜂鸣器长鸣
            }
        }
        else
        {
            sprintf(buf, "心率:--- ");
            OLED_ShowCH(0, 0, (u8 *)buf);
            sprintf(buf, "血氧:--- ");
            OLED_ShowCH(0, 2, (u8 *)buf);
        }

        vTaskDelay(1000); //延时1s，也就是1000个时钟节拍
    }
}
