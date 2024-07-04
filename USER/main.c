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
//�������ȼ�
#define START_TASK_PRIO 1
//�����ջ��С
#define START_STK_SIZE 128
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define TASK1_TASK_PRIO 2
//�����ջ��С
#define TASK1_STK_SIZE 50
//������
TaskHandle_t Task1Task_Handler;
//������
void task1_task(void *pvParameters);

//�������ȼ�
#define TASK2_TASK_PRIO 3
//�����ջ��С
#define TASK2_STK_SIZE 50
//������
TaskHandle_t Task2Task_Handler;
//������
void task2_task(void *pvParameters);

#define FLASH_SAVE_ADDR 0X080024B0 //����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)

char Data_buff[128]; //���ݻ�����
char Data_len[5];    //���ݳ��Ȼ�����
char Data_id[5];     //���ݷ����ߵ�id������
char buf[20];        //��������
//�������ݻ���
int i;
uint32_t aun_ir_buffer[500];  // ����LED���������ݻ�����
int32_t n_ir_buffer_length;   // ���⴫�������ݳ���
uint32_t aun_red_buffer[500]; // ��ɫLED���������ݻ�����
int32_t n_sp02;
int8_t ch_spo2_valid; // SP02������Ч��ָʾ��
int32_t n_heart_rate; // ����ֵ
int8_t ch_hr_valid;   // ���ʼ�����Ч��ָʾ��
uint8_t uch_dummy;
uint32_t un_min, un_max, un_prev_data;
int32_t n_brightness;
float f_temp;
u8 temp_num = 0;
u8 temp[6];
u8 str[100];
u8 dis_hr = 0, dis_spo2 = 0;
#define MAX_BRIGHTNESS 255
//������
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
    un_min = 0x3FFFF;         // ��ʼ����СֵΪ������ֵ
    un_max = 0;               // ��ʼ�����ֵΪ��С����ֵ
    n_ir_buffer_length = 500; // ���û���������Ϊ500���洢5������������������Ϊ100sps��

    // ��ȡǰ500����������ȷ���źŷ�Χ
    for (i = 0; i < n_ir_buffer_length; i++)
    {
        while (MAX30102_INT == 1)
            ; // �ȴ��ж����Ŷ���

        max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp);                                                        // ��MAX30102��FIFO��ȡ����
        aun_red_buffer[i] = (long)((long)((long)temp[0] & 0x03) << 16) | (long)temp[1] << 8 | (long)temp[2]; // ������ݵõ�ʵ�ʵĺ��ֵ
        aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03) << 16) | (long)temp[4] << 8 | (long)temp[5];  // ������ݵõ�ʵ�ʵ�IRֵ

        if (un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i]; // �����ź���Сֵ
        if (un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i]; // �����ź����ֵ
    }
    un_prev_data = aun_red_buffer[i]; // ����ǰһ�����ݵ�

    // �ڲɼ�ǰ500��������������ʺ�Ѫ�����Ͷ�
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
}

void get_xinlv()
{
    i = 0;
    un_min = 0x3FFFF; // ���³�ʼ����Сֵ
    un_max = 0;       // ���³�ʼ�����ֵ

    // ����ǰ100���������������400�������Ƶ�����
    for (i = 100; i < 500; i++)
    {
        aun_red_buffer[i - 100] = aun_red_buffer[i]; // �ƶ���⻺����
        aun_ir_buffer[i - 100] = aun_ir_buffer[i];   // �ƶ�IR������

        // �����ź���Сֵ�����ֵ
        if (un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i];
        if (un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i];
    }

    // �ɼ�100���������ټ�������
    for (i = 400; i < 500; i++)
    {
        un_prev_data = aun_red_buffer[i - 1]; // ����ǰһ�����ݵ�
        while (MAX30102_INT == 1)
            ;                                                                                                // �ȴ��ж����Ŷ���
        max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp);                                                        // ��MAX30102��FIFO��ȡ����
        aun_red_buffer[i] = (long)((long)((long)temp[0] & 0x03) << 16) | (long)temp[1] << 8 | (long)temp[2]; // ������ݵõ�ʵ�ʵĺ��ֵ
        aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03) << 16) | (long)temp[4] << 8 | (long)temp[5];  // ������ݵõ�ʵ�ʵ�IRֵ

        if (aun_red_buffer[i] > un_prev_data)
        {
            f_temp = aun_red_buffer[i] - un_prev_data; // ���㵱ǰ������ǰһ�������Ĳ�ֵ
            f_temp /= (un_max - un_min);               // ��һ����ֵ
            f_temp *= MAX_BRIGHTNESS;                  // ����������ȵ���
            n_brightness -= (int)f_temp;               // ��������
            if (n_brightness < 0)
                n_brightness = 0; // ȷ�����Ȳ�С��0
        }
        else
        {
            f_temp = un_prev_data - aun_red_buffer[i]; // ���㵱ǰ������ǰһ�������Ĳ�ֵ
            f_temp /= (un_max - un_min);               // ��һ����ֵ
            f_temp *= MAX_BRIGHTNESS;                  // ����������ȵ���
            n_brightness += (int)f_temp;               // ��������
            if (n_brightness > MAX_BRIGHTNESS)
                n_brightness = MAX_BRIGHTNESS; // ȷ�����Ȳ��������ֵ
        }

        if (ch_hr_valid == 1 && n_heart_rate < 120)
        {                          // ���������Ч��С��120
            dis_hr = n_heart_rate; // ��������
            dis_spo2 = n_sp02;     // ����Ѫ�����Ͷ�
        }
        else
        {
            dis_hr = 0;   // ��Чʱ������Ϊ0
            dis_spo2 = 0; // ��ЧʱѪ�����Ͷ���Ϊ0
        }
    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); // �������ʺ�Ѫ�����Ͷ�
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //����ϵͳ�ж����ȼ�����4
    delay_init();                                   //��ʱ������ʼ��
    uart_init(115200);                              //��ʼ������
    LED_Init();
    beeps1(); //��ʼ��LED
    delay_ms(100);
    OLED_Init();  // OLED��ʼ��
    OLED_Clear(); //����
    OLED_ShowCH(0, 0, (u8 *)"Init.....");
    Usart2_DMAInit(115200); //����2 ����DMA �� �����ж�  wifi
    WiFi_ResetIO_Init();    //��ʼ��WiFi�ĸ�λIO
    while (WiFi_InitServer_AP())
    {                  //ѭ������ʼ����������������ֱ���ɹ�
        delay_ms(200); //��ʱ
    }
    WiFi_RxCounter = 0;                       // WiFi������������������
    memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE); //���WiFi���ջ�����
    OLED_Clear();                             //����

    max30102_init(); //����Ѫ����ʼ��
    xinlv_init();    //���ʳ�ʼ���ݲɼ�
    sprintf(buf, "����:--- ");
    OLED_ShowCH(0, 0, (u8 *)buf);
    sprintf(buf, "Ѫ��:--- ");
    OLED_ShowCH(0, 2, (u8 *)buf);

    //������ʼ����
    xTaskCreate((TaskFunction_t)start_task,          //������
                (const char *)"start_task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
    vTaskStartScheduler();                           //�����������
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //�����ٽ���
    //����TASK1����
    xTaskCreate((TaskFunction_t)task1_task,
                (const char *)"task1_task",
                (uint16_t)TASK1_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TASK1_TASK_PRIO,
                (TaskHandle_t *)&Task1Task_Handler);
    //����TASK2����
    xTaskCreate((TaskFunction_t)task2_task,
                (const char *)"task2_task",
                (uint16_t)TASK2_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TASK2_TASK_PRIO,
                (TaskHandle_t *)&Task2Task_Handler);
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

// task1������
void task1_task(void *pvParameters)
{
    u8 task1_num = 0;

    while (1)
    {
        get_xinlv(); //��ȡ����Ѫ��
                     //		sprintf(buf,"%03d %03d",dis_hr,dis_spo2);
                     //		STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)buf,7);//FLASH����

        LEDC = !LEDC;
        WiFi_Get_LinkSta(); //������޿ͻ������ӻ��ǶϿ�

        if (WiFi_Get_Data(Data_buff, Data_len, Data_id))
        { //��������

            if (strstr((char *)Data_buff, "tcpL"))
            {                                                                                //��������֡
                sprintf(buf, "s %d %d ", dis_hr, dis_spo2);                                  //����������ֵ
                WiFi_SendData(Char_to_Hex(Data_id, strlen(Data_id)), buf, strlen(buf), 500); //�������ʺ�Ѫ��
            }
        }
        vTaskDelay(100);
    }
}

// task2������
void task2_task(void *pvParameters)
{

    while (1)
    {

        if (dis_hr != 0 && dis_spo2 != 0) //��0�ж�
        {
            if (dis_hr > 99)
            {
                dis_hr = 99;
            }
            sprintf(buf, "����:%3d ", dis_hr); //**HR:%3d SpO2:%3d
            OLED_ShowCH(0, 0, (u8 *)buf);
            sprintf(buf, "Ѫ��:%3d ", dis_spo2); //**HR:%3d SpO2:%3d
            OLED_ShowCH(0, 2, (u8 *)buf);

            if (dis_spo2 < 80)
            {             //Ѫ������
                beeps1(); //����������
            }
            if (dis_hr<60 | dis_hr> 120)
            {             //���ʹ���  ���ʹ���
                beeps2(); //����������
            }
        }
        else
        {
            sprintf(buf, "����:--- ");
            OLED_ShowCH(0, 0, (u8 *)buf);
            sprintf(buf, "Ѫ��:--- ");
            OLED_ShowCH(0, 2, (u8 *)buf);
        }

        vTaskDelay(1000); //��ʱ1s��Ҳ����1000��ʱ�ӽ���
    }
}
