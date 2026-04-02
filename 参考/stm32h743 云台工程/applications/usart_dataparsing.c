#include "usart_dataparsiring.h"

#include "string.h"
#include "stdio.h"
#include "ulog.h"
#include "laser_gimbal_vision_tracker.h"

extern vision_tracker_t g_vision_tracker;
int is_serach = 0;

// 协议:@data1,data2,data3,....#
Data_Parsiring_Typedef camera_data;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转整形数字 数据范围是 [-32768,32767]
// 参数说明     *str            传入字符串 可带符号
// 返回参数     int32           转换后的数据
// 使用示例     int32 dat = func_str_to_int("-100");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
int32_t func_str_to_int(char *str)
{
    uint8_t sign = 0; // 标记符号 0-正数 1-负数
    int32_t temp = 0; // 临时计算变量
    do
    {
        if (NULL == str)
        {
            break;
        }

        if ('-' == *str) // 如果第一个字符是负号
        {
            sign = 1; // 标记负数
            str++;
        }
        else if ('+' == *str) // 如果第一个字符是正号
        {
            str++;
        }

        while (('0' <= *str) && ('9' >= *str)) // 确定这是个数字
        {
            temp = temp * 10 + ((uint8_t)(*str) - 0x30); // 计算数值
            str++;
        }

        if (sign)
        {
            temp = -temp;
        }
    } while (0);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转整形数字 数据范围是 [0,65535]
// 参数说明     *str            传入字符串 无符号
// 返回参数     uint32          转换后的数据
// 使用示例     uint32 dat = func_str_to_uint("100");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32_t func_str_to_uint(char *str)
{
    uint32_t temp = 0; // 临时计算变量

    do
    {
        if (NULL == str)
        {
            break;
        }

        while (('0' <= *str) && ('9' >= *str)) // 确定这是个数字
        {
            temp = temp * 10 + ((uint8_t)(*str) - 0x30); // 计算数值
            str++;
        }
    } while (0);

    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转浮点数 有效累计精度为小数点后六位
// 参数说明     *str            传入字符串 可带符号
// 返回参数     float           转换后的数据
// 使用示例     float dat = func_str_to_float("-100.2");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float func_str_to_float(char *str)
{
    uint8_t sign = 0;       // 标记符号 0-正数 1-负数
    float temp = 0.0;       // 临时计算变量 整数部分
    float temp_point = 0.0; // 临时计算变量 小数部分
    float point_bit = 1;    // 小数累计除数

    do
    {
        if (NULL == str)
        {
            break;
        }

        if ('-' == *str) // 负数
        {
            sign = 1; // 标记负数
            str++;
        }
        else if ('+' == *str) // 如果第一个字符是正号
        {
            str++;
        }

        // 提取整数部分
        while (('0' <= *str) && ('9' >= *str)) // 确定这是个数字
        {
            temp = temp * 10 + ((uint8_t)(*str) - 0x30); // 将数值提取出来
            str++;
        }
        if ('.' == *str)
        {
            str++;
            while (('0' <= *str) && ('9' >= *str) && point_bit < 1000000.0) // 确认这是个数字 并且精度控制还没到六位
            {
                temp_point = temp_point * 10 + ((uint8_t)(*str) - 0x30); // 提取小数部分数值
                point_bit *= 10;                                         // 计算这部分小数的除数
                str++;
            }
            temp_point /= point_bit; // 计算小数
        }
        temp += temp_point; // 将数值拼合

        if (sign)
        {
            temp = -temp;
        }
    } while (0);
    return temp;
}

// 判断字符串形式的数据的实际类型
// 返回值：无符号整数为0
//         有符号整数为1
//         浮点数为2
uint8_t determine_type(char *text)
{
    uint8_t point_flag = 0;
    uint8_t signed_flag = 0;
    while (*text != '\0')
    {
        if (*text == '-')
        {
            signed_flag = 1;
        }
        if (*text == '.')
        {
            point_flag = 1;
        }
        text++;
    }
    if (point_flag != 0)
    {
        return 2;
    }
    if (signed_flag != 0)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief    数据解析结构体初始化
 * @param    data:数据解析结构体指针
 * @retval   None
 */
void Usart_Dataparsiring_Init(Data_Parsiring_Typedef *data)
{
    data->statue = 0; // 解析状态清零
    data->frame_start = '@';
    data->frame_end = '#';
    data->frame_lenght = 0;

    data->data_end = ','; // 数据分割符
    data->data_lenght = 0;

    data->data_buff.float_data_lenght = 0;
    data->data_buff.uint32_data_lenght = 0;
    data->data_buff.int32_data_lenght = 0;
}

/**
 * @brief    数据解析中断处理函数，
 * @param    data:数据解析结构体指针
 * 					rx_data:接收到的数据
 * @retval   None
 */
void Usart_Dataparsiring_Handle(Data_Parsiring_Typedef *data, uint8_t rx_data)
{
    if (data->statue == 0 && rx_data == data->frame_start)
    {
        memset(data->frame_string, '\0', 100); // 清零接收buff
        data->frame_lenght = 0;
        data->statue = 1;
    }
    else if (data->statue == 1 && rx_data != data->frame_end) // 数据部分
    {
        data->frame_string[data->frame_lenght] = rx_data;
        data->frame_lenght++;
    }
    else if (data->statue == 1 && rx_data == data->frame_end) // 数据接收完成
    {
        // 数据中断解析或者释放信号量进行异步解析
        // 这里直接解析
        Usart_Dataparsiring(data);

        // if (data->data_buff.float_data[2] > 0.5f)
        // {
        //     static int cnt = 0;
        //     if (cnt++ > 2)
        //     {
        //         cnt = 0;
        //         is_serach = 0; // 开启视觉跟踪
        //     }
        //     // 正常巡线
        // }
        // else
        // {
        //     if (g_vision_tracker.enabled == 1)
        //     {
        //                     static int cnt = 0;
        //     if (cnt++ > 200)
        //     {
        //         vision_tracker_enable(&g_vision_tracker, 0); // 关闭视觉跟踪
        //         is_serach = 1;
        //     }

        //     }
        //     // // 给定四个角度寻找
        //     // static int cnt = 0;
        //     // if (cnt++ > 10)
        //     // {
        //     //     cnt = 0;
        //     //     vision_tracker_enable(&g_vision_tracker, 0); // 关闭视觉跟踪，执行固定位置搜索
        //     //     is_serach = 1;
        //     // }
        // }
				vision_tracker_input_target(&g_vision_tracker, camera_data.data_buff.float_data[0], camera_data.data_buff.float_data[1],  camera_data.data_buff.float_data[2]);

        data->statue = 0;
    }
}
/**
 * @brief    实际解析函数，
 * @param    data:数据解析结构体指针
 * @retval   None
 */
void Usart_Dataparsiring(Data_Parsiring_Typedef *data)
{
    uint8_t run_flag = 0;
    data->data_buff.float_data_lenght = 0;
    data->data_buff.int32_data_lenght = 0;
    data->data_buff.uint32_data_lenght = 0;
    const uint8_t *text = data->frame_string;
    while (*text != '\0')
    {
        if (*text != data->data_end) // 数据段
        {
            data->data_string[data->data_lenght] = *text;
            data->data_lenght++;
        }
        else // 数据段结束,进行数据提取
        {

            uint8_t type = determine_type((char *)data->data_string);
            if (type == 0)
            {
                data->data_buff.uint32_data[data->data_buff.uint32_data_lenght] =
                    func_str_to_uint((char *)data->data_string);
                data->data_buff.uint32_data_lenght++;
            }
            else if (type == 1)
            {
                data->data_buff.int32_data[data->data_buff.int32_data_lenght] =
                    func_str_to_int((char *)data->data_string);
                data->data_buff.int32_data_lenght++;
            }
            else if (type == 2)
            {
                data->data_buff.float_data[data->data_buff.float_data_lenght] =
                    func_str_to_float((char *)data->data_string);
                data->data_buff.float_data_lenght++;
            }
            memset(data->data_string, '\0', 10); // 提取完成后，清空buff
            data->data_lenght = 0;
        }
        text++;
    }
}
void Usart_Dataparising_Debug(Data_Parsiring_Typedef *data)
{
    if (data->data_buff.uint32_data_lenght != 0)
    {
        for (uint8_t i = 0; i < data->data_buff.uint32_data_lenght; i++)
        {
            LOG_D("uint32:%d,%d", i, data->data_buff.uint32_data[i]);
        }
        data->data_buff.uint32_data_lenght = 0;
    }
    if (data->data_buff.int32_data_lenght != 0)
    {
        for (uint8_t i = 0; i < data->data_buff.int32_data_lenght; i++)
        {
            LOG_D("int32:%d,%d", i, data->data_buff.int32_data[i]);
        }
        data->data_buff.int32_data_lenght = 0;
    }
    if (data->data_buff.float_data_lenght != 0)
    {
        for (uint8_t i = 0; i < data->data_buff.float_data_lenght; i++)
        {
            LOG_D("float:%d,%f", i, data->data_buff.float_data[i]);
        }
        data->data_buff.float_data_lenght = 0;
    }
}

//// 从提取的到所有字符里面提取出数据，并存进结构体的数据BUFF里面
// void get_data_from_text(const char *text)
//{
//     uint8_t data_flag = 0;
//     // 清空实际数据长度
//     data_frame_struct.data_buff.float_data_lenght = 0;
//     data_frame_struct.data_buff.int32_data_lenght = 0;
//     data_frame_struct.data_buff.uint32_data_lenght = 0;
//     while(*text != '\0')
//     {
//         if(data_flag == 0 && *text == data_frame_struct.data_start)
//         {
//             memset(data_frame_struct.data_text, '\0', 10);
//             data_frame_struct.data_lenght = 0;
//             data_flag = 1;
//         }
//         else if(data_flag == 1 && *text != data_frame_struct.data_end)
//         {
//             data_frame_struct.data_text[data_frame_struct.data_lenght] = *text;
//             data_frame_struct.data_lenght++;
//         }
//         else if(data_flag == 1 && *text == data_frame_struct.data_end)
//         {
//             uint8_t type = determine_type((char *)data_frame_struct.data_text);
//             if(type == 0)
//             {
//                 data_frame_struct.data_buff.uint32_data[data_frame_struct.data_buff.uint32_data_lenght] =
//                 func_str_to_uint((char *)data_frame_struct.data_text);
//                 data_frame_struct.data_buff.uint32_data_lenght++;
//             }
//             else if(type == 1)
//             {
//                 data_frame_struct.data_buff.int32_data[data_frame_struct.data_buff.int32_data_lenght] =
//                 func_str_to_int((char *)data_frame_struct.data_text);
//                 data_frame_struct.data_buff.int32_data_lenght++;
//             }
//             else if(type == 2)
//             {
//                 printf("%s\n", data_frame_struct.data_text);
//                 data_frame_struct.data_buff.float_data[data_frame_struct.data_buff.float_data_lenght] =
//                 func_str_to_float((char *)data_frame_struct.data_text);
//                 data_frame_struct.data_buff.float_data_lenght++;
//             }
//             data_flag = 0;
//         }
//         text++;
//     }
// }

static rt_thread_t thread_usart = RT_NULL;
#define USART_NAME "uart4"
static rt_device_t serial4;        // 串口设备句柄
static struct rt_semaphore rx_sem; /* 用于接收消息的信号量 */
static void thread_usart_entry(void *parameter)
{
    uint8_t rx_data = 0;
    Usart_Dataparsiring_Init(&camera_data);

    while (1)
    {

        // if (is_serach)
        // {
            
        //     static int step = 0;


        //     if(camera_data.data_buff.float_data[2]>0.5f)
        //     {
        //         is_serach = 0; // 关闭视觉跟踪
        //         vision_tracker_enable(&g_vision_tracker, 1);
        //         step = 0;
        //     }

        //     if (step > 140)
        //     {
        //         step = 0;
        //     }
        //     else if (step > 120)
        //     {
        //         laser_gimbal_set_angle(g_vision_tracker.gimbal, 0, 0);
        //     }
        //     else if (step > 80)
        //     {
        //         laser_gimbal_set_angle(g_vision_tracker.gimbal, -90, 0);
        //     }
        //     else if (step > 60)
        //     {
        //         laser_gimbal_set_angle(g_vision_tracker.gimbal, 180, 0);
        //     }
        //     else if (step > 40)
        //     {
        //         laser_gimbal_set_angle(g_vision_tracker.gimbal, 90, 0);
        //     }
        //     else
        //     {
        //         laser_gimbal_set_angle(g_vision_tracker.gimbal, 0, 0);
        //     }
        //     step++;
        // }
        // rt_device_write(serial4,0,"@123,4.56,#",11);
        // Usart_Dataparising_Debug(&camera_data);	//进行数据Debug
				if(camera_data.data_buff.float_data_lenght != 0){
				camera_data.data_buff.float_data_lenght=0;
				}
        rt_thread_mdelay(10);
    }
}
// 串口接收回调函数
static rt_err_t usart_rx_callback(rt_device_t dev, rt_size_t size)
{
    uint8_t rx_data = 0;
    rt_device_read(serial4, 0, &rx_data, 1);
    Usart_Dataparsiring_Handle(&camera_data, rx_data); // 数据解析
    // LOG_D("RX:%c",rx_data);
    return RT_EOK;
}
int usart_init(void)
{

    thread_usart =
        rt_thread_create("thread_usart", thread_usart_entry, RT_NULL, 1024, 10, 1);

    /* 查找串口设备 */
    serial4 = rt_device_find(USART_NAME);
    if (!serial4)
    {
        LOG_D("find %s failed!\n", USART_NAME);
        return RT_ERROR;
    }
    rt_device_open(serial4, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_BLOCKING);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial4, usart_rx_callback);

    if (thread_usart != RT_NULL)
    {
        rt_thread_startup(thread_usart);
    }

    return 0;
}
INIT_APP_EXPORT(usart_init);
