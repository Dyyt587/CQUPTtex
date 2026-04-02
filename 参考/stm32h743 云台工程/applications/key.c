#include "key.h"
#include "ulog.h"
#include <board.h>
#include <rtthread.h>
#include "flexible_button.h"


//#define KEY1_PIN    GET_PIN(A, 4)
//#define KEY2_PIN    GET_PIN(A, 5)
//#define KEY3_PIN    GET_PIN(C, 4)
//#define KEY4_PIN    GET_PIN(C, 5)




#ifndef KEY1_PIN
#define KEY1_PIN GET_PIN(A, 4)
#endif

#ifndef KEY2_PIN
#define KEY2_PIN GET_PIN(A, 5)
#endif

#ifndef KEY3_PIN
#define KEY3_PIN GET_PIN(C, 4)
#endif

#ifndef KEY4_PIN
#define KEY4_PIN GET_PIN(C, 5)
#endif

#define ENUM_TO_STR(e) (#e)

typedef enum
{
    USER_BUTTON_1 = 0,
    USER_BUTTON_2,
    USER_BUTTON_3,
    USER_BUTTON_4,
    USER_BUTTON_MAX
} user_button_t;

static char *enum_event_string[] = {
    ENUM_TO_STR(FLEX_BTN_PRESS_DOWN),
    ENUM_TO_STR(FLEX_BTN_PRESS_CLICK),
    ENUM_TO_STR(FLEX_BTN_PRESS_DOUBLE_CLICK),
    ENUM_TO_STR(FLEX_BTN_PRESS_REPEAT_CLICK),
    ENUM_TO_STR(FLEX_BTN_PRESS_SHORT_START),
    ENUM_TO_STR(FLEX_BTN_PRESS_SHORT_UP),
    ENUM_TO_STR(FLEX_BTN_PRESS_LONG_START),
    ENUM_TO_STR(FLEX_BTN_PRESS_LONG_UP),
    ENUM_TO_STR(FLEX_BTN_PRESS_LONG_HOLD),
    ENUM_TO_STR(FLEX_BTN_PRESS_LONG_HOLD_UP),
    ENUM_TO_STR(FLEX_BTN_PRESS_MAX),
    ENUM_TO_STR(FLEX_BTN_PRESS_NONE),
};

static char *enum_btn_id_string[] = {
    ENUM_TO_STR(USER_BUTTON_1),
    ENUM_TO_STR(USER_BUTTON_2),
    ENUM_TO_STR(USER_BUTTON_3),
    ENUM_TO_STR(USER_BUTTON_4),
    ENUM_TO_STR(USER_BUTTON_MAX),
};

static flex_button_t user_button[USER_BUTTON_MAX];

static uint8_t common_btn_read(void *arg)
{
    uint8_t value = 0;

    flex_button_t *btn = (flex_button_t *)arg;

    switch (btn->id)
    {
    case USER_BUTTON_1:
        value = rt_pin_read(KEY1_PIN);
        break;
    case USER_BUTTON_2:
        value = rt_pin_read(KEY2_PIN);
        break;
    case USER_BUTTON_3:
        value = rt_pin_read(KEY3_PIN);
        break;
    case USER_BUTTON_4:
        value = rt_pin_read(KEY4_PIN);
        break;
    default:
        RT_ASSERT(0);
    }
    return value;
}

static void common_btn_evt_cb(void *arg)
{
    flex_button_t *btn = (flex_button_t *)arg;

//    LOG_D("id: [%d - %s]  event: [%d - %30s]  repeat: %d\n", 
//        btn->id, enum_btn_id_string[btn->id],
//        btn->event, enum_event_string[btn->event],
//        btn->click_cnt);

//    if ((flex_button_event_read(&user_button[USER_BUTTON_1]) == FLEX_BTN_PRESS_CLICK) &&\
//        (flex_button_event_read(&user_button[USER_BUTTON_2]) == FLEX_BTN_PRESS_CLICK))
//    {
//        LOG_D("[combination]: button 0 and button 1\n");
//    }
	LOG_D("id:%d,event:%d,btn_cnt:%d",btn->id,btn->event,btn->click_cnt);
}


static void user_button_init(void)
{
    int i;
    
    rt_memset(&user_button[0], 0x0, sizeof(user_button));

    rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT_PULLUP); /* set KEY pin mode to input */
    rt_pin_mode(KEY2_PIN, PIN_MODE_INPUT_PULLUP); /* set KEY pin mode to input */
    rt_pin_mode(KEY3_PIN, PIN_MODE_INPUT_PULLUP); /* set KEY pin mode to input */
    rt_pin_mode(KEY4_PIN, PIN_MODE_INPUT_PULLUP); /* set KEY pin mode to input */

    for (i = 0; i < USER_BUTTON_MAX; i ++)
    {
        user_button[i].id = i;
        user_button[i].usr_button_read = common_btn_read;
        user_button[i].cb = common_btn_evt_cb;
        user_button[i].pressed_logic_level = 0;
        user_button[i].short_press_start_tick = FLEX_MS_TO_SCAN_CNT(1500);
        user_button[i].long_press_start_tick = FLEX_MS_TO_SCAN_CNT(3000);
        user_button[i].long_hold_start_tick = FLEX_MS_TO_SCAN_CNT(4500);

//        if (i == USER_BUTTON_3)
//        {
//            user_button[USER_BUTTON_3].pressed_logic_level = 1;
//        }

        flex_button_register(&user_button[i]);
    }
}

//static void key_thread_entry(void *parameter)
//{
//	  rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT);
//    rt_pin_mode(KEY2_PIN, PIN_MODE_INPUT);
//    rt_pin_mode(KEY3_PIN, PIN_MODE_INPUT);
//		rt_pin_mode(KEY4_PIN, PIN_MODE_INPUT);
//    while (1)
//    {
//			if(rt_pin_read(KEY1_PIN) == PIN_LOW)
//			{
//				rt_thread_mdelay(30);//延时消抖
//				if(rt_pin_read(KEY1_PIN) == PIN_LOW)
//				{
//					KEY_FLAG=1;
//					LOG_D("KEY1 DOWN");
//					//break;
//				}
//			}
//			else if(rt_pin_read(KEY2_PIN) == PIN_LOW)
//			{
//				rt_thread_mdelay(30);//延时消抖
//				if(rt_pin_read(KEY2_PIN) == PIN_LOW)
//				{
//					KEY_FLAG=2;
//					LOG_D("KEY2 DOWN");
//					//break;
//				}
//			}
//			else if(rt_pin_read(KEY3_PIN) == PIN_LOW)
//			{
//				rt_thread_mdelay(30);//延时消抖
//				if(rt_pin_read(KEY3_PIN) == PIN_LOW)
//				{
//					KEY_FLAG=3;
//					LOG_D("KEY3 DOWN");
//					//break;
//				}
//			}
//			else if(rt_pin_read(KEY4_PIN) == PIN_LOW)
//			{
//				rt_thread_mdelay(30);//延时消抖
//				if(rt_pin_read(KEY4_PIN) == PIN_LOW)
//				{
//					KEY_FLAG=4;
//					LOG_D("KEY4 DOWN");
//					//break;
//				}
//			}
//			rt_thread_mdelay(20);//延时消抖
//    }
//}

//按键线程
static void key_thread_entry(void *arg)
{
    while(1)
    {
        flex_button_scan();
        rt_thread_mdelay(20); // 20 ms
    }
}

int key_init(void)
{
		rt_err_t ret = RT_EOK;
		user_button_init();//按键初始化
    /* 创建key 线程 */
    rt_thread_t thread = rt_thread_create("key", key_thread_entry, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }

    return ret;
}
INIT_APP_EXPORT(key_init);