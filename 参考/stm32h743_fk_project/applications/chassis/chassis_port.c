/*
 * @Author: error: error: git config user.name & please set dead value or
 * install git && error: git config user.email & please set dead value or
 * install git & please set dead value or install git
 * @Date: 2024-03-16 21:52:49
 * @LastEditors: Dyyt587 805207319@qq.com
 * @LastEditTime: 2024-03-24 16:34:07
 * @FilePath: \project\applications\chassis\chassis_port.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "chassis_port.h"
#include "chassis_module_mai.h"
// #include "abus_topic.h"
#include "../drv_emm_v5.h"
#include "math.h"

#include "A_star.h"
#include "apid.h"
#include "drv_gray_sensor.h"
#include <rtdbg.h>
#include <rtthread.h>

#define DBG_TAG "Chassis.port"
#define DBG_LVL DBG_LOG

chassis_t chassis_mai;

chassis_speed_t chassis_speed;
chassis_pos_t chassis_pos;

// 定位矫正的PID
extern apid_t pid_gary_x;
extern apid_t pid_gary_y;
extern apid_t pid_gary_z;

// 0.55 正中间 400 90度
void chassis_port_handle(void *parameter) {
  // int chassis_set_speed(chassis_t *chassis, chassis_speed_t *data);
  // int chassis_set_pos(chassis_t *chassis, chassis_pos_t *data);
  // chassis_speed.x_m_s = 10;
  // uint8_t old_start_dir_index = 0;
  // uint8_t point_flag = 0;
  // int state_flag = 0;
  chassis_speed.y_m_s = 0;
  chassis_speed.z_rad_s = 0.001;

  chassis_pos.x_m = 0;
  chassis_pos.y_m = 0;
  chassis_pos.z_rad = 0;
  // chassis_set_speed(&chassis_mai, &chassis_speed);
  chassis_set_pos(&chassis_mai, &chassis_pos);
  rt_thread_mdelay(1500); // 一定要加，保证电机先使能再控制
  // static uint8_t state_start = 0;
  while (1) {

    // if (find_finish) {
    //   if (state_start == 0)
    //     state_start = motor_moveOK();
    //   if (state_flag < state_num && state_start) {

    //     chassis_pos.x_m = 0;
    //     chassis_pos.y_m = state_y_z[state_flag][0];
    //     chassis_pos.z_rad = state_y_z[state_flag][1];

    //     state_flag++;
    //     chassis_set_pos(&chassis_mai, &chassis_pos);
    //     state_start = 0;
    //   } else if (state_flag == state_num && state_start) {

    //     chassis_pos.x_m = 0;
    //     chassis_pos.y_m = 0;
    //     old_start_dir_index = start_dir_index;

    //     start_dir_index = path[point_num - 1][4];
    //     map[path[point_num - 1][2]][path[point_num - 1][3]] = 1;
    //     chassis_pos.z_rad =
    //         car_arc[start_dir_index] - car_arc[old_start_dir_index];
    //     if (chassis_pos.z_rad == 3 * PI / 2.0)
    //       chassis_pos.z_rad = -PI / 2.0;
    //     else if (chassis_pos.z_rad == -3 * PI / 2.0)
    //       chassis_pos.z_rad = PI / 2.0;
    //     // 上面是转向
    //     // 在这里写前进/夹爪
    //     chassis_set_pos(&chassis_mai, &chassis_pos);
    //     // 所有操作执行完成后才能运行下面的代码
    //     state_flag = 0;
    //     find_finish = 0;
    //     // }
    //     state_start = 0;
    //   }
    // }

    //			chassis_set_pos(&chassis_mai, &chassis_pos);
    // chassis_set_speed(&chassis_mai, &chassis_speed);
    // APID_Get_Out(&pid)
    //  chassis_speed.x_m_s=0.1;
    //  chassis_speed.y_m_s=0;
    //  chassis_speed.z_rad_s=0;
    // chassis_set_speed(&chassis_mai, &chassis_speed);
#if defined(CHASSIS_MODULE_MAI) && defined(CHASSIS_MODULE_MAI)
    chassis_handle(&chassis_mai, 0);
#endif
    rt_thread_mdelay(10);
  }
}

// int chassis_sub_callback(abus_topic_t *sub)
//{
//     chassis_ctrl_t ctrl;
//     uint16_t size;
//     size = afifo_out_data(sub->datafifo, (uint8_t*)&ctrl,
//     sizeof(chassis_ctrl_t)); if (size! =sizeof(chassis_ctrl_t))
//     {
//         LOG_E("abus_topic_subscribe  afifo_out_data error\n");
//         return -1;
//     }
//     if (ctrl.type == 0)
//     {
//         LOG_D("speed x:%f y:%f
//         w:%f",ctrl.speed.x_m_s,ctrl.speed.y_m_s,ctrl.speed.z_rad_s);
//         chassis_set_speed(&chassis_mai, &ctrl.speed);
//     }
//     else
//     {
//         LOG_D("pos x:%f y:%f w:%f",ctrl.pos.x_m,ctrl.pos.y_m,ctrl.pos.z_rad);
//         chassis_set_pos(&chassis_mai, &ctrl.pos);
//     }
//     return 0;
// }

int chassis_port_init(void) {
#if defined(CHASSIS_MODULE_MAI) && defined(CHASSIS_MODULE_MAI)
  chassis_init(&chassis_mai, &ops_mai);
#endif

  rt_thread_t tid_chassis = RT_NULL;

  /* 创建线程， 名称是 thread_test， 入口是 thread_entry*/
  tid_chassis = rt_thread_create("chassis_mai", chassis_port_handle, RT_NULL,
                                 4096, 5, 1);
  /* 线程创建成功，则启动线程 */
  if (tid_chassis != RT_NULL) {
    rt_thread_startup(tid_chassis);
  }
  return 0;
}
//INIT_ENV_EXPORT(chassis_port_init);
