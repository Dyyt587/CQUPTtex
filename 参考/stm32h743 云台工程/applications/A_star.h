#ifndef A_STAR_H
#define A_STAR_H
#ifdef __cpluscplus
extern "C" {
#endif
#include <rtthread.h>

#define LENGTH 11
#define WIDTH 9
#define MAX_NODES (WIDTH * LENGTH)
#define DIR_NUM 4
#define PI (3.14159265359f)
// 结点的结构体
typedef struct POINT {
  uint8_t pos[2];       // 坐标
  uint8_t dir_index;    // 方向
  struct POINT *parent; // 父节点
  int F, G, H;          // F，G，H的值
} point;
// 最小堆实现
typedef struct {
  int *indices; // 节点池索引数组
  int size;
  int capacity;
} MinHeap;

extern const float car_arc[DIR_NUM];

extern uint8_t point_num;

// 当车到达目标点后，若进行了拿取操作，记得更新地图，将对应点置1
extern uint8_t map[WIDTH][LENGTH];

// 当车到达目标点后，一定要在完成所有操作后，将该值最后置0
extern uint8_t find_finish;
extern uint8_t state_num;
extern float state_y_z[MAX_NODES][2];

// 当车到达目标点后，一定要在完成所有操作后，返回目标点，然后给出车现在的方向
extern uint8_t start_dir_index;



// A*算法，返回值为目标节点，通过回溯父节点得到路径
point *A_star(point *start_point, point *target_point);
// 路径寻找，传入起点坐标，起点方向，终点坐标，最终路径数组
// 返回值为步数
uint8_t Path_find(uint8_t start_x, uint8_t start_y, uint8_t start_dir,
                  uint8_t target_x, uint8_t target_y,
                  uint8_t final_road[WIDTH * LENGTH][3]);

#ifdef __cpluscplus
}
#endif
#endif