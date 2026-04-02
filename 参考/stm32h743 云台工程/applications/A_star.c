#include "A_star.h"
#include "chassis_module_mai.h"
#include "chassis_port.h"
//#include "drv_mech_arm.h"
#include "key.h"
#include "ulog.h"
#include <math.h>
#include <rtdbg.h>
#include <rtthread.h>
#include <stdint.h>
#include <string.h>

uint8_t KEY_FLAG=0;	//按键标志位，临时加在这里

#define STEP (0.4f)

#define FALSE 0
#define TRUE 1

#define INF 0xFFFF

const int step_dir[DIR_NUM][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
//   1
// 2   0
//   3
const float car_arc[DIR_NUM] = {0, PI / 2.0f, PI, -PI / 2.0f};

int arm_finish_flag =0;
int arm_run_flag =0;

// 地图：1表示可行区域，0表示障碍物
uint8_t map[WIDTH][LENGTH] =
{
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0},
  {0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0}, {0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0},
  {0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0}, {0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
// uint8_t map[WIDTH][LENGTH] =
//{
//   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0},
//   {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0}, {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
//   {0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0}, {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
//   {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0}, {0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0},
//   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
// };

point node_pool[MAX_NODES];
int node_count = 0;

MinHeap open_heap = {0};

// 初始化最小堆
void heap_init(MinHeap *heap, int capacity)
{
  heap->indices = rt_malloc(capacity * sizeof(int));
  heap->size = 0;
  heap->capacity = capacity;
}

// 释放堆内存
void heap_free(MinHeap *heap)
{
  if (heap->indices)
    {
      rt_free(heap->indices);
      heap->indices = NULL;
    }
  heap->size = heap->capacity = 0;
}

// 堆上浮操作
void heapify_up(MinHeap *heap, int index, point *nodes)
{
  while (index > 0)
    {
      int parent = (index - 1) / 2;
      if (nodes[heap->indices[index]].F < nodes[heap->indices[parent]].F)
        {
          // 交换索引
          int temp = heap->indices[index];
          heap->indices[index] = heap->indices[parent];
          heap->indices[parent] = temp;
          index = parent;
        }
      else
        {
          break;
        }
    }
}

// 堆下沉操作
void heapify_down(MinHeap *heap, int index, point *nodes)
{
  int left = 2 * index + 1;
  int right = 2 * index + 2;
  int smallest = index;

  if (left < heap->size &&
      nodes[heap->indices[left]].F < nodes[heap->indices[smallest]].F)
    {
      smallest = left;
    }

  if (right < heap->size &&
      nodes[heap->indices[right]].F < nodes[heap->indices[smallest]].F)
    {
      smallest = right;
    }

  if (smallest != index)
    {
      // 交换索引
      int temp = heap->indices[index];
      heap->indices[index] = heap->indices[smallest];
      heap->indices[smallest] = temp;
      heapify_down(heap, smallest, nodes);
    }
}

// 向堆中推入元素
void heap_push(MinHeap *heap, int index, point *nodes)
{
  if (heap->size < heap->capacity)
    {
      heap->indices[heap->size] = index;
      heapify_up(heap, heap->size, nodes);
      heap->size++;
    }
}

// 从堆中弹出最小元素
int heap_pop(MinHeap *heap, point *nodes)
{
  if (heap->size == 0)
    return -1;

  int min_index = heap->indices[0];
  heap->indices[0] = heap->indices[heap->size - 1];
  heap->size--;
  heapify_down(heap, 0, nodes);

  return min_index;
}

// 获取新节点
point *get_new_node()
{
  if (node_count < MAX_NODES)
    {
      return &node_pool[node_count++];
    }
  return NULL;
}

// 重置节点池
void reset_node_pool()
{
  node_count = 0;
}

// 可行区域判断：1表示可行区域
uint8_t isTrueNode(int x, int y)
{
  return (x >= 0 && x < WIDTH && y >= 0 && y < LENGTH && map[x][y] == 1);
}

// H的计算-曼哈顿距离
//void H_cal(point *current_point, point *target_point)
//{
//  current_point->H = ABS(target_point->pos[0] - current_point->pos[0]) +
//                     ABS(target_point->pos[1] - current_point->pos[1]);
//}


//// G的计算-包含移动代价和转向代价
//void G_cal(point *current_point, point *parent_point)
//{
//  int dir_diff = ABS(parent_point->dir_index - current_point->dir_index);
//  int turn_cost = dir_diff % 2;
//  //int turn_cost = MIN(dir_diff, DIR_NUM - dir_diff);  // 最小转向代价
//  current_point->G = parent_point->G + 1 + turn_cost; // 基础移动代价为1
//}

// H的计算-曼哈顿距离
void H_cal(point *current_point, point *target_point) {
    int tempx = 0;
    int tempy = 0;
    tempx = target_point->pos[0] - current_point->pos[0];
    tempy = target_point->pos[1] - current_point->pos[1];
  current_point->H = ABS(tempx)+ABS(tempy);
}

// G的计算-包含移动代价和转向代价
void G_cal(point *current_point, point *parent_point) {
  int dir_diff = parent_point->dir_index - current_point->dir_index;
    dir_diff = ABS(dir_diff);
  int turn_cost = dir_diff % 2;
  // int turn_cost = MIN(dir_diff, DIR_NUM - dir_diff);  // 最小转向代价
  current_point->G = parent_point->G + 1 + turn_cost; // 基础移动代价为1
}

// A*算法实现
point *A_star(point *start_point, point *target_point)
{
  reset_node_pool();
  heap_init(&open_heap, MAX_NODES);
  uint8_t close_pos[WIDTH][LENGTH] = {0}; // 关闭列表标记
  int value_F[WIDTH][LENGTH];             // 存储每个位置的最小F值

  // 初始化F值为极大值
  for (int i = 0; i < WIDTH; i++)
    {
      for (int j = 0; j < LENGTH; j++)
        {
          value_F[i][j] = INF;
        }
    }

  // 创建起点
  point *start = get_new_node();
  if (!start)
    return NULL;
  *start = *start_point;
  start->parent = NULL;
  start->G = 0;
  H_cal(start, target_point);
  start->F = start->G + start->H;

  heap_push(&open_heap, node_count - 1, node_pool);
  value_F[start->pos[0]][start->pos[1]] = start->F;

  while (open_heap.size > 0)
    {
      // 找到F值最小的节点
      int min_index = heap_pop(&open_heap, node_pool);
      point *current = &node_pool[min_index];

      // 检查是否到达目标
      if (current->pos[0] == target_point->pos[0] &&
          current->pos[1] == target_point->pos[1])
        {
          heap_free(&open_heap);
          return current;
        }

      // 标记为已关闭
      close_pos[current->pos[0]][current->pos[1]] = 1;

      // 探索四个方向
      for (int dir_index = 0; dir_index < DIR_NUM; dir_index++)
        {
          int new_x = current->pos[0] + step_dir[dir_index][0];
          int new_y = current->pos[1] + step_dir[dir_index][1];

          // 检查是否有效节点
          if (isTrueNode(new_x, new_y) && !close_pos[new_x][new_y])
            {
              // 计算新节点的G和F
              point neighbor_proto =
              {
                .pos = {new_x, new_y}, .dir_index = dir_index, .parent = current
              };

              G_cal(&neighbor_proto, current);
              H_cal(&neighbor_proto, target_point);
              neighbor_proto.F = neighbor_proto.G + neighbor_proto.H;

              // 检查是否需要更新
              if (neighbor_proto.F < value_F[new_x][new_y])
                {
                  value_F[new_x][new_y] = neighbor_proto.F;

                  // 创建新节点
                  point *neighbor = get_new_node();
                  if (!neighbor)
                    continue;

                  *neighbor = neighbor_proto;

                  // 添加到开放列表
                  heap_push(&open_heap, node_count - 1, node_pool);
                }
            }
        }
    }
  heap_free(&open_heap);
  return NULL; // 未找到路径
}
int back_flag = 1;
uint8_t Astar_state_make(uint8_t step_num,
                         uint8_t final_road[WIDTH * LENGTH][3],
                         float state_y_z[MAX_NODES][2])
{

  float y_m = 0.0;
  float z_rad = 0.0;
  uint8_t state = 0;
  if (step_num > 1)
    {
      for (int i = step_num - 2; i >= 0; i--)
        {
          if (final_road[i][2] == final_road[i + 1][2])
            {
              y_m += STEP;
            }
          else
            {
              state_y_z[state][0] = y_m * back_flag;
              state_y_z[state][1] = 0 * back_flag;
              state++;
              y_m = STEP;
              z_rad = (car_arc[final_road[i][2]] - car_arc[final_road[i + 1][2]]);
              if (ABS(z_rad) == PI)
                {
                  back_flag = -1;
                }
              else
                {
                  if (z_rad == 3 * PI / 2.0)
                    z_rad = -PI / 2.0;
                  else if (z_rad == -3 * PI / 2.0)
                    z_rad = PI / 2.0;
                  state_y_z[state][0] = 0 * back_flag;
                  state_y_z[state][1] = z_rad * back_flag;
                  back_flag = 1;
                  state++;
                }
            }
        }
      state_y_z[state][0] = y_m * back_flag;
      state_y_z[state][1] = 0;
      state++;
      if (back_flag == -1)
        {
          int temp = final_road[0][2];
					if(temp == 1 || temp == 3)
						final_road[0][2] = temp*7%4;
					else{
						temp = temp -2;
						final_road[0][2] = ABS(temp);
					}
						
          back_flag = 1;
        }
      return state;
    }
  return state;
}

uint8_t Path_find(uint8_t start_x, uint8_t start_y, uint8_t start_dir,
                  uint8_t target_x, uint8_t target_y,
                  uint8_t final_road[WIDTH * LENGTH][3])
{
  uint8_t step_num = 0;
  point start_point = {{start_x, start_y}, start_dir, NULL, 0, 0, 0};
  point target_point = {{target_x, target_y}, 0, NULL, 0, 0, 0};
  // 检查起点终点是否相同
  if (start_point.pos[0] == target_point.pos[0] &&
      start_point.pos[1] == target_point.pos[1])
    {
      return 1;
    }
  if (isTrueNode(start_point.pos[0], start_point.pos[1]) &&
      isTrueNode(target_point.pos[0], target_point.pos[1]))
    {

      point *result = A_star(&start_point, &target_point);

      if (result)
        {
          // 回溯路径
          point *current = result;
          while (current != NULL)
            {
              final_road[step_num][0] = current->pos[0];
              final_road[step_num][1] = current->pos[1];
              final_road[step_num][2] = current->dir_index;
              step_num++;
              current = current->parent;
            }
          return step_num;
        }
      return 0;
    }
  return 0;
}
uint8_t find_finish = 0;
uint8_t state_num = 0;
uint8_t point_num = 0;

float state_y_z[MAX_NODES][2] = {0};

uint8_t start_dir_index = 0;
// 目标点设定,终点x,y，地图修改点x,y，最终方向dir，机械臂动作组0:不动；1：抓取；2：放1分区；3：放2分区；4：放3分区类型1；5：放三分区类型2

uint8_t path1[20][6] = {
    {1, 2, 2, 2, 0, 1}, {1, 4, 1, 1, 2, 2}, {3, 4, 3, 3, 3, 1},
    {1, 4, 1, 1, 2, 2}, {3, 3, 3, 2, 3, 1}, {3, 4, 1, 1, 0, 4},
    {3, 2, 3, 1, 3, 1}, {2, 5, 1, 1, 2, 3}, {3, 3, 4, 3, 0, 1},
    {2, 3, 1, 1, 2, 3}, {4, 3, 5, 3, 0, 1}, {5, 4, 1, 1, 1, 4},
    {4, 3, 4, 2, 3, 1}, {6, 3, 1, 1, 0, 3}, {6, 3, 6, 2, 3, 1},
    {1, 4, 1, 1, 2, 2}, {4, 2, 5, 2, 0, 1}, {1, 4, 1, 1, 2, 2},
    {4, 1, 5, 1, 0, 1}, {6, 5, 1, 1, 0, 3}};

uint8_t path2[20][6] = {
    {4, 1, 3, 1, 2, 1}, {1, 2, 1, 1, 1, 3}, {1, 2, 2, 2, 0, 1},
    {1, 4, 1, 1, 2, 2}, {3, 4, 3, 3, 3, 1}, {1, 4, 1, 1, 2, 2},
    {3, 3, 3, 2, 3, 1}, {3, 4, 1, 1, 0, 4}, {3, 3, 4, 3, 0, 1},
    {2, 5, 1, 1, 2, 3}, {4, 3, 5, 3, 0, 1}, {5, 4, 1, 1, 1, 4},
    {4, 3, 4, 2, 3, 1}, {6, 3, 1, 1, 0, 3}, {6, 3, 6, 2, 3, 1},
    {1, 4, 1, 1, 2, 2}, {4, 2, 5, 2, 0, 1}, {1, 4, 1, 1, 2, 2},
    {4, 1, 5, 1, 0, 1}, {6, 5, 1, 1, 0, 3}};

uint8_t path3[20][6] = {
    {6, 1, 5, 1, 2, 1}, {7, 2, 1, 1, 1, 3}, {4, 1, 3, 1, 2, 1},
    {1, 2, 1, 1, 1, 3}, {1, 2, 2, 2, 0, 1}, {1, 4, 1, 1, 2, 2},
    {3, 4, 3, 3, 3, 1}, {1, 4, 1, 1, 2, 2}, {3, 3, 3, 2, 3, 1},
    {3, 4, 1, 1, 0, 4}, {3, 3, 4, 3, 0, 1}, {2, 5, 1, 1, 2, 3},
    {4, 3, 5, 3, 0, 1}, {5, 4, 1, 1, 1, 4}, {5, 3, 5, 2, 3, 1},
    {1, 4, 1, 1, 2, 2}, {6, 3, 6, 2, 3, 1}, {1, 4, 1, 1, 2, 2},
    {3, 2, 4, 2, 0, 1}, {6, 5, 1, 1, 0, 3}};

uint8_t path[20][6] = {0};


uint8_t point_flag = 0;
uint8_t state_flag = 0;
uint8_t state_finish = 1;
void thread_A_star(void *A)
{
		
  // 初始化数据结构
  uint8_t final_road[WIDTH * LENGTH][3] = {0}; // 最终路径

  uint8_t start_pos[3][2] = {{1, 1}, {4, 1}, {7, 1}}; // 三个起点
  uint8_t step_num = 0;                               // 步数

  uint8_t start_x = 0; // 每次寻路的起点x
  uint8_t start_y = 0; // 每次寻路的起点y

  uint8_t old_start_dir_index = 0;
	
	rt_thread_mdelay(2000);
	
	  while (!KEY_FLAG) {
     rt_thread_mdelay(10);
   }
	 //向前走
		chassis_pos.x_m = 0;
		chassis_pos.y_m = 0.25;
		chassis_pos.z_rad = 0;
		chassis_set_pos(&chassis_mai, &chassis_pos);
		send_start = 1;
	 	rt_thread_mdelay(50);
	 while(motor_moveOK()==0)
	 {
		 rt_thread_mdelay(20);
	 }


  switch (KEY_FLAG)
    {
    case 1:
      start_x = start_pos[0][0];
      start_y = start_pos[0][1];
      memcpy(path, path1, 20 * 6);
      break;
    case 2:
      start_x = start_pos[1][0];
      start_y = start_pos[1][1];
      memcpy(path, path2, 20 * 6);
      break;
    case 3:
      start_x = start_pos[2][0];
      start_y = start_pos[2][1];
      memcpy(path, path3, 20 * 6);
      break;
    }
  start_dir_index = 1;
	//rt_thread_mdelay(1000);
  while (1)
    {

      if (!find_finish && point_num < 20)
        {
					if(start_x == path[point_num][0] && start_y == path[point_num][1])
					{
						state_num = 0;
						find_finish = 1;
						point_num++;
					}else{
						LOG_D("final path start: %d,%d", start_x, start_y);
						LOG_D("final path start_dir: %d", start_dir_index);
						step_num = Path_find(start_x, start_y, start_dir_index,
																 path[point_num][0], path[point_num][1], final_road);
						// 重设起点，这里的方向可以在外面更改
						start_x = path[point_num][0];
						start_y = path[point_num][1];
						state_num = Astar_state_make(step_num, final_road, state_y_z);
						start_dir_index = final_road[0][2];
						point_num++;
						if (state_num)
							{
								find_finish = 1;
							}
						// 打印路径
						LOG_D("final path step: %d", step_num);
						LOG_D("final path state: %d", state_num);
						LOG_D("final path final: %d,%d", start_x, start_y);
						LOG_D("final path final_dir: %d", start_dir_index);
						for (int i = 0; i < step_num; i++)
							{
								LOG_D("final path x: %d, y: %d, dir: %d", final_road[i][0],
											final_road[i][1], final_road[i][2]);
							}
					}
          
        }
      else if (find_finish && arm_finish_flag)
        {
          if (state_finish)
            {
              if (state_flag < state_num)
                {
                  chassis_pos.x_m = 0;
                  chassis_pos.y_m = state_y_z[state_flag][0];
                  chassis_pos.z_rad = state_y_z[state_flag][1];
                  state_flag++;
                  chassis_set_pos(&chassis_mai, &chassis_pos);
                  send_start = 1;
                  state_finish = 0;
                }
              else if (state_flag == state_num)
                {

                  chassis_pos.x_m = 0;
                  chassis_pos.y_m = 0;
                  old_start_dir_index = start_dir_index;

                  start_dir_index = path[point_num - 1][4];
                  map[path[point_num - 1][2]][path[point_num - 1][3]] = 1;
                  chassis_pos.z_rad =
                    car_arc[start_dir_index] - car_arc[old_start_dir_index];
									LOG_D("final path start_dir_index: %d, old_start_dir_index :%d", start_dir_index, old_start_dir_index);
                  if (chassis_pos.z_rad == 3 * PI / 2.0)
                    chassis_pos.z_rad = -PI / 2.0;
                  else if (chassis_pos.z_rad == -3 * PI / 2.0)
                    chassis_pos.z_rad = PI / 2.0;
									LOG_D("final path z_rad: %f", chassis_pos.z_rad);
                  // 上面是转向
                  // 在这里写前进/夹爪
                  chassis_set_pos(&chassis_mai, &chassis_pos);
                  send_start = 1;
                  // 所有操作执行完成后才能运行下面的代码
                  state_finish = 0;
                  state_flag++;
                }
              else if (state_flag > state_num)
                {
                  arm_run_flag = path[point_num-1][5];
                  find_finish = 0;
                  arm_finish_flag = 0;
                  state_flag = 0;
                }
            }
          else
            {
              while (!state_finish)
                {
                  state_finish = motor_moveOK();
                  rt_thread_mdelay(20);
                }
            }
        }

      rt_thread_mdelay(10);
    }
}

int A_star_init(void)
{

  rt_thread_t A_star = RT_NULL;

  /* 创建线程， 名称是 A_star 入口是 thread_entry*/
  A_star = rt_thread_create("A_star", thread_A_star, RT_NULL, 4096, 7, 1);
  /* 线程创建成功，则启动线程 */
  if (A_star != RT_NULL)
    {
      rt_thread_startup(A_star);
    }
  return 0;
}
//INIT_ENV_EXPORT(A_star_init);