#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include <vector>
#include "data_struct.h"

using namespace std;

extern int srand_l;
extern int cycle_times;//给定循环次数
extern int range;//迭代起始范围
extern int step;//迭代步长
extern all_car_charact car_char[2];

//extern double busy_turn;//道路拥堵系数
//extern double weight_turn;//拐弯带来的权重
extern int range_now;//起始迭代起始范围
extern double percent_k;//每个plantime里取前percent_k为快车

//自动调参
void chan_parm_auto();

//迭代器
void iteration(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<int>&plan_time, vector<vector<int> >&best_cars_route, int preset_cnt);

//模拟器
int simulator(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross);

//初始化cars
void init_cars(vector<Car>&cars);

//初始化roads
void init_roads(vector<Road>&roads);

//设定出发顺序
void set_start_order(vector<Car>&cars,vector<int>&car_run_order);

//最终更新汽车出发时间
void final_update_time(vector<Car>&cars,vector<int>&plan_time, vector<vector<int> >&best_cars_route);

//检查车状态是否全部到达目的地
bool check_car_status(vector<Car>&cars);

//输出调试信息
void cout_msg(vector<Car>&cars, int stage);

// 判断优先级，穿越路口返回1,继续等待返回0
bool judge_prior(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, int car_id);

//汽车第一次上路
void car_first_run(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, bool prior = false);

//第一阶段模拟器
void simulator_first_stage(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross);

//第二阶段更新wait状态
bool simulator_second_stage(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross);

//确定car的状态
void confirm_car_status(vector<Car>&cars, vector<Road>&roads, vector<vector<int> >&map);

//一轮循环后更新car状态为待改变
void init_cars_status(vector<Car>&cars);

//统计按道路统计map中wait状态的数量
int cnt_wait_car_num(vector<Car>&cars);

//第二阶段，更新waitcar状态
int confirm_prior_status(int car_id, vector<vector<int> >&map, vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross);

//第二阶段每确定一个DONE状态都要对本车道进行一次扫描
void update_channel(int pos, vector<int>&map, vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross);

//更新路口车辆信息
int update_car_status(int car_id, vector<vector<int> >&map, vector<Road>&roads, vector<Car>&cars);

//更新一脸车的方向
void update_car_DLR(Car&car,  vector<Cross>&cross,bool ifCheckStatus=true);

//对车辆按照速度排序
void planTime_sort(vector<Car>&cars, vector<int>&car_speed_order);

//按照同一速度按照car，planTime升序
void speed_sort(vector<Car>&cars,vector<int>&Vec);

void car_speed_sort_old(vector<Car>&cars, vector<int>&car_speed_order);

//实时按照路口统计汽车数量
void synchronize_cnt_car(vector<Car>&cars, vector<Cross>&cross, vector<double>&cross_flow);

//获取上路权限
bool get_right_on_road(Car&car, vector<double>&cross_flow);

//计算系数
void compute_modulus(double &a, double &b,vector<Car>&cars, vector<Cross>&cross);

//汽车出生
void cars_birth_in_road(vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross, vector<int>&car_birth);

//对所有的道路上能上路的车辆排序
void sort_no_start(vector<Road>&roads, vector<Car>&cars);

//调度最高有优先级车辆汽车上路
bool car_run_to_road(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<vector<int> >&map, int car_index);

//调度该路等待上车的车辆，依次上路
void each_dir_run(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, bool prior, vector<int>&car_birth);

//计算第一优先级车辆
void create_car_sequeue(vector<Car>&cars, vector<Road>&roads);

//计算优先与所有的总调度时间
void compute_all_time(long &s_all_schedule_time, long &all_schedule_time,vector<Car>&cars);

//输出地图信息
void cout_road_msg(vector<Road>&roads);

//对车辆按照speed——plantime排序  zhp
void planT_speed_sort(vector<Car>&cars,vector<vector<int> >&ptime_speed_vec, vector<vector<int> >&ptime_speed_preset, vector<vector<int> >&ptime_speed_priority);

//将一个ptime_speed_vec中的慢车单独放到最后一个vector中
void cut_vector(vector<Car>&cars, vector<vector<int> >&vec, double k);

//每个时间片提取车并排序准备上路
void SureStartTime(vector<Car>&cars,vector<Road>&roads, vector<Cross>&cross, vector<int>&car_run_order, int &InCarsPer);

//给定car的id的vector,按照速度排序
void speed_low_sort(vector<Car> &cars, vector<int> &Vec);

//给定car的id的vector,按照在路上的时间排序
void dis_low_sort(vector<Car>&cars,vector<int>&Vec);

//根据拥堵情况释放新车上路
void choose_car(vector<int> CarOnRoad, vector<int>&car_run_order,vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross);

//速度降序排列
void sort_speed_part(vector<Car>&cars,vector<int>&Vec,int N);

//统计道路情况
void cnt_road_busy(vector<int>& CarOnRoad, vector<int>&car_run_order,vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross);

//最好参数
void best_para_run(vector<Car>&cars);

//判断预置车辆能否更改路径
void IfChangeRoute( std::vector<Car>&cars, std::vector<Road> &road, std::vector<Cross> &cross);

//删除车辆在路径后面的路径
void FindAndDelete(int pos, Car &car);

int FindPreRoute(int pos, Car &cars);

//一定时间段后对路上的车路径重新规划一下
void ChangeRounteInTime(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<int> CarOnRoad);

void ChangeRounteInTime(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, Car &carr);

bool IfChangePresetRoute(Car&car, vector<Road>&roads, vector<Cross>&cross);

//改变预置车时间，给优先车辆让路
void IfChangeTime(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross);

//计算全图车位数，并且根据此设定车辆保有量的起始值
int get_roads_length(vector<Road>&roads);

#endif // __SIMULATOR_H__
