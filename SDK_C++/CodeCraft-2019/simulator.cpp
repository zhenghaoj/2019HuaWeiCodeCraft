#include "simulator.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <numeric>
#include "BFS.h"
#include <fstream>
#include <sstream>
#include"preprocess.h"

long long sys_time = 0; //系统时间
int best_range = 0;//最好的地图car保有量
int step = 101;//RANGE迭代步长
int range_now = 3500;//起始迭代起始范围
int range_end = 40000;//迭代结束值

all_car_charact car_char[2];//0表示所有车辆特性，１表示优先车辆的特性
long long best_schedule_time = INF;//最短系统调度时间

int best_s_schedule_time =0;//优先车辆完结时间
int best_o_schedule_time =0;//所有车辆跑完时间

int rest_car_num = 0;//每次调度剩余车辆的总数
int INF_time = 0;//统计连续无解的次数
int answer_time = 0;//统计连续有解的次数
int time_keep = 0;//第一段保持时间,用于保证前n秒稳定
int time_keep2 =0;//第二段时间
int range_keep = 0;//第一段保持数量,用于保证前n秒稳定
int range_keep2 = 0;

int best_time_keep, best_time_keep2, best_range_keep, best_range_keep2;//保存最好的参数


vector<vector<int> >ptime_speed_vec;//按照planTime排序的车 普通
vector<vector<int> >ptime_speed_preset;//按照planTime排序的车 预置
vector<vector<int> >ptime_speed_priority;//按照planTime排序的车 优先

vector<int> rest_p_vec;//此前时间片剩下优先没上路的car
vector<int> rest_c_vec;//此前时间片剩余普通没上路的car
vector<int>speed_value;//速度梯度
vector<vector<double> > busy_road;//(roads.size(), vector<double>(2, 0))

//关于前10%预置车辆
vector<int>cars_id;//车辆id
vector<vector<int> > cars_route;//对应car的路径
int c_t_preset_num;//最大能够改变时间或路线的预置车辆总数

//变步长迭代器，调节参数为全图汽车保有量
void iteration(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<int>&best_start_time, vector<vector<int> >&best_cars_route, int preset_cnt)
{
	//最大能够改变时间或路线的预置车辆总数
	c_t_preset_num = floor(preset_cnt * 0.1);

	//记录系统运行时间
	clock_t start_time,end_time;
	start_time = clock();

	//计算系数
	double a,b;
	compute_modulus(a, b, cars, cross);

	//cross链接路口按照id排序
	sort_road_id(cross);

	//优先车辆和普通车辆的调度时间、总调度时间、系数加权后的时间
	long s_all_schedule_time = 0, o_all_schedule_time = 0;
	long s_schedule_time = 0, o_schedule_time = 0;
	long final_schedule_time = INF, final_all_schedule_time = INF;

	//对车辆按照plantime、speed排序，全部放到二位数组里面。并且将预置车辆，非预置有限车辆，非预置普通车辆非开
	planT_speed_sort(cars, ptime_speed_vec, ptime_speed_preset, ptime_speed_priority);//对car按照planTime——speed排序

	//设定起始保有量
	range_now = 3500;//0.2 * get_roads_length(roads);

	//前10%预置车辆挑选
	//IfChangeRoute(cars, roads, cross);
	IfChangeTime(cars, roads, cross);//改变时间

	while(range_now < range_end)
	{
		//起始保有量
		cout<<range_now <<endl;

		//初始化cars
		init_cars(cars);

		//初始化道路
		init_roads(roads);

		// 模拟系统调度
		o_schedule_time = simulator(cars, roads, cross);

		if(o_schedule_time != INF)
		{

			//无解统计归零
			INF_time = 0;
			answer_time++;
			if(answer_time > 3)
			{
				step += 50;
				answer_time = 0;
			}

			//计算各种时间
			s_schedule_time = car_char[1].end_time - car_char[1].earlist;
			final_schedule_time = round(o_schedule_time + a * s_schedule_time);
			compute_all_time(s_all_schedule_time, o_all_schedule_time,cars);
			final_all_schedule_time = round(o_all_schedule_time + b * s_all_schedule_time);

			cout<<"系数a:"<<a<<"系数b:"<<b<<endl;
			cout<<"specialResult is Result{scheduleTime = "<<s_schedule_time<<", allScheduleTime = "<<s_all_schedule_time<<"}"<<endl;
			cout<<"originResult is Result{scheduleTime = "<<o_schedule_time<<", allScheduleTime = "<<o_all_schedule_time<<"}"<<endl;
			cout<<"CodeCraftJudge end schedule time is "<<final_schedule_time<<" allScheduleTime is "<<final_all_schedule_time<<endl;
		}
		else
		{
			INF_time++;
			answer_time = 0;
		}

		// 保存最优参数
		if(final_schedule_time < best_schedule_time)
		{
			for(int i = 0; i < cars.size(); i++)
			{
				best_start_time[i] = cars[i].start_time;
				if(cars[i].preset && !cars[i].change_route)
					continue;
				best_cars_route[i] = cars[i].route;
			}
			best_range = range_now;//最好的
			best_s_schedule_time = s_schedule_time;
			best_o_schedule_time = o_schedule_time;
			best_schedule_time = final_schedule_time;
		}

		//自动调参
		chan_parm_auto();

		range_now += step;

		//设定迭代时间,最大900秒，超时退出
		end_time = clock();
		if(end_time - start_time > 860000000)
		{
			cout<<"最优解"<<best_schedule_time<<endl;
			return;
		}
	}

}


//判题系统模拟
int simulator(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross)
{
	//初始化时间片
	sys_time=0;

	//库存清零
	rest_p_vec.clear();
	rest_c_vec.clear();

	//减去到达终点车辆，记录剩余车辆
	rest_car_num = cars.size();

	//获取该时间片可以上路的car
	vector<int>car_birth;

	while(rest_car_num > 0)
	{
		sys_time++;

		//确定出生车辆
		SureStartTime(cars, roads, cross, car_birth, range_now);

		//到时间的汽车出生在roads里面
		cars_birth_in_road(roads, cars, cross, car_birth);

		//对所有的道路上能上路的车辆队列排序
		sort_no_start(roads, cars);

		//第一阶段，确定状态
		simulator_first_stage(cars, roads, cross);

		//优先车辆上路
		car_first_run(cars, roads, cross, true);

		//计算第一优先级车辆
		create_car_sequeue(cars, roads);

		//第二阶段，对WAIT状态车辆移动
		if(!simulator_second_stage(cars, roads, cross))
		{
			return INF;
		}

		if(sys_time > 10000)
		{
			cout<<"超时退出"<<endl;
			return INF;
		}

		//所有新车上路
		car_first_run(cars, roads, cross, false);

		//初始化car状态为待改变
		init_cars_status(cars);

		//cout_road_msg(roads);

		if(sys_time > best_schedule_time)
		{
			return INF;
		}
	}
	return sys_time;
}

//计算第一优先级车辆
void create_car_sequeue(vector<Car>&cars, vector<Road>&roads)
{
	for(int i = 0; i < roads.size(); i++)
	{
		roads[i].prior_cars[0] = roads[i].get_prior_car(roads[i].forward_map, cars);
		roads[i].prior_cars[1] = roads[i].get_prior_car(roads[i].back_map, cars);
	}
}
//初始化cars
void init_cars(vector<Car>&cars)
{
	//预置车辆路径返回，改变预置车辆路线时使用
//		for(int i=0;i<cars_id.size();i++)
//		{
//			cars[cars_id[i]].route.clear();
//			cars[cars_id[i]].route = cars_route[i];
//		}

	for(int i = 0; i < cars.size() ; i++)
	{
		cars[i].real_speed = 0;
		cars[i].pos = 0;
		cars[i].channel  = 0;
		cars[i].DLR = DIRICT;
		cars[i].status =NO_START;
		cars[i].next_cross_id = -1;
		cars[i].pre_road_id = -1;
		cars[i].next_road_id = -1;
		cars[i].route_pos = 0;
		cars[i].IfHasReplaned = false;
		cars[i].ReplanTimes = 1;
	}
}

//初始化roads
void init_roads(vector<Road>&roads)
{
	for(int i = 0; i < roads.size() ; i++)
	{
		if(roads[i].isDuplex)
		{
			for(int ch = 0; ch < roads[i].forward_map.size(); ch++)
			{
				for(int p = 0; p < roads[i].forward_map[ch].size(); p++)
				{
					roads[i].forward_map[ch][p] = -1;
					roads[i].back_map[ch][p] = -1;
				}
			}
		}
		else
		{
			for(int ch = 0; ch < roads[i].forward_map.size(); ch++)
			{
				for(int p = 0; p < roads[i].forward_map[ch].size(); p++)
				{
					roads[i].forward_map[ch][p] = -1;
				}
			}
		}
		roads[i].prior_cars[0] = -1;
		roads[i].prior_cars[1] = -1;
		roads[i].garage_from_to.clear();
		roads[i].garage_to_from.clear();
		roads[i].times[0] = 0;
		roads[i].times[1] = 0;
	}
}





//第一阶段先更新状态
void simulator_first_stage(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross)
{
	for(int i = 0; i < roads.size(); i++)
	{
		confirm_car_status(cars, roads, roads[i].forward_map);
		confirm_car_status(cars, roads, roads[i].back_map);
	}
}

//第二阶段
bool simulator_second_stage(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross)
{

	int whole_status = WAIT;//整体状态有一辆车WAIT,就循环
	int pre_wait_car_num = 10000000, last_wait_car_num = 1000000;
	int deak_lock_time = 0;
	while(whole_status == WAIT)
	{
		whole_status = DONE;//每次循环初始化为DONE
		pre_wait_car_num = 0;
		for(int i = 0; i < cross.size(); i++)//先按路口循环
		{
				for(int j = 0; j < 4; j++)//四个道路循环
				{
					if(cross[i].road_id_order[j] == -1)//没路cross[i].road_id_order[j] != -1&&roads[road_index].from == cross[i].id
					{
						continue;
					}
					else
					{
						int road_index = cross[i].road_id_order[j] - roads[0].id;//路索引
						//进入此路口的地图
						vector<vector<int> >&map = (roads[road_index].from == cross[i].id) ? roads[road_index].back_map : roads[road_index].forward_map;
						int car_id = 0;
						int prior_status = DONE;//第一优先级状态
						while(prior_status == DONE)//每条路循环多次，直到出现wait状态为止
						{
							car_id = (roads[road_index].from == cross[i].id) ? roads[road_index].prior_cars[1] : roads[road_index].prior_cars[0];//找到第一优先级车辆Id,没车就是-1
							if(car_id == -1)//地图上没车，或者没有wait状态车辆
								break;
							int pos = cars[car_id - cars[0].id].pos;
							int channel = cars[car_id - cars[0].id].channel;
							int target_road_id = cars[car_id - cars[0].id].next_road_id;
							prior_status = confirm_prior_status(car_id, map, roads, cars, cross);//第一优先级状态，如果变成DONE，继续这条道路地图寻找
							if(prior_status == DONE)
							{
								//刷新本通道
								update_channel(pos, map[channel], cars, roads, cross);

								//刷新第一优先级,与原路的优先级上路
								if(roads[road_index].from == cross[i].id)
								{
									each_dir_run(cars, roads, cross, true, roads[road_index].garage_to_from);
									roads[road_index].prior_cars[1] = roads[road_index].get_prior_car(map,cars);
								}
								else
								{
									each_dir_run(cars, roads, cross, true, roads[road_index].garage_from_to);
									roads[road_index].prior_cars[0] = roads[road_index].get_prior_car(map,cars);
								}
							}
						}
						if(prior_status == WAIT)//存在wait车辆，大循环继续
						{
							whole_status = WAIT;
						}
					}
				}
			//cout<<"等待车辆数量"<<wait_num<<"结束车数量"<<done_num<<endl;
		}
		pre_wait_car_num = cnt_wait_car_num(cars);
		if(pre_wait_car_num == last_wait_car_num && pre_wait_car_num != 0)//一轮循环过后，车数量不变
		{
			deak_lock_time++;
			cout<<"死循环"<<endl;
			cout<<sys_time<<endl;
			return false;
		}
		else
		{
			 last_wait_car_num = pre_wait_car_num;
		}
	}
	return true;
}

//刷新本车道能够在本路中达到终止状态的车
void update_channel(int pos, vector<int>&map, vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross)
{
	int sv1;
	int car_index;
	for(int p = pos + 1; p < map.size(); p++)//通道位置循环，位置越小，表示越靠近路口
	{
		if(map[p] != -1 && cars[map[p] - cars[0].id].status == WAIT)//找到待改变状态车辆
		{
			car_index = map[p] - cars[0].id;
			sv1 = cars[car_index].real_speed;
			for(int q = p - 1; q >= 0 &&  q >= (p - sv1); q--)//先查前面有没有车阻挡
			{
				if(map[q] != -1)//发现有车
				{
					if(cars[map[q] - cars[0].id].status == INIT)//前车状态为INIT
						cout<<"第一阶段先后顺序出错"<<endl;
					if(cars[map[q] - cars[0].id].status == WAIT)//前车状态为WAIT,本车只能WAIT
					{
						cout<<"第二阶段先后顺序出错"<<endl;
						break;
					}
					if(cars[map[q] - cars[0].id].status == DONE)//前车状态为DONE,本车DONE
					{
						cars[car_index].status = DONE;//更新状态
						cars[car_index].pos = q + 1;//汽车位置更新
						map[p] = -1;//地图车原位置清除,先清除原位置，防止q = p - 1，本车不动，被清除
						map[q + 1] = cars[car_index].id;//地图车新位置更新
						break;
					}
				}
			}
			if(cars[car_index].status != WAIT)//表示前车有阻挡，已经确定状态
				continue;

			//前方没有发现车
			if(sv1 <= p)//本路段限速导致不能穿越路口，本车状态DONE
			{
				cars[car_index].status = DONE;//更新状态
				cars[car_index].pos = p - sv1;//汽车位置更新
				map[p - sv1] = cars[car_index].id;//地图车新位置更新
				map[p] = -1;//地图车原位置清除
			}
			else//本路段速度能穿越路口，等待并参与优先级排序
			{
				cars[car_index].status = WAIT;
				return;
			}
		}
	}
}

//汽车第一次上路
void car_first_run(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, bool prior)
{
	for(int i = 0; i < roads.size(); i++)
	{
		each_dir_run(cars, roads, cross, prior, roads[i].garage_to_from);
		each_dir_run(cars, roads, cross, prior, roads[i].garage_from_to);
	}
}

//每条路的两个方向分别上路
void each_dir_run(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, bool prior, vector<int>&car_birth)
{
	for(int r = car_birth.size() - 1; r >= 0; r--)
	{
		if(prior)//要求优先车辆上路
		{
			if(cars[car_birth[r]].priority)
			{
				int car_index = car_birth[r];
				int road_index = cars[car_index].route.front();
				vector<vector<int> > &map = (roads[road_index].from == cars[car_index].from) ? roads[road_index].forward_map : roads[road_index].back_map;
				if(car_run_to_road(cars, roads, cross, map, car_index))
				{
					car_birth.erase(car_birth.begin() + r);//上路成功，弹出，最高优先级车辆,上路失败，后续车辆允许上路
				}
			}
			else
			{
				return;//如果只允许优先车辆上路,碰到非优先车辆，直接return
			}
		}
		else
		{
			int car_index = car_birth[r];
			int road_index = cars[car_index].route.front();
			vector<vector<int> > &map = (roads[road_index].from == cars[car_index].from) ? roads[road_index].forward_map : roads[road_index].back_map;
			if(car_run_to_road(cars, roads, cross, map, car_index))
			{
				car_birth.erase(car_birth.begin() + r);//弹出，最高优先级车辆，上路失败，次路后续车辆不允许上路
			}
		}
	}
	return ;//全部上路成功，或者根本没车等待上路，切换下一条路
}

//汽车上路
bool car_run_to_road(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<vector<int> >&map, int car_index)
{
	int road_index = cars[car_index].route.front();
	int SV2 = min(cars[car_index].max_speed, roads[road_index].speed);
	for(int ch = 0; ch < map.size(); ch++)//循环车道
	{
		int p;
		for(p = roads[road_index].length - 1; p >= (roads[road_index].length - SV2) && p >= 0; p--)
		{
			if(map[ch][p] != -1)//表示前方有车
			{
				if(cars[map[ch][p]].status == DONE)//前车是DONE，可以插入
				{
					if(p == roads[road_index].length - 1 && ch == map.size() - 1)
					{
						return false;//每个车道的末尾都有DONE状态车，上路失败
					}
					else if(p == roads[road_index].length - 1)
					{
						break;//换车道搜索
					}
					else
					{
						//上路实时规划路径
						if(!cars[car_index].preset)
							first_run_bfs_weight(cars[car_index], roads, cross, cars[car_index].route, busy_road);
						else if(cars[car_index].change_route)
						{
							first_run_bfs_weight(cars[car_index], roads, cross, cars[car_index].route, busy_road);
						}
						map[ch][p + 1] = cars[car_index].id;
						cars[car_index].status = DONE;
						cars[car_index].channel = ch;
						cars[car_index].pos = p + 1;
						cars[car_index].real_speed = SV2;

						cars[car_index].pre_road_id = cars[car_index].route[cars[car_index].route_pos];
						cars[car_index].route_pos++;
						cars[car_index].next_cross_id = cars[car_index].route[cars[car_index].route_pos];
						cars[car_index].route_pos++;

						if(cars[car_index].next_cross_id != cars[car_index].to)
						{
							cars[car_index].next_road_id = cars[car_index].route[cars[car_index].route_pos];
						}
						else
						{
							cars[car_index].next_road_id = -1;
						}
						update_car_DLR(cars[car_index], cross);//更新方向
						return true;
					}
				}
				else if(cars[map[ch][p]].status == WAIT)//发现等待车辆，上路失败
				{
					return false;
				}
			}
		}
		//该通道没有车，可以上路
		if(p == (roads[road_index].length - SV2 - 1) || p == -1)
		{
			//上路实时规划路径
			if(!cars[car_index].preset)
				first_run_bfs_weight(cars[car_index], roads, cross, cars[car_index].route, busy_road);
			else if(cars[car_index].change_route)
			{
				first_run_bfs_weight(cars[car_index], roads, cross, cars[car_index].route, busy_road);
			}
			map[ch][p + 1] = cars[car_index].id;

			cars[car_index].status = DONE;
			cars[car_index].channel = ch;
			cars[car_index].pos = p + 1;
			cars[car_index].real_speed = SV2;

			cars[car_index].pre_road_id = cars[car_index].route[cars[car_index].route_pos];
			cars[car_index].route_pos++;
			cars[car_index].next_cross_id = cars[car_index].route[cars[car_index].route_pos];
			cars[car_index].route_pos++;

			if(cars[car_index].next_cross_id != cars[car_index].to)
			{
				cars[car_index].next_road_id = cars[car_index].route[cars[car_index].route_pos];
				//cars[car_index].route_pos++;
			}
			else
			{
				cars[car_index].next_road_id = -1;
			}
			update_car_DLR(cars[car_index], cross);//更新方向
			return true;
		}
	}
}


//第一阶段， 确定car的状态
void confirm_car_status(vector<Car>&cars, vector<Road>&roads, vector<vector<int> >&map)
{
	int s1,sv1;
	int car_index;
	for(int ch = 0; ch < map.size(); ch++)//通道循环
	{
		for(int p = 0; p < map[ch].size(); p++)//通道位置循环，位置越小，表示越靠近路口
		{
			if(map[ch][p] != -1 && cars[map[ch][p] - cars[0].id].status == INIT)//找到待改变状态车辆
			{
				car_index = map[ch][p] - cars[0].id;
				sv1 = cars[car_index].real_speed;
				for(int q = p - 1; q >= 0 &&  q >= (p - sv1); q--)//先查前面有没有车阻挡
				{
					if(map[ch][q] != -1)//发现有车
					{
						if(cars[map[ch][q] - cars[0].id].status == INIT)//前车状态为INIT
							cout<<"第一阶段先后顺序出错"<<endl;
						if(cars[map[ch][q] - cars[0].id].status == WAIT)//前车状态为WAIT,本车只能WAIT
						{
							cars[car_index].status = WAIT;
							break;
						}
						if(cars[map[ch][q] - cars[0].id].status == DONE)//前车状态为DONE,本车DONE
						{
							cars[car_index].status = DONE;//更新状态
							cars[car_index].pos = q + 1;//汽车位置更新
							map[ch][p] = -1;//地图车原位置清除,先清除原位置，防止q = p - 1，本车不动，被清除
							map[ch][q + 1] = cars[car_index].id;//地图车新位置更新
							break;
						}
					}
				}
				if(cars[car_index].status != INIT)//表示前车有阻挡，已经确定状态
					continue;

				//前方没有发现车
				if(sv1 <= p)//本路段限速导致不能穿越路口，本车状态DONE
				{
					cars[car_index].status = DONE;//更新状态
					cars[car_index].pos = p - sv1;//汽车位置更新
					map[ch][p - sv1] = cars[car_index].id;//地图车新位置更新
					map[ch][p] = -1;//地图车原位置清除
				}
				else//本路段速度能穿越路口，分情况
				{
					cars[car_index].status = WAIT;
				}
			}
		}
	}
}

//第二阶段，判断优先级
int confirm_prior_status(int car_id, vector<vector<int> >&map, vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross)
{
	//本路段速度能穿越路口，分情况:1：即将到达终点，2：下一段路限速不能穿越，3：下一短路有车阻挡不能穿越，
	int car_index = car_id - cars[0].id;
	if(judge_prior(cars, roads, cross, car_id))//返回true,能够穿越路口
	{
		//即将达到终点的车辆,按照直行处理
		if(cars[car_index].next_cross_id == cars[car_index].to)
		{
			cars[car_index].status = END;//更新状态为END
			rest_car_num--;
			cars[car_index].end_time = sys_time;
			map[cars[car_index].channel][cars[car_index].pos] = -1;//地图车原位置清除
			car_char[0].end_time = sys_time;//记录所有车辆最后达到时间

			if(cars[car_index].priority)
			{
				car_char[1].end_time = sys_time;//记录优先车辆最后达到时间
			}
			return DONE;
		}

		//非即将达到终点车辆
		int sv2 = min(roads[cars[car_index].next_road_id - roads[0].id].speed, cars[car_index].max_speed);
		int s2 = sv2 - cars[car_index].pos;
		if(s2 > 0)//能穿越路口,分情况：1：下路口有阻挡，2：别的路口有优先级更高的车辆要进入此路
		{
			//先查询下路口状态
			int next_road_index= cars[car_index].next_road_id - roads[0].id;
			vector<vector<int> > &next_road_map = (roads[next_road_index].from == cars[car_index].next_cross_id) ? roads[next_road_index].forward_map : roads[next_road_index].back_map;
			if(next_road_map.size() == 0)
			{
				cout<<"路径规划错误"<<endl;
			}
			else
			{
				for(int ch = 0; ch < next_road_map.size(); ch++)//首先从小通道查起
				{
					int p;
					for(p = next_road_map[ch].size() - 1; p >= next_road_map[ch].size() - s2 && p >= 0; p--)//从每个通道的末尾查起
					{
						if(next_road_map[ch][p] != -1)//分情况，车状态WAIT和DONE,以及是否在末尾
						{
							if(cars[next_road_map[ch][p] - cars[0].id].status == WAIT)//阻挡车辆为WAIT状态
							{
								//cars[car_index].status = WAIT;//继续等待
								return WAIT;
							}
							else//分情况，１车在最后车道末尾，不能穿路口，２在前车道末尾不是最后车道，换车道搜索，不在末尾看优先级
							{
								if(p == next_road_map[ch].size() - 1 && ch == next_road_map.size() - 1)//在最后车道的末尾，不能穿越路口
								{
									cars[car_index].status = DONE;//更新状态
									map[cars[car_index].channel][cars[car_index].pos] = -1;//地图车原位置清除
									cars[car_index].pos = 0;//汽车位置更新
									map[cars[car_index].channel][cars[car_index].pos] = cars[car_index].id;//地图车新位置更新
									return DONE;
								}
								else
								{
									if(p == next_road_map[ch].size() - 1)//是最后位置，但不是最后车道，换车道搜索
									{
										break;
									}
									else
									{
										cars[car_index].status = DONE;//更新状态
										cars[car_index].real_speed = min(cars[car_index].max_speed,roads[cars[car_index].next_road_id - roads[0].id].speed);
										map[cars[car_index].channel][cars[car_index].pos] = -1;//地图车原位置清除
										cars[car_index].pos = p + 1;//汽车位置更新
										cars[car_index].channel = ch;//汽车车道更新
										next_road_map[ch][cars[car_index].pos] = cars[car_index].id;//地图车新位置更新

										cars[car_index].pre_road_id = cars[car_index].route[cars[car_index].route_pos];//更新当前所在路
										cars[car_index].route_pos++;//弹出
										cars[car_index].next_cross_id = cars[car_index].route[cars[car_index].route_pos];//更新下一个路口id
										cars[car_index].route_pos++;//弹出
										if(cars[car_index].route_pos < cars[car_index].route.size())
										{
											cars[car_index].next_road_id = cars[car_index].route[cars[car_index].route_pos];//更新下一条路id
											//cars[car_index].route_pos++;//弹出
										}
										else
										{
											cars[car_index].next_road_id = -1;
											//cars[car_index].route_pos++;//弹出
										}
										update_car_DLR(cars[car_index], cross);//更新方向
										return DONE;
									}
								}
							}
						}
					}

					//一次循环没有找到车，走到下条路s2
					if(p == next_road_map[ch].size() - s2  - 1 ||  p == -1)
					{
						cars[car_index].status = DONE;//更新状态
						cars[car_index].real_speed = min(cars[car_index].max_speed,roads[cars[car_index].next_road_id - roads[0].id].speed);
						map[cars[car_index].channel][cars[car_index].pos] = -1;//地图车原位置清除
						cars[car_index].pos = p + 1;//汽车位置更新
						cars[car_index].channel = ch;//汽车车道更新
						next_road_map[ch][cars[car_index].pos] = cars[car_index].id;//地图车新位置更新

						cars[car_index].pre_road_id = cars[car_index].route[cars[car_index].route_pos];//更新当前所在路
						cars[car_index].route_pos++;//弹出
						cars[car_index].next_cross_id = cars[car_index].route[cars[car_index].route_pos];//更新下一个路口id
						cars[car_index].route_pos++;//弹出
						if(cars[car_index].route_pos < cars[car_index].route.size())
						{
							cars[car_index].next_road_id = cars[car_index].route[cars[car_index].route_pos];//更新下一条路id
							//cars[car_index].route_pos++;//弹出
						}
						else
						{
							cars[car_index].next_road_id = -1;
							//cars[car_index].route_pos++;//弹出
						}

						update_car_DLR(cars[car_index], cross);//更新方向
						return DONE;
					}

				}
			}
		}
		else//否则本车只能走到本路尽头，本车状态为DONE
		{
			cars[car_index].status = DONE;//更新状态
			map[cars[car_index].channel][cars[car_index].pos] = -1;//地图车原位置清除
			cars[car_index].pos = 0;//汽车位置更新
			map[cars[car_index].channel][0] = cars[car_index].id;//地图车新位置更新
			return DONE;
		}
	}
	else
	{
		return WAIT;
	}

}

//判断优先级，false表示竞争失败，true表示竞争获胜
bool judge_prior(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, int car_id)
{
	int car_index = car_id;
	int left_r_index,right_r_index,down_r_index;
	int	left_car_id = -1, right_car_id = -1,down_car_id = -1;//没有车或者没有路为－１
	//否则分上下左右路,首先人为旋转，均假设当前道路在上方，可得出相对左右前后方向
	if(cars[car_index].pre_road_id == cross[cars[car_index].next_cross_id].up_road_id)
	{
		left_r_index = cross[cars[car_index].next_cross_id].left_road_id;
		right_r_index = cross[cars[car_index].next_cross_id].right_road_id;
		down_r_index = cross[cars[car_index].next_cross_id].down_road_id;
	}
	if(cars[car_index].pre_road_id == cross[cars[car_index].next_cross_id].left_road_id)
	{
		left_r_index = cross[cars[car_index].next_cross_id].down_road_id;
		right_r_index = cross[cars[car_index].next_cross_id].up_road_id;
		down_r_index = cross[cars[car_index].next_cross_id].right_road_id;
	}
	if(cars[car_index].pre_road_id == cross[cars[car_index].next_cross_id].down_road_id)
	{
		left_r_index = cross[cars[car_index].next_cross_id].right_road_id;
		right_r_index = cross[cars[car_index].next_cross_id].left_road_id;
		down_r_index = cross[cars[car_index].next_cross_id].up_road_id;
	}
	if(cars[car_index].pre_road_id == cross[cars[car_index].next_cross_id].right_road_id)
	{
		left_r_index = cross[cars[car_index].next_cross_id].up_road_id;
		right_r_index = cross[cars[car_index].next_cross_id].down_road_id;
		down_r_index = cross[cars[car_index].next_cross_id].left_road_id;
	}

	//直行并且是优先车辆，竞争成功
	if(cars[car_index].DLR == DIRICT && cars[car_index].priority)
	{
		return true;
	}

	//直行并且不是优先车辆
	if(cars[car_index].DLR == DIRICT && !cars[car_index].priority)
	{
		if(left_r_index != -1)//左侧有路
		{
			//vector<vector<int> >&map = (roads[left_r_index].from == cars[car_index].next_cross_id) ? roads[left_r_index].back_map : roads[left_r_index].forward_map;
			//记录下最高优先级车辆ｉd
			left_car_id = (roads[left_r_index].from == cars[car_index].next_cross_id) ? roads[left_r_index].prior_cars[1] : roads[left_r_index].prior_cars[0];
		}
		//左侧道路右转并且优先级高,竞争失败
		if(left_car_id != -1 && cars[left_car_id].DLR == RIGHT && cars[left_car_id].priority)
		{
			return false;
		}

		if(right_r_index != -1)//右侧有路
		{
			//vector<vector<int> >&map = (roads[right_r_index].from == cars[car_index].next_cross_id) ? roads[right_r_index].back_map : roads[right_r_index].forward_map;
			//right_car_id = roads[right_r_index].confirm_prior_car(map, cars);
			//记录下最高优先级车辆ｉd
			right_car_id = (roads[right_r_index].from == cars[car_index].next_cross_id) ? roads[right_r_index].prior_cars[1] : roads[right_r_index].prior_cars[0];
		}

		//右侧道路左转并且优先级高，竞争失败
		if(right_car_id != -1 && cars[right_car_id].DLR == LEFT && cars[right_car_id].priority)
		{
			return false;
		}
		//左右侧没车或者没有优先级高的车
		return true;
	}

	//左转并且是优先车辆
	if(cars[car_index].DLR == LEFT && cars[car_index].priority)
	{
		if(left_r_index != -1)//左侧有路
		{
			//vector<vector<int> >&map = (roads[left_r_index].from == cars[car_index].next_cross_id) ? roads[left_r_index].back_map : roads[left_r_index].forward_map;
			//left_car_id = roads[left_r_index].confirm_prior_car(map, cars);//记录下最高优先级车辆ｉd
			left_car_id = (roads[left_r_index].from == cars[car_index].next_cross_id) ? roads[left_r_index].prior_cars[1] : roads[left_r_index].prior_cars[0];
		}
		//左侧道路直行转并且优先级高，竞争失败
		if(left_car_id != -1 && cars[left_car_id].DLR == DIRICT && cars[left_car_id].priority)
		{
			return false;
		}

		return true;
	}

	//左转并且不是优先车辆
	if(cars[car_index].DLR == LEFT && !cars[car_index].priority)
	{
		if(left_r_index != -1)//左侧有路
		{
			//vector<vector<int> >&map = (roads[left_r_index].from == cars[car_index].next_cross_id) ? roads[left_r_index].back_map : roads[left_r_index].forward_map;
			//left_car_id = roads[left_r_index].confirm_prior_car(map, cars);//记录下最高优先级车辆ｉd
			left_car_id = (roads[left_r_index].from == cars[car_index].next_cross_id) ? roads[left_r_index].prior_cars[1] : roads[left_r_index].prior_cars[0];
		}
		//左侧道路直行，竞争失败
		if(left_car_id != -1 && cars[left_car_id].DLR == DIRICT)
		{
			return false;
		}

		if(down_r_index != -1)//下方有路
		{
			//vector<vector<int> >&map = (roads[down_r_index].from == cars[car_index].next_cross_id) ? roads[down_r_index].back_map : roads[down_r_index].forward_map;
			//down_car_id = roads[down_r_index].confirm_prior_car(map, cars);
			//记录下最高优先级车辆ｉd
			down_car_id = (roads[down_r_index].from == cars[car_index].next_cross_id) ? roads[down_r_index].prior_cars[1] : roads[down_r_index].prior_cars[0];
		}
		//下方道路右转并且优先级高，竞争失败
		if(down_car_id != -1 && cars[down_car_id].DLR == RIGHT && cars[down_car_id].priority)
		{
			return false;
		}
		return true;
	}

	//右转并且是优先车辆
	if(cars[car_index].DLR == RIGHT && cars[car_index].priority)
	{

		//右侧有路
		if(right_r_index != -1)
		{
			//vector<vector<int> >&map = (roads[right_r_index].from == cars[car_index].next_cross_id) ? roads[right_r_index].back_map : roads[right_r_index].forward_map;
			//right_car_id = roads[right_r_index].confirm_prior_car(map, cars);
			//记录下最高优先级车辆ｉd
			right_car_id = (roads[right_r_index].from == cars[car_index].next_cross_id) ? roads[right_r_index].prior_cars[1] : roads[right_r_index].prior_cars[0];
		}
		//右侧道路直行并且优先级高，竞争失败
		if(right_car_id != -1 && cars[right_car_id].DLR == DIRICT && cars[right_car_id].priority)
		{
			return false;
		}

		//下方有路
		if(down_r_index != -1)
		{
			//vector<vector<int> >&map = (roads[down_r_index].from == cars[car_index].next_cross_id) ? roads[down_r_index].back_map : roads[down_r_index].forward_map;
			//down_car_id = roads[down_r_index].confirm_prior_car(map, cars);
			//记录下最高优先级车辆ｉd
			down_car_id = (roads[down_r_index].from == cars[car_index].next_cross_id) ? roads[down_r_index].prior_cars[1] : roads[down_r_index].prior_cars[0];
		}
		//下方道路左转并且优先级高，竞争失败
		if(down_car_id != -1 && cars[down_car_id].DLR == LEFT && cars[down_car_id].priority)
		{
			return false;
		}
		return true;
	}

	//右转并且不是优先车辆
	if(cars[car_index].DLR == RIGHT && !cars[car_index].priority)
	{
		//右侧有路
		if(right_r_index != -1)
		{
			//vector<vector<int> >&map = (roads[right_r_index].from == cars[car_index].next_cross_id) ? roads[right_r_index].back_map : roads[right_r_index].forward_map;
			//right_car_id = roads[right_r_index].confirm_prior_car(map, cars);
			//记录下最高优先级车辆ｉd
			right_car_id = (roads[right_r_index].from == cars[car_index].next_cross_id) ? roads[right_r_index].prior_cars[1] : roads[right_r_index].prior_cars[0];
		}
		//右侧道路直行，竞争失败
		if(right_car_id != -1 && cars[right_car_id].DLR == DIRICT)
		{
			return false;
		}

		//下方有路
		if(down_r_index != -1)
		{
			//vector<vector<int> >&map = (roads[down_r_index].from == cars[car_index].next_cross_id) ? roads[down_r_index].back_map : roads[down_r_index].forward_map;
			//down_car_id = roads[down_r_index].confirm_prior_car(map, cars);
			//记录下最高优先级车辆ｉd
			down_car_id = (roads[down_r_index].from == cars[car_index].next_cross_id) ? roads[down_r_index].prior_cars[1] : roads[down_r_index].prior_cars[0];
		}
		//下方道路左转，竞争失败
		if(down_car_id != -1 && cars[down_car_id].DLR == LEFT)
		{
			return false;
		}
		return true;
	}
}

int cnt_wait_car_num(vector<Car>&cars)
{
	int num = 0;
	for(int i = 0; i < cars.size(); i++)
	{
		if(cars[i].status == WAIT)
			num++;
	}
	return num;
}

//一轮循环后，更新状态为待改变
void init_cars_status(vector<Car>&cars)
{
	for(int i = 0; i < cars.size(); i++)
	{
		if(cars[i].status == DONE)
			cars[i].status = INIT;
	}
}

//一轮循环后，更新车方向
void update_car_DLR(Car&car,  vector<Cross>&cross, bool ifCheckStatus)
{
	if(car.status != DONE && ifCheckStatus)
	{
		cout<<"error"<<endl;
		return;
	}
	if(car.next_cross_id == car.to)//未上路车辆，一定是直行.到终点车辆和即将到终点车辆
	{
		car.DLR = DIRICT;
		return;
	}

	//道路位于上方
	if(car.pre_road_id== cross[car.next_cross_id - cross[0].id].up_road_id)
	{
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].left_road_id)
		{
			car.DLR = RIGHT;
			return;
		}
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].down_road_id)
		{
			car.DLR = DIRICT;
			return;
		}
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].right_road_id)
		{
			car.DLR = LEFT;
			return;
		}
	}
	//道路位于右侧
	if(car.pre_road_id == cross[car.next_cross_id - cross[0].id].right_road_id)
	{
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].left_road_id)
		{
			car.DLR = DIRICT;
			return;
		}
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].down_road_id)
		{
			car.DLR = LEFT;
			return;
		}
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].up_road_id)
		{
			car.DLR = RIGHT;
			return;
		}
	}
	//道路位于下方
	if(car.pre_road_id == cross[car.next_cross_id - cross[0].id].down_road_id)
	{
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].left_road_id)
		{
			car.DLR = LEFT;
			return;
		}
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].up_road_id)
		{
			car.DLR = DIRICT;
			return;
		}
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].right_road_id)
		{
			car.DLR = RIGHT;
			return;
		}
	}

	//道路位于左侧
	if(car.pre_road_id == cross[car.next_cross_id - cross[0].id].left_road_id)
	{
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].up_road_id)
		{
			car.DLR = LEFT;
			return;
		}
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].down_road_id)
		{
			car.DLR = RIGHT;
			return;
		}
		if(car.next_road_id == cross[car.next_cross_id - cross[0].id].right_road_id)
		{
			car.DLR = DIRICT;
			return;
		}
	}
}

//最终更新plan_time到cars.start_time车出发时间
void final_update_time(vector<Car>&cars,vector<int>&plan_time, vector<vector<int> >&best_cars_route)
{
	for(int i = 0; i < cars.size() ; i++)//出发时间最大是车数量
	{
		cars[i].start_time = plan_time[i];
		if(cars[i].preset && !cars[i].change_route)
			continue;
		cars[i].route = best_cars_route[i];
	}
}

//输出道路信息，调试用
void cout_road_msg(vector<Road>&roads)
{
	if(sys_time > 500)
		return;
	stringstream sstr;
	sstr<<"time:"<<sys_time<<endl;
	int road_id,car_id;
	for(int i = 0; i < roads.size(); i++)
	{
		road_id = roads[i].id;
		sstr<<"road_id:"<<road_id<<endl;//<<" 容积率"<<roads[i].plot_ratio
		sstr<<"from - to"<<endl;
		int sum = 0;
		for(int ch = 0; ch < roads[i].forward_map.size(); ch++)
		{
			for(int p = roads[i].forward_map[ch].size() - 1; p >= 0 ; p--)
			{
				if(roads[i].forward_map[ch][p] == -1)
				{
					sstr<<"-1"<<" ";
				}
				else
				{
					car_id = roads[i].forward_map[ch][p];
					sum++;
					sstr<<car_id<<" ";
				}

			}
			sstr<<endl;
		}
		if(!roads[i].isDuplex)
			continue;
		sstr<<endl;
		sstr<<"to - from"<<endl;
		sum = 0;
		for(int ch = 0; ch < roads[i].back_map.size(); ch++)
		{
			for(int p = roads[i].back_map[ch].size() - 1; p >= 0 ; p--)
			{
				if(roads[i].back_map[ch][p] == -1)
				{
					sstr<<"-1"<<" ";
				}
				else
				{
					sum++;
					car_id = roads[i].back_map[ch][p];
					sstr<<car_id<<" ";
				}
			}
			sstr<<endl;
		}
		sstr<<endl;
	}
	write_file(0, sstr.str().c_str(), "output.txt");
}


//输出调试信息
void cout_msg(vector<Car>&cars, int stage)
{
	int done_num = 0;
	int wait_num = 0;
	int end_num = 0;
	if(stage == 1)
	{
		for(int i = 0; i < cars.size() ; i++)
		{
			if(cars[i].status == WAIT)
			{
				wait_num++;
//				if(wait_num % 10 == 0)
//					cout<<endl;
			}
		}

		for(int i = 0; i < cars.size() ; i++)
		{
			if(cars[i].status == DONE)
			{
				done_num++;
//				if(done_num % 10 == 0)
//					cout<<endl;
			}
		}
		cout<<"等待车数量"<<wait_num<<"终止车数量"<<done_num<<endl;
	}
	if(stage == 2 && sys_time > 900)
	{
		for(int i = 0; i < cars.size() ; i++)
		{
			if(cars[i].status != END)
			{
				cout<<i<<endl;
			}
		}
	}
	if(stage == 3)
	{
		for(int i = 0; i < cars.size() ; i++)
		{
			//cout<<"车"<<cars[i].id<<" 出发"<<cars[i].start_time<<" 路"<<" "<<cars[i].pre_road_id<<" "<<"速度"<<cars[i].real_speed<<" "<<" 车道"<<cars[i].channel<<" 位置"<<cars[i].pos<<" next路"<<cars[i].next_road_id<<" "<<"方向"<<cars[i].DLR<< " 状态"<<cars[i].status<<endl;
			if(cars[i].status == WAIT)
			{
				cout<<"车id"<<i<<" 路口"<<cars[i].next_cross_id<<endl;
			}
		}
	}
	if(stage == 4)
	{
		for(int i = 0; i < cars.size() ; i++)
		{
			if(cars[i].status == END)
			{
				end_num++;
			}
		}
		//cout<<"时间片:"<<sys_time<<" 到达终点个数:"<<end_num<<endl;
	}
}

//出发时间与出发地分布计算
void time_space_distribution(vector<Car>&cars, vector<Cross>&cross)
{
	vector<int>start_s(cross.size(),0);
	vector<int>end_s(cross.size(),0);
	vector<int>p_start_s(cross.size(),0);
	vector<int>p_end_s(cross.size(),0);
	for(int i = 0; i < cars.size(); i++)
	{
		//所有车的最早和最晚出发时间
		if(cars[i].planTime < car_char[0].earlist)
			car_char[0].earlist = cars[i].planTime;
		if(cars[i].planTime > car_char[0].latest)
			car_char[0].latest = cars[i].planTime;

		//所有车辆的出发地和终止地分布
		start_s[cars[i].from] = 1;
		end_s[cars[i].from] = 1;
		//优先车辆的最早和最晚出发时间
		if(cars[i].priority)
		{
			if(cars[i].planTime < car_char[1].earlist)
				car_char[1].earlist = cars[i].planTime;
			if(cars[i].planTime > car_char[1].latest)
				car_char[1].latest = cars[i].planTime;
			p_start_s[cars[i].from] = 1;
			p_end_s[cars[i].from] = 1;
		}
	}
	//出发地分布和终止地分布
	car_char[0].start_s = accumulate(start_s.begin(),start_s.end(),0);
	car_char[0].end_s = accumulate(end_s.begin(),end_s.end(),0);
	car_char[1].start_s = accumulate(p_start_s.begin(),p_start_s.end(),0);
	car_char[1].end_s = accumulate(p_end_s.begin(),p_end_s.end(),0);
}

//计算系数a,b
void compute_modulus(double &a, double &b,vector<Car>&cars, vector<Cross>&cross)
{
	//time_space_distribution(cars, cross);
	vector<int>start_s(cross.size(),0);
	vector<int>end_s(cross.size(),0);
	vector<int>p_start_s(cross.size(),0);
	vector<int>p_end_s(cross.size(),0);
	car_char[0].car_num = cars.size();
	car_char[1].car_num = 0;
	speed_value.push_back(cars[0].max_speed);
	for(int i = 0; i < cars.size(); i++)
	{
		for(int j = 0; j < speed_value.size(); j++)
		{
			if(cars[i].max_speed == speed_value[j])
				break;
			if(j == speed_value.size() - 1)
				speed_value.push_back(cars[i].max_speed);
		}
		//记录所有车的最高和最低速度
		if(cars[i].max_speed > car_char[0].max_speed)
			car_char[0].max_speed = cars[i].max_speed;
		if(cars[i].max_speed < car_char[0].min_speed)
			car_char[0].min_speed = cars[i].max_speed;

		//所有车的最早和最晚出发时间
		if(cars[i].planTime < car_char[0].earlist)
			car_char[0].earlist = cars[i].planTime;
		if(cars[i].planTime > car_char[0].latest)
			car_char[0].latest = cars[i].planTime;

		//所有车辆的出发地和终止地分布
		start_s[cars[i].from] = 1;
		end_s[cars[i].to] = 1;

		//优先车辆的最早和最晚出发时间
		if(cars[i].priority)
		{
			car_char[1].car_num++;//记录优先车辆总数
			//记录所有车的最高和最低速度
			if(cars[i].max_speed > car_char[1].max_speed)
				car_char[1].max_speed = cars[i].max_speed;
			if(cars[i].max_speed < car_char[1].min_speed)
				car_char[1].min_speed = cars[i].max_speed;

			//所有车的最早和最晚出发时间
			if(cars[i].planTime < car_char[1].earlist)
				car_char[1].earlist = cars[i].planTime;
			if(cars[i].planTime > car_char[1].latest)
				car_char[1].latest = cars[i].planTime;

			p_start_s[cars[i].from] = 1;
			p_end_s[cars[i].to] = 1;
		}
	}
	sort(speed_value.begin(), speed_value.end(),greater<int>());
	//出发地分布和终止地分布
	car_char[0].start_s = accumulate(start_s.begin(),start_s.end(),0);
	car_char[0].end_s = accumulate(end_s.begin(),end_s.end(),0);
	car_char[1].start_s = accumulate(p_start_s.begin(),p_start_s.end(),0);
	car_char[1].end_s = accumulate(p_end_s.begin(),p_end_s.end(),0);

	a = (double)car_char[0].car_num/car_char[1].car_num * 0.05 + (double)car_char[0].max_speed / car_char[0].min_speed * car_char[1].min_speed / car_char[1].max_speed * 0.2375
		+ (double)car_char[0].latest / car_char[0].earlist * car_char[1].earlist / car_char[1].latest * 0.2375 + (double)car_char[0].start_s / car_char[1].start_s * 0.2375 + (double)car_char[0].end_s / car_char[1].end_s * 0.2375;
	b = (double)car_char[0].car_num/car_char[1].car_num * 0.8 + (double)car_char[0].max_speed / car_char[0].min_speed * car_char[1].min_speed / car_char[1].max_speed * 0.05
		+ (double)car_char[0].latest / car_char[0].earlist * car_char[1].earlist / car_char[1].latest * 0.05 + (double)car_char[0].start_s / car_char[1].start_s * 0.05 + (double)car_char[0].end_s / car_char[1].end_s * 0.05;
}

//汽车出生
void cars_birth_in_road(vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross, vector<int>&car_birth)
{
	for(int j = 0; j < car_birth.size(); j++)
	{
		int id = car_birth[j];
		if(cars[id].status == NO_START && cars[id].start_time == sys_time)//到了上路时间，就出生
		{
			if(roads[cars[id].route[0]].from == cars[id].from)
			{
				roads[cars[id].route[0]].garage_from_to.push_back(cars[id].id);
			}
			else
			{
				roads[cars[id].route[0]].garage_to_from.push_back(cars[id].id);
			}
		}
	}
	car_birth.clear();
}

//对所有的道路上能上路的车辆排序
void sort_no_start(vector<Road>&roads, vector<Car>&cars)
{
	for(int i = 0; i < roads.size(); i++)
	{
		roads[i].sort_cars(cars, roads[i].garage_from_to, sys_time);
		roads[i].sort_cars(cars, roads[i].garage_to_from, sys_time);
	}
}

void compute_all_time(long &s_all_schedule_time, long &all_schedule_time,vector<Car>&cars)
{
	all_schedule_time = 0;
	s_all_schedule_time = 0;
	for(int i = 0; i < cars.size(); i++)
	{
		all_schedule_time += (cars[i].end_time - cars[i].planTime);
		if(cars[i].priority)
		{
			s_all_schedule_time += (cars[i].end_time - cars[i].planTime);
		}
	}
}

//对车辆按照plantime、speed排序
void planT_speed_sort(vector<Car>&cars, vector<vector<int> >&ptime_speed_vec, vector<vector<int> >&ptime_speed_preset, vector<vector<int> >&ptime_speed_priority)
{
	//按照路口存放车辆index
	int preset_size = 0;
	int priority_size=0;
	int common_size = 0;

	//speed_value.push_back();
	for(unsigned int i = 0;i < cars.size() ; i ++)
	{
		if(!cars[i].preset)
		{
			if(cars[i].priority)//priority
			{
				if(cars[i].planTime > priority_size)
					priority_size = cars[i].planTime;
			}
			else
			{
				if(cars[i].planTime > common_size)
					common_size = cars[i].planTime;
			}

		}else
		{
			if(cars[i].start_time > preset_size)
				preset_size = cars[i].start_time;
		}
	}

	ptime_speed_vec.clear();
	ptime_speed_priority.clear();
	ptime_speed_preset.clear();

	vector<int> EmptyVec;
	for(int i=0;i<preset_size+1;i++)
	{
		ptime_speed_preset.push_back(EmptyVec);
	}
	for(int i=0;i<priority_size+1;i++)
	{
		ptime_speed_priority.push_back(EmptyVec);
	}
	for(int i=0;i<common_size+1;i++)
	{
		ptime_speed_vec.push_back(EmptyVec);
	}

	//填充二维数组
	for(unsigned int i = 0;i < cars.size() ; i ++)
	{
		if(cars[i].preset)
			ptime_speed_preset[cars[i].start_time].push_back(i);
		else if(cars[i].priority)
			ptime_speed_priority[cars[i].planTime].push_back(i);
		else
			ptime_speed_vec[cars[i].planTime].push_back(i);
	}

	//普通车和优先车速度排序
	for(int i = 0;i < common_size ; i++){
		speed_low_sort(cars,ptime_speed_vec[i]);
	}
	for(int i = 0;i < priority_size ; i++){
		speed_low_sort(cars,ptime_speed_priority[i]);
	}
}

//将一个ｖector截取部分放入另一个ｖector
void cut_vector(vector<Car>&cars, vector<vector<int> >&vec, double k)
{
    int N;
    vector<int> add_vec;
    int size=0;
    for(int i = 0; i < vec.size(); i++)
    {
    	if(vec[i].size() == 0)
    		continue;

		N = vec[i].size() * k;
		size += vec[i].size();

		//不能把预置车辆剔除！！！！
		while(N < vec[i].size() && cars[vec[i][N]].preset)
			N++;
		if(N == vec[i].size())
			continue;

		add_vec.insert(add_vec.end(), vec[i].begin() + N, vec[i].end());
		vec[i].erase(vec[i].begin() + N, vec[i].end());

    }
    //cout<<size<<endl;
    speed_low_sort(cars,add_vec);//速度降序排列
    vec.push_back(add_vec);
}

//每个时间片提取车并排序准备上路
void SureStartTime(vector<Car>&cars,vector<Road>&roads, vector<Cross>&cross, vector<int>&car_run_order,int &InCarsPer)
{
	int win = 2;//冗余系数
	//merge前面剩下的vec和此次可以上路的vec
	if(ptime_speed_vec.size() - 1 >= sys_time)
		rest_c_vec.insert(rest_c_vec.end(),ptime_speed_vec[sys_time].begin(),ptime_speed_vec[sys_time].end());
	if(ptime_speed_priority.size() - 1 >= sys_time)
		rest_p_vec.insert(rest_p_vec.begin(),ptime_speed_priority[sys_time].begin(),ptime_speed_priority[sys_time].end());

	//对地图上跑的车统计数量
	vector<int> CarOnRoad;
	for(int i = 0;i<cars.size();i++)
	{
		if(cars[i].status == INIT)
			CarOnRoad.push_back(i);
	}

	//if(sys_time%50 == 0)//每50个时间片重新规划一次
		ChangeRounteInTime(cars,roads,cross,CarOnRoad);

	int need_car;
	if(sys_time < time_keep)
	{
		need_car = range_keep - CarOnRoad.size();//这次需要上多少车
	}
	else
	{
		need_car = InCarsPer - CarOnRoad.size();//这次需要上多少车
	}

	for(int r = 0; r < roads.size(); r++)
	{
		for(int j = 0; j < roads[r].garage_from_to.size(); j++)
		{
			CarOnRoad.push_back(roads[r].garage_from_to[j]);
		}
		for(int j = 0; j < roads[r].garage_to_from.size(); j++)
		{
			CarOnRoad.push_back(roads[r].garage_to_from[j]);
		}
	}
	cnt_road_busy(CarOnRoad, car_run_order, roads, cars, cross);



	if((rest_p_vec.size() == 0 && rest_c_vec.size() == 0 ) || need_car <= 0)
	{
		//预置车辆直塞
		if(ptime_speed_preset.size() > sys_time && ptime_speed_preset[sys_time].size() > 0)
			car_run_order.insert(car_run_order.end(),ptime_speed_preset[sys_time].begin(),ptime_speed_preset[sys_time].end());
			return;
	}
	int size_preset = car_run_order.size();

	vector<int> vec;
	int real_need = need_car * win;//多存入20%，以免有些车由于拥挤上路不了
	if(rest_p_vec.size() >= real_need)
	{
		if(rest_p_vec.size() > sys_time)
			sort_speed_part(cars,rest_p_vec,0);
		vec.insert(vec.begin(),rest_p_vec.begin(),rest_p_vec.begin() + real_need);//从中抽取车上路
		rest_p_vec.erase(rest_p_vec.begin(),rest_p_vec.begin() + real_need);
		choose_car(CarOnRoad, vec, roads, cars, cross);
		car_run_order.insert(car_run_order.end(),vec.begin(),vec.begin() + need_car);//从中抽取车上路
		vector<int>rest(vec.begin() + need_car, vec.end());
		//speed_low_sort(cars,rest);
		if(rest_p_vec.size() <= sys_time)
			sort_speed_part(cars,rest,0);
		rest_p_vec.insert(rest_p_vec.begin(),rest.begin(), rest.end());//回收
	}
	else if(rest_p_vec.size() >= need_car)
	{
		if(rest_p_vec.size() > sys_time)
			sort_speed_part(cars,rest_p_vec,0);
		vec.insert(vec.begin(),rest_p_vec.begin(),rest_p_vec.end());//从中抽取车上路
		rest_p_vec.clear();
		choose_car(CarOnRoad, vec, roads, cars, cross);
		car_run_order.insert(car_run_order.end(),vec.begin(),vec.begin() + need_car);//从中抽取车上路
		vector<int>rest(vec.begin() + need_car, vec.end());
		//speed_low_sort(cars,rest);
		if(rest_p_vec.size() <= sys_time)
			sort_speed_part(cars,rest,0);
		rest_p_vec.insert(rest_p_vec.begin(),rest.begin(), rest.end());//回收
	}
	else
	{
		car_run_order.insert(car_run_order.end(),rest_p_vec.begin(),rest_p_vec.end());//从中抽取车上路
		need_car = need_car - rest_p_vec.size() - size_preset;
			
		choose_car(CarOnRoad, car_run_order,roads, cars, cross);
		//把优先车辆当作在路上
		CarOnRoad.insert(CarOnRoad.end(), rest_p_vec.begin(), rest_p_vec.end());

		real_need = need_car * win;//real_need - rest_p_vec.size();// - size_preset;
		rest_p_vec.clear();
		if(need_car > 0)
		{
			if(real_need > 0 && real_need <= rest_c_vec.size())//插入一部分优先车辆后，还需要插入普通车辆
			{
				if(rest_c_vec.size() == sys_time)
					sort_speed_part(cars,rest_c_vec,0);
				vec.insert(vec.begin(),rest_c_vec.begin(),rest_c_vec.begin() + real_need);//从中抽取车上路
				rest_c_vec.erase(rest_c_vec.begin(),rest_c_vec.begin() + real_need);
				choose_car(CarOnRoad, vec, roads, cars, cross);
				car_run_order.insert(car_run_order.end(),vec.begin(),vec.begin() + need_car);//从中抽取车上路
				vector<int>rest(vec.begin() + need_car, vec.end());
				//speed_low_sort(cars,rest);
				if(rest_c_vec.size() <= sys_time)
					sort_speed_part(cars,rest,0);
				rest_c_vec.insert(rest_c_vec.begin(),rest.begin(), rest.end());//回收
			}
			else if(rest_c_vec.size() > need_car)//普通车辆+优先车，比2×need_car少，比1×need_car多
			{
				vec.insert(vec.begin(),rest_c_vec.begin(),rest_c_vec.end());//从中抽取车上路;
				rest_c_vec.clear();
				choose_car(CarOnRoad, vec, roads, cars, cross);
				car_run_order.insert(car_run_order.end(),vec.begin(),vec.begin() + need_car);//从中抽取车上路
				vector<int>rest(vec.begin() + need_car, vec.end());
				//speed_low_sort(cars,rest);
				//sort_speed_part(cars,rest,0);
				rest_c_vec.insert(rest_c_vec.begin(),rest.begin(), rest.end());//回收
			}
			else//普通车辆也比较少，全部插入
			{
				car_run_order.insert(car_run_order.end(),rest_c_vec.begin(),rest_c_vec.end());//从中抽取车上路
				choose_car(CarOnRoad, rest_c_vec, roads, cars, cross);
				rest_c_vec.clear();
			}
		}
	}

	//赋值起始时间
	for(int i =0 ;i<car_run_order.size();i++)
	{
		if(!cars[car_run_order[i]].preset)
			cars[car_run_order[i]].start_time = sys_time;
	}
	//预置车辆直塞
	if(ptime_speed_preset.size() > sys_time && ptime_speed_preset[sys_time].size() > 0)
		car_run_order.insert(car_run_order.end(),ptime_speed_preset[sys_time].begin(),ptime_speed_preset[sys_time].end());
}

//给定car的id的vector,按照速度排序
void speed_low_sort(vector<Car>&cars,vector<int>&Vec)
{
	int size = Vec.size();
	for (int j = 0; j < size-1; j++){
		for (int k = 0; k < size-1-j; k++){
			if (cars[Vec[k]].max_speed < cars[Vec[k+1]].max_speed){
				int temp = Vec[k+1];
				Vec[k+1] = Vec[k];
				Vec[k] = temp;
			}
		}
	}
}

//速度降序排列
void sort_speed_part(vector<Car>&cars,vector<int>&Vec,int N)
{
	int size = Vec.size();
	int diff_speed = speed_value.size();
	vector<vector<int> >temp(diff_speed, vector<int>(0, 0));
	for (int j = 0; j < size; j++)
	{
		for(int i = 0; i < diff_speed; i++)
		{
			if(cars[Vec[j]].max_speed == speed_value[i])
			{
				temp[i].push_back(Vec[j]);
				break;
			}
		}
	}
	Vec.clear();
	for(int i = 0; i < diff_speed; i++)
	{
		Vec.insert(Vec.end(), temp[i].begin(), temp[i].end());
	}
}

////给定car的id的vector,按照在路上的时间排序
//void dis_low_sort(vector<Car>&cars,vector<int>&Vec)
//{
//	for (int j = 0; j < Vec.size()-1; j++){
//		for (int k = 0; k < Vec.size()-1-j; k++){
//			if (cars[Vec[k]].predict_runtime < cars[Vec[k+1]].predict_runtime){
//				int temp = Vec[k+1];
//				Vec[k+1] = Vec[k];
//				Vec[k] = temp;
//			}
//		}
//	}
//}

//根据拥堵情况释放新车上路
void choose_car(vector<int> CarOnRoad, vector<int>&car_run_order,vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross)
{

	double k=0.7;

	//每辆车实时规划路径
	BFS_Online(cars, roads, cross, car_run_order, busy_road);

	vector<double> clculate_car(car_run_order.size(),0);
	int road1;
	int cross1;
	for(int i=0;i<car_run_order.size();i++)
	{
		int route_pos = cars[car_run_order[i]].route_pos;
		double weigth = 1;
		double sum =0 ;
		while(route_pos<cars[car_run_order[i]].route.size())
		{
			road1 = cars[car_run_order[i]].route[route_pos];
			cross1 = cars[car_run_order[i]].route[route_pos+1];
			if(cross1 == roads[road1].to)
				clculate_car[i] += (weigth*busy_road[road1][0]);
			else
				clculate_car[i] += (weigth*busy_road[road1][1]);

			sum += weigth;
			weigth *= k;
			route_pos +=2;
		}

		//对有多段路的car求平均
		if(sum!=0)
			clculate_car[i]/=sum;
	}

	//按照拥堵状况重新排序,从小到大
	int Size = clculate_car.size();
	for (int j = 0; j < Size -1; j++){
		for (int k = 0; k < Size-1-j; k++){
			if (clculate_car[k] > clculate_car[k+1]){
				int temp = car_run_order[k+1];
				car_run_order[k+1] = car_run_order[k];
				car_run_order[k] = temp;

				temp = clculate_car[k+1];
				clculate_car[k+1] = clculate_car[k];
				clculate_car[k] = temp;
			}
		}
	}
}

void cnt_road_busy(vector<int>& CarOnRoad, vector<int>&car_run_order,vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross)
{
	double k=0.7;
	vector<vector<double> > temp_busy_road(roads.size(), vector<double>(2, 0));
	int i;
	for(int j =0;j<CarOnRoad.size();j++)
	{
		i = CarOnRoad[j];
		int route_pos;

		if(cars[i].status == INIT)
			route_pos = cars[i].route_pos - 2;
		else if(cars[i].status == NO_START)
			route_pos = 0;

		//当前道路
		double weigth = 1;
		while(route_pos + 1 < cars[i].route.size())
		{
			if(cars[i].route[route_pos+1] == roads[cars[i].route[route_pos]].to)
				temp_busy_road[cars[i].route[route_pos]][0] +=weigth;
			else
				temp_busy_road[cars[i].route[route_pos]][1] +=weigth;

			weigth *= k;
			route_pos +=2;
		}
	}

	for(int i=0;i<roads.size();i++)
	{
		temp_busy_road[i][0] /= (roads[i].channel*roads[i].length);
		if(roads[i].isDuplex)
			temp_busy_road[i][1] /= (roads[i].channel*roads[i].length);
	}
	busy_road = temp_busy_road;
}

//改变预置车辆路线
void IfChangeRoute( std::vector<Car>&cars, std::vector<Road> &road, std::vector<Cross> &cross)
{
	//ptime_speed_preset
	vector<int> car_id;//存车辆id
	vector<double> length;//存车辆路径长度差值
	vector<int> temproute;//临时路径
	double time1,time2;//预存路径与重新规划路径时间
	int thisroad,thiscar;
	for(int i=0;i<ptime_speed_preset.size();i++)
	{
		for(int j=0;j<ptime_speed_preset[i].size();j++)
		{
			BFS(cars[ptime_speed_preset[i][j]],road, cross, temproute);
			time1 = 0;
			time2 = 0;
			thiscar = ptime_speed_preset[i][j];
			for(int k=0;k<cars[thiscar].route.size();k+=2)
			{
				thisroad = cars[thiscar].route[k];
				time1 +=road[thisroad].length/(min(road[thisroad].speed, cars[thiscar].max_speed));
			}
			for(int k=0;k<temproute.size();k+=2)
			{
				thisroad = temproute[k];
				time2 +=road[thisroad].length/(min(road[thisroad].speed, cars[thiscar].max_speed));
			}
			length.push_back(time1 - time2);//存入时间差
			car_id.push_back(thiscar);
			//temproute.clear();
		}
	}

	//按照时间差重新排序,从大小
	int Size = length.size();
	int neednum = floor((double)Size*0.1);
	for (int j = 0; j < neednum; j++){
		for (int k = 0; k < Size-1-j; k++){
			if (length[k] > length[k+1]){
				int temp = length[k+1];
				length[k+1] = length[k];
				length[k] = temp;

				temp = car_id[k+1];
				car_id[k+1] = car_id[k];
				car_id[k] = temp;
			}
		}
		cars[car_id[Size-1-j]].change_route = true;
		cars_route.push_back(cars[car_id[Size-1-j]].route);//存下路径
		cars_id.push_back(car_id[Size-1-j]);//存下id
		//BFS(cars[car_id[Size-1-j]],road, cross, cars[car_id[Size-1-j]].route);
	}
}

void ChangeRounteInTime(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<int> CarOnRoad)
{
	//cars[CarOnRoad[i]] 当前在路上的车
	int pos = 0;
	int temp = 0;
	int currentPos = 0;
	int routeSize = 0;
	bool replan = false;
	for(int i=0; i<CarOnRoad.size(); i++)//对所有在路上的车重新进行规划
	{
		//添加对行驶距离的判断，走了一定距离后才进行重新规划，不再用时间来限制规划次数
		currentPos = cars[CarOnRoad[i]].route_pos-2;//当前位置
		routeSize = cars[CarOnRoad[i]].route.size();//路径长度

		if(currentPos<routeSize*cars[CarOnRoad[i]].ReplanTimes/4)
			continue;
		//

		if(cars[CarOnRoad[i]].preset && !cars[CarOnRoad[i]].change_route)//预置车辆不重新规划
			continue;
		if(cars[CarOnRoad[i]].route_pos<2)//刚出发一个路口都没有经过的，防止再次重复规划
			continue;
		if(cars[CarOnRoad[i]].pre_road_id != -1)
		{
			pos = cars[CarOnRoad[i]].next_cross_id;//将起点改为当前道路的终止路口
			FindAndDelete(cars[CarOnRoad[i]].route_pos-1, cars[CarOnRoad[i]] );//删除当前车辆在所在道路后面的路径
			choose_car_bfs_weight_A(cars[CarOnRoad[i]], roads, cross, cars[CarOnRoad[i]].route, busy_road,pos);

			replan = true;
			cars[CarOnRoad[i]].IfHasReplaned = true;
			cars[CarOnRoad[i]].ReplanTimes++;
			if(cars[CarOnRoad[i]].route_pos<cars[CarOnRoad[i]].route.size())
			{
				//更新下一个路口id
				cars[CarOnRoad[i]].next_cross_id = cars[CarOnRoad[i]].route[cars[CarOnRoad[i]].route_pos-1];
				//更新下一条路id
				cars[CarOnRoad[i]].next_road_id = cars[CarOnRoad[i]].route[cars[CarOnRoad[i]].route_pos];
				//更新下一次方向
				update_car_DLR(cars[CarOnRoad[i]],cross,false);
			}
		}
	}
}

void FindAndDelete(int pos, Car &car)
{
	car.route.erase(car.route.begin()+pos+1,car.route.end());
}

//改变预置车辆时间
void IfChangeTime(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross)
{
	int cnt = 0;
	bool mark = false;
	for(int i=ptime_speed_preset.size() - 1;i>=0;i--)
	{
		for(int j=ptime_speed_preset[i].size() - 1;j>=0;j--)
		{
			if(cars[ptime_speed_preset[i][j]].priority && cars[ptime_speed_preset[i][j]].start_time > cars[ptime_speed_preset[i][j]].planTime)
			{
				cars[ptime_speed_preset[i][j]].change_time = true;
				ptime_speed_preset[cars[ptime_speed_preset[i][j]].planTime].push_back(ptime_speed_preset[i][j]);
				cars[ptime_speed_preset[i][j]].start_time = cars[ptime_speed_preset[i][j]].planTime;
				ptime_speed_preset[i].erase(ptime_speed_preset[i].begin() + j);
				//ptime_speed_preset[i].erase(ptime_speed_preset[i].begin() + j);

				cnt++;
			}
			if(cnt >= c_t_preset_num)
			{
				mark = true;
				break;
			}
		}
		if(mark)
		{
			break;
		}
	}
}

//自动调参
void chan_parm_auto()
{
	//连续十次无解
	if(INF_time == 10 && step!=2)
	{
		INF_time = 0;
		//初始起点给大了，
		if(best_range == 0)
		{
			range_now = range_now - 1401 > 0 ? range_now - 1401 : 0;
		}
		else//找到过解
		{
				if(step == 14 || time_keep != 0)
				{
					if(time_keep ==0)//保证只进来一次
					{
						range_now = best_range;
						time_keep = best_s_schedule_time - 10;//最优优先车辆时间提前一点
						range_keep = range_now;
						step = 100;
						cout<<"rang_keep is :"<<range_keep<<endl;
						cout<<"The Time1 can be sure!The time is: "<<time_keep<<endl<<"The range_now is:"<<range_now<<endl;
					}
					else //if(time_keep2 ==0)
					{
						range_now = best_range;
						step = 4;//最后放弃阶段，小步慢走
						cout<<"OK! The scores can be Accept!Start from "<<range_now<<endl;
					}
				}
				else
				{
					range_now = best_range;
					step = 14;
					cout<<"The RangeStart can be sure:"<<range_now<<endl;
				}
		}
	}
}

//计算全图车位数，并且根据此设定车辆保有量的起始值
int get_roads_length(vector<Road>&roads)
{
	int length = 0;
	for(int i=0; i<roads.size(); i++)
	{
		length += roads[i].length*roads[i].channel;
		if(roads[i].isDuplex)//双向车道加两次
			length += roads[i].length*roads[i].channel;
	}
	return length;
}
