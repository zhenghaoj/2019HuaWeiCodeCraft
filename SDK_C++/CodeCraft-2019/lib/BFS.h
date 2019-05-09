#ifndef __BFS_H__
#define __BFS_H__

#include "iostream"
#include "cstdio"
#include<vector>
#include"data_struct.h"
#include <queue>
#include <stack>
#include <cmath>

using namespace std;

#define INF 0x7fffffff
//#define min(a,b)    (((a) < (b)) ? (a) : (b))

extern double busy_turn;
extern double weight_turn;//拐弯带来的权重
extern double cross_weight;
//单个车辆的路径规划
void BFS(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route);

//所有车辆的
void cars_bfs(vector<Car>&cars, vector<Road> &roads, std::vector<Cross>&cross);

//统计路口和路出现车的次数,返回最大车辆数量
void cnt_cars(vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross);

//加拥堵权重
void BFS_weight(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route);

//统计每辆车经过路口ｉd
void cnt_route_cross(vector<Car>&cars, vector<Cross>&cross);

//每隔一定数量车规划完路径后，重新学习权重
void cnt_busy(vector<Car>&cars, vector<Road>&roads,vector<Cross>&cross, vector<int>&bfs_done_car);

//初赛版，整体规划学习
void cars_bfs_origin(vector<Car>&cars, vector<Road> &roads, std::vector<Cross>&cross);

//实时规划路径
void BFS_Online(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<int>&car_id, vector<vector<double> >&busyroad);

//上路实时规划路径
void first_run_bfs_weight(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route, vector<vector<double> > &busyroad);
//不用学习的路径规划
void first_run_bfs(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route);

//选车时路径规划
void choose_car_bfs_weight(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route, vector<vector<double> >&busyroad);

//选车时路径规划
void choose_car_bfs_weight_A(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route, vector<vector<double> >&busyroad, int pos);

//实时规划路径
void BFS_Online_A(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<int>&car_id, vector<vector<double> >&busyroad, int pos);

//单个车辆的路径规划
void BFS_A(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route,int pos);

#endif
