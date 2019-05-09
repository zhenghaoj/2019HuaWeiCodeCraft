#ifndef __DATA_STRUCT_H__
#define __DATA_STRUCT_H__

#include <vector>
#include <queue>
#include <map>
using namespace std;
//定义所需要的数据结构
#define INF 0x7fffffff
//DONE:一次循环结束，WAIT：等待过路口。INIT：下一次循环开始。NO_START：未上路车辆。END;到达终点车辆。COME_END：即将到到终点
enum car_status {DONE=1, WAIT=0, INIT = -1, NO_START = -2, END = -3};//COME_END = -4
enum car_dlr {DIRICT=0, LEFT=1, RIGHT = 2};
class Car
{
public:
    //成员变量

    unsigned short from;                //汽车起点
    unsigned short to;                  //汽车终点
    unsigned short max_speed;           //汽车限速
    unsigned short planTime;            //汽车出发时间
    unsigned short real_speed = 0;      //汽车实际速度
    unsigned short pos = 0;        		//距离即将驶入路口距离
    unsigned short channel = 0;			//所在道路的
    unsigned short DLR = DIRICT;        //即将行走方向：0：直行；1：左转；2：右转;
    unsigned short start_time;			//测试用
    short status =NO_START;      		//汽车状态，0:等待移动；1：本时间片已经移动；-1：待判断状态

    bool change_route = false;
    bool change_time = false;
    bool priority;
    bool preset;
    int end_time;
    int id;                             //汽车ID
    int next_cross_id = -1;             //下一个路口ID
    int pre_road_id = -1;               //当前道路ID
    int next_road_id  ;                 //下一个道路ID

    double predict_runtime;				//预期汽车跑完所用时间

    int route_pos = 0;
    vector<int>route;					//规划的路线
    vector<int>preset_original_route;	//存预置车辆原路线
    vector<int>pass_cross_id;
    bool IfHasReplaned = false;
    int ReplanTimes = 1;
    //成员函数

};

class Road
{
public:
    //成员变量
    int id;                            	//道路ID
    unsigned short length;             	//道路长度
    unsigned short speed;             	//道路限速
    unsigned short channel;             //道路车道数量
    unsigned short from;              	//道路起点
    unsigned short to;                	//道路终点
    bool isDuplex;                    	//是否双向车道
    int prior_cars[2] = {-1,-1};		//第一优先级车辆，0：从from-to,即forward方向，1：相反的方向
    double times[2] = {0};				//0：从from-to,即forward方向，1：相反的方向

    vector<vector<int> >forward_map;    //从from到to的地图
    vector<vector<int> >back_map;      	//从to到from的地图

    vector<int>garage_from_to;			//从from-to上路车辆
    vector<int>garage_to_from;			//从to-from上路车辆

    //成员函数
    void init_map();           			//申请内存，并初始化地图
    int get_prior_car(vector<vector<int> >&map, vector<Car>&cars); //确定第一优先级车辆
    void sort_cars(vector<Car>&cars, vector<int>&run_order,int sys_t);
};


class Cross
{
public:
    //成员变量
    int id;                             //路口ID
    int up_road_id;                     //连接道路ID，顺时针
    int right_road_id;
    int down_road_id;
    int left_road_id;
    double dis = INF;                   //距离起点位置，默认无穷大

    int times = 0;
    int road_id_order[4];               //道路ID排序
    double Four_Dis[4];					//四个方向到达该路口的最短路径
    //成员函数
    void put_road_order();              //对道路id按从小到大排序

};

struct all_car_charact
{
	double car_num = 0;//所有车数量
	double max_speed = 0;//最高速度
	double min_speed = INF;//最低速度
	double latest = 0;//最晚出发时间
	double earlist = INF;//最早出发时间
	double start_s;//出发地分布
	double end_s;//终止点分布
	double end_time;//终止时间
};

//自定义排序函数
bool SortRoad(const Road &v1, const Road &v2);
bool SortCross(const Cross &v1,const Cross &v2);
bool SortCar(const Car &v1,const Car &v2);
void map_id(vector<Car>&cars, vector<Cross>&cross, vector<Road>&roads, map<int,int>&map_car_i_to_id, map<int,int>&map_car_id_to_i, map<int,int>&map_road_i_to_id, map<int,int>&map_road_id_to_i, map<int,int>&map_cross_i_to_id, map<int,int>&map_cross_id_to_i);

//void back_map(vector<Car>&cars, vector<Road>&roads,vector<int>&car_id_b, vector<int>&road_id_b);
#endif
