#ifndef __PREPROCESS_H__
#define __PREPROCESS_H__

#include<vector>
#include<string>
#include <map>
#include "data_struct.h"
using namespace std;

#define MAX_INFO_NUM    50
#define MAX_DATA_NUM    100



//读取文件，并存到vector
int read_file(vector<string> &data, const char * const filename);

//将车id,start_time以及路线，写入
void write_result(vector<Car>&cars,const char * const filename,map<int , int > map_car_id_to_i, map<int , int > map_road_id_to_i);

//将result缓冲区中的内容写入文件，写入方式为覆盖写入
void write_file(const bool cover, const char * const buff, const char * const filename);

//只写入start_time
void write_start_time(vector<Car>&cars,const char * const filename);

//car数据解析
void car_info_analysis(vector<string> &data, vector<Car>&cars);

//road数据解析
void road_info_analysis(vector<string> &data, vector<Road>&roads);

//cross数据解析
void cross_info_analysis(vector<string> &data, vector<Cross>&cross);

//预置车辆
void preset_info_analysis(vector<Car>&cars, vector<Cross>&cross, vector<Road>&roads, vector<string>&data, map<int,int>&map_car_id_to_i,map<int,int>&map_road_id_to_i);

//道路地图初始化
void roads_map_init(vector<Road>&roads);

//路口数据初始化
void sort_road_id(vector<Cross>&cross);

//直接读入anser文件
void anser_analysis(vector<Car>&cars, vector<Cross>&cross, vector<Road>&roads, const char *const filename,  vector<int>&car_id_back, vector<int>&road_id_back);

#endif
