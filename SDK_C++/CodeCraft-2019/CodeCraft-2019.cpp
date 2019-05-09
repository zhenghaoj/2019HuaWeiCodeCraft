#include <iostream>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <map>

#include "stdlib.h"
#include "preprocess.h"
#include "data_struct.h"
#include "BFS.h"
#include "simulator.h"
using namespace std;

//在线测试
//#define _TEST_ONLINE
//本地测试
#define _TEST_LOCAL
int main(int argc, char *argv[])
{
	std::cout << "Begin" << std::endl;

#ifdef _TEST_ONLINE
	if(argc < 6){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}

	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string presetAnswerPath(argv[4]);
	std::string answerPath(argv[5]);
#endif

#ifdef _TEST_LOCAL
	std::string carPath("./3-map-training-2/car.txt");//config_10
	std::string roadPath("./3-map-training-2/road.txt");
	std::string crossPath("./3-map-training-2/cross.txt");//1-map-training-1
	std::string presetAnswerPath("./3-map-training-2/presetAnswer.txt");
	std::string answerPath("./answer.txt");
#endif

	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "presetAnswerPath is " << presetAnswerPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;

	//文件数据读取
	vector<string>car_data, road_data, cross_data, preset_data;

	int car_cnt = read_file(car_data, carPath.data()) - 1;
	int road_cnt = read_file(road_data, roadPath.data()) - 1;
	int cross_cnt = read_file(cross_data, crossPath.data()) - 1;
	int preset_cnt = read_file(preset_data, presetAnswerPath.data()) - 1;

	//string 数据解读
	vector<Car>cars(car_cnt);
	vector<Road>roads(road_cnt);
	vector<Cross>cross(cross_cnt);

	car_info_analysis(car_data, cars);
	road_info_analysis(road_data, roads);
	cross_info_analysis(cross_data, cross);

	//建立一个map表
	map<int, int> map_car_i_to_id;
	map<int, int> map_road_i_to_id;
	map<int, int> map_cross_i_to_id;

	map<int, int> map_car_id_to_i;
	map<int, int> map_road_id_to_i;
	map<int, int> map_cross_id_to_i;

	//根据id大小排序
	sort(roads.begin(),roads.end(), SortRoad);
	sort(cars.begin(),cars.end(), SortCar);
	sort(cross.begin(),cross.end(), SortCross);

	//将id映射到0～n
	map_id(cars, cross, roads, map_car_i_to_id, map_car_id_to_i, map_road_i_to_id, map_road_id_to_i, map_cross_i_to_id, map_cross_id_to_i);
	preset_info_analysis(cars, cross, roads, preset_data, map_car_id_to_i,map_road_id_to_i);

	//直接读取answer
	//anser_analysis(cars, cross, roads, "./2-map-training-2/answer.txt", car_id_back, road_id_back);

	//记录最好出发时间,记录最好的路径
	vector<int>best_start_time(car_cnt, 0);
	vector<vector<int> >best_cars_route(car_cnt, vector<int>(0));

	//迭代器
	iteration(cars, roads, cross, best_start_time, best_cars_route, preset_cnt);

	//更新出发时间
	final_update_time(cars, best_start_time, best_cars_route);

	//输出文件
	write_result(cars,answerPath.c_str(),map_car_i_to_id, map_road_i_to_id);

	return 0;
}
