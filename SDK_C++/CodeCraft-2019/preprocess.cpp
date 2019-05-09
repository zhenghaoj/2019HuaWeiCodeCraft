#include <string>
#include <fstream>
#include <sstream>
#include"preprocess.h"
#include <iostream>
#include <stdlib.h>
#include "data_struct.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "simulator.h"

using namespace std;

//文件读入
int read_file(vector<string> &data, const char *const filename)
{
    int line_cnt = 0;

    ifstream in(filename);
    if(!in)
    {
        cout << __DATE__ << " " << __TIME__
                  << __FILE__ << " " << __LINE__
                  << ": dict read error" << endl;
        exit(-1);
    }

    string line;
    while(getline(in, line))
    {
        line_cnt++;
        data.push_back(line);//.substr(1,line.size() - 2)
    }
    in.close();
    return line_cnt;

}

void write_result(vector<Car>&cars,const char * const filename,map<int , int > map_car_i_to_id, map<int , int > map_road_i_to_id)
{
	//string anser = "";
	stringstream sstr;
	string str = sstr.str();
	sstr <<"#(carID,StartTime,RoadId...)\n";
	int car_id,road_id;
	for(int i = 0; i < cars.size(); i++)
	{
		if(cars[i].preset && !cars[i].change_route && !cars[i].change_time)
			continue;

		sstr <<"(";
		car_id = map_car_i_to_id[cars[i].id];
		sstr << car_id;
		sstr <<",";
		sstr << cars[i].start_time;
		sstr <<",";
		int j;
		for( j= 0; j < cars[i].route.size() - 2; j += 2)
		{
			road_id = map_road_i_to_id[cars[i].route[j]];
			sstr << road_id;
			sstr <<",";
		}
		road_id = map_road_i_to_id[cars[i].route[j]];
		sstr  << road_id;
		sstr << ")";
		if(i <  cars.size() - 1)
			sstr <<"\n";
	}
	// 以覆盖的方式写入
    write_file(1, sstr.str().c_str(), filename);

}

 void write_file(const bool cover, const char * const buff, const char * const filename)
{
	if (buff == NULL)
		return;

	const char *write_type = cover ? "w" : "a";//1:覆盖写文件，0:追加写文件
	FILE *fp = fopen(filename, write_type);
	if (fp == NULL)
	{
		printf("Fail to open file %s, %s.\n", filename);
		return;
	}
	printf("Open file %s OK.\n", filename);
	fputs(buff, fp);
	fputs("\n", fp);
	fclose(fp);
}

//write_start_time
void write_start_time(vector<Car>&cars,const char * const filename)
{
	stringstream sstr;
	string str = sstr.str();
	sstr <<"{";
	for(int i = 0; i < cars.size(); i++)
	{
		sstr << cars[i].start_time;
		if(i <  cars.size() - 1)
			sstr <<",";
		if(i%1000 == 0 && i > 0)
			sstr <<"\n";
	}
	sstr << "}";
	// 以覆盖的方式写入
    write_file(1, sstr.str().c_str(), filename);

}
//car数据解析
void car_info_analysis(vector<string> &data, vector<Car>&cars)
{
    string str = "";
    unsigned short pos = 0;

    for(unsigned int i = 1; i < data.size(); i++)
    {
        str = "";
        pos = 0;
        for(unsigned int j = 1; j < data[i].size(); j++)
        {
            if(data[i][j] >= '0' && data[i][j] <= '9')
            {
                str += data[i][j];
                continue;
            }
            if(data[i][j] == ',' || data[i][j] == ')')
            {
                switch(pos)
                {
                case 0:
                    cars[i - 1].id = atoi(str.c_str());
                    break;
                case 1:
                    cars[i - 1].from = atoi(str.c_str());
                    break;
                case 2:
                    cars[i - 1].to = atoi(str.c_str());
                    break;
                case 3:
                    {
                    	cars[i - 1].max_speed = atoi(str.c_str());
                        break;
                    }
                case 4:
                    cars[i - 1].planTime = atoi(str.c_str());
                    break;
                case 5:
					{
						cars[i - 1].priority = atoi(str.c_str());
						break;
					}
                case 6:
					cars[i - 1].preset = atoi(str.c_str());
					break;
                }
                pos++;
                str = "";
            }
        }
    }
    data.clear();
}

//road数据解析
void road_info_analysis(vector<string> &data, vector<Road>&roads)
{
    string str = "";
    unsigned short pos = 0;
    for(unsigned int i = 1; i < data.size(); i++)
    {
        str = "";
        pos = 0;
        for(unsigned int j = 1; j < data[i].size(); j++)
        {
            if(data[i][j] >= '0' && data[i][j] <= '9')
            {
                str += data[i][j];
                continue;
            }
            if(data[i][j] == ',' || data[i][j] == ')')
            {
                switch(pos)
                {
                case 0:
                    roads[i - 1].id = atoi(str.c_str());
                    break;
                case 1:
                    roads[i - 1].length = atoi(str.c_str());
                    break;
                case 2:
                    roads[i - 1].speed = atoi(str.c_str());
                    break;
                case 3:
                    roads[i - 1].channel = atoi(str.c_str());
                    break;
                case 4:
                    roads[i - 1].from = atoi(str.c_str());
                    break;
                case 5:
                    roads[i - 1].to = atoi(str.c_str());
                    break;
                case 6:
                    roads[i - 1].isDuplex = atoi(str.c_str());
                    break;
                }
                pos++;
                str = "";
            }
        }
    }
    data.clear();

    //道路地图初始化
    roads_map_init(roads);
}

//cross数据解析
void cross_info_analysis(vector<string> &data, vector<Cross>&cross)
{
    string str = "";
    unsigned short pos = 0;
    for(unsigned int i = 1; i < data.size(); i++)
    {
        str = "";
        pos = 0;
        for(unsigned int j = 1; j < data[i].size(); j++)
        {
            if(data[i][j] >= '0' && data[i][j] <= '9' || data[i][j] == '-')
            {
                str += data[i][j];
                continue;
            }
            if(data[i][j] == ',' || data[i][j] == ')')
            {
                switch(pos)
                {
                case 0:
                    cross[i - 1].id = atoi(str.c_str());
                    break;
                case 1:
                    cross[i - 1].up_road_id = atoi(str.c_str());
                    break;
                case 2:
                    cross[i - 1].right_road_id = atoi(str.c_str());
                    break;
                case 3:
                    cross[i - 1].down_road_id = atoi(str.c_str());
                    break;
                default:
                    cross[i - 1].left_road_id = atoi(str.c_str());
                    break;
                }
                pos++;
                str = "";
            }
        }
    }
    data.clear();
}

//cross数据解析
void preset_info_analysis(vector<Car>&cars, vector<Cross>&cross, vector<Road>&roads, vector<string>&data, map<int,int>&map_car_id_to_i,map<int,int>&map_road_id_to_i)
{
	string str = "";
	unsigned short pos = 0;
	vector<int>road;
	int car_index;
	for(unsigned int i = 1; i < data.size(); i++)
	{
		str = "";
		pos = 0;
		road.clear();
		for(unsigned int j = 1; j < data[i].size(); j++)
		{
			if(data[i][j] >= '0' && data[i][j] <= '9' || data[i][j] == '-')
			{
				str += data[i][j];
				continue;
			}
			if(data[i][j] == ',' || data[i][j] == ')')
			{
				switch(pos)
				{
				case 0:
					car_index = map_car_id_to_i[atoi(str.c_str())];
					break;
				case 1:
					cars[car_index].start_time = atoi(str.c_str());
					break;
				default:
					road.push_back(map_road_id_to_i[atoi(str.c_str())]);
					break;
				}
				pos++;
				str = "";
			}
		}
		for(unsigned int j = 0; j < road.size() - 1; j++)
		{
			cars[car_index].route.push_back(road[j]);
			if(roads[road[j] - roads[0].id].to == roads[road[j + 1] - roads[0].id].from
					||roads[road[j] - roads[0].id].to == roads[road[j + 1] - roads[0].id].to)
			{
				cars[car_index].route.push_back(roads[road[j] - roads[0].id].to);
			}
			else
			{
				cars[car_index].route.push_back(roads[road[j] - roads[0].id].from);
			}

		}
		cars[car_index].route.push_back(road[road.size() - 1]);
		if(roads[road[road.size() - 1] - roads[0].id].to == cars[car_index].to)
		{
			cars[car_index].route.push_back(roads[road[road.size() - 1] - roads[0].id].to);
		}
		else
		{
			cars[car_index].route.push_back(roads[road[road.size() - 1] - roads[0].id].from);
		}
	}
	data.clear();
}

//道路地图初始化
void roads_map_init(vector<Road>&roads)
{
    for(unsigned int i = 0; i < roads.size(); i++)
    {
        roads[i].init_map();
    }
}

//路口数据初始化
void sort_road_id(vector<Cross>&cross)
{
    for(int i = 0; i < cross.size(); i++)
    {
        cross[i].put_road_order();    //按道路ID升序
    }
}

//解析anser.txt
void anser_analysis(vector<Car>&cars, vector<Cross>&cross, vector<Road>&roads, const char *const filename, vector<int>&car_id_back, vector<int>&road_id_back)
{
	vector<string>data;
	read_file(data, filename);
	string str = "";
	unsigned short pos = 0;
	vector<int>road;
	int car_index;
	for(unsigned int i = 1; i < data.size(); i++)
	{
		str = "";
		pos = 0;
		road.clear();
		for(unsigned int j = 1; j < data[i].size(); j++)
		{
			if(data[i][j] >= '0' && data[i][j] <= '9' || data[i][j] == '-')
			{
				str += data[i][j];
				continue;
			}
			if(data[i][j] == ',' || data[i][j] == ')')
			{
				switch(pos)
				{
				case 0:
					car_index = car_id_back[atoi(str.c_str())];
					break;
				case 1:
					cars[car_index].start_time = atoi(str.c_str());
					break;
				default:
					road.push_back(road_id_back[atoi(str.c_str())]);
					break;
				}
				pos++;
				str = "";
			}
		}

		int road_size = road.size();
		for(int j = 0; j < road_size - 1; j++)
		{
			cars[car_index].route.push_back(road[j]);
			if(roads[road[j] - roads[0].id].to == roads[road[j + 1] - roads[0].id].from
					||roads[road[j] - roads[0].id].to == roads[road[j + 1] - roads[0].id].to)
			{
				cars[car_index].route.push_back(roads[road[j] - roads[0].id].to);
			}
			else
			{
				cars[car_index].route.push_back(roads[road[j] - roads[0].id].from);
			}

		}
		cars[car_index].route.push_back(road[road.size() - 1]);
		if(roads[road[road.size() - 1] - roads[0].id].to == cars[car_index].to)
		{
			cars[car_index].route.push_back(roads[road[road.size() - 1] - roads[0].id].to);
		}
		else
		{
			cars[car_index].route.push_back(roads[road[road.size() - 1] - roads[0].id].from);
		}
	}
	data.clear();
}

