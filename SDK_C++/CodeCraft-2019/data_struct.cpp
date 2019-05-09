#include "data_struct.h"

//申请内存，并初始化地图
void Road :: init_map()
{

    vector<int> m(length, -1);
    for(int i = 0; i < channel; i++)
    {
        forward_map.push_back(m);
    }
    //双向道路
    if(isDuplex)
    {
        for(int i = 0; i < channel; i++)
        {
            back_map.push_back(m);
        }
    }
}

//确定第一优先级车辆
int Road :: get_prior_car(vector<vector<int> >&map, vector<Car>&cars)//确定第一优先级车辆
{
	if(map.size() == 0)//没有地图
		return -1;

	int prior_car = -1;
	int pre_car;
	for(int j = 0; j < channel; j++)//先遍历通道，把每个通道最前面的wait状态车辆记下来再比较
	{
		for(int i = 0; i < length; i++)
		{
			if(map[j][i] != -1 && cars[map[j][i] - cars[0].id].status == DONE)//该通道找到DONE车，后面的肯定是DONE
			{
				break;
			}
			if(map[j][i] != -1 && cars[map[j][i] - cars[0].id].status == WAIT)
			{
				pre_car = map[j][i];
				if(prior_car == -1)//第一次找到
				{
					prior_car = pre_car;
				}
				else
				{
					//两个都是优先车辆或者都不是优先车辆
					if((cars[prior_car].priority && cars[pre_car].priority) || (!cars[prior_car].priority && !cars[pre_car].priority))
					{
						if(cars[pre_car].pos < cars[prior_car].pos)//同优先级，位置在前优先，prior通道肯定比pre_car小
						{
							prior_car = pre_car;
						}
					}
					else//否则哪个是优先车辆，哪个是第一优先级
					{
						if(!cars[prior_car].priority)
						{
							prior_car = pre_car;
						}
					}
				}
				break;//每个通道比较最前面的车
			}
		}
	}
	return prior_car;
}

//第一次出发排序，按照时间，优先级，车辆id排序
void Road:: sort_cars(vector<Car>&cars, vector<int>&run_order, int sys_t)
{
	bool odd_exist = false;//是否存在上个时间片剩余车辆
	for(int j = 0; j < run_order.size(); j++)
	{
		if(cars[run_order[j]].start_time < sys_t)
		{
			odd_exist = true;
			break;
		}
	}
	if(odd_exist)//存在剩余车辆，按照优先级，时间，car_id排序
	{
		//先按照优先级排序
		for(int j = 0; j < run_order.size(); j++)
		{
			for(int i = 0; i <run_order.size() - 1 - j; i++)
			{
				if(cars[run_order[i]].priority && !cars[run_order[i + 1]].priority)
				{
					swap(run_order[i],run_order[i + 1]);
				}
			}
		}
		//相同优先级，按照时间排序
		for(int j = 0; j < run_order.size(); j++)
		{
			for(int i = 0; i <run_order.size() - 1 - j; i++)
			{
				if(cars[run_order[i]].priority == cars[run_order[i + 1]].priority
					&& cars[run_order[i]].start_time < cars[run_order[i + 1]].start_time)
				{
					swap(run_order[i],run_order[i + 1]);
				}
			}
		}
		//相同优先级，相同时间，按照id排序
		for(int j = 0; j < run_order.size(); j++)
		{
			for(int i = 0; i <run_order.size() - 1 - j; i++)
			{
				if(cars[run_order[i]].priority == cars[run_order[i + 1]].priority
					&& cars[run_order[i]].start_time == cars[run_order[i + 1]].start_time
					&& cars[run_order[i]].id < cars[run_order[i + 1]].id)
				{
					swap(run_order[i],run_order[i + 1]);
				}
			}
		}

	}
	else//不存在剩余车辆，按照优先级，车辆id排序
	{
		//先按照优先级排序
		for(int j = 0; j < run_order.size(); j++)
		{
			for(int i = 0; i <run_order.size() - 1 - j; i++)
			{
				if(cars[run_order[i]].priority && !cars[run_order[i + 1]].priority)
				{
					swap(run_order[i],run_order[i + 1]);
				}
			}
		}
		//相同优先级，按照id排序
		for(int j = 0; j < run_order.size(); j++)
		{
			for(int i = 0; i <run_order.size() - 1 - j; i++)
			{
				if(cars[run_order[i]].priority == cars[run_order[i + 1]].priority
					&& cars[run_order[i]].id < cars[run_order[i + 1]].id)
				{
					swap(run_order[i],run_order[i + 1]);
				}
			}
		}
	}

}


//每一个cross按路口id排序
void Cross:: put_road_order()
{
    road_id_order[0] = up_road_id;
    road_id_order[1] = right_road_id;
    road_id_order[2] = down_road_id;
    road_id_order[3] = left_road_id;
    int temp;
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 3 - i; j++)
        {
            if(road_id_order[j] > road_id_order[j + 1])
            {
                swap(road_id_order[j], road_id_order[j + 1]);
            }
        }
    }
}
//自定义排序函数
bool SortRoad(const Road &v1,const Road &v2)//注意：本函数的参数的类型一定要与vector中元素的类型一致
{
    return v1.id < v2.id;//升序排列
}
//自定义排序函数
bool SortCross(const Cross &v1,const Cross &v2)//注意：本函数的参数的类型一定要与vector中元素的类型一致
{
    return v1.id < v2.id;//升序排列
}
//自定义排序函数
bool SortCar(const Car &v1,const Car &v2)//注意：本函数的参数的类型一定要与vector中元素的类型一致
{
    return v1.id < v2.id;//升序排列
}
void map_id(vector<Car>&cars, vector<Cross>&cross, vector<Road>&roads,map<int,int>&map_car_i_to_id,map<int,int>&map_car_id_to_i,map<int,int>&map_road_i_to_id,map<int,int>&map_road_id_to_i,map<int,int>&map_cross_i_to_id,map<int,int>&map_cross_id_to_i)
{
	//hash表
	for(int i = 0; i < cross.size(); i++)
	{
		map_cross_i_to_id[i] = cross[i].id;
		map_cross_id_to_i[cross[i].id] = i;
		cross[i].id = i;
	}
	//
	for(int i = 0; i < cars.size(); i++)
	{
		map_car_i_to_id[i] = cars[i].id;
		map_car_id_to_i[cars[i].id] = i;
		cars[i].id = i;
		cars[i].from = map_cross_id_to_i[cars[i].from];
		cars[i].to = map_cross_id_to_i[cars[i].to];
	}

	//
	for(int i = 0; i < roads.size(); i++)
	{
		map_road_i_to_id[i] = roads[i].id;
		map_road_id_to_i[roads[i].id] = i;
		roads[i].id = i;
		roads[i].from = map_cross_id_to_i[roads[i].from];
		roads[i].to = map_cross_id_to_i[roads[i].to];
	}
	//
	for(int i = 0; i < cross.size(); i++)
	{
		if(cross[i].up_road_id != -1)
		{
			cross[i].up_road_id = map_road_id_to_i[cross[i].up_road_id];
		}
		if(cross[i].down_road_id != -1)
		{
			cross[i].down_road_id = map_road_id_to_i[cross[i].down_road_id];
		}
		if(cross[i].left_road_id != -1)
		{
			cross[i].left_road_id = map_road_id_to_i[cross[i].left_road_id];
		}
		if(cross[i].right_road_id != -1)
		{
			cross[i].right_road_id = map_road_id_to_i[cross[i].right_road_id];
		}
	}

}


