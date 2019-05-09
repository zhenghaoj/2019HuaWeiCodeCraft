#include "BFS.h"
#include "simulator.h"
#include <assert.h>
using namespace std;

double max_car_num = 0,min_car_num = INF;
double busy_turn = 50.0;//道路拥堵权重
double max_weight=0,min_weight=INF,max_route_len = 0,min_route_len = INF;
double weight_turn = 1;//拐弯带来的权重
double cross_weight = 0;//过路口权重


//所有车统一规划路径
void cars_bfs(vector<Car>&cars, vector<Road> &roads, std::vector<Cross>&cross)
{

	vector<int>bfs_done_car;
    for(unsigned int j = 0; j < cars.size(); j++)
    {
        if(cars[j].preset)
		{
        	bfs_done_car.push_back(cars[j].id);
		}
    }
    cnt_busy(cars, roads, cross, bfs_done_car);

	for(unsigned int j = 0; j < cars.size(); j++)
	{
		if(!cars[j].preset)//非预置车辆
		{
			BFS_weight(cars[j], roads, cross, cars[j].route);
			bfs_done_car.push_back(cars[j].id);
		}
		if(bfs_done_car.size() > 1000)
		{
			cnt_busy(cars, roads, cross, bfs_done_car);
		}
	}

    //统计每辆车经过路ｉd
    //cnt_cars(roads, cars, cross);
    //统计每辆车经过路口ｉd
    //cnt_route_cross(cars, cross);
}

//最短时间搜索
void BFS(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route)
{
	route.clear();
    int start = car.from;
    int target = car.to;

    queue<int> Q;
    Q.push(start);
    //清空所有路口到达标记
    for(unsigned int i=0;i<cross.size();i++)
    {
        cross[i].dis = INF;
    }
    cross[start].dis = 0;

    int road_id;
    int road_index;
    int next_cross_id;

    int index;//队列弹出路口id转化为下标index
    while(!Q.empty())
    {
        //队列头进行遍历
    	index = Q.front();
        Q.pop();

        for(int i=0;i<4;i++)
        {
        	switch(i)
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
            road_index = road_id;
            if(road_id != -1)
            {
                if(road[road_index].isDuplex==true)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?-1:road[road_index].to;//此路可能不通
                }

                if(next_cross_id != -1)
                {
                	double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);//car在此道路上停留的时间
                	double cross_time = cross_weight;// * road[road_index].channel;
                	double minlen = cross[index].dis + stop_time + cross_time;//car到下一个路口的时间

                    //如果第一次访问　或　此路径更优
                    if(cross[next_cross_id].dis>minlen)
                    {
                        //路口距离
                        cross[next_cross_id].dis = minlen;

                        if(next_cross_id != target)
                        {
                            Q.push(next_cross_id);
                        }
                    }
                }
            }
        }
    }
    //回溯路径
    //寻找速度与car匹配的路径
    int temp_cross_id = -1;
    int temp_road_id = -1;
    int temp_len_v = INF;

    stack<int> C;//装载回溯路径Cross
    stack<int> R;//装载回溯路径Road
    C.push(target);
    while(C.top()!=start)
    {
    	index = C.top();
        temp_len_v = INF;
        for(int i = 0; i < 4; i++)
        {
        	switch(i)
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
            road_index = road_id;
            if(road_id != -1 )//存在
            {
                if(road[road_index].isDuplex)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:-1;//此路可能不通
                }

                if(next_cross_id != -1)
                {
                    double stop_time = (double)road[road_index].length / min(car.max_speed,road[road_index].speed);
                    double cross_time = cross_weight;// * road[road_index].channel;
                    double levlen = cross[index].dis - stop_time - cross_time;//剩余路段长

                    if(abs(cross[next_cross_id].dis - levlen) < 0.000001)
                    {
                        if(abs(car.max_speed-road[road_index].speed) < temp_len_v)
                        {
                            //记录该次最优路径
                            temp_cross_id = next_cross_id;
                            temp_road_id = road_id;
                        }
                    }
                }
            }
        }
        assert(temp_cross_id >= cross[0].id && temp_cross_id <= cross[cross.size() - 1].id);
        C.push(temp_cross_id);
        R.push(temp_road_id);
    }

   C.pop();
    while(!R.empty())
    {
        route.push_back(R.top());
        route.push_back(C.top());
        C.pop();
        R.pop();
    }

}

//加拥堵权重
void BFS_weight(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route)
{
	car.route.clear();
    int start = car.from;
    int target = car.to;

    queue<int> Q;
    Q.push(start);
    //清空所有路口到达标记
    for(int i=0;i<cross.size();i++)
    {
        for(int j=0;j<4;j++)
            cross[i].Four_Dis[j] = INF;
    }
    for(int j=0;j<4;j++)
        cross[start].Four_Dis[j] = 0;

    //int *p ;//路口id的地址
    int road_id;
    int road_index;
    int next_cross_id;

    int index;//队列弹出路口id转化为下标index
    int drct;//当前道路在上一个路口的方向
    
    while(!Q.empty())
    {
        //队列头进行遍历
    	index = Q.front();
        Q.pop();

        for(int i=0;i<4;i++)
        {
            //p++;
            //road_id = *p;
        	switch(i)//↑ ↓ ← →
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
            road_index = road_id;
            if(road_id != -1)
            {
                if(road[road_index].isDuplex==true)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?-1:road[road_index].to;//此路可能不通
                }

                if(next_cross_id!=-1)
                {
                    double lenth_drct = INF; //最短方向的路径
                    for(int j=0;j<4;j++)
                    {
                        if(abs(cross[index].Four_Dis[j] - INF)<0.001 || i==j)//同一方向 或 来的方向没有路
                            continue;
                        if(i+j==1||i+j==5)//直行
                            lenth_drct = lenth_drct < cross[index].Four_Dis[j] ? lenth_drct : cross[index].Four_Dis[j];
                        else
                            lenth_drct = lenth_drct < cross[index].Four_Dis[j] + weight_turn ? lenth_drct : cross[index].Four_Dis[j] + weight_turn;
                    }
					
                	double crowd;
            		if(road[road_id - road[0].id].from == cross[next_cross_id - cross[0].id].id)
					{
            			crowd = (double) (road[road_index].times[0]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
					}
            		else
            		{
            			crowd = (double) (road[road_index].times[1]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
            		}
            		//car在此道路上停留的时间
                	double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
					double cross_time = cross_weight;// * road[road_index].channel;
                	double minlen = lenth_drct + stop_time + crowd + cross_time;//car到下一个路口的时间

                    drct=0;//该路在下一个路口的方向

					if(cross[next_cross_id - cross[0].id].up_road_id == road_id)
						drct=0;
					if(cross[next_cross_id - cross[0].id].down_road_id == road_id)
						drct=1;
					if(cross[next_cross_id - cross[0].id].left_road_id == road_id)
						drct=2;
					if(cross[next_cross_id - cross[0].id].right_road_id == road_id)
						drct=3;

                    if(cross[next_cross_id].Four_Dis[drct] > minlen )
                    {
                        //路口距离
                        cross[next_cross_id].Four_Dis[drct] = minlen;

                        if(next_cross_id != target)
                        {
                            Q.push(next_cross_id);
                        }
                    }
                }
            }
        }
    }
    //回溯路径
    //寻找速度与car匹配的路径

    stack<int> C;//装载回溯路径Cross
    stack<int> R;//装载回溯路径Road
    C.push(target);

    double lenth_drct = INF;//上一路口的路径长度
    for(int j=0;j<4;j++)
    {
        if(lenth_drct > cross[target].Four_Dis[j])
        {
            lenth_drct = cross[target].Four_Dis[j];
            drct = j;//最短路径的方向
        }
    }

    car.predict_runtime = lenth_drct;
    while(C.top()!=start)
    {
    	index = C.top();

        switch(drct)
        {
        case 0:
            road_id = cross[index].up_road_id;
            break;
        case 1:
            road_id = cross[index].down_road_id;
            break;
        case 2:
            road_id = cross[index].left_road_id;
            break;
        default:
            road_id = cross[index].right_road_id;
            break;
        }
        road_index = road_id;
        next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;

        double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
        //double crowd = ((double) road[road_index].times / max_car_num) * proportion;
    	double crowd;
		if(road[road_id - road[0].id].from != cross[next_cross_id - cross[0].id].id)
		{
			crowd = (double) (road[road_index].times[0]  - min_car_num) / (max_car_num - min_car_num) * busy_turn;
		}
		else
		{
			crowd = (double) (road[road_index].times[1]  - min_car_num) / (max_car_num - min_car_num) * busy_turn;
		}
		double cross_time = cross_weight;// * road[road_index].channel;
		double levlen = lenth_drct - stop_time - crowd - cross_time;//剩余路段长



		int a=0;//该路在下一个路口的方向

		if(cross[next_cross_id - cross[0].id].up_road_id == road_id)
			a=0;
		if(cross[next_cross_id - cross[0].id].down_road_id == road_id)
			a=1;
		if(cross[next_cross_id - cross[0].id].left_road_id == road_id)
			a=2;
		if(cross[next_cross_id - cross[0].id].right_road_id == road_id)
			a=3;


        for(int i=0;i<4;i++)
        {
            if(a==i)
                continue;

            if(a+i==1||a+i==5)//直行
            {
                if(abs(cross[next_cross_id].Four_Dis[i] - levlen) < 0.001)
                {
                    lenth_drct = cross[next_cross_id].Four_Dis[i];
                    drct = i;
                    C.push(next_cross_id);
                    R.push(road_id);
                    break;
                }
            }
            else
            {
                if(abs(cross[next_cross_id].Four_Dis[i] - levlen + weight_turn) < 0.001)
                {
                    lenth_drct = cross[next_cross_id].Four_Dis[i];
                    drct = i;
                    C.push(next_cross_id);
                    R.push(road_id);
                    break;
                }
            }
        }
    }

   C.pop();
    while(!R.empty())
    {
        route.push_back(R.top());
        route.push_back(C.top());
        C.pop();
        R.pop();
    }
}


//第一次上路时，不拥挤，用最短路径规划
void first_run_bfs(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route)
{
	int first_road = route[0];//保存第一条路
	int first_cross = route[1];
	car.route.clear();
    int start = first_cross;
    int target = car.to;

    queue<int> Q;
    Q.push(start);
    //清空所有路口到达标记
    for(unsigned int i=0;i<cross.size();i++)
    {
        cross[i].dis = INF;
    }
    cross[start].dis = 0;

    //int *p ;//路口id的地址
    int road_id;
    int road_index;
    int next_cross_id;

    int index;//队列弹出路口id转化为下标index
    while(!Q.empty()){
        //队列头进行遍历
    	index = Q.front();
        Q.pop();

        for(int i=0;i<4;i++)
        {
            //p++;
            //road_id = *p;
        	switch(i)
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
        	if(road_id == first_road)
        		continue;
            road_index = road_id;
            if(road_id != -1)
            {
                if(road[road_index].isDuplex)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?-1:road[road_index].to;//此路可能不通
                }

                if(next_cross_id!=-1)
                {
                	double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);//car在此道路上停留的时间
                	double cross_time = cross_weight;// * road[road_index].channel;
					double minlen = cross[index].dis + stop_time + cross_time;//car到下一个路口的时间

                    //如果第一次访问　或　此路径更优
                    if(cross[next_cross_id].dis>minlen)
                    {
                        //路口距离
                        cross[next_cross_id].dis = minlen;

                        if(next_cross_id != target)
                        {
                            Q.push(next_cross_id);
                        }
                    }
                }
            }
        }
    }
    //回溯路径
    //寻找速度与car匹配的路径
    int temp_cross_id = -1;
    int temp_road_id = -1;
    int temp_len_v = INF;

    stack<int> C;//装载回溯路径Cross
    stack<int> R;//装载回溯路径Road
    C.push(target);
    while(C.top()!=start)
    {
    	index = C.top();

        temp_len_v = INF;
        for(int i=0;i<4;i++)
        {
           // p++;
            //road_id = *p;
        	switch(i)
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
            road_index = road_id;

            if(road_id != -1 )//存在
            {
                if(road[road_index].isDuplex)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:-1;//此路可能不通
                }

                if(next_cross_id!=-1)
                {
                    double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
                    double cross_time = cross_weight;// * road[road_index].channel;
		    		double levlen = cross[index].dis - stop_time - cross_time;//剩余路段长

                    if(abs(cross[next_cross_id].dis - levlen) < 0.000001)
                    {
                        if(abs(car.max_speed-road[road_index].speed)<temp_len_v)
                        {
                            //记录该次最优路径
                            temp_cross_id = next_cross_id;
                            temp_road_id = road_id;
                        }
                    }
                }
            }
        }
        assert(temp_cross_id >= cross[0].id && temp_cross_id <= cross[cross.size() - 1].id);
        C.push(temp_cross_id);
        R.push(temp_road_id);
    }

	route.push_back(first_road);
	route.push_back(first_cross);
	C.pop();
    while(!R.empty())
    {
        route.push_back(R.top());
        route.push_back(C.top());
        C.pop();
        R.pop();
    }
}

//第一次上路时路径规划
void first_run_bfs_weight(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route, vector<vector<double> > &busyroad)
{
	max_car_num = 0,min_car_num = INF;
	for(int i = 0; i < busyroad.size() ; i++)
	{
		if(busyroad[i][1] > max_car_num)
			max_car_num = busyroad[i][1];
		if(busyroad[i][0] > max_car_num)
			max_car_num = busyroad[i][0];
		if(busyroad[i][1] < min_car_num)
			min_car_num = busyroad[i][1];
		if(busyroad[i][0] < min_car_num)
			min_car_num = busyroad[i][0];
	}
	if(max_car_num == 0)
	{
		first_run_bfs(car, road, cross, car.route);
		return;
	}

	int first_road = route[0];//保存第一条路
	int first_cross = route[1];

	car.route.clear();
    int start = first_cross;
    int target = car.to;

    queue<int> Q;
    Q.push(start);
    //清空所有路口到达标记
    for(int i=0;i<cross.size();i++)
    {
        for(int j=0;j<4;j++)
            cross[i].Four_Dis[j] = INF;
    }
    for(int j=0;j<4;j++)
        cross[start].Four_Dis[j] = 0;

    //int *p ;//路口id的地址
    int road_id;
    int road_index;
    int next_cross_id;

    int index;//队列弹出路口id转化为下标index
    int drct;//当前道路在上一个路口的方向

    while(!Q.empty()){
        //队列头进行遍历
    	index = Q.front();
        Q.pop();

        //p = (int*)&(cross[index]);//路口id的地址

        for(int i=0;i<4;i++)
        {
            //p++;
            //road_id = *p;
        	switch(i)//↑ ↓ ← →
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
        	if(road_id == first_road)
        		continue;
            road_index = road_id;
            if(road_id != -1)
            {
                if(road[road_index].isDuplex==true)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?-1:road[road_index].to;//此路可能不通
                }

                if(next_cross_id!=-1)
                {
                    double lenth_drct = INF; //最短方向的路径
                    for(int j=0;j<4;j++)
                    {
                        if(abs(cross[index].Four_Dis[j] - INF)<0.001 || i==j)//同一方向 或 来的方向没有路
                            continue;
                        if(i+j==1||i+j==5)//直行
                            lenth_drct = lenth_drct < cross[index].Four_Dis[j] ? lenth_drct : cross[index].Four_Dis[j];
                        else
                            lenth_drct = lenth_drct < cross[index].Four_Dis[j] + weight_turn ? lenth_drct : cross[index].Four_Dis[j] + weight_turn;
                    }

                	double crowd;
            		if(road[road_id - road[0].id].from == cross[next_cross_id - cross[0].id].id)
					{
            			crowd = (double) (busyroad[road_index][1]  - min_car_num) / (max_car_num - min_car_num) * busy_turn;
					}
            		else
            		{
            			crowd = (double) (busyroad[road_index][0]  - min_car_num) / (max_car_num - min_car_num) * busy_turn;
            		}

            		//car在此道路上停留的时间
                	double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
					double cross_time = cross_weight;// * road[road_index].channel;	
                	double minlen = lenth_drct + stop_time + crowd + cross_time;//car到下一个路口的时间

                    drct=0;//该路在下一个路口的方向

					if(cross[next_cross_id - cross[0].id].up_road_id == road_id)
						drct=0;
					if(cross[next_cross_id - cross[0].id].down_road_id == road_id)
						drct=1;
					if(cross[next_cross_id - cross[0].id].left_road_id == road_id)
						drct=2;
					if(cross[next_cross_id - cross[0].id].right_road_id == road_id)
						drct=3;

                    if(cross[next_cross_id].Four_Dis[drct] > minlen )
                    {
                        //路口距离
                        cross[next_cross_id].Four_Dis[drct] = minlen;

                        if(next_cross_id != target)
                        {
                            Q.push(next_cross_id);
                        }
                    }
                }
            }
        }
    }
    //回溯路径
    //寻找速度与car匹配的路径

    stack<int> C;//装载回溯路径Cross
    stack<int> R;//装载回溯路径Road
    C.push(target);

    double lenth_drct = INF;//上一路口的路径长度
    for(int j=0;j<4;j++)
    {
        if(lenth_drct > cross[target].Four_Dis[j])
        {
            lenth_drct = cross[target].Four_Dis[j];
            drct = j;//最短路径的方向
        }
    }
    //cross[target - FirstCrossId].dis = lenth_drct;
    car.predict_runtime = lenth_drct;
    while(C.top()!=start)
    {
    	index = C.top();
        switch(drct)
        {
        case 0:
            road_id = cross[index].up_road_id;
            break;
        case 1:
            road_id = cross[index].down_road_id;
            break;
        case 2:
            road_id = cross[index].left_road_id;
            break;
        default:
            road_id = cross[index].right_road_id;
            break;
        }
        road_index = road_id;
        next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;

        double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
        //double crowd = ((double) road[road_index].times / max_car_num) * proportion;
    	double crowd;
		if(road[road_id - road[0].id].from != cross[next_cross_id - cross[0].id].id)
		{
			crowd = (double) (busyroad[road_index][1]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
		}
		else
		{
			crowd = (double) (busyroad[road_index][0]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
		}
		double cross_time = cross_weight;// * road[road_index].channel;
		double levlen = lenth_drct - stop_time - crowd - cross_time;//剩余路段长



		int a=0;//该路在下一个路口的方向

		if(cross[next_cross_id - cross[0].id].up_road_id == road_id)
			a=0;
		if(cross[next_cross_id - cross[0].id].down_road_id == road_id)
			a=1;
		if(cross[next_cross_id - cross[0].id].left_road_id == road_id)
			a=2;
		if(cross[next_cross_id - cross[0].id].right_road_id == road_id)
			a=3;

        for(int i=0;i<4;i++)
        {
            if(a==i)
                continue;

            if(a+i==1||a+i==5)//直行
            {
                if(abs(cross[next_cross_id].Four_Dis[i] - levlen) < 0.001)
                {
                    lenth_drct = cross[next_cross_id].Four_Dis[i];
                    drct = i;
                    C.push(next_cross_id);
                    R.push(road_id);
                    break;
                }
            }
            else
            {
                if(abs(cross[next_cross_id].Four_Dis[i] - levlen + weight_turn) < 0.001)
                {
                    lenth_drct = cross[next_cross_id].Four_Dis[i];
                    drct = i;
                    C.push(next_cross_id);
                    R.push(road_id);
                    break;
                }
            }
        }
    }

    route.push_back(first_road);
    route.push_back(first_cross);
    C.pop();
    while(!R.empty())
    {
        route.push_back(R.top());
        route.push_back(C.top());
        //cout<<C.top()<<"  "<<R.top()<<endl;
        C.pop();
        R.pop();
    }
}

//统计道路拥堵情况
void cnt_busy(vector<Car>&cars, vector<Road>&roads,vector<Cross>&cross, vector<int>&bfs_done_car)
{
	//每次统计之前清零、初始化
	max_car_num = 0,min_car_num = INF;
	int next_cross_id, pre_road_id;
	int cnt = bfs_done_car.size();
	for(unsigned int k = 0; k < bfs_done_car.size() ; k++)
	{
		int i = bfs_done_car[k];
		for(unsigned int j = 0; j < cars[i].route.size() ; j += 2)
		{
			next_cross_id = cars[i].route[j + 1];
			pre_road_id = cars[i].route[j];
			if(roads[pre_road_id].from == cross[next_cross_id].id)
			{
				roads[pre_road_id].times[0] += (1.0 / roads[cars[i].route[j]].channel);
			}
			else
			{
				roads[pre_road_id].times[1] += (1.0 / roads[cars[i].route[j]].channel);
			}
		}
	}
	bfs_done_car.clear();
	for(unsigned int i = 0; i < roads.size() ; i++)
	{
		if(max_car_num < roads[i].times[0])
		{
			max_car_num = roads[i].times[0];
		}
		if(max_car_num < roads[i].times[1])
		{
			max_car_num = roads[i].times[1];
		}
		if(min_car_num > roads[i].times[0])
		{
			min_car_num = roads[i].times[0];
		}
		if(min_car_num > roads[i].times[1])
		{
			min_car_num = roads[i].times[1];
		}
	}
}

//统计路口和路出现车的次数
void cnt_cars(vector<Road>&roads, vector<Car>&cars, vector<Cross>&cross)
{

	//每次统计之前清零、初始化
	max_car_num = 0,min_car_num = INF;
	for(unsigned int i = 0; i < roads.size() ; i++)
	{
		roads[i].times[0] = 0;
		roads[i].times[1] = 0;
	}

	int next_cross_id, pre_road_id;
	//统计所有车辆
	for(unsigned int i = 0; i < cars.size() ; i++)
	{
		for(unsigned int j = 0; j < cars[i].route.size() ; j += 2)
		{
			next_cross_id = cars[i].route[j + 1];
			pre_road_id = cars[i].route[j];
			if(roads[pre_road_id - roads[0].id].from == cross[next_cross_id - cross[0].id].id)
			{
				roads[cars[i].route[j] - roads[0].id].times[0]++;
			}
			else
			{
				roads[cars[i].route[j] - roads[0].id].times[1]++;
			}
		}
	}

	//按照通道数，平均，但是弱化多通道
	for(unsigned int i = 0; i < roads.size() ; i++)
	{
		roads[i].times[0] /= roads[i].channel;
		roads[i].times[1] /= roads[i].channel;
	}

	for(unsigned int i = 0; i < roads.size() ; i++)
	{
		if(max_car_num < roads[i].times[0])
		{
			max_car_num = roads[i].times[0];
		}
		if(max_car_num < roads[i].times[1])
		{
			max_car_num = roads[i].times[1];
		}
		if(min_car_num > roads[i].times[0])
		{
			min_car_num = roads[i].times[0];
		}
		if(min_car_num > roads[i].times[1])
		{
			min_car_num = roads[i].times[1];
		}
	}
}

//统计每辆车经过路口ｉd
void cnt_route_cross(vector<Car>&cars, vector<Cross>&cross)
{
	vector<int>temp(cross.size(),0);
	for(int i = 0; i < cars.size(); i++)
	{
		cars[i].pass_cross_id = temp;
	}
	int cross_index;
	for(int i = 0; i < cars.size(); i++)
	{
		for(int j = 1; j < cars[i].route.size(); j += 2)
		{
			cross_index = cars[i].route[j] - cross[0].id;
			cars[i].pass_cross_id[cross_index]++;
		}
	}
}

void BFS_Online(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<int>&car_id, vector<vector<double> >&busyroad)
{
	max_car_num = 0,min_car_num = INF;
	for(unsigned int i = 0; i < busyroad.size() ; i++)
	{
		if(busyroad[i][1] > max_car_num)
			max_car_num = busyroad[i][1];
		if(busyroad[i][0] > max_car_num)
			max_car_num = busyroad[i][0];
		if(busyroad[i][1] < min_car_num)
			min_car_num = busyroad[i][1];
		if(busyroad[i][0] < min_car_num)
			min_car_num = busyroad[i][0];
	}

	int car_index;
	for(int i = 0; i < car_id.size(); i++)
	{
		car_index = car_id[i];
		if(cars[car_index].preset)
			continue;
		if(max_car_num == 0)
			BFS(cars[car_index], roads, cross, cars[car_index].route);
		else
			choose_car_bfs_weight(cars[car_index], roads, cross, cars[car_index].route, busyroad);
			//BFS_weight(cars[car_index], roads, cross, cars[car_index].route);
	}
}

//选车时用路径规划
void choose_car_bfs_weight(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route, vector<vector<double> >&busyroad)
{
	car.route.clear();
    int start = car.from;
    int target = car.to;

    queue<int> Q;
    Q.push(start);
    //清空所有路口到达标记
    for(int i=0;i<cross.size();i++)
    {
        for(int j=0;j<4;j++)
            cross[i].Four_Dis[j] = INF;
    }
    for(int j=0;j<4;j++)
        cross[start].Four_Dis[j] = 0;

    int road_id;
    int road_index;
    int next_cross_id;

    int index;//队列弹出路口id转化为下标index
    int drct;//当前道路在上一个路口的方向

    while(!Q.empty()){
        //队列头进行遍历
        index = Q.front();
        Q.pop();
        for(int i=0;i<4;i++)
        {
        	switch(i)//↑ ↓ ← →
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
            road_index = road_id ;
            if(road_id != -1)
            {
                if(road[road_index].isDuplex==true)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?-1:road[road_index].to;//此路可能不通
                }

                if(next_cross_id!=-1)
                {
                    double lenth_drct = INF; //最短方向的路径
                    for(int j=0;j<4;j++)
                    {
                        if(abs(cross[index].Four_Dis[j] - INF)<0.001 || i==j)//同一方向 或 来的方向没有路
                            continue;
                        if(i+j==1||i+j==5)//直行
                            lenth_drct = lenth_drct < cross[index].Four_Dis[j] ? lenth_drct : cross[index].Four_Dis[j];
                        else
                            lenth_drct = lenth_drct < cross[index].Four_Dis[j] + weight_turn ? lenth_drct : cross[index].Four_Dis[j] + weight_turn;
                    }

                	double crowd;
            		if(road[road_id - road[0].id].from == cross[next_cross_id - cross[0].id].id)
					{
            			crowd = (double) (busyroad[road_index][1]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
					}
            		else
            		{
            			crowd = (double) (busyroad[road_index][0]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
            		}

            		//car在此道路上停留的时间
                	double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
					double cross_time = cross_weight;// * road[road_index].channel;
                	double minlen = lenth_drct + stop_time + crowd + cross_time;//car到下一个路口的时间

                    drct=0;//该路在下一个路口的方向

					if(cross[next_cross_id - cross[0].id].up_road_id == road_id)
						drct=0;
					if(cross[next_cross_id - cross[0].id].down_road_id == road_id)
						drct=1;
					if(cross[next_cross_id - cross[0].id].left_road_id == road_id)
						drct=2;
					if(cross[next_cross_id - cross[0].id].right_road_id == road_id)
						drct=3;

                    if(cross[next_cross_id].Four_Dis[drct] > minlen )
                    {
                        //路口距离
                        cross[next_cross_id].Four_Dis[drct] = minlen;

                        if(next_cross_id != target)
                        {
                            Q.push(next_cross_id);
                        }
                    }
                }
            }
        }
    }
    //回溯路径
    //寻找速度与car匹配的路径

    stack<int> C;//装载回溯路径Cross
    stack<int> R;//装载回溯路径Road
    C.push(target);

    double lenth_drct = INF;//上一路口的路径长度
    for(int j=0;j<4;j++)
    {
        if(lenth_drct > cross[target].Four_Dis[j])
        {
            lenth_drct = cross[target].Four_Dis[j];
            drct = j;//最短路径的方向
        }
    }
    //cross[target - FirstCrossId].dis = lenth_drct;
    car.predict_runtime = lenth_drct;
    while(C.top()!=start)
    {
        index = C.top();
        switch(drct)
        {
        case 0:
            road_id = cross[index].up_road_id;
            break;
        case 1:
            road_id = cross[index].down_road_id;
            break;
        case 2:
            road_id = cross[index].left_road_id;
            break;
        default:
            road_id = cross[index].right_road_id;
            break;
        }
        road_index = road_id ;
        next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;

        double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
        //double crowd = ((double) road[road_index].times / max_car_num) * proportion;
    	double crowd;
		if(road[road_id - road[0].id].from != cross[next_cross_id - cross[0].id].id)
		{
			crowd = (double) (busyroad[road_index][1]  - min_car_num) / (max_car_num - min_car_num) * busy_turn;
		}
		else
		{
			crowd = (double) (busyroad[road_index][0]  - min_car_num) / (max_car_num - min_car_num) * busy_turn;
		}
		double cross_time = cross_weight;// * road[road_index].channel;
		double levlen = lenth_drct - stop_time - crowd - cross_time;//剩余路段长



		int a=0;//该路在下一个路口的方向

		if(cross[next_cross_id - cross[0].id].up_road_id == road_id)
			a=0;
		if(cross[next_cross_id - cross[0].id].down_road_id == road_id)
			a=1;
		if(cross[next_cross_id - cross[0].id].left_road_id == road_id)
			a=2;
		if(cross[next_cross_id - cross[0].id].right_road_id == road_id)
			a=3;


        for(int i=0;i<4;i++)
        {
            if(a==i)
                continue;

            if(a+i==1||a+i==5)//直行
            {
                if(abs(cross[next_cross_id].Four_Dis[i] - levlen) < 0.001)
                {
                    lenth_drct = cross[next_cross_id].Four_Dis[i];
                    drct = i;
                    C.push(next_cross_id);
                    R.push(road_id);
                    break;
                }
            }
            else
            {
                if(abs(cross[next_cross_id].Four_Dis[i] - levlen + weight_turn) < 0.001)
                {
                    lenth_drct = cross[next_cross_id].Four_Dis[i];
                    drct = i;
                    C.push(next_cross_id);
                    R.push(road_id);
                    break;
                }
            }
        }
    }

   C.pop();
    while(!R.empty())
    {
        route.push_back(R.top());
        route.push_back(C.top());
        C.pop();
        R.pop();
    }
}

//单个车辆的路径规划
void BFS_A(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route,int pos)
{
    int start = pos;
    int target = car.to;

    queue<int> Q;
    Q.push(start);
    //清空所有路口到达标记
    for(unsigned int i=0;i<cross.size();i++)
    {
        cross[i].dis = INF;
    }
    cross[start].dis = 0;

    //int *p ;//路口id的地址
    int road_id;
    int road_index;
    int next_cross_id;

    int index;//队列弹出路口id转化为下标index
    while(!Q.empty()){
        //队列头进行遍历
    	index = Q.front();
        Q.pop();


        //p = (int*)&(cross[index]);//路口id的地址

        for(int i=0;i<4;i++)
        {
            //p++;
            //road_id = *p;
        	switch(i)
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
            road_index = road_id ;
            if(road_id != -1)
            {
                if(road[road_index].isDuplex==true)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?-1:road[road_index].to;//此路可能不通
                }

                if(next_cross_id!=-1)
                {
                	double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);//car在此道路上停留的时间
                	double cross_time = cross_weight;// * road[road_index].channel;
			double minlen = cross[index].dis + stop_time + cross_time;//car到下一个路口的时间

                    //如果第一次访问　或　此路径更优
                    if(cross[next_cross_id].dis>minlen)
                    {
                        //路口距离
                        cross[next_cross_id].dis = minlen;

                        if(next_cross_id != target)
                        {
                            Q.push(next_cross_id);
                        }
                    }
                }
            }
        }
    }
    //回溯路径
    //寻找速度与car匹配的路径
    int temp_cross_id = -1;
    int temp_road_id = -1;
    int temp_len_v = INF;

    stack<int> C;//装载回溯路径Cross
    stack<int> R;//装载回溯路径Road
    C.push(target);
    while(C.top()!=start)
    {
    	index = C.top();
        temp_len_v = INF;
        for(int i=0;i<4;i++)
        {
           // p++;
            //road_id = *p;
        	switch(i)
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
            road_index = road_id ;

            if(road_id != -1 )//存在
            {
                if(road[road_index].isDuplex==true)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:-1;//此路可能不通
                }

                if(next_cross_id!=-1)
                {
                    double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
          	        double cross_time = cross_weight;// * road[road_index].channel;
		   			double levlen = cross[index].dis - stop_time - cross_time;//剩余路段长

                    if(abs(cross[next_cross_id].dis - levlen) < 0.000001)
                    {
                        if(abs(car.max_speed-road[road_index].speed)<temp_len_v)
                        {
                            //记录该次最优路径
                            temp_cross_id = next_cross_id;
                            temp_road_id = road_id;
                        }
                    }
                }
            }
        }
        assert(temp_cross_id >= cross[0].id && temp_cross_id <= cross[cross.size() - 1].id);
        C.push(temp_cross_id);
        R.push(temp_road_id);
    }
   //int test = C.top();
   C.pop();
    while(!R.empty())
    {
        route.push_back(R.top());
        route.push_back(C.top());
        //cout<<C.top()<<"  "<<R.top()<<endl;
        C.pop();
        R.pop();
    }
}

//实时规划路径
void BFS_Online_A(vector<Car>&cars, vector<Road>&roads, vector<Cross>&cross, vector<int>&car_id, vector<vector<double> >&busyroad, int pos)
{
	max_car_num = 0,min_car_num = INF;
	for(unsigned int i = 0; i < busyroad.size() ; i++)
	{
		if(busyroad[i][1] > max_car_num)
			max_car_num = busyroad[i][1];
		if(busyroad[i][0] > max_car_num)
			max_car_num = busyroad[i][0];
		if(busyroad[i][1] < min_car_num)
			min_car_num = busyroad[i][1];
		if(busyroad[i][0] < min_car_num)
			min_car_num = busyroad[i][0];
	}

	int car_index;
	for(int i = 0; i < car_id.size(); i++)
	{
		car_index = car_id[i];
		if(cars[car_index].preset)
			continue;
		if(max_car_num == 0)
			BFS_A(cars[car_index], roads, cross, cars[car_index].route, pos);
		else
			choose_car_bfs_weight_A(cars[car_index], roads, cross, cars[car_index].route, busyroad,pos);
			//BFS_weight(cars[car_index], roads, cross, cars[car_index].route);
	}
}

//选车时路径规划
void choose_car_bfs_weight_A(Car &car, std::vector<Road> &road, std::vector<Cross> &cross, vector<int> &route, vector<vector<double> >&busyroad, int pos)
{
    int start = pos;
    int target = car.to;

    queue<int> Q;
    Q.push(start);
    //清空所有路口到达标记
    for(int i=0;i<cross.size();i++)
    {
        for(int j=0;j<4;j++)
            cross[i].Four_Dis[j] = INF;
    }
    for(int j=0;j<4;j++)
        cross[start].Four_Dis[j] = 0;

    //int *p ;//路口id的地址
    int road_id;
    int road_index;
    int next_cross_id;

    int index;//队列弹出路口id转化为下标index
    int drct;//当前道路在上一个路口的方向

    while(!Q.empty()){
        //队列头进行遍历
    	index = Q.front();
        Q.pop();
        for(int i=0;i<4;i++)
        {
        	switch(i)//↑ ↓ ← →
        	{
        	case 0:
        		road_id = cross[index].up_road_id;
        		break;
        	case 1:
        		road_id = cross[index].down_road_id;
        		break;
        	case 2:
        		road_id = cross[index].left_road_id;
        		break;
        	default:
        		road_id = cross[index].right_road_id;
        		break;
        	}
            road_index = road_id ;
            if(road[road_index].id == car.pre_road_id)//防止掉头
            	continue;
            if(road_id != -1)
            {
                if(road[road_index].isDuplex==true)//双车道
                {
                    next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;
                }
                else
                {
                    next_cross_id = road[road_index].to == index?-1:road[road_index].to;//此路可能不通
                }

                if(next_cross_id!=-1)
                {
                    double lenth_drct = INF; //最短方向的路径
                    for(int j=0;j<4;j++)
                    {
                        if(abs(cross[index].Four_Dis[j] - INF)<0.001 || i==j)//同一方向 或 来的方向没有路
                            continue;
                        if(i+j==1||i+j==5)//直行
                            lenth_drct = lenth_drct < cross[index].Four_Dis[j] ? lenth_drct : cross[index].Four_Dis[j];
                        else
                            lenth_drct = lenth_drct < cross[index].Four_Dis[j] + weight_turn ? lenth_drct : cross[index].Four_Dis[j] + weight_turn;
                    }

                	double crowd;
            		if(road[road_id - road[0].id].from == cross[next_cross_id - cross[0].id].id)
					{
            			crowd = (double) (busyroad[road_index][1]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
					}
            		else
            		{
            			crowd = (double) (busyroad[road_index][0]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
            		}

            		//car在此道路上停留的时间
                	double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
                	double cross_time = cross_weight;// * road[road_index].channel;
                	double minlen = lenth_drct + stop_time + crowd + cross_time;//car到下一个路口的时间

                    drct=0;//该路在下一个路口的方向

					if(cross[next_cross_id - cross[0].id].up_road_id == road_id)
						drct=0;
					if(cross[next_cross_id - cross[0].id].down_road_id == road_id)
						drct=1;
					if(cross[next_cross_id - cross[0].id].left_road_id == road_id)
						drct=2;
					if(cross[next_cross_id - cross[0].id].right_road_id == road_id)
						drct=3;

                    if(cross[next_cross_id].Four_Dis[drct] > minlen )
                    {
                        //路口距离
                        cross[next_cross_id].Four_Dis[drct] = minlen;

                        if(next_cross_id != target)
                        {
                            Q.push(next_cross_id);
                        }
                    }
                }
            }
        }
    }
    //回溯路径
    //寻找速度与car匹配的路径

    stack<int> C;//装载回溯路径Cross
    stack<int> R;//装载回溯路径Road
    C.push(target);

    double lenth_drct = INF;//上一路口的路径长度
    for(int j=0;j<4;j++)
    {
        if(lenth_drct > cross[target].Four_Dis[j])
        {
            lenth_drct = cross[target].Four_Dis[j];
            drct = j;//最短路径的方向
        }
    }
    //cross[target - FirstCrossId].dis = lenth_drct;
    car.predict_runtime = lenth_drct;
    while(C.top()!=start)
    {
    	index = C.top();
        switch(drct)
        {
        case 0:
            road_id = cross[index].up_road_id;
            break;
        case 1:
            road_id = cross[index].down_road_id;
            break;
        case 2:
            road_id = cross[index].left_road_id;
            break;
        default:
            road_id = cross[index].right_road_id;
            break;
        }
        road_index = road_id ;
        next_cross_id = road[road_index].to == index?road[road_index].from:road[road_index].to;

        double stop_time = (double)road[road_index].length/min(car.max_speed,road[road_index].speed);
        //double crowd = ((double) road[road_index].times / max_car_num) * proportion;
    	double crowd;
		if(road[road_id - road[0].id].from != cross[next_cross_id - cross[0].id].id)
		{
			crowd = (double) (busyroad[road_index][1]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
		}
		else
		{
			crowd = (double) (busyroad[road_index][0]  - min_car_num)/ (max_car_num - min_car_num) * busy_turn;
		}
		double cross_time = cross_weight;// * road[road_index].channel;
		double levlen = lenth_drct - stop_time - crowd - cross_time;//剩余路段长



		int a=0;//该路在下一个路口的方向

		if(cross[next_cross_id - cross[0].id].up_road_id == road_id)
			a=0;
		if(cross[next_cross_id - cross[0].id].down_road_id == road_id)
			a=1;
		if(cross[next_cross_id - cross[0].id].left_road_id == road_id)
			a=2;
		if(cross[next_cross_id - cross[0].id].right_road_id == road_id)
			a=3;


        for(int i=0;i<4;i++)
        {
            if(a==i)
                continue;
            if(a+i==1||a+i==5)//直行
            {
                if(abs(cross[next_cross_id].Four_Dis[i] - levlen) < 0.001)
                {
                    lenth_drct = cross[next_cross_id].Four_Dis[i];
                    drct = i;
                    C.push(next_cross_id);
                    R.push(road_id);
                    break;
                }
            }else{
                if(abs(cross[next_cross_id].Four_Dis[i] - levlen + weight_turn) < 0.001)
                {
                    lenth_drct = cross[next_cross_id].Four_Dis[i];
                    drct = i;
                    C.push(next_cross_id);
                    R.push(road_id);
                    break;
                }
            }
        }
    }

   C.pop();
    while(!R.empty())
    {
        route.push_back(R.top());
        route.push_back(C.top());
        C.pop();
        R.pop();
    }
}

