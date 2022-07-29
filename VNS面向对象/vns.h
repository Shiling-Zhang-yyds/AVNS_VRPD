#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>
#include <random>
#include <fstream>
#include <map>
#include <utility>
#include <numeric>

using namespace std;

#define V 10 //客户数量+2
#define FILE_PATH "../demo/R201.txt"
#define COEF_PATH "../demo/拥堵系数.txt"
//创建解的类
class SolutionElement {
public:
	int upper;//上层属性，代表服务的客户点
	bool lower;//下层属性，代表服务方式：0 (卡车); 1 (无人机)
};

//创建点的属性类
class Vertice {
public:
	int index;//序号
	double x;//x坐标
	double y;//y坐标
	double demand;//需求
};

//创建无人机路线类
class UavSortie {
public:
	int lau;//起点
	int dro;//服务点
	int red;//降落点
};

//该类用于将Solution中的无人机路线与卡车路线分离，方便计算Solution的成本
class DetachedSolution : public UavSortie {
public:
	vector<vector<int>> truck_route;
	vector<UavSortie> drone_route;
};

//邻域权重类，用于每一次寻优后对所有的邻域进行权重分配
class NeighborhoodWeight {
public:
	int index;//邻域序号
	double proportion;//占用的比例
};

//解的呈现方案类
class SOLUTION {
public:
	vector<SolutionElement> solution;
};

//继承自解的派生类，用于返回权重
class SolutionWithNeighborWeight : public SOLUTION
{
public:
	vector<NeighborhoodWeight> sorting;//返回各个邻域对应编号以及对应的权重
};

//用于对所有符合无人机重量约束的点进行路径匹配的所有方案集合
class DroneExtractionScheme {
public:
	int lau;//无人机起点
	int dro;//无人机服务点
	int rdz;//无人机降落点
	int routeIndex;//dro点对应的卡车路径
	int interIndex;//路径中的降落点所在下标
	double addCost;//提取dro点的每个方案对应的成本变化
};

//谓词函数
extern bool AscendPairSort(const pair<int, double>& p1, const pair<int, double>& p2);
extern bool DescendWeightSort(const NeighborhoodWeight& p1, const NeighborhoodWeight& p2);//根据权重大小降序排列
extern bool DescendExtractDroneCostSort(const DroneExtractionScheme& p1, const DroneExtractionScheme& p2);//用于按照节约的成本降序排列不同插入方式


//解决方案类
class SOLVE {
public:
	SOLVE() //默认构造函数，用于初始化所需数据
	{
		fstream input;
		input.open(FILE_PATH);
		for (int i = 0; i < V; i++) {
			input >> vertex[i].index;
			input >> vertex[i].x;
			input >> vertex[i].y;
			input >> vertex[i].demand;
		}
		//计算所有点之间的距离
		for (int i = 0; i < V; i++)
			for (int j = 0; j < V; j++) {
				distace_matrix[i][j] = pow((vertex[i].x - vertex[j].x) * (vertex[i].x - vertex[j].x) +
					(vertex[i].y - vertex[j].y) * (vertex[i].y - vertex[j].y), 0.5);
			}
		input.close();

		fstream readCoef(COEF_PATH);
		for (int i = 0; i < V; i++)
			for (int j = 0; j < V; j++) {
				readCoef >> conjestion_coefficient_matrix[i][j];
			}
		readCoef.close();
		/*cout << "拥堵系数" << endl;
		for (int i = 0; i < V; i++)
		{
			for (int j = 0; j < V; j++) {
				cout << conjestion_coefficient_matrix[i][j] << "\t";
			}
			cout << endl;
		}*/
	}

	/**
	* part1->InitialSolution
	*/
	//InitialSolution的成本计算函数
	double FitnessOfTruckRoute(DetachedSolution& s);
	//将所有点根据容量约束连接起来，作为起始解
	DetachedSolution GenerateInitialTruckRoute();
	//用于判断当前3个点能否组成无人机路线
	bool JudgeFeaibilityOfThreeNodes(int l, int d, int r, vector<DroneExtractionScheme>& uav_rout);
	//清空InitialSolution中不包含任何客户点的路线
	void ClearEmptyRouteInInitialSolution(DetachedSolution& s);
	
	DetachedSolution VnsOfInitialSolution(DetachedSolution& s);//VNS of InitialSolution
	DetachedSolution VndOfInitialSolution(DetachedSolution& s);//VND of InitialSolution
	DetachedSolution TwoExchangeInInitialSolution(DetachedSolution& s);//对初始解进行 2-opt操作
	DetachedSolution RelocateNodeInInitialSolution(DetachedSolution& s);//对初始解进行relocate操作
	SOLUTION  ExtractDroneRouteFromInitialSolution(DetachedSolution& s);//提取无人机路线。提取规则在 JudgeFeaibilityOfThreeNodes

	/**
	* part2->Solution
	*/
	//打印输出Solution
	void PrintSolution(SOLUTION& s);
	//Solution的成本计算函数
	double FitnessOfDroneAndTruckRoute(SOLUTION& s1) {
		vector<DroneExtractionScheme> uav;
		SOLUTION s = s1;
		//路段超载惩罚
		double part = 0, whole = 0; double cost = 0;
		for (auto it = s.solution.begin(); it != s.solution.end(); it++) {
			if (it->upper != 0) { part += vertex[it->upper].demand; }
			else {
				if (part > truck_capacity) {
					whole += part - truck_capacity;
				}
				part = 0;
			}
		}
		cost += whole * 10000;
		for (int i = s.solution.size() - 2; i > 0; i--) {
			//进行分流， 将 0 1 的一对和后面的提取出为一个无人机路线，
			if (s.solution[i - 1].lower == 0 && s.solution[i].lower == 1) {
				uav.push_back({ s.solution[i - 1].upper, s.solution[i].upper,s.solution[i + 1].upper });
				s.solution.erase(s.solution.begin() + i);
			}
		}

		for (int i = 0; i < s.solution.size() - 1; i++) {
			cost += distace_matrix[s.solution[i].upper][s.solution[i + 1].upper] * truck_unit_cost;
		}

		for (auto it = uav.begin(); it != uav.end(); it++) {
			cost += (distace_matrix[it->lau][it->dro] + distace_matrix[it->dro][it->rdz]) * drone_unit_cost;
			//设置惩罚成本
			//1、duration penalty
			double uav_time = (distace_matrix[it->lau][it->dro] + distace_matrix[it->dro][it->rdz]) / drone_speed,
				van_time = distace_matrix[it->lau][it->rdz] / (truck_speed * conjestion_coefficient_matrix[it->lau][it->rdz]);
			double first_uav_time = distace_matrix[it->lau][it->dro] / drone_speed,
				second_uav_time = distace_matrix[it->dro][it->rdz] / drone_speed;
			//如果无人机飞行时间超过卡车最大等待时间
			if (uav_time > van_time + max_waiting_time) {
				cost += (uav_time - van_time) * 10000;
			}

			//考虑重量变化
			//如果无人机飞行时间超过卡车行驶时间
			if (first_uav_time + second_uav_time > van_time) {
				if (first_uav_time * battery_consume_rate * (drone_self_weight + vertex[it->dro].demand) +
					(second_uav_time + first_uav_time) * battery_consume_rate * drone_self_weight > battery_capacity) {
					cost += ((first_uav_time * battery_consume_rate * (drone_self_weight + vertex[it->dro].demand) +
						(second_uav_time + first_uav_time) * battery_consume_rate * drone_self_weight) - battery_capacity) * 10000;
				}
			}
			//如果卡车行驶时间超过无人机飞行时间
			else {
				if (first_uav_time * battery_consume_rate * (drone_self_weight + vertex[it->dro].demand) +
					(van_time - first_uav_time) * battery_consume_rate * drone_self_weight > battery_capacity) {
					cost += ((first_uav_time * battery_consume_rate * (drone_self_weight + vertex[it->dro].demand) +
						(van_time - first_uav_time) * battery_consume_rate * drone_self_weight) - battery_capacity) * 10000;
				}
			}

			if (vertex[it->dro].demand > drone_capacity) {
				cost += -(drone_capacity - vertex[it->dro].demand) * 10000;
			}
		}
		return cost;
	}
	/*         VNS 主框架         */
	SOLUTION VNS(SOLUTION& s);
	void ShakingSolution(SOLUTION& s);
	SolutionWithNeighborWeight VND(SOLUTION& s, vector<NeighborhoodWeight>& weight);
	/*        扰动以及局部搜索函数和算子         */
	SOLUTION OperatorOfRelocatedAsDrone(SOLUTION& s);
	SOLUTION OperatorOfToggleService(SOLUTION& s);
	SOLUTION SwapNodeOfShaking(SOLUTION& s);
	SOLUTION RandomSwapNode(SOLUTION& s);//随机交换一个点
	SOLUTION RandomSwapWhole(SOLUTION& s);//随机交换一个点及其方式
	SOLUTION RelocateWhole(SOLUTION& s);
	SOLUTION RelocateAsDrone(SOLUTION& s);
	SOLUTION RandomToggleService(SOLUTION& s);//更改一个点为1
	SOLUTION RandomReverseNode(SOLUTION& s);//随机反转
	SOLUTION RandomReverseWhole(SOLUTION& s);//随机反转全部

private:
	double battery_consume_rate = 0.2;//电池消耗
	double max_waiting_time = 1;//卡车等无人机的最大时间
	const double truck_speed = 0.8;//卡车速度
	const double drone_speed = 1.5;//飞机速度
	const double truck_capacity = 20;//卡车容量
	// 大规模情况下，调整卡车容量
	const double drone_capacity = 5;//飞机容量
	const double drone_self_weight = 1;//飞机自重
	const double truck_unit_cost = 10;//卡车成本
	const double drone_unit_cost = 0.1;//飞机成本
	const double consumeRate = 0.2;//飞机电量消耗率
	const double battery_capacity = 20;//飞机电池大小
	double w1 = 1, w2 = 1, w3 = 1, w4 = 1, w5 = 1, w6 = 1, w7 = 1;//每个领域的权重初始值
	const int max_iteration = 15 * V;//最大迭代次数
	double distace_matrix[V][V];//数组存放所有点之间的距离
	Vertice vertex[V];//数组存放所有点的属性
	double conjestion_coefficient_matrix[V][V];//存放速度矩阵
};