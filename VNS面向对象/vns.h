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

#define V 10 //�ͻ�����+2
#define FILE_PATH "../demo/R201.txt"
#define COEF_PATH "../demo/ӵ��ϵ��.txt"
//���������
class SolutionElement {
public:
	int upper;//�ϲ����ԣ��������Ŀͻ���
	bool lower;//�²����ԣ��������ʽ��0 (����); 1 (���˻�)
};

//�������������
class Vertice {
public:
	int index;//���
	double x;//x����
	double y;//y����
	double demand;//����
};

//�������˻�·����
class UavSortie {
public:
	int lau;//���
	int dro;//�����
	int red;//�����
};

//�������ڽ�Solution�е����˻�·���뿨��·�߷��룬�������Solution�ĳɱ�
class DetachedSolution : public UavSortie {
public:
	vector<vector<int>> truck_route;
	vector<UavSortie> drone_route;
};

//����Ȩ���࣬����ÿһ��Ѱ�ź�����е��������Ȩ�ط���
class NeighborhoodWeight {
public:
	int index;//�������
	double proportion;//ռ�õı���
};

//��ĳ��ַ�����
class SOLUTION {
public:
	vector<SolutionElement> solution;
};

//�̳��Խ�������࣬���ڷ���Ȩ��
class SolutionWithNeighborWeight : public SOLUTION
{
public:
	vector<NeighborhoodWeight> sorting;//���ظ��������Ӧ����Լ���Ӧ��Ȩ��
};

//���ڶ����з������˻�����Լ���ĵ����·��ƥ������з�������
class DroneExtractionScheme {
public:
	int lau;//���˻����
	int dro;//���˻������
	int rdz;//���˻������
	int routeIndex;//dro���Ӧ�Ŀ���·��
	int interIndex;//·���еĽ���������±�
	double addCost;//��ȡdro���ÿ��������Ӧ�ĳɱ��仯
};

//ν�ʺ���
extern bool AscendPairSort(const pair<int, double>& p1, const pair<int, double>& p2);
extern bool DescendWeightSort(const NeighborhoodWeight& p1, const NeighborhoodWeight& p2);//����Ȩ�ش�С��������
extern bool DescendExtractDroneCostSort(const DroneExtractionScheme& p1, const DroneExtractionScheme& p2);//���ڰ��ս�Լ�ĳɱ��������в�ͬ���뷽ʽ


//���������
class SOLVE {
public:
	SOLVE() //Ĭ�Ϲ��캯�������ڳ�ʼ����������
	{
		fstream input;
		input.open(FILE_PATH);
		for (int i = 0; i < V; i++) {
			input >> vertex[i].index;
			input >> vertex[i].x;
			input >> vertex[i].y;
			input >> vertex[i].demand;
		}
		//�������е�֮��ľ���
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
		/*cout << "ӵ��ϵ��" << endl;
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
	//InitialSolution�ĳɱ����㺯��
	double FitnessOfTruckRoute(DetachedSolution& s);
	//�����е��������Լ��������������Ϊ��ʼ��
	DetachedSolution GenerateInitialTruckRoute();
	//�����жϵ�ǰ3�����ܷ�������˻�·��
	bool JudgeFeaibilityOfThreeNodes(int l, int d, int r, vector<DroneExtractionScheme>& uav_rout);
	//���InitialSolution�в������κοͻ����·��
	void ClearEmptyRouteInInitialSolution(DetachedSolution& s);
	
	DetachedSolution VnsOfInitialSolution(DetachedSolution& s);//VNS of InitialSolution
	DetachedSolution VndOfInitialSolution(DetachedSolution& s);//VND of InitialSolution
	DetachedSolution TwoExchangeInInitialSolution(DetachedSolution& s);//�Գ�ʼ����� 2-opt����
	DetachedSolution RelocateNodeInInitialSolution(DetachedSolution& s);//�Գ�ʼ�����relocate����
	SOLUTION  ExtractDroneRouteFromInitialSolution(DetachedSolution& s);//��ȡ���˻�·�ߡ���ȡ������ JudgeFeaibilityOfThreeNodes

	/**
	* part2->Solution
	*/
	//��ӡ���Solution
	void PrintSolution(SOLUTION& s);
	//Solution�ĳɱ����㺯��
	double FitnessOfDroneAndTruckRoute(SOLUTION& s1) {
		vector<DroneExtractionScheme> uav;
		SOLUTION s = s1;
		//·�γ��سͷ�
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
			//���з����� �� 0 1 ��һ�Ժͺ������ȡ��Ϊһ�����˻�·�ߣ�
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
			//���óͷ��ɱ�
			//1��duration penalty
			double uav_time = (distace_matrix[it->lau][it->dro] + distace_matrix[it->dro][it->rdz]) / drone_speed,
				van_time = distace_matrix[it->lau][it->rdz] / (truck_speed * conjestion_coefficient_matrix[it->lau][it->rdz]);
			double first_uav_time = distace_matrix[it->lau][it->dro] / drone_speed,
				second_uav_time = distace_matrix[it->dro][it->rdz] / drone_speed;
			//������˻�����ʱ�䳬���������ȴ�ʱ��
			if (uav_time > van_time + max_waiting_time) {
				cost += (uav_time - van_time) * 10000;
			}

			//���������仯
			//������˻�����ʱ�䳬��������ʻʱ��
			if (first_uav_time + second_uav_time > van_time) {
				if (first_uav_time * battery_consume_rate * (drone_self_weight + vertex[it->dro].demand) +
					(second_uav_time + first_uav_time) * battery_consume_rate * drone_self_weight > battery_capacity) {
					cost += ((first_uav_time * battery_consume_rate * (drone_self_weight + vertex[it->dro].demand) +
						(second_uav_time + first_uav_time) * battery_consume_rate * drone_self_weight) - battery_capacity) * 10000;
				}
			}
			//���������ʻʱ�䳬�����˻�����ʱ��
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
	/*         VNS �����         */
	SOLUTION VNS(SOLUTION& s);
	void ShakingSolution(SOLUTION& s);
	SolutionWithNeighborWeight VND(SOLUTION& s, vector<NeighborhoodWeight>& weight);
	/*        �Ŷ��Լ��ֲ���������������         */
	SOLUTION OperatorOfRelocatedAsDrone(SOLUTION& s);
	SOLUTION OperatorOfToggleService(SOLUTION& s);
	SOLUTION SwapNodeOfShaking(SOLUTION& s);
	SOLUTION RandomSwapNode(SOLUTION& s);//�������һ����
	SOLUTION RandomSwapWhole(SOLUTION& s);//�������һ���㼰�䷽ʽ
	SOLUTION RelocateWhole(SOLUTION& s);
	SOLUTION RelocateAsDrone(SOLUTION& s);
	SOLUTION RandomToggleService(SOLUTION& s);//����һ����Ϊ1
	SOLUTION RandomReverseNode(SOLUTION& s);//�����ת
	SOLUTION RandomReverseWhole(SOLUTION& s);//�����תȫ��

private:
	double battery_consume_rate = 0.2;//�������
	double max_waiting_time = 1;//���������˻������ʱ��
	const double truck_speed = 0.8;//�����ٶ�
	const double drone_speed = 1.5;//�ɻ��ٶ�
	const double truck_capacity = 20;//��������
	// ���ģ����£�������������
	const double drone_capacity = 5;//�ɻ�����
	const double drone_self_weight = 1;//�ɻ�����
	const double truck_unit_cost = 10;//�����ɱ�
	const double drone_unit_cost = 0.1;//�ɻ��ɱ�
	const double consumeRate = 0.2;//�ɻ�����������
	const double battery_capacity = 20;//�ɻ���ش�С
	double w1 = 1, w2 = 1, w3 = 1, w4 = 1, w5 = 1, w6 = 1, w7 = 1;//ÿ�������Ȩ�س�ʼֵ
	const int max_iteration = 15 * V;//����������
	double distace_matrix[V][V];//���������е�֮��ľ���
	Vertice vertex[V];//���������е������
	double conjestion_coefficient_matrix[V][V];//����ٶȾ���
};