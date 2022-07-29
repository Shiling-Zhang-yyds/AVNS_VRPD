#include "vns.h"
void SOLVE::ShakingSolution(SOLUTION& s)
{
	int i = rand() % 3;
	switch (i)
	{
	case 0:
		s = SwapNodeOfShaking(s);
		break;
	case 1:
		s = OperatorOfRelocatedAsDrone(s);
		break;
	case 2:
		s = OperatorOfToggleService(s);
		break;
	default:
		break;
	}
}

void SOLVE::PrintSolution(SOLUTION& s) {

	for (int i = 0; i < s.solution.size(); i++) {
		cout << s.solution[i].upper << "\t";
	}cout << endl;
	for (int i = 0; i < s.solution.size(); i++) {
		cout << s.solution[i].lower << "\t";
	}
	cout << endl;
}

void SOLVE::ClearEmptyRouteInInitialSolution(DetachedSolution& s) {
	for (int i = s.truck_route.size() - 1; i >= 0; i--) {
		if (s.truck_route[i].size() <= 2) {
			s.truck_route.erase(s.truck_route.begin() + i);
		}
	}
}

bool AscendPairSort(const pair<int, double>& p1, const pair<int, double>& p2)
{
	return p1.second < p2.second;//升序排列
}

bool DescendWeightSort(const NeighborhoodWeight& p1, const NeighborhoodWeight& p2) {
	return p1.proportion > p2.proportion;//根据权重大小降序排列，权重大的排在前面
}

bool DescendExtractDroneCostSort(const DroneExtractionScheme& p1, const DroneExtractionScheme& p2) {
	return p1.addCost > p2.addCost;
}

bool SOLVE::JudgeFeaibilityOfThreeNodes(int l, int d, int r, vector<DroneExtractionScheme>& uav_route)//用于判断当前3个点能否组成无人机路线
{
	double firstPart = distace_matrix[l][d] / drone_speed;//第一段飞行时间
	double secondPart = distace_matrix[d][r] / drone_speed;//第二段飞行时间
	double temdom = distace_matrix[l][r] / truck_speed;//卡车的行驶时间
	bool check_exist;
	for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
		if (//1、l点 和 r点都已经存在在现有路线中
			it->lau == l || it->rdz == r
			) {
			return false;
		}
		if (//2、l点 或者 r点是已经组成无人机路线的无人机点
			it->dro == l || it->dro == r
			) {
			return false;
		}
	}

	if (firstPart + secondPart <= temdom && firstPart * (drone_self_weight + vertex[d].demand) * consumeRate +
		secondPart * drone_self_weight * consumeRate <= battery_capacity) {//如果时间满足以及电量消耗满足，返回真
		return true;
	}
	else { return false; }
}

double SOLVE::FitnessOfTruckRoute(DetachedSolution& s) {//计算初始解的适应度值
	double cost = 0;
	double overLoad = 0;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (int j = 0; j < s.truck_route[i].size() - 1; j++) {
			cost += truck_unit_cost * distace_matrix[s.truck_route[i][j]][s.truck_route[i][j + 1]];
		}
	}
	//计算超载的惩罚成本
	for (int i = 0; i < s.truck_route.size(); i++) {
		double overLoadInOne = 0;
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			overLoadInOne += vertex[*it].demand;
		}
		if (overLoadInOne > truck_capacity) { overLoad += (overLoadInOne - truck_capacity); }
	}
	return cost + overLoad * 1000;
}

DetachedSolution SOLVE::GenerateInitialTruckRoute() {
	vector<int> unarranged;
	for (int i = 1; i < V - 1; i++) {
		unarranged.push_back(i);
	}
	vector<vector<int> > vrp;
	vector<int> current;
	current.push_back(0);
	int selected = 0;//当前选择的点
	double load = 0;//存放累积容量
	while (unarranged.size() != 0) {
		//map<int, double> compair;//仿函数使map发生重载，按照value值升序排列
		double nearDis = 0;
		vector<pair<int, double>> arr;
		for (int i = 0; i < unarranged.size(); i++) {
			nearDis = distace_matrix[selected][unarranged[i]];
			arr.push_back(make_pair(unarranged[i], nearDis));
		}
		sort(arr.begin(), arr.end(), AscendPairSort);
		auto iter = arr.begin();
		//计算添加该点是否会超载
		load += vertex[iter->first].demand;
		if (load > truck_capacity) {
			current.push_back(0);
			if (current[0] != 0) {
				current.insert(current.begin(), 0);
			}
			vrp.push_back(current);
			current.clear();
			load = 0;
			selected = 0;
		}
		else {
			current.push_back(iter->first);//存放距离最近的点在当前路线
			selected = iter->first;
			for (int i = unarranged.size() - 1; i >= 0; i--) {
				if (unarranged[i] == iter->first) {
					unarranged.erase(unarranged.begin() + i);
					break;
				}
			}
		}
	}
	current.push_back(0);
	auto start = current.begin();
	if (*start != 0) { current.insert(current.begin(), 0); }
	vrp.push_back(current);
	DetachedSolution s;
	for (int i = 0; i < vrp.size(); i++) {
		s.truck_route.push_back(vrp[i]);
	}
	return s;
}

DetachedSolution SOLVE::TwoExchangeInInitialSolution(DetachedSolution& s) {
	DetachedSolution c = s;//当前解
	DetachedSolution g = s;//最优解
	int maxIter = max_iteration;
	int count = 0;
	double global_opt = 0;
	double local_opt = 0;
	while (count < maxIter) {
		int route1 = rand() % c.truck_route.size();
		int route2 = rand() % c.truck_route.size();
		if (route1 == route2) {//如果选择的是单条路线
			int first = rand() % (c.truck_route[route1].size() - 2) + 1;
			int second = rand() % (c.truck_route[route1].size() - 2) + 1;
			if (first > second) {
				int temp = first;
				first = second;
				second = temp;
			}
			reverse(c.truck_route[route1].begin() + first, c.truck_route[route1].begin() + second);
			if (FitnessOfTruckRoute(c) < FitnessOfTruckRoute(g)) {
				g = c;
				c = g;
				global_opt = FitnessOfTruckRoute(c);
				count = 0;
			}
			else {
				count++;
			}
		}
		else {//选择的是两条路线，则选取某一段互换
			int route1First = rand() % (c.truck_route[route1].size() - 2) + 1;
			int route1Second = rand() % (c.truck_route[route1].size() - 2) + 1;

			int route2First = rand() % (c.truck_route[route2].size() - 2) + 1;
			int route2Second = rand() % (c.truck_route[route2].size() - 2) + 1;

			int Class;
			if (route1First == route1Second && route2First == route2Second) {
				Class = 0;//两条路线都是选择的一个点
			}
			if (route1First == route1Second && route2First != route2Second) {
				if (route2First > route2Second) {
					int temp = route2First;
					route2First = route2Second;
					route2Second = temp;
				}
				Class = 1;//route1 选了一个点， route2 选了两个点
			}
			if (route1First != route1Second && route2First == route2Second) {
				if (route1First > route1Second) {
					int temp = route1First;
					route1First = route1Second;
					route1Second = temp;
				}
				Class = 2;//route1 选了两个点， route2 选了一个点
			}
			if (route1First != route1Second && route2First != route2Second) {
				if (route2First > route2Second) {
					int temp = route2First;
					route2First = route2Second;
					route2Second = temp;
				}
				if (route1First > route1Second) {
					int temp = route1First;
					route1First = route1Second;
					route1Second = temp;
				}
				Class = 3;//两条路线都是选择的两个点
			}
			switch (Class) {
			case 0:
				//单纯的交换这两个点
			{int temp = c.truck_route[route1][route1First];
			c.truck_route[route1][route1First] = c.truck_route[route2][route2First];
			c.truck_route[route2][route2First] = temp; }
			break;
			case 1:
				//点和序列交换
			{int selected = c.truck_route[route1][route1First];
			c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);//清除点

			for (auto it = c.truck_route[route2].begin() + route2First; it != c.truck_route[route2].begin() + route2Second; it++) {
				c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, *it);//反转插入
			}
			for (int i = route2First; i < route2Second; i++) {
				c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);//全部清除
			}
			c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, selected);//插入点
			}
			break;
			case 2:
				//点和序列交换
			{int selected = c.truck_route[route2][route2First];
			c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);//清除点

			for (auto it = c.truck_route[route1].begin() + route1First; it != c.truck_route[route1].begin() + route1Second; it++) {
				c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, *it);//反转插入
			}
			for (int i = route1First; i < route1Second; i++) {
				c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);//全部清除
			}
			c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, selected);//插入点
			}
			break;
			case 3:
				//序列和序列交换
			{
				vector<int> tempRoute1, tempRoute2;
				for (int i = route1First; i <= route1Second; i++) {
					tempRoute1.push_back(c.truck_route[route1][i]);
				}
				for (int i = route2First; i <= route2Second; i++) {
					tempRoute2.push_back(c.truck_route[route2][i]);
				}

				for (int i = route1First; i <= route1Second; i++) {
					c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);
				}
				for (int i = route2First; i <= route2Second; i++) {
					c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);
				}
				//逆向插入
				for (auto it = tempRoute1.begin(); it != tempRoute1.end(); it++) {
					c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, *it);
				}
				for (auto it = tempRoute2.begin(); it != tempRoute2.end(); it++) {
					c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, *it);
				}
			}
			break;
			default:break;
			}
			if (FitnessOfTruckRoute(c) < FitnessOfTruckRoute(g)) {
				g = c;
				c = g;
				global_opt = FitnessOfTruckRoute(c);
				count = 0;
			}
			else { count++; }
		}
	}
	return g;
}

DetachedSolution SOLVE::RelocateNodeInInitialSolution(DetachedSolution& s) {
	DetachedSolution c = s;//当前解
	DetachedSolution g = s;//最优解
	int maxIter = max_iteration;
	int count = 0;
	double global_opt = 0;
	double local_opt = 0;
	while (count < maxIter) {
		int route1 = rand() % c.truck_route.size();
		int route2 = rand() % c.truck_route.size();
		while (c.truck_route[route1].size() < 4 || c.truck_route[route2].size() < 4) {
			route1 = rand() % c.truck_route.size();
			route2 = rand() % c.truck_route.size();
		}
		int index1 = rand() % (c.truck_route[route1].size() - 2) + 1;
		int index2 = rand() % (c.truck_route[route2].size() - 2) + 1;
		int selected = c.truck_route[route1][index1];
		c.truck_route[route1].erase(c.truck_route[route1].begin() + index1);
		c.truck_route[route2].insert(c.truck_route[route2].begin() + index2, selected);
		if (FitnessOfTruckRoute(c) < FitnessOfTruckRoute(g)) {
			g = c;
			count = 0;
		}
		else {
			count++;
			c = g;
		}
	}
	ClearEmptyRouteInInitialSolution(g);
	return g;
}

DetachedSolution SOLVE::VndOfInitialSolution(DetachedSolution& s) {
	int index = 0;
	bool noExit = 1;
	DetachedSolution current = s;
	DetachedSolution best = s;
	while (noExit) {
		switch (index) {
		case 0:
			current = TwoExchangeInInitialSolution(current);
			if (FitnessOfTruckRoute(current) < FitnessOfTruckRoute(best)) {
				best = current;
				index = 0;
			}
			else {
				current = best;
				index++;
			}
			break;
		case 1:
			current = RelocateNodeInInitialSolution(current);
			if (FitnessOfTruckRoute(current) < FitnessOfTruckRoute(best)) {
				best = current;
				index = 0;
			}
			else {
				current = best;
				index++;
			}
			break;
		default:
			noExit = 0;
			break;
		}
	}
	return best;
}

DetachedSolution SOLVE::VnsOfInitialSolution(DetachedSolution& s) {
	DetachedSolution best = s;
	DetachedSolution current = s;
	int count = 0;//当前迭代次数
	int maxIter = 40;//最大迭代次数
	while (count < maxIter) {
		//current = shaking(current);
		current = VndOfInitialSolution(current);
		if (FitnessOfTruckRoute(current) < FitnessOfTruckRoute(best)) {
			best = current;
			count = 0;
		}
		else {
			current = best;
			count++;
		}
	}
	return best;
}

SOLUTION SOLVE::ExtractDroneRouteFromInitialSolution(DetachedSolution& s) {
	//建立一个集合存放所有的无人机可行点
	vector<int> candidates;
	DetachedSolution s2 = s;
	SOLUTION s1;
	s1.solution.push_back({ 0, 0 });

	for (int i = 0; i < s.truck_route.size(); i++) {
		for (int j = 1; j < s.truck_route[i].size(); j++) {
			s1.solution.push_back({ s.truck_route[i][j], 0 });
			if (s.truck_route[i][j] != 0 && vertex[s.truck_route[i][j]].demand <= drone_capacity) {
				candidates.push_back(s.truck_route[i][j]);
			}
		}
	}

	vector<DroneExtractionScheme> uav_route;//存放选定的无人机路线

	//开始尝试提取无人机路线
	for (int i = 0; i < candidates.size(); i++) {
		bool skip = false;
		//因为最外层 for循环 已经在遍历了，所以不需要删除，只需要遇到已经存在在无人机路线中的点直接跳过即可
		for (auto iter1 = uav_route.begin(); iter1 != uav_route.end(); iter1++) {
			if (candidates[i] == iter1->lau || candidates[i] == iter1->dro || candidates[i] == iter1->rdz) {
				skip = true;
			}
		}
		if (skip) {
			//cout << candidates[i] << "在路线中已经有了" << endl;
			;
		}
		else {
			int currentNode = candidates[i];
			double sub_cost;//从卡车路线中移除该点后减少的成本k
			for (int j = 0; j < s.truck_route.size(); j++) {
				for (int k = 1; k < s.truck_route[j].size() - 1; k++) {
					if (currentNode == s.truck_route[j][k]) {
						//从卡车路线移除当前点节约出来的成本，当前是负数
						sub_cost = (distace_matrix[s.truck_route[j][k - 1]][s.truck_route[j][k + 1]] -
							(distace_matrix[s.truck_route[j][k - 1]][s.truck_route[j][k]] + distace_matrix[s.truck_route[j][k]][s.truck_route[j][k + 1]])) * truck_unit_cost;
						s.truck_route[j].erase(s.truck_route[j].begin() + k);//从原路线中移除当前无人机待选点
					}
				}
			}
			bool* feasi = new bool[s.truck_route.size()];//用于判断当前点插入某条路径时是否超载
			for (int j = 0; j < s.truck_route.size(); j++) {
				double acLoad = 0;
				for (int k = 0; k < s.truck_route[j].size(); k++) {
					acLoad += vertex[s.truck_route[j][k]].demand;
				}
				if (uav_route.size() > 0) {
					for (auto iter_uav = uav_route.begin(); iter_uav != uav_route.end(); iter_uav++) {
						if (iter_uav->routeIndex == j) {
							acLoad += vertex[iter_uav->dro].demand;//添加上插入该路径的无人机的重量
						}
					}
				}
				if (vertex[currentNode].demand + acLoad <= truck_capacity) {
					feasi[j] = true;
				}
				else {
					feasi[j] = false;
				}
			}
			vector<DroneExtractionScheme> add_cost_m;//存放添加无人机之后总成本的变化

			for (int j = 0; j < s.truck_route.size(); j++) {
				if (feasi[j] == 1) {//如果当前路线插入无人机不会超载，则可以选择尝试

					for (int k = 0; k < s.truck_route[j].size() - 1; k++) {
						//1、判断当前选择的弧是否已有无人机
						//if ()
						//检验当前组成的路线是否可行
						/*cout << "路线 {" << s.truck_route[j][k] << " " << currentNode << " " << s.truck_route[j][k + 1] << "}" <<
							judgeFeaibility(s.truck_route[j][k], currentNode, s.truck_route[j][k + 1], uav_route) << endl;*/
						if (JudgeFeaibilityOfThreeNodes(s.truck_route[j][k], currentNode, s.truck_route[j][k + 1], uav_route)) {
							double add = (distace_matrix[s.truck_route[j][k]][currentNode] + distace_matrix[currentNode][s.truck_route[j][k + 1]]) * drone_unit_cost;
							//cout << "减少了 " << sub_cost << "  增加了" << add << " ";
							double saving = -(add + sub_cost);
							//cout << "savings = " << saving << endl;
							if (saving >= 0) {
								add_cost_m.push_back({ s.truck_route[j][k], currentNode,s.truck_route[j][k + 1], j, k + 1, saving });
								//依次是	[ 起飞点、无人机点、降落点、所在路径、路径中下标、节约值 ]
							}
						}
					}
				}
			}
			sort(add_cost_m.begin(), add_cost_m.end(), DescendExtractDroneCostSort);//
			//cout << "输出所有与 " << currentNode << " 组成的最优无人机路线" << endl;

			//选择 节约值最大 的作为无人机路线加入
			if (add_cost_m.size() > 0) {
				auto iter = add_cost_m.begin();
				//cout << iter->lau << "\t" << iter->dro << "\t" << iter->rdz << "\t" << iter->addCost << endl;

				uav_route.push_back(*iter);
			}
			s = s2;
			delete[] feasi;//释放内存
		}

	}
	if (uav_route.size() > 0) {
		//1、清楚原路线中的无人机点
		for (int i = s1.solution.size() - 1; i >= 0; i--) {
			for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
				if (s1.solution[i].upper == it->dro) {
					s1.solution.erase(s1.solution.begin() + i);
				}
			}
		}

		//2、将这些无人机点按照最优路线重新插入原路线
		for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
			for (int i = 0; i <= s1.solution.size() - 2; i++) {
				if (it->lau == s1.solution[i].upper && it->rdz == s1.solution[i + 1].upper) {
					s1.solution.insert(s1.solution.begin() + i + 1, { it->dro, 1 });
				}

			}
		}
	}

	return s1;
}

SOLUTION SOLVE::VNS(SOLUTION& s) {
	SolutionWithNeighborWeight best, current;
	best.solution = s.solution;
	current.solution = s.solution;
	int count = 0;//当前迭代次数
	int maxIter = V;//最大迭代次数
		//创建一个数组，用来存放对各个邻域的权重值计算
	vector<NeighborhoodWeight> weight_values;
	for (int i = 0; i < 8; i++) {
		weight_values.push_back({ i, 0 });//每个地方的权重都初始化为0
	}

	while (count < maxIter) {
		ShakingSolution(current);
		/*cout << "每个邻域的权重为： " << endl;
		for (int i = 0; i < 7; i++) {
			cout << "邻域：" << weight_values[i].index << "\t" << "权重：" << weight_values[i].proportion << endl;
		}*/
		current = VND(current, weight_values);
		if (FitnessOfDroneAndTruckRoute(current) < FitnessOfDroneAndTruckRoute(best)) {
			best = current;
			for (auto it2 = 0; it2 != current.sorting.size(); it2++) {
				weight_values[it2].index = current.sorting[it2].index;
				weight_values[it2].proportion = current.sorting[it2].proportion;
			}
			count = 0;
		}
		else {
			current = best;
			count++;
		}
	}

	return best;
}

SolutionWithNeighborWeight SOLVE::VND(SOLUTION& s, vector<NeighborhoodWeight>& values) {

	bool noExit = 1;
	SOLUTION current = s;
	SOLUTION best = s;
	vector<NeighborhoodWeight> tempValues;
	tempValues = values;
	double accumulation[7] = { 0,0,0,0,0,0,0 };
	sort(tempValues.begin(), tempValues.end(), DescendWeightSort);

	/*cout << "VND中邻域的排序如下" << endl;
	for (auto it = tempValues.begin(); it != tempValues.end(); it++) {
		cout << it->index << "--" << it->proportion << endl;
	}*/
	auto it = tempValues.begin();
	while (noExit) {
		switch (it->index) {
		case 0:
			current = RandomSwapNode(current);
			if (FitnessOfDroneAndTruckRoute(current) < FitnessOfDroneAndTruckRoute(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				best = current;
				accumulation[it->index] += 1;
				it = tempValues.begin();

				//cout << "RandomSwapNode" << endl;

			}
			else {
				current = best;

				it++;
			}
			break;
		case 1:
			current = RandomSwapWhole(current);
			if (FitnessOfDroneAndTruckRoute(current) < FitnessOfDroneAndTruckRoute(best)) {
				best = current;
				accumulation[it->index] += 1;
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				it = tempValues.begin();
				//cout << "RandomSwapWhole" << endl;

			}
			else {
				current = best;

				it++;
			}
			break;
		case 2:
			current = RelocateWhole(current);
			if (FitnessOfDroneAndTruckRoute(current) < FitnessOfDroneAndTruckRoute(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				best = current;
				accumulation[it->index] += 1;
				it = tempValues.begin();
				//cout << "Relocate" << endl;


			}
			else {
				current = best;

				it++;
			}
			break;
		case 3:
			current = RelocateAsDrone(current);
			if (FitnessOfDroneAndTruckRoute(current) < FitnessOfDroneAndTruckRoute(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				accumulation[it->index] += 1;

				best = current;
				//cout << "RelocateAsDrone" << endl;


				it = tempValues.begin();
			}
			else {
				current = best;

				it++;
			}
			break;
		case 4:
			current = RandomToggleService(current);
			if (FitnessOfDroneAndTruckRoute(current) < FitnessOfDroneAndTruckRoute(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				accumulation[it->index] += 1;
				it = tempValues.begin();
				best = current;
				//cout << "AddDroneSortie" << endl;


			}
			else {
				current = best;

				it++;
			}
			break;
		case 5:
			current = RandomReverseNode(current);
			if (FitnessOfDroneAndTruckRoute(current) < FitnessOfDroneAndTruckRoute(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				accumulation[it->index] += 1;
				it = tempValues.begin();
				best = current;
				//cout << "RandomReverseWhole" << endl;


			}
			else {
				current = best;

				it++;
			}
			break;
		case 6:
			current = RandomReverseWhole(current);
			if (FitnessOfDroneAndTruckRoute(current) < FitnessOfDroneAndTruckRoute(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				accumulation[it->index] += 1;
				it = tempValues.begin();
				best = current;
				//cout << "RandomReverseNode" << endl;


			}
			else {
				current = best;

				it++;
			}
			break;
		default:
			noExit = 0;
			break;
		}
	}

	for (int i = 0; i < 7; i++) {
		for (auto it1 = tempValues.begin(); it1 != tempValues.end(); it1++) {
			if (i == it->index) {
				it->proportion = accumulation[i];
			}
		}
	}
	SolutionWithNeighborWeight ui;
	ui.solution = best.solution;
	ui.sorting = tempValues;
	values = tempValues;
	return ui;
}

SOLUTION SOLVE::RelocateWhole(SOLUTION& s1) {

	int count = 0;
	int maxIter = 200;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		while (s.solution[first].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
		}
		SolutionElement temp = s.solution[first];
		s.solution.erase(s.solution.begin() + first);
		int second = rand() % (s.solution.size() - 1) + 1;
		while (first == second) {
			second = rand() % (s.solution.size() - 1) + 1;
		}
		s.solution.insert(s.solution.begin() + second, temp);
		if (FitnessOfDroneAndTruckRoute(s) < FitnessOfDroneAndTruckRoute(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION SOLVE::OperatorOfRelocatedAsDrone(SOLUTION& s) {
	int first = rand() % (s.solution.size() - 2) + 1;
	while (s.solution[first].upper == 0) {
		first = rand() % (s.solution.size() - 2) + 1;
	}
	SolutionElement temp = s.solution[first];
	temp.lower = 1;
	s.solution.erase(s.solution.begin() + first);
	int second = rand() % (s.solution.size() - 1) + 1;
	while (first == second) {
		second = rand() % (s.solution.size() - 1) + 1;
	}
	s.solution.insert(s.solution.begin() + second, temp);
	return s;
}

SOLUTION SOLVE::RelocateAsDrone(SOLUTION& s1) {

	int count = 0;
	int maxIter = 100;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		s = OperatorOfRelocatedAsDrone(s);
		if (FitnessOfDroneAndTruckRoute(s) < FitnessOfDroneAndTruckRoute(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION SOLVE::OperatorOfToggleService(SOLUTION& s) {
	int first = rand() % (s.solution.size() - 2) + 1;
	while (s.solution[first].upper == 0) {
		first = rand() % (s.solution.size() - 2) + 1;
	}
	if (s.solution[first].lower == 0) {
		s.solution[first].lower = 1;
	}
	else {
		s.solution[first].lower = 0;
	}
	return s;
}

SOLUTION SOLVE::RandomToggleService(SOLUTION& s1) {
	int count = 0;
	int maxIter = 50;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		s = OperatorOfToggleService(s);
		if (FitnessOfDroneAndTruckRoute(s) < FitnessOfDroneAndTruckRoute(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION SOLVE::SwapNodeOfShaking(SOLUTION& s) {
	int first = rand() % (s.solution.size() - 2) + 1;
	int second = rand() % (s.solution.size() - 2) + 1;
	while (first == second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
		first = rand() % (s.solution.size() - 2) + 1;
		second = rand() % (s.solution.size() - 2) + 1;
	}
	//交换两个点
	int temp = s.solution[first].upper;
	s.solution[first].upper = s.solution[second].upper;
	s.solution[second].upper = temp;
	return s;
}

SOLUTION SOLVE::RandomSwapNode(SOLUTION& s1) {

	int count = 0;
	int maxIter = max_iteration;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		int second = rand() % (s.solution.size() - 2) + 1;
		while (first == second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
			second = rand() % (s.solution.size() - 2) + 1;
		}
		//交换两个点
		int temp = s.solution[first].upper;
		s.solution[first].upper = s.solution[second].upper;
		s.solution[second].upper = temp;
		if (FitnessOfDroneAndTruckRoute(s) < FitnessOfDroneAndTruckRoute(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION SOLVE::RandomSwapWhole(SOLUTION& s1) {
	int count = 0;
	int maxIter = max_iteration;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		int second = rand() % (s.solution.size() - 2) + 1;
		while (first == second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
			second = rand() % (s.solution.size() - 2) + 1;
		}
		//交换两个点

		SolutionElement temp1 = s.solution[first];
		s.solution[first] = s.solution[second];
		s.solution[second] = temp1;

		if (FitnessOfDroneAndTruckRoute(s) < FitnessOfDroneAndTruckRoute(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION SOLVE::RandomReverseNode(SOLUTION& s1) {
	int count = 0;
	int maxIter = max_iteration;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		int second = rand() % (s.solution.size() - 2) + 1;
		while (first >= second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
			second = rand() % (s.solution.size() - 2) + 1;
		}
		//交换两个点
		reverse(s.solution.begin() + first, s.solution.begin() + second);


		if (FitnessOfDroneAndTruckRoute(s) < FitnessOfDroneAndTruckRoute(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION SOLVE::RandomReverseWhole(SOLUTION& s1) {
	int count = 0;
	int maxIter = max_iteration;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		int second = rand() % (s.solution.size() - 2) + 1;
		while (first >= second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
			second = rand() % (s.solution.size() - 2) + 1;
		}
		//逆序上面的点
		vector<int> temp;
		for (int i = 0; i < first; i++) {
			temp.push_back(s.solution[i].upper);
		}
		for (int i = second; i >= first; i--) {
			temp.push_back(s.solution[i].upper);
		}
		for (int i = second + 1; i < s.solution.size(); i++) {
			temp.push_back(s.solution[i].upper);
		}
		for (int i = 0; i < s.solution.size(); i++) {
			s.solution[i].upper = temp[i];
			if (s.solution[i].upper == 0) {
				s.solution[i].lower = 0;
			}
		}
		if (FitnessOfDroneAndTruckRoute(s) < FitnessOfDroneAndTruckRoute(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}