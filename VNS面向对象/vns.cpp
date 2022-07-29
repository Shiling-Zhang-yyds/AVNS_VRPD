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
	return p1.second < p2.second;//��������
}

bool DescendWeightSort(const NeighborhoodWeight& p1, const NeighborhoodWeight& p2) {
	return p1.proportion > p2.proportion;//����Ȩ�ش�С�������У�Ȩ�ش������ǰ��
}

bool DescendExtractDroneCostSort(const DroneExtractionScheme& p1, const DroneExtractionScheme& p2) {
	return p1.addCost > p2.addCost;
}

bool SOLVE::JudgeFeaibilityOfThreeNodes(int l, int d, int r, vector<DroneExtractionScheme>& uav_route)//�����жϵ�ǰ3�����ܷ�������˻�·��
{
	double firstPart = distace_matrix[l][d] / drone_speed;//��һ�η���ʱ��
	double secondPart = distace_matrix[d][r] / drone_speed;//�ڶ��η���ʱ��
	double temdom = distace_matrix[l][r] / truck_speed;//��������ʻʱ��
	bool check_exist;
	for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
		if (//1��l�� �� r�㶼�Ѿ�����������·����
			it->lau == l || it->rdz == r
			) {
			return false;
		}
		if (//2��l�� ���� r�����Ѿ�������˻�·�ߵ����˻���
			it->dro == l || it->dro == r
			) {
			return false;
		}
	}

	if (firstPart + secondPart <= temdom && firstPart * (drone_self_weight + vertex[d].demand) * consumeRate +
		secondPart * drone_self_weight * consumeRate <= battery_capacity) {//���ʱ�������Լ������������㣬������
		return true;
	}
	else { return false; }
}

double SOLVE::FitnessOfTruckRoute(DetachedSolution& s) {//�����ʼ�����Ӧ��ֵ
	double cost = 0;
	double overLoad = 0;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (int j = 0; j < s.truck_route[i].size() - 1; j++) {
			cost += truck_unit_cost * distace_matrix[s.truck_route[i][j]][s.truck_route[i][j + 1]];
		}
	}
	//���㳬�صĳͷ��ɱ�
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
	int selected = 0;//��ǰѡ��ĵ�
	double load = 0;//����ۻ�����
	while (unarranged.size() != 0) {
		//map<int, double> compair;//�º���ʹmap�������أ�����valueֵ��������
		double nearDis = 0;
		vector<pair<int, double>> arr;
		for (int i = 0; i < unarranged.size(); i++) {
			nearDis = distace_matrix[selected][unarranged[i]];
			arr.push_back(make_pair(unarranged[i], nearDis));
		}
		sort(arr.begin(), arr.end(), AscendPairSort);
		auto iter = arr.begin();
		//������Ӹõ��Ƿ�ᳬ��
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
			current.push_back(iter->first);//��ž�������ĵ��ڵ�ǰ·��
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
	DetachedSolution c = s;//��ǰ��
	DetachedSolution g = s;//���Ž�
	int maxIter = max_iteration;
	int count = 0;
	double global_opt = 0;
	double local_opt = 0;
	while (count < maxIter) {
		int route1 = rand() % c.truck_route.size();
		int route2 = rand() % c.truck_route.size();
		if (route1 == route2) {//���ѡ����ǵ���·��
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
		else {//ѡ���������·�ߣ���ѡȡĳһ�λ���
			int route1First = rand() % (c.truck_route[route1].size() - 2) + 1;
			int route1Second = rand() % (c.truck_route[route1].size() - 2) + 1;

			int route2First = rand() % (c.truck_route[route2].size() - 2) + 1;
			int route2Second = rand() % (c.truck_route[route2].size() - 2) + 1;

			int Class;
			if (route1First == route1Second && route2First == route2Second) {
				Class = 0;//����·�߶���ѡ���һ����
			}
			if (route1First == route1Second && route2First != route2Second) {
				if (route2First > route2Second) {
					int temp = route2First;
					route2First = route2Second;
					route2Second = temp;
				}
				Class = 1;//route1 ѡ��һ���㣬 route2 ѡ��������
			}
			if (route1First != route1Second && route2First == route2Second) {
				if (route1First > route1Second) {
					int temp = route1First;
					route1First = route1Second;
					route1Second = temp;
				}
				Class = 2;//route1 ѡ�������㣬 route2 ѡ��һ����
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
				Class = 3;//����·�߶���ѡ���������
			}
			switch (Class) {
			case 0:
				//�����Ľ�����������
			{int temp = c.truck_route[route1][route1First];
			c.truck_route[route1][route1First] = c.truck_route[route2][route2First];
			c.truck_route[route2][route2First] = temp; }
			break;
			case 1:
				//������н���
			{int selected = c.truck_route[route1][route1First];
			c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);//�����

			for (auto it = c.truck_route[route2].begin() + route2First; it != c.truck_route[route2].begin() + route2Second; it++) {
				c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, *it);//��ת����
			}
			for (int i = route2First; i < route2Second; i++) {
				c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);//ȫ�����
			}
			c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, selected);//�����
			}
			break;
			case 2:
				//������н���
			{int selected = c.truck_route[route2][route2First];
			c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);//�����

			for (auto it = c.truck_route[route1].begin() + route1First; it != c.truck_route[route1].begin() + route1Second; it++) {
				c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, *it);//��ת����
			}
			for (int i = route1First; i < route1Second; i++) {
				c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);//ȫ�����
			}
			c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, selected);//�����
			}
			break;
			case 3:
				//���к����н���
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
				//�������
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
	DetachedSolution c = s;//��ǰ��
	DetachedSolution g = s;//���Ž�
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
	int count = 0;//��ǰ��������
	int maxIter = 40;//����������
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
	//����һ�����ϴ�����е����˻����е�
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

	vector<DroneExtractionScheme> uav_route;//���ѡ�������˻�·��

	//��ʼ������ȡ���˻�·��
	for (int i = 0; i < candidates.size(); i++) {
		bool skip = false;
		//��Ϊ����� forѭ�� �Ѿ��ڱ����ˣ����Բ���Ҫɾ����ֻ��Ҫ�����Ѿ����������˻�·���еĵ�ֱ����������
		for (auto iter1 = uav_route.begin(); iter1 != uav_route.end(); iter1++) {
			if (candidates[i] == iter1->lau || candidates[i] == iter1->dro || candidates[i] == iter1->rdz) {
				skip = true;
			}
		}
		if (skip) {
			//cout << candidates[i] << "��·�����Ѿ�����" << endl;
			;
		}
		else {
			int currentNode = candidates[i];
			double sub_cost;//�ӿ���·�����Ƴ��õ����ٵĳɱ�k
			for (int j = 0; j < s.truck_route.size(); j++) {
				for (int k = 1; k < s.truck_route[j].size() - 1; k++) {
					if (currentNode == s.truck_route[j][k]) {
						//�ӿ���·���Ƴ���ǰ���Լ�����ĳɱ�����ǰ�Ǹ���
						sub_cost = (distace_matrix[s.truck_route[j][k - 1]][s.truck_route[j][k + 1]] -
							(distace_matrix[s.truck_route[j][k - 1]][s.truck_route[j][k]] + distace_matrix[s.truck_route[j][k]][s.truck_route[j][k + 1]])) * truck_unit_cost;
						s.truck_route[j].erase(s.truck_route[j].begin() + k);//��ԭ·�����Ƴ���ǰ���˻���ѡ��
					}
				}
			}
			bool* feasi = new bool[s.truck_route.size()];//�����жϵ�ǰ�����ĳ��·��ʱ�Ƿ���
			for (int j = 0; j < s.truck_route.size(); j++) {
				double acLoad = 0;
				for (int k = 0; k < s.truck_route[j].size(); k++) {
					acLoad += vertex[s.truck_route[j][k]].demand;
				}
				if (uav_route.size() > 0) {
					for (auto iter_uav = uav_route.begin(); iter_uav != uav_route.end(); iter_uav++) {
						if (iter_uav->routeIndex == j) {
							acLoad += vertex[iter_uav->dro].demand;//����ϲ����·�������˻�������
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
			vector<DroneExtractionScheme> add_cost_m;//���������˻�֮���ܳɱ��ı仯

			for (int j = 0; j < s.truck_route.size(); j++) {
				if (feasi[j] == 1) {//�����ǰ·�߲������˻����ᳬ�أ������ѡ����

					for (int k = 0; k < s.truck_route[j].size() - 1; k++) {
						//1���жϵ�ǰѡ��Ļ��Ƿ��������˻�
						//if ()
						//���鵱ǰ��ɵ�·���Ƿ����
						/*cout << "·�� {" << s.truck_route[j][k] << " " << currentNode << " " << s.truck_route[j][k + 1] << "}" <<
							judgeFeaibility(s.truck_route[j][k], currentNode, s.truck_route[j][k + 1], uav_route) << endl;*/
						if (JudgeFeaibilityOfThreeNodes(s.truck_route[j][k], currentNode, s.truck_route[j][k + 1], uav_route)) {
							double add = (distace_matrix[s.truck_route[j][k]][currentNode] + distace_matrix[currentNode][s.truck_route[j][k + 1]]) * drone_unit_cost;
							//cout << "������ " << sub_cost << "  ������" << add << " ";
							double saving = -(add + sub_cost);
							//cout << "savings = " << saving << endl;
							if (saving >= 0) {
								add_cost_m.push_back({ s.truck_route[j][k], currentNode,s.truck_route[j][k + 1], j, k + 1, saving });
								//������	[ ��ɵ㡢���˻��㡢����㡢����·����·�����±ꡢ��Լֵ ]
							}
						}
					}
				}
			}
			sort(add_cost_m.begin(), add_cost_m.end(), DescendExtractDroneCostSort);//
			//cout << "��������� " << currentNode << " ��ɵ��������˻�·��" << endl;

			//ѡ�� ��Լֵ��� ����Ϊ���˻�·�߼���
			if (add_cost_m.size() > 0) {
				auto iter = add_cost_m.begin();
				//cout << iter->lau << "\t" << iter->dro << "\t" << iter->rdz << "\t" << iter->addCost << endl;

				uav_route.push_back(*iter);
			}
			s = s2;
			delete[] feasi;//�ͷ��ڴ�
		}

	}
	if (uav_route.size() > 0) {
		//1�����ԭ·���е����˻���
		for (int i = s1.solution.size() - 1; i >= 0; i--) {
			for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
				if (s1.solution[i].upper == it->dro) {
					s1.solution.erase(s1.solution.begin() + i);
				}
			}
		}

		//2������Щ���˻��㰴������·�����²���ԭ·��
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
	int count = 0;//��ǰ��������
	int maxIter = V;//����������
		//����һ�����飬������ŶԸ��������Ȩ��ֵ����
	vector<NeighborhoodWeight> weight_values;
	for (int i = 0; i < 8; i++) {
		weight_values.push_back({ i, 0 });//ÿ���ط���Ȩ�ض���ʼ��Ϊ0
	}

	while (count < maxIter) {
		ShakingSolution(current);
		/*cout << "ÿ�������Ȩ��Ϊ�� " << endl;
		for (int i = 0; i < 7; i++) {
			cout << "����" << weight_values[i].index << "\t" << "Ȩ�أ�" << weight_values[i].proportion << endl;
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

	/*cout << "VND���������������" << endl;
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
	//����������
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
		//����������
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
		//����������

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
		//����������
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
		//��������ĵ�
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