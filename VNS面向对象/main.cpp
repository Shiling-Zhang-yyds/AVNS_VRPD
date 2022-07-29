#include "vns.h"

int main()
{
	srand((unsigned int)time(0));
	SOLVE* solve = new SOLVE();

	clock_t start, end;
	start = clock();

	//��һ���֣�InitialSolution
	DetachedSolution s = solve->GenerateInitialTruckRoute();
	cout << "��������⣺" << solve->FitnessOfTruckRoute(s) << endl;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			cout << *it << "\t";
		}
		cout << endl;
	}
	s = solve->VnsOfInitialSolution(s);
	cout << "�Ż���ĳ�ʼ�⣺" << solve->FitnessOfTruckRoute(s) << endl;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			cout << *it << "\t";
		}
		cout << endl;
	}

	//�ڶ����֣�Solution
	SOLUTION solut = solve->ExtractDroneRouteFromInitialSolution(s);
	cout << "��ȡ��ĳ�ʼ�⣺" << solve->FitnessOfDroneAndTruckRoute(solut) << endl;
	solve->PrintSolution(solut);
	solut = solve->VNS(solut);
	cout << "�Ľ���Ľ⣺" << solve->FitnessOfDroneAndTruckRoute(solut) << endl;
	solve->PrintSolution(solut);
	end = clock();
	double time = double(end - start) / CLOCKS_PER_SEC;
	cout << "�ܼ���ʱ " << time << "��" << endl;
	system("pause");
	ofstream out;

	//�������֣�������
	out.open("../demo/result.txt");
	out << "Ŀ��ֵ��" << solve->FitnessOfDroneAndTruckRoute(solut) << endl;
	out << "·��Ϊ��" << endl;
	for (int i = 0; i < solut.solution.size(); i++) {
		out << solut.solution[i].upper << "\t";
	}out << endl;
	for (int i = 0; i < solut.solution.size(); i++) {
		out << solut.solution[i].lower << "\t";
	}
	out << endl;
	out << "�ܼ���ʱ " << time << "��" << endl;
	out.close();


	return 0;
}