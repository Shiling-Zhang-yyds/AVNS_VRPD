#include "vns.h"

int main()
{
	srand((unsigned int)time(0));
	SOLVE* solve = new SOLVE();

	clock_t start, end;
	start = clock();

	//第一部分：InitialSolution
	DetachedSolution s = solve->GenerateInitialTruckRoute();
	cout << "最近搜索解：" << solve->FitnessOfTruckRoute(s) << endl;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			cout << *it << "\t";
		}
		cout << endl;
	}
	s = solve->VnsOfInitialSolution(s);
	cout << "优化后的初始解：" << solve->FitnessOfTruckRoute(s) << endl;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			cout << *it << "\t";
		}
		cout << endl;
	}

	//第二部分：Solution
	SOLUTION solut = solve->ExtractDroneRouteFromInitialSolution(s);
	cout << "提取后的初始解：" << solve->FitnessOfDroneAndTruckRoute(solut) << endl;
	solve->PrintSolution(solut);
	solut = solve->VNS(solut);
	cout << "改进后的解：" << solve->FitnessOfDroneAndTruckRoute(solut) << endl;
	solve->PrintSolution(solut);
	end = clock();
	double time = double(end - start) / CLOCKS_PER_SEC;
	cout << "总计用时 " << time << "秒" << endl;
	system("pause");
	ofstream out;

	//第三部分：输出结果
	out.open("../demo/result.txt");
	out << "目标值：" << solve->FitnessOfDroneAndTruckRoute(solut) << endl;
	out << "路线为：" << endl;
	for (int i = 0; i < solut.solution.size(); i++) {
		out << solut.solution[i].upper << "\t";
	}out << endl;
	for (int i = 0; i < solut.solution.size(); i++) {
		out << solut.solution[i].lower << "\t";
	}
	out << endl;
	out << "总计用时 " << time << "秒" << endl;
	out.close();


	return 0;
}