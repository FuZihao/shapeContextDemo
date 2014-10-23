#include "shapeContext.h"
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

int main()
{
	const int N = 3;
	int cost[N][N] = {{7,4,3},{3,1,2},{3,0,0}};
	Mat cost_mat(N, N, CV_32SC1,cost);

	hungarian_problem p(cost_mat, 1);
	p.print_cost_matrix();

	p.solve();
	p.print_assigment_array();

	return 0;
}
