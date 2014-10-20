#ifndef SHAPE_CONTEXT_H
#define SHAPE_CONTEXT_H

#include <iostream>
#include <opencv2/core/core.hpp>
/*
void add_to_tree(int x, int prevx);
void update_labels();
cv::Mat point_assignment_for_sc(const cv::Mat& cost_matrix);
*/

class hungarian_problem {
public:
	hungarian_problem();
	hungarian_problem(const cv::Mat& cost_mat, int mode);
	~hungarian_problem();
	
	void solve();
	void print_cost_matrix();
	void print_assigment_matrix();
	void print_status();

private:
	cv::Mat cost_matrix;
	vector<int> assignment_array;

	vector<int> xy, yx;
	vector<int> lx, ly;
	vector<bool> S, T;
	vector<int> slack, slackx;
	vector<int> prev;
	int max_match;

	void update_labels();
	void add_to_tree(int x, int prevx);
};

#endif
