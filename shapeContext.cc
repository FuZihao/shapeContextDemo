#include "shapeContext.h"
#include <algorithm>
#include <vector>
#include <limits.h>
#include <iostream>

using namespace std;

hungarian_problem::hungarian_problem(const cv::Mat& cost_mat, int mode)
{
	max_match = cost_mat.cols;
	cost_matrix =  cost_mat;
	assignment_array.resize(max_match, 0);
	xy.resize(max_match, -1);
	yx.resize(max_match, -1);
	lx.resize(max_match, 0);
	ly.resize(max_match, 0);
	S.resize(max_match, false);
	T.resize(max_match, false);
	prev.resize(max_match, -1);
	slack.resize(max_match, -1);
	slackx.resize(max_match, -1);

	if (!mode)
	{
	}
	for (int x = 0; x < max_match; x++)
		for (int y = 0; y < max_match; y++)
			lx[x] = max(lx[x], cost_matrix.at<int>(x, y));
}

hungarian_problem::~hungarian_problem()
{
}

void hungarian_problem::solve()
{
	int matching = 0;

	while (matching != max_match)
	{
		int root, x, y;
		const int n = max_match;
		int q[n], wr = 0, rd = 0;
		for (x = 0; x < max_match; x++)            //finding root of the tree
			if (xy[x] == -1)
			{
				q[wr++] = root = x;
				prev[x] = -2;
				S[x] = true;
				break;
			}
		for (y = 0; y < max_match; y++)            //initializing slack array
		{
			slack[y] = lx[root] + ly[y] - cost_matrix.at<int>(root,y);
			slackx[y] = root;
		}

		while (true)
		{
			while (rd < wr)
			{
				x =  q[rd++];
				for (y = 0; y < max_match; y++)
					if (cost_matrix.at<int>(x,y) == lx[x]+ly[y] && !T[y])
					{
						if (yx[y] == -1)
							break;
						T[y] = true;
						q[wr++] = yx[y];
						add_to_tree(yx[y], x);
					}
				if (y < max_match) break;
			}
			if (y < max_match) break;

			update_labels();
			wr = rd = 0;
			for (y = 0; y < n; y++)
			{
				if (!T[y] &&  slack[y] == 0)
				{
					if (yx[y] == -1)
					{
						x = slackx[y];
						break;
					}
					else
					{
						T[y] = true;
						if (!S[yx[y]])
						{
							q[wr++] = yx[y];
							add_to_tree(yx[y], slackx[y]);
						}
					}
				}
			}
			if (y < max_match) break;
		}
		if (y < max_match)
		{
			matching++;
			for (int cx = x, cy = y, ty; cx != -2; cx = prev[cx], cy = ty)
			{
				ty = xy[cx];
				yx[cy] = cx;
				xy[cx] = cy;
			}
		}
	}
	for (int i = 0; i < max_match; i++)
		assignment_array[i] = xy[i]+1;
}

void hungarian_problem::update_labels()
{
	int delta = INT_MAX;
	for (int y = 0; y < max_match; y++)            //calculate delta using slack
		if (!T[y])
			delta = min(delta, slack[y]);
	for (int x = 0; x < max_match; x++)            //update X labels
		if (S[x]) lx[x] -= delta;
	for (int y = 0; y < max_match; y++)            //update Y labels
		if (T[y]) ly[y] += delta;
	for (int y = 0; y < max_match; y++)            //update slack array
		if (!T[y])
			slack[y] -= delta;
}

void hungarian_problem::add_to_tree(int x, int prevx)
{
	S[x] = true;                    //add x to S
	prev[x] = prevx;                //we need this when augmenting
	for (int y = 0; y < max_match; y++)    //update slacks, because we add new vertex to S
		if (lx[x] + ly[y] - cost_matrix.at<int>(x,y) < slack[y])
		{
			slack[y] = lx[x] + ly[y] - cost_matrix.at<int>(x,y);
			slackx[y] = x;
		}
}

void hungarian_problem::print_assigment_array()
{
	cout << "assignment array is : " << endl;
	cout << "[ ";
	for (int i = 0; i < assignment_array.size(); i++)
		cout << assignment_array[i];
	cout << " ]" << endl;
	cout << "total cost is: ";
	int ret = 0;
	for (int i = 0; i < max_match; i++)
	{
		ret += cost_matrix.at<int>(i,xy[i]);
	}
	cout << ret << endl;

}

void hungarian_problem::print_cost_matrix()
{
	cout << "cost matrix is : " << endl;
	cout << "[ ";
	for(int i = 0; i < cost_matrix.rows; i++)
	{
		for (int j = 0; j < cost_matrix.cols; j++)
			cout << cost_matrix.at<int>(i,j) << " ";
		if (i != cost_matrix.rows-1)
			cout << endl;
	}
	cout << " ]" << endl;
}
