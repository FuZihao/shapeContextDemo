#include "shapeContext.h"
#include <algorithm>
#include <vector>
#include <Limits.h>

using namespace std;

hungarian_problem::hungarian_problem(const cv::Mat& cost_mat, int mode)
{

    cost_matrix =  cost_mat;
    max_match = cost_matrix.cols;
    assignment_array.resize(max_match, 0);
    xy.resize(max_match, -1);
    yx.resize(max_match, -1);
    lx.resize(max_match, 0);
    ly.resize(max_match, 0);
    S.resize(max_match, false);
    T.resize(max_match, false);
    prev.resize(max_match, -1);

    if (!mode)
    {
    }
    for (int x = 0; x < max_match; x++)
        for (int y = 0; y < max_match; y++)
            lx[x] = max(lx[x], cost_matrix.at<int>(x, y));
}

hungarian_problem::solve()
{
    int matching = 0;

    while (matching != max_match)
    {
        int root = -1;
        int x, y;
        const int n = max_match;
        int q[max_match], wr = 0, rd = 0;
        for (x = 0; x < max_match; x++)            //finding root of the tree
            if (xy[x] == -1)
            {
                q[wr++] = root = x;
                prev[x] = -2;
                S[x] = true;
                break;
            }
        for (y = 0; y < n; y++)            //initializing slack array
        {
            slack[y] = lx[root] + ly[y] - cost[root][y];
            slackx[y] = root;
        }

        while (1)
        {
            while (rd < wr)
            {
                int x =  q[rd++];
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
            for (y = 0; y < max_match; y++)
                if (!T[y] && slack[y] == 0)
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
                            add_to_tree(yx[y] prevx:slackx[y]);
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
}

hungarian_problem::update_labels()
{
    int delta = INT_MAX;
    for (int y = 0; y < n; y++)            //calculate delta using slack
        if (!T[y])
            delta = min(delta, slack[y]);
    for (int x = 0; x < n; x++)            //update X labels
        if (S[x]) lx[x] -= delta;
    for (int y = 0; y < n; y++)            //update Y labels
        if (T[y]) ly[y] += delta;
    for (int y = 0; y < n; y++)            //update slack array
        if (!T[y])
            slack[y] -= delta;
}

hungarian_problem::add_to_tree(int x, int prevx)
{
    S[x] = true;                    //add x to S
    prev[x] = prevx;                //we need this when augmenting
    for (int y = 0; y < n; y++)    //update slacks, because we add new vertex to S
        if (lx[x] + ly[y] - cost_matrix.at<int>(x,y) < slack[y])
        {
            slack[y] = lx[x] + ly[y] - cost_matrix.at<int>(x,y);
            slackx[y] = x;
        }
}
