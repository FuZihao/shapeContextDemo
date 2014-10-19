#ifndef SHAPE_CONTEXT_H
#define SHAPE_CONTEXT_H

#include <iostream>
#include <opencv2/core/core.hpp>

void update_labels();
cv::Mat point_assignment_for_sc(const cv::Mat& cost_matrix);

#endif
