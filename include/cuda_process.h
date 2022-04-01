#ifndef NORMAL_ESTIMASTION_CUDA_H
#define NORMAL_ESTIMASTION_CUDA_H

#include <vector>
#include <sensor_msgs/PointCloud.h>

extern void transformPc(sensor_msgs::PointCloud& pc_ros, double x_m, double y_m, double z_m, double r_deg, double p_deg, double y_deg);

#endif