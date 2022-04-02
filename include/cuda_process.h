#ifndef CUDA_PROCESS_H
#define CUDA_PROCESS_H

void transformPc(sensor_msgs::PointCloud2& pc_ros, float x_m, float y_m, float z_m, float r_deg, float p_deg, float y_deg);

#endif