#include <stdio.h>

#include "cuda_process.h"

typedef struct{
	double x;
	double y;
	double z;
}Point;

typedef struct{
	double m00, m10, m20;
	double m01, m11, m21;
	double m02, m12, m22;
}Mat;

void pcRosToPtr(const sensor_msgs::PointCloud& pc_ros, Point* pc_ptr){
	for(size_t i = 0; i < pc_ros.points.size(); i++){
		pc_ptr[i].x = pc_ros.points[i].x;
		pc_ptr[i].y = pc_ros.points[i].y;
		pc_ptr[i].z = pc_ros.points[i].z;
	}
}

double degToRad(double deg)
{
	double rad = deg / 180.0 * M_PI;
	rad = atan2(sin(rad), cos(rad));
	return rad;
}

Mat getRotMatrix(double r_deg, double p_deg, double y_deg)
{
	double r_rad = degToRad(r_deg);
	double p_rad = degToRad(p_deg);
	double y_rad = degToRad(y_deg);

	Mat rot;

	// rot.m00 = cos(r_rad) * cos(y_rad) - sin(r_rad) * cos(p_rad) * sin(y_rad);
	// rot.m10 = -cos(r_rad) * sin(y_rad) - sin(r_rad) * cos(p_rad) * cos(y_rad);
	// rot.m20 = sin(r_rad) * sin(p_rad);

	// rot.m01 = sin(r_rad) * cos(y_rad) + cos(r_rad) * cos(p_rad) * sin(y_rad);
	// rot.m11 = -sin(r_rad) * sin(y_rad) + cos(r_rad) * cos(p_rad) * cos(y_rad);
	// rot.m21 = -cos(r_rad) * sin(p_rad);

	// rot.m02 = sin(p_rad) * sin(y_rad);
	// rot.m12 = sin(p_rad) * cos(y_rad);
	// rot.m22 = cos(p_rad);

	// rot.m00 = cos(r_rad) * cos(p_rad);
	// rot.m10 = cos(r_rad) * sin(p_rad) * sin(y_rad) - sin(r_rad) * cos(y_rad);
	// rot.m20 = cos(r_rad) * sin(p_rad) * cos(y_rad) + sin(r_rad) * sin(y_rad);

	// rot.m01 = sin(r_rad) * cos(p_rad);
	// rot.m11 = sin(r_rad) * sin(p_rad) * sin(y_rad) + cos(r_rad) * cos(y_rad);
	// rot.m21 = sin(r_rad) * sin(p_rad) * cos(y_rad) - cos(r_rad) * sin(y_rad);

	// rot.m02 = -sin(y_rad);
	// rot.m12 = cos(p_rad) * sin(y_rad);
	// rot.m22 = cos(p_rad) * cos(y_rad);

	rot.m00 = cos(p_rad)*cos(y_rad);
	rot.m10 = sin(r_rad)*sin(p_rad)*cos(y_rad) - cos(r_rad)*sin(y_rad);
	rot.m20 = cos(r_rad)*sin(p_rad)*cos(y_rad) + sin(r_rad)*sin(y_rad);

	rot.m01 = cos(p_rad)*sin(y_rad);
	rot.m11 = sin(r_rad)*sin(p_rad)*sin(y_rad) + cos(r_rad)*cos(y_rad);
	rot.m21 = cos(r_rad)*sin(p_rad)*sin(y_rad) - sin(r_rad)*cos(y_rad);

	rot.m02 = -sin(p_rad);
	rot.m12 = sin(r_rad)*cos(p_rad);
	rot.m22 = cos(r_rad)*cos(p_rad);

	// rot.m00 = cos(p_rad)*cos(y_rad);
	// rot.m10 = cos(p_rad)*sin(y_rad);
	// rot.m20 = -sin(p_rad);

	// rot.m01 = sin(r_rad)*sin(p_rad)*cos(y_rad) - cos(r_rad)*sin(y_rad);
	// rot.m11 = sin(r_rad)*sin(p_rad)*sin(y_rad) + cos(r_rad)*cos(y_rad);
	// rot.m21 = sin(r_rad)*cos(p_rad);

	// rot.m02 = cos(r_rad)*sin(p_rad)*cos(y_rad) + sin(r_rad)*sin(y_rad);
	// rot.m12 = cos(r_rad)*sin(p_rad)*sin(y_rad) - sin(r_rad)*cos(y_rad);
	// rot.m22 = cos(r_rad)*cos(p_rad);

	return rot;
}

__global__
void process(Point* pc_ptr, size_t num_points, double x_m, double y_m, double z_m, Mat rot)
{
	size_t index = threadIdx.x + blockIdx.x * blockDim.x;
	size_t grid_stride = gridDim.x * blockDim.x;

	for(size_t i = index; i < num_points; i += grid_stride){
		pc_ptr[i].x = rot.m00 * pc_ptr[i].x + rot.m10 * pc_ptr[i].y + rot.m20 * pc_ptr[i].z + x_m;
		pc_ptr[i].y = rot.m01 * pc_ptr[i].x + rot.m11 * pc_ptr[i].y + rot.m21 * pc_ptr[i].z + y_m;
		pc_ptr[i].z = rot.m02 * pc_ptr[i].x + rot.m12 * pc_ptr[i].y + rot.m22 * pc_ptr[i].z + z_m;
	}
}

void pcPtrToRos(Point* pc_ptr, sensor_msgs::PointCloud& pc_ros){
	for(size_t i = 0; i < pc_ros.points.size(); i++){
		pc_ros.points[i].x = pc_ptr[i].x;
		pc_ros.points[i].y = pc_ptr[i].y;
		pc_ros.points[i].z = pc_ptr[i].z;
	}
}

void cudaProcess(sensor_msgs::PointCloud& pc_ros, double x_m, double y_m, double z_m, double r_deg, double p_deg, double y_deg)
{
	/*device query*/
	int device_id;
	cudaGetDevice(&device_id);
	int num_sm;
	cudaDeviceGetAttribute(&num_sm, cudaDevAttrMultiProcessorCount, device_id);

	/*memory setting*/
	int bytes = pc_ros.points.size() * sizeof(Point);
	Point* pc_ptr;
	cudaMallocManaged(&pc_ptr, bytes);
	cudaMemPrefetchAsync(pc_ptr, bytes, device_id);

	/*cpu*/
	pcRosToPtr(pc_ros, pc_ptr);

	/*gpu*/
	int num_blocks = num_sm * 32;
	int threads_per_block = 1024;
	process<<<num_blocks, threads_per_block>>>(pc_ptr, pc_ros.points.size(), x_m, y_m, z_m, getRotMatrix(r_deg, p_deg, y_deg));

	/*cpu*/
	cudaDeviceSynchronize();
	cudaMemPrefetchAsync(pc_ptr, bytes, cudaCpuDeviceId);
	pcPtrToRos(pc_ptr, pc_ros);
	cudaFree(pc_ptr);

	printf("num_blocks * threads_per_block = %d\n", num_blocks * threads_per_block);
	printf("pc_ros.points.size() = %d\n", int(pc_ros.points.size()));
}