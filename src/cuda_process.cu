#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>

typedef struct{
	float x;
	float y;
	float z;
}Point;

typedef struct{
	float m00, m10, m20,
		m01, m11, m21,
		m02, m12, m22;
}Mat;

void pcRosToPtr(const sensor_msgs::PointCloud2& pc_ros, Point* pc_ptr){
	int x_idx, y_idx, z_idx;
	for(size_t i = 0; i < pc_ros.fields.size(); ++i){
		if(pc_ros.fields[i].name == "x")	x_idx = i;
		else if(pc_ros.fields[i].name == "y")	y_idx = i;
		else if(pc_ros.fields[i].name == "z")	z_idx = i;
	}
	int x_offset = pc_ros.fields[x_idx].offset;
	int y_offset = pc_ros.fields[y_idx].offset;
	int z_offset = pc_ros.fields[z_idx].offset;
	uint8_t x_datatype = pc_ros.fields[x_idx].datatype;
	uint8_t y_datatype = pc_ros.fields[y_idx].datatype;
	uint8_t z_datatype = pc_ros.fields[z_idx].datatype;

	for (size_t i = 0; i < pc_ros.height * pc_ros.width; ++i){
		pc_ptr[i].x = sensor_msgs::readPointCloud2BufferValue<float>(&pc_ros.data[i * pc_ros.point_step + x_offset], x_datatype);
		pc_ptr[i].y = sensor_msgs::readPointCloud2BufferValue<float>(&pc_ros.data[i * pc_ros.point_step + y_offset], y_datatype);
		pc_ptr[i].z = sensor_msgs::readPointCloud2BufferValue<float>(&pc_ros.data[i * pc_ros.point_step + z_offset], z_datatype);
	}
}

float degToRad(float deg)
{
	float rad = deg / 180.0 * M_PI;
	rad = atan2(sin(rad), cos(rad));
	return rad;
}

Mat getRotMatrix(float r_deg, float p_deg, float y_deg)
{
	float r_rad = degToRad(r_deg);
	float p_rad = degToRad(p_deg);
	float y_rad = degToRad(y_deg);

	Mat rot;

	rot.m00 = cos(p_rad)*cos(y_rad);
	rot.m10 = sin(r_rad)*sin(p_rad)*cos(y_rad) - cos(r_rad)*sin(y_rad);
	rot.m20 = cos(r_rad)*sin(p_rad)*cos(y_rad) + sin(r_rad)*sin(y_rad);

	rot.m01 = cos(p_rad)*sin(y_rad);
	rot.m11 = sin(r_rad)*sin(p_rad)*sin(y_rad) + cos(r_rad)*cos(y_rad);
	rot.m21 = cos(r_rad)*sin(p_rad)*sin(y_rad) - sin(r_rad)*cos(y_rad);

	rot.m02 = -sin(p_rad);
	rot.m12 = sin(r_rad)*cos(p_rad);
	rot.m22 = cos(r_rad)*cos(p_rad);

	return rot;
}

__global__
void transformPcCuda(Point* pc_ptr, size_t num_points, float x_m, float y_m, float z_m, Mat rot)
{
	size_t index = threadIdx.x + blockIdx.x * blockDim.x;
	size_t grid_stride = gridDim.x * blockDim.x;

	for(size_t i = index; i < num_points; i += grid_stride){
		Point tmp;
		tmp.x = rot.m00 * pc_ptr[i].x + rot.m10 * pc_ptr[i].y + rot.m20 * pc_ptr[i].z + x_m;
		tmp.y = rot.m01 * pc_ptr[i].x + rot.m11 * pc_ptr[i].y + rot.m21 * pc_ptr[i].z + y_m;
		tmp.z = rot.m02 * pc_ptr[i].x + rot.m12 * pc_ptr[i].y + rot.m22 * pc_ptr[i].z + z_m;
		pc_ptr[i] = tmp;
	}
}

void pcPtrToRos(Point* pc_ptr, sensor_msgs::PointCloud2& pc_ros){
	for(size_t i = 0; i < pc_ros.height * pc_ros.width; ++i){
		memcpy(&pc_ros.data[i * pc_ros.point_step + pc_ros.fields[0].offset], &pc_ptr[i].x, sizeof(float));
		memcpy(&pc_ros.data[i * pc_ros.point_step + pc_ros.fields[1].offset], &pc_ptr[i].y, sizeof(float));
		memcpy(&pc_ros.data[i * pc_ros.point_step + pc_ros.fields[2].offset], &pc_ptr[i].z, sizeof(float));
	}
}

void transformPc(sensor_msgs::PointCloud2& pc_ros, float x_m, float y_m, float z_m, float r_deg, float p_deg, float y_deg)
{
	/*device query*/
	int device_id;
	cudaGetDevice(&device_id);
	int num_sm;
	cudaDeviceGetAttribute(&num_sm, cudaDevAttrMultiProcessorCount, device_id);

	/*memory setting*/
	int num_points = pc_ros.height * pc_ros.width;
	int bytes = num_points * sizeof(Point);
	Point* pc_ptr;
	cudaMallocManaged(&pc_ptr, bytes);
	cudaMemPrefetchAsync(pc_ptr, bytes, device_id);

	/*cpu*/
	pcRosToPtr(pc_ros, pc_ptr);

	// /*gpu*/
	int num_blocks = num_sm * 32;
	int threads_per_block = 1024;
	transformPcCuda<<<num_blocks, threads_per_block>>>(pc_ptr, num_points, x_m, y_m, z_m, getRotMatrix(r_deg, p_deg, y_deg));
	cudaDeviceSynchronize();

	// /*cpu*/
	cudaMemPrefetchAsync(pc_ptr, bytes, cudaCpuDeviceId);
	pcPtrToRos(pc_ptr, pc_ros);
	cudaFree(pc_ptr);

	// std::cout << "num_blocks = " << num_blocks << std::endl;
	// std::cout << "threads_per_block = " << threads_per_block << std::endl;
	// std::cout << "num_blocks * threads_per_block = " << num_blocks * threads_per_block << std::endl;
	// std::cout << "num_points = " << num_points << std::endl;
}