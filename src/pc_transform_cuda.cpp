#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "cuda_process.h"

class PcTransform{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber sub_;
		/*publisher*/
		ros::Publisher pub_;
		/*parameter*/
		double x_m_, y_m_, z_m_;
		double r_deg_, p_deg_, y_deg_;
	public:
		PcTransform();
		void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void publication(void);
};

PcTransform::PcTransform()
	: nh_private_("~")
{
	/*parameter*/
	nh_private_.param("x_m", x_m_, 0.0);
	std::cout << "x_m_ = " << x_m_ << std::endl;
	nh_private_.param("y_m", y_m_, 0.0);
	std::cout << "y_m_ = " << y_m_ << std::endl;
	nh_private_.param("z_m", z_m_, 0.0);
	std::cout << "z_m_ = " << z_m_ << std::endl;
	nh_private_.param("r_deg", r_deg_, 0.0);
	std::cout << "r_deg_ = " << r_deg_ << std::endl;
	nh_private_.param("p_deg", p_deg_, 0.0);
	std::cout << "p_deg_ = " << p_deg_ << std::endl;
	nh_private_.param("y_deg", y_deg_, 0.0);
	std::cout << "y_deg_ = " << y_deg_ << std::endl;
	/*subscriber*/
	sub_ = nh_.subscribe("/point_cloud", 1, &PcTransform::callback, this);
	/*publisher*/
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud/transformed", 1);
}

void PcTransform::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	ros::Time t_start = ros::Time::now();

	sensor_msgs::PointCloud pc1;
	sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc1);

	cudaProcess(pc1, x_m_, y_m_, z_m_, r_deg_, p_deg_, y_deg_);

	sensor_msgs::PointCloud2 pc2;
	sensor_msgs::convertPointCloudToPointCloud2(pc1, pc2);

	pub_.publish(pc2);

	std::cout << "cuda time: " << (ros::Time::now() - t_start).toSec() << " [s]" << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_transform");

	PcTransform pc_transform;

	ros::spin();
}