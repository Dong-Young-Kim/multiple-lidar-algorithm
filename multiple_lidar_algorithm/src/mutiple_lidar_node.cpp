#include <multiple_lidar_algorithm/node_declare.h>

ros::Publisher pub;

//short 
PCXYZI frst_lidar_buffer;
PCXYZI scnd_lidar_buffer;


void data_integrate(){
    PCXYZI integrated_data;

    integrated_data = frst_lidar_buffer + scnd_lidar_buffer;

    sensor_msgs::PointCloud2 output;                        
    pub_process(integrated_data, output);
    pub.publish(output);
    
}

void first_lidar_input(const sensor_msgs::PointCloud2ConstPtr& scan){
    RT::start();
    PCXYZI rawData;
    PCXYZI cropedData;
    pcl::fromROSMsg(*scan, rawData);

    frst_lidar_buffer = rawData;

    // sensor_msgs::PointCloud2 output;                        
    // pub_process(rawData,output);
    // pub.publish(output);

    RT::end_cal("first_lidar");
}

void second_lidar_input(const sensor_msgs::PointCloud2ConstPtr& scan){
    RT::start();
    PCXYZI rawData;
    PCXYZI cropedData;
    pcl::fromROSMsg(*scan, rawData);

    scnd_lidar_buffer = rawData;

    // sensor_msgs::PointCloud2 output;                        
    // pub_process(rawData,output);
    // pub.publish(output);

    RT::end_cal("second_lidar");


    data_integrate();
}




int main(int argc, char** argv){
	ros::init(argc, argv, "input_ROI");         //node name 
	ros::NodeHandle nh;                         //nodehandle


	ros::Subscriber sub_lidar1 = nh.subscribe<sensor_msgs::PointCloud2> ("/lidar1/velodyne_points", 1, first_lidar_input);
	ros::Subscriber sub_lidar2 = nh.subscribe<sensor_msgs::PointCloud2> ("/lidar2/velodyne_points", 1, second_lidar_input);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_integrated", 1);
    
	ros::spin();
}