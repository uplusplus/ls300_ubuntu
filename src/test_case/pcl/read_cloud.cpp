/*
 * write_point.cpp
 *
 *  Created on: Jun 14, 2013
 *      Author: uplusplus
 */

#if TEST_PCL

#include <iostream>           //标准C++库中的输入输出类相关头文件。
#include <pcl/io/pcd_io.h>   //pcd 读写类相关的头文件。
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
int read_cloud()
{
	std::cout << "\rread_cloud" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1) //打开点云文件
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return (-1);
	}
	sensor_msgs::PointCloud2 cloud_blob; //PointCloud2适合版本低的点云文件
	pcl::io::loadPCDFile("test_pcd.pcd", cloud_blob);
	pcl::fromROSMsg(cloud_blob, *cloud);
	//* sensor_msgs/PointCloud2 转换为 pcl::PointCloud<T>
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y
				<< " " << cloud->points[i].z << std::endl;
	return 0;
}

#endif
