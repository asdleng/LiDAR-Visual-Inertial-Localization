/*
 * @Author: asdleng lengjianghao2006@163.com
 * @Date: 2023-06-13 16:28:47
 * @LastEditors: asdleng lengjianghao2006@163.com
 * @LastEditTime: 2023-07-16 21:43:40
 * @FilePath: /vio_in_lidar_map/src/vio_in_lidar_map/src/EuRoC_reader.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
    std::string txt_filename = "/media/i/8A3C72723C725961/EuRoC/V2_01_easy/mav0/pointcloud0/data2.txt";
    std::string pcd_filename = "output.pcd";

    // Define the point type to match the columns in the text file
    typedef pcl::PointXYZI PointType;

    // Create a point cloud
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

    // Read the text file
    std::ifstream file(txt_filename);
    if (!file.is_open())
    {
        std::cout << "Error opening file: " << txt_filename << std::endl;
        return -1;
    }

    std::string line;
    while (std::getline(file, line))
    {
        PointType point;
        std::stringstream ss(line);
        ss >> point.x >> point.y >> point.z >> point.intensity;
        cloud->push_back(point);
    }
    file.close();

    // Set the width, height, and is_dense values
    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Write the point cloud to a PCD file
    pcl::PCDWriter writer;
    if (writer.write<PointType>(pcd_filename, *cloud, true) == -1)
    {
        std::cout << "Error writing PCD file: " << pcd_filename << std::endl;
        return -1;
    }

    std::cout << "Text file converted to PCD: " << pcd_filename << std::endl;

    return 0;
}
