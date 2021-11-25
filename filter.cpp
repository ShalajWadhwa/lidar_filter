#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>

using namespace std;

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../PCD_Files/1.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    // Create the filtering object

    // Passthrough Filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1000, 0);
    pass.setFilterLimitsNegative(true);
    pass.filter(*cloud_pass);


    // Crop Box Filter
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(cloud);
    Eigen::Vector4f min_pt(0, -3.0f, -1.0f, 1.0f);
    Eigen::Vector4f max_pt(12.0f, 3.0f, 3.0f, 1.0f);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cloud_crop);


    // Visualisation
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = simpleVis(cloud);
    viewer = simpleVis(cloud_pass);
    viewer = simpleVis(cloud_crop);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return (0);
}