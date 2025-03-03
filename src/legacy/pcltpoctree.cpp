#include <stonefish_boids/OctreeScn.hpp>

#include <chrono>
#include <thread>
#include <iostream>

// #include <pcl/io/io.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <octomap/octomap.h>

OctreeScn::OctreeScn()
{
    std::cout << "justa a simple constructor" << "\n";
}

void OctreeScn::genOctreeScn(sf::SimulationManager *sim)
{
    auto enti = sim->getEntity("Brain");
    btVector3 bbox_center;
    if (enti != NULL)
    {
        std::cout << "got entity of type " << int(enti->getType()) << "\n";

        sf::Vector3 min;
        sf::Vector3 max;
        enti->getAABB(min, max);

        std::cout << "minX " << min.getX() << " maxX " << max.getX() << "\n";
        std::cout << "minY " << min.getY() << " maxY " << max.getY() << "\n";
        std::cout << "minZ " << min.getZ() << " maxZ " << max.getZ() << "\n";

        bbox_center = (max - min) / 2;

        std::cout << "X " << bbox_center.getX() << "\n";
        std::cout << "Y " << bbox_center.getY() << "\n";
        std::cout << "Z " << bbox_center.getZ() << "\n";
    }

    // std::this_thread::sleep_for(std::chrono::seconds(3));

    pcl::PointCloud<pcl::PointXYZ> cloud_og;
    pcl::io::loadOBJFile("/home/srealper/ros_ws/doc_ws/src/mythings/stonefish_boids/resources/corals/brain.obj", cloud_og);

    // Create the filtering object
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(cloud);
    // sor.setLeafSize(0.01f, 0.01f, 0.01f);
    // sor.filter(*cloud_filtered);

    Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();
    m1(0, 3) = 5.0;
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::transformPointCloud(cloud_og, cloud1, m1);

    Eigen::Matrix4f m2 = Eigen::Matrix4f::Identity();
    m2(1, 3) = 5.0;
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    pcl::transformPointCloud(cloud_og, cloud2, m2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>(cloud_og + cloud1 + cloud2));

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(final_cloud);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 10cm
    ne.setRadiusSearch(0.1);
    // Compute the features
    ne.compute(*cloud_normals);
    // cloud_normals->size () should have the same size as the input cloud->size ()*

    std::cout << final_cloud->size() << "\n";
    std::cout << cloud_normals->size() << "\n";

    octomap::OcTree mytree(0.4);
    std::cout << "printing points" << "\n";
    for (const auto &point : final_cloud->points)
    {

        mytree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }

 

    // Save the ocmytree
    auto suc = mytree.writeBinary("/home/srealper/ros_ws/doc_ws/mymap.bt");
    std::cout << "\n";

    // see normal at z=1.4
    auto query = octomap::point3d(0.2, 0.6, 1.4);
    auto result = mytree.search(query);

    if (result != NULL)
        std::cout << "occupied? " << mytree.isNodeOccupied(result) << "\n";

    std::vector<octomap::point3d> normals;
    if (mytree.getNormals(query, normals,false))
    {
        std::cout << std::endl;
        std::string s_norm = (normals.size() > 1) ? " normals " : " normal ";
        std::cout << "MC algorithm gives " << normals.size() << s_norm << "in voxel at " << query << std::endl;
        for (unsigned i = 0; i < normals.size(); ++i)
            std::cout << "\t" << normals[i].x() << "; " << normals[i].y() << "; " << normals[i].z() << std::endl;
    }
    else
    {
        std::cout << "query point unknown (no normals)\n";
    }
}