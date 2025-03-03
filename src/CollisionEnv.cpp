#include <stonefish_boids/CollisionEnv.hpp>

CollisionEnv::CollisionEnv()
{
    std::cout << "default collision env constructor" << "\n";

    double resolution = 0.1;
    collisionTree = std::make_shared<OcForceTree>(resolution);

    // sphere
    octomap::point3d center(0.0, 0.0, 2.0);
    double radius = 2.5;
    // double r2 = radius * radius;
    double r1 = 0;
    for (double x = center.x() - radius; x <= center.x() + radius; x += resolution / 2)
    {
        for (double y = center.y() - radius; y <= center.y() + radius; y += resolution / 2)
        {
            for (double z = center.z() - radius; z <= center.z() + radius; z += resolution / 2)
            {
                // Check if the point is inside the sphere
                double check_r = (x - center.x()) * (x - center.x()) +
                                 (y - center.y()) * (y - center.y()) +
                                 (z - center.z()) * (z - center.z());
                check_r = sqrt(check_r);
                if (r1 < check_r && check_r <= radius)
                {
                    Eigen::Vector3f normal((x - center.x()), (y - center.y()), (z - center.z()));
                    normal.stableNormalize();
                    auto node = collisionTree->updateNode(octomap::point3d(x, y, z), true, true); // Mark occupied
                    node->setForce(normal * ((1 / check_r) + __FLT_EPSILON__));
                }
            }
        }
    }

    collisionTree->updateInnerOccupancy(); // Ensure consistency in octree
    // auto suc = collisionTree->writeBinaryConst("/home/srealper/ros_ws/doc_ws/forcemap.bt");
}

CollisionEnv::~CollisionEnv()
{
    std::cout << "default collision env desconstructor" << "\n";
}