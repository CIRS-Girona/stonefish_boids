#include <stonefish_boids/CollisionEnv.hpp>

CollisionEnv::CollisionEnv()
{
    std::cout << "default collision env constructor" << "\n";

    resolution_ = 0.8;
    collisionTree = std::make_shared<OcForceTree>(resolution_);
}

CollisionEnv::~CollisionEnv()
{
    std::cout << "default collision env desconstructor" << "\n";
}

void CollisionEnv::addCollisionSphere(const Eigen::Vector3f &center, double radius)
{

    // Step sizes for θ (azimuth) and φ (elevation)
    double dTheta = M_PI / 70.0; // fraction of PI increments
    double dPhi = M_PI / 70.0;   // fraction of PI increments
    double dR = resolution_ / 2; // Step along the radius

    // Iterate in spherical coordinates
    for (double r = 0; r <= radius; r += dR)
    {
        for (double theta = 0; theta < 2 * M_PI; theta += dTheta)
        {
            for (double phi = -M_PI / 2; phi <= 0; phi += dPhi)
            {
                // Convert spherical coordinates to Cartesian
                double x = center.x() + r * cos(phi) * cos(theta);
                double y = center.y() + r * cos(phi) * sin(theta);
                double z = center.z() + r * sin(phi);

                Eigen::Vector3f normal((x - center.x()), (y - center.y()), (z - center.z()));
                normal.stableNormalize();
                auto node = collisionTree->updateNode(octomap::point3d(x, y, z), true, true); // Mark occupied
                node->setForce(normal * ((1 / r) + __FLT_EPSILON__));
                // node->setForce(normal * (r));
            }
        }
    }
    collisionTree->updateInnerOccupancy(); // Ensure consistency in octree
    // collisionTree->prune();
}

void CollisionEnv::addCollisionSurface(const std::vector<Eigen::Vector3f> &points,
                                       const std::vector<Eigen::Vector3d> &normals)
{
    int i = 0;
    for (auto &point : points)
    {
        auto node = collisionTree->updateNode(octomap::point3d(point.x(), point.y(), point.z()), true, true);
        node->setForce(normals[i].cast<float>());
    }
}

bool CollisionEnv::writeMaptoFile(std::string filepath)
{
    bool suc = collisionTree->writeBinaryConst(filepath);
    std::cout << "\n";
    return suc;
}

// write map to file
// auto suc = collisionTree->writeBinaryConst("/home/srealper/ros_ws/doc_ws/forcemap.bt");