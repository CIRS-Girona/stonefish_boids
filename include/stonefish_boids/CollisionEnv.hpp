#include <stonefish_boids/OcForceTree.hpp>
#include <memory>

class CollisionEnv
{
private:
    double resolution_ = 0;

public:
    CollisionEnv(/* args */);
    ~CollisionEnv();

    std::shared_ptr<OcForceTree> collisionTree;

    void addCollisionSphere(const Eigen::Vector3f &center, double radius);
    void addCollisionSurface(const std::vector<Eigen::Vector3f> &points, const std::vector<Eigen::Vector3d> &normals);

    bool writeMaptoFile(std::string filepath);
};
