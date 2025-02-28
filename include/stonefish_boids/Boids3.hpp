#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

using Eigen::Vector3f;

struct Boid3
{
    Vector3f position;
    Vector3f velocity;
    Vector3f aceleration;
};

class Boids3
{
private:
    std::vector<Boid3> boids;
    Vector3f leaderPosition;
    double alignmentWeight = 0.0;
    double cohesionWeight = 0.0;
    double separationWeight = 0.0;
    double leaderAttractionWeight = 0.0;
    double maxSpeed = 0.0;
    double maxAccel = 0.0;
    double perceptionRadius = 0.0;
    double separationRadius = 0.0;
    double mass = 1.0;
    Vector3f groupExternalForce = Vector3f::Zero();

    // Alignment: Match the velocity of nearby boids
    // Cohesion: Steer towards the average position of nearby boids
    // Separation: Avoid crowding nearby boids
    Vector3f computeRules(const Boid3 &boid);

    // follow the leader
    Vector3f computeLeaderAttraction(const Boid3 &boid);

    // avoid going over the surface
    Vector3f computeSurfaceRepulsion(const Boid3 &boid);

    // Limit the speed of a vector to the maximum speed
    Vector3f limitSpeed(const Vector3f &velocity);

    // Limit the acceleration
    Vector3f limitAccel(const Vector3f &accel);

public:
    Boids3();
    Boids3(int numBoids, const Vector3f &initialLeaderPosition, int seed);
    void updateBoids(double dt);

    const std::vector<Boid3> &getBoids() const;

    void setLeaderPosition(const Vector3f &position);
    const Vector3f &getLeaderPosition() const;

    const double &getMass() const;

    void printSettings();

    void setAlignmentWeight(const double value);
    void setCohesionWeight(const double value);
    void setSeparationWeight(const double value);
    void setLeaderAttractionWeight(const double value);
    void setMaxSpeed(const double value);
    void setMaxAccel(const double value);
    void setPerceptionRadius(const double value);
    void setSeparationRadius(const double value);
    void setMass(const double value);

    void AddGroupExternalForce(Vector3f force, double weight);
};
