#include <stonefish_boids/Boids2.hpp>

#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

using Eigen::Vector3f;

// Alignment: Match the velocity of nearby boids
Vector3f Boids2::computeAlignment(const Boid2 &boid)
{
    Vector3f avgVelocity = Vector3f::Zero();
    int count = 0;

    for (const auto &other : boids)
    {
        if ((boid.position - other.position).norm() < perceptionRadius && &boid != &other)
        {
            avgVelocity += other.velocity;
            count++;
        }
    }

    return (count > 0) ? (avgVelocity / count).normalized() : Vector3f::Zero();
    // if (count > 0)
    //     return avgVelocity / float(count);
    // else
    //     return Vector3f::Zero();
}

// Cohesion: Steer towards the average position of nearby boids
Vector3f Boids2::computeCohesion(const Boid2 &boid)
{
    Vector3f avgPosition = Vector3f::Zero();
    int count = 0;

    for (const auto &other : boids)
    {
        if ((boid.position - other.position).norm() < perceptionRadius && &boid != &other)
        {
            avgPosition += other.position;
            count++;
        }
    }

    return (count > 0) ? ((avgPosition / count) - boid.position).normalized() : Vector3f::Zero();
    // if (count > 0)
    //     return (avgPosition / float(count)) - boid.position;
    // else
    //     return Vector3f::Zero();
}

// Separation: Avoid crowding nearby boids
Vector3f Boids2::computeSeparation(const Boid2 &boid)
{
    Vector3f repulsion = Vector3f::Zero();

    for (const auto &other : boids)
    {
        double distance = (boid.position - other.position).norm();
        if (distance < separationRadius && &boid != &other)
        {
            repulsion -= (other.position - boid.position) / distance;
        }
    }

    return repulsion.normalized();
    // return repulsion;
}

Vector3f Boids2::computeLeaderAttraction(const Boid2 &boid)
{
    Vector3f toLeader = leaderPosition - boid.position;
    return toLeader.normalized();
    // return toLeader;
}

// Limit the speed of a vector to the maximum speed
Vector3f Boids2::limitSpeed(const Vector3f &velocity)
{
    if (velocity.norm() > maxSpeed)
    {
        return velocity.normalized() * maxSpeed;
    }

    if (velocity.norm() < 0.5)
    {
        // std::cout << "applyint minimun" << "\n";
        return velocity.normalized() * 0.5;
    }

    return velocity;
}

// Limit the acceleration
Vector3f Boids2::limitAccel(const Vector3f &accel)
{
    if (accel.norm() > maxAccel)
    {
        return accel.normalized() * maxAccel;
    }
    return accel;
}

// repulsion force near surface
Vector3f Boids2::computeSurfaceRepulsion(const Boid2 &boid)
{

    if (boid.position.z() - 1.0 < 0.5)
    {
        // std::cout << "near surface limit, pushing down" << "\n";
        float repulsion = 1 / (boid.position.z() + __FLT_EPSILON__);
        return Vector3f(0, 0, repulsion);
    }
    else
    {
        return Vector3f::Zero();
    }
}

void Boids2::updateBoids(double dt)
{
    std::vector<Vector3f> newAccelerations;

    for (const auto &boid : boids)
    {
        Vector3f alignment = computeAlignment(boid) * alignmentWeight;
        Vector3f cohesion = computeCohesion(boid) * cohesionWeight;
        Vector3f separation = computeSeparation(boid) * separationWeight;
        Vector3f leaderAttraction = computeLeaderAttraction(boid) * leaderAttractionWeight;

        Vector3f surfaceRepulsion = computeSurfaceRepulsion(boid) * 2;

        Vector3f newAccel = (alignment + cohesion + separation + leaderAttraction + surfaceRepulsion) / mass;
        newAccel = limitAccel(newAccel);
        newAccelerations.push_back(newAccel);
    }

    for (size_t i = 0; i < boids.size(); i++)
    {
        boids[i].aceleration = newAccelerations[i];
        boids[i].velocity += boids[i].aceleration * dt;
        boids[i].velocity = limitSpeed(boids[i].velocity);
        boids[i].position += boids[i].velocity * dt;
    }
}

Boids2::Boids2()
{
    // std::cout << "default boids constructor" << "\n";
}

Boids2::Boids2(int numBoids, const Vector3f &initialLeaderPosition, int seed) : leaderPosition(initialLeaderPosition)
{
    std::random_device rd;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> dis(-2.0, 2.0);

    for (int i = 0; i < numBoids; i++)
    {
        boids.push_back({Vector3f(dis(gen) + leaderPosition[0],
                                  dis(gen) + leaderPosition[1],
                                  leaderPosition[2]),
                         Vector3f(dis(gen) * 0.1f, dis(gen) * 0.1f, dis(gen) * 0.1f)});
    }
}

const std::vector<Boid2> &Boids2::getBoids() const
{
    return boids;
}

void Boids2::printSettings()
{
    std::cout << "alignmentWeight: " << alignmentWeight << "\n"
              << "cohesionWeight: " << cohesionWeight << "\n"
              << "separationWeight: " << separationWeight << "\n"
              << "leaderAttractionWeight: " << leaderAttractionWeight << "\n"
              << "maxSpeed: " << maxSpeed << "\n"
              << "maxAccel: " << maxAccel << "\n"
              << "perceptionRadius: " << perceptionRadius << "\n"
              << "separationRadius: " << separationRadius << "\n"
              << "mass: " << mass << "\n";
}

void Boids2::setLeaderPosition(const Vector3f &position)
{
    leaderPosition = position;
}

const Vector3f &Boids2::getLeaderPosition() const
{
    return leaderPosition;
}

const double &Boids2::getMass() const
{
    return mass;
}

void Boids2::setAlignmentWeight(const double weight)
{
    alignmentWeight = weight;
}

void Boids2::setCohesionWeight(const double weight)
{
    cohesionWeight = weight;
}

void Boids2::setSeparationWeight(const double weight)
{
    separationWeight = weight;
}

void Boids2::setLeaderAttractionWeight(const double weight)
{
    leaderAttractionWeight = weight;
}

void Boids2::setMaxSpeed(const double speed)
{
    maxSpeed = speed;
}

void Boids2::setMaxAccel(const double accel)
{
    maxAccel = accel;
}

void Boids2::setPerceptionRadius(const double radius)
{
    perceptionRadius = radius;
}

void Boids2::setSeparationRadius(const double radius)
{
    separationRadius = radius;
}

void Boids2::setMass(const double value)
{
    mass = value;
}
