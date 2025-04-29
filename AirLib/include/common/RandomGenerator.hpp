#pragma once

#include <random>
#include "common/Common.hpp"

namespace msr { namespace airlib {

class RandomGenerator {
public:
    RandomGenerator()
        : rng_(std::random_device{}()),
          normal_dist_(0.0f, 1.0f)
    {
    }

    // Uniform distribution: [min, max]
    float uniformReal(float min, float max)
    {
        std::uniform_real_distribution<float> dist(min, max);
        return dist(rng_);
    }

    // Gaussian distribution with mean and stddev
    float gaussianFloat(float mean, float stddev)
    {
        return stddev * normal_dist_(rng_) + mean;
    }

    // Integer random in range [min, max]
    int uniformInt(int min, int max)
    {
        std::uniform_int_distribution<int> dist(min, max);
        return dist(rng_);
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<float> normal_dist_;
};

}} // namespace msr::airlib
