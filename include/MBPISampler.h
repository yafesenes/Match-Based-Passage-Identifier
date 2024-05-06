#ifndef MBPISAMPLER_H
#define MBPISAMPLER_H

#include <ompl/base/StateSampler.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <NarrowFinder.h>
#include <thread>
#include <vector>

namespace ompl
{
    namespace base
    {
        class MBPIValidStateSampler : public ValidStateSampler
        {
        public:
            MBPIValidStateSampler(const SpaceInformation* si, const std::vector<std::vector<bool>>& occupancyGrid, const double resolution, const std::pair<double, double>& origin, int uniformPerNarrow = 1, double maxPassageRatio = 0.05);
            ~MBPIValidStateSampler() override = default;

            bool sample(State* state) override;
            bool sampleNear(State* state, const State* near, double distance) override;
        private:
            std::vector<std::vector<float>> calculatePassageValues(const std::vector<std::vector<bool>>& occupancyGrid);
            void calculateCumulative(const std::vector<std::vector<float>>& passageValues);
            bool sampleUniform(State* state);
            bool sampleNarrow(State* state);

        protected:
            StateSamplerPtr sampler_;

        private:
            std::vector<float> cumulative_;
            size_t mapWidth_;
            size_t mapHeight_;

            const double resolution_;
            std::pair<double, double> origin_;

            std::random_device rd_;
            std::mt19937 gen_;
            std::uniform_real_distribution<> dis_;

            size_t counter_;
            const int uniformPerNarrow_;

            double maxPassageRatio_;
        };
    }
}

#endif // MBPISAMPLER_H
