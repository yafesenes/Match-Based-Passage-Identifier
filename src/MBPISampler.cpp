#include "MBPISampler.h"

ompl::base::MBPIValidStateSampler::MBPIValidStateSampler(const SpaceInformation* si, const std::vector<std::vector<bool>>& occupancyGrid, const double resolution, const std::pair<double, double>& origin, int uniformPerNarrow, double maxPassageRatio)
        : ValidStateSampler(si)
        , sampler_(si->allocStateSampler())
        , resolution_(resolution)
        , origin_(origin)
        , gen_(rd_())
        , dis_(0.0, 1.0)
        , uniformPerNarrow_(uniformPerNarrow)
        , counter_(0)
        , maxPassageRatio_(maxPassageRatio)
{
    name_ = "mbpi";

    // calculate passage values
    std::vector<std::vector<float>> heatMap = calculatePassageValues(occupancyGrid);
    mapHeight_ = heatMap.size();
    mapWidth_ = heatMap[0].size();

    calculateCumulative(heatMap);
}

bool ompl::base::MBPIValidStateSampler::sample(State* state)
{
    if (uniformPerNarrow_ > 0)
    {
        if (counter_++ < uniformPerNarrow_)
        {
            return sampleNarrow(state);
        }
        counter_ = 0;

        return sampleUniform(state);
    }
    else
    {
        if (counter_++ < -uniformPerNarrow_)
        {
            return sampleUniform(state);
        }
        counter_ = 0;

        return sampleNarrow(state);
    }

}

bool ompl::base::MBPIValidStateSampler::sampleUniform(State* state)
{
    unsigned int attempts = 0;
    bool valid = false;
    do
    {
        sampler_->sampleUniform(state);
        valid = si_->isValid(state);
        attempts++;
    }
    while (!valid && attempts < attempts_);

    return valid;
}

bool ompl::base::MBPIValidStateSampler::sampleNarrow(State* state)
{
    float random = dis_(gen_);

    auto it = std::lower_bound(cumulative_.begin(), cumulative_.end(), random);

    // get 2d coordinates
    int index = std::distance(cumulative_.begin(), it);
    int yIndex = index / mapWidth_;
    int xIndex = index % mapWidth_;

    // convert to real world coordinates
    double convertedX = origin_.first + (xIndex + 0.5) * resolution_;
    double convertedY = origin_.second + (yIndex + 0.5) * resolution_;

    auto* rstate = state->as<RealVectorStateSpace::StateType>();
    rstate->values[0] = convertedX;
    rstate->values[1] = convertedY;

    bool isValid = si_->isValid(state);
    return isValid;
}

bool ompl::base::MBPIValidStateSampler::sampleNear(State* state, const State* near, const double distance)
{
    return false;
}

void ompl::base::MBPIValidStateSampler::calculateCumulative(const std::vector<std::vector<float>>& passageValues)
{
    float obstaclePassageVal = static_cast<float>(std::min(passageValues.size(), passageValues[0].size()));

    std::vector<float> oneDimPasVal(passageValues.size() * passageValues[0].size());
    for (size_t i = 0; i < passageValues.size(); i++)
    {
        for (size_t j = 0; j < passageValues[i].size(); j++)
        {
            oneDimPasVal[i*passageValues[i].size() + j] = std::pow((obstaclePassageVal - passageValues[i][j]) / obstaclePassageVal, 1);
        }
    }

    float sum = std::accumulate(oneDimPasVal.begin(), oneDimPasVal.end(), 0.0f);
    for (float & i : oneDimPasVal) {
        i = i / sum; // Normalize
    }

    cumulative_.resize(oneDimPasVal.size());
    std::partial_sum(oneDimPasVal.begin(), oneDimPasVal.end(), cumulative_.begin());
}

std::vector<std::vector<float>> ompl::base::MBPIValidStateSampler::calculatePassageValues(
        const std::vector<std::vector<bool>> &occupancyGrid) {
    NarrowFinder narrowFinder(occupancyGrid, maxPassageRatio_);
    narrowFinder.calculatePassageValues();
    return narrowFinder.getPassageValues();
}