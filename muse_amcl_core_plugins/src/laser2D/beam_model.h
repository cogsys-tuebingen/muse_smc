#pragma once

#include <muse_amcl/functions/update.hpp>

namespace muse_amcl {
class BeamModel : public Update
{
public:
    BeamModel();

    virtual void setup(const std::string &name) override;
    virtual double apply(ParticleSet::WeightIterator set) override;
};
}

