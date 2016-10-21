#pragma once

#include <muse_amcl/functions/propagation.hpp>

namespace muse_amcl {
class OmniDrive : public Propagation
{
public:
    OmniDrive();

    virtual void setup(const std::string &name) override;
    virtual void apply(ParticleSet::PoseIterator set) override;

};
}
