#pragma once

#include <muse_amcl/functions/propagation.hpp>

namespace muse_amcl {
class MockPropagation : public Propagation
{
public:
    MockPropagation();

    virtual void setup(const std::string &ns) override;
    virtual void apply(ParticleSet::PoseIterator set) override;

};
}

