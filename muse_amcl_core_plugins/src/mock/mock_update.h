#pragma once

#include <muse_amcl/functions/update.hpp>

namespace muse_amcl {
class MockUpdate : public Update
{
public:
    MockUpdate();

    virtual void setup(const std::string &name) override;
    virtual double apply(ParticleSet::WeightIterator set) override;

};
}
