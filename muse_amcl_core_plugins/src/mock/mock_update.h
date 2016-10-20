#pragma once

#include <muse_amcl/functions/update.hpp>

namespace muse_amcl {
class MockUpdate : public Update
{
public:
    MockUpdate();

    virtual double apply(ParticleSet::WeightIterator set) override;

};
}
