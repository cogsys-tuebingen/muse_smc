#pragma once

#include <muse_amcl/functions/update.hpp>

namespace muse_amcl {
class LikelihoodFieldProbModel : public Update
{
public:
    LikelihoodFieldProbModel();

    virtual void   setup(const std::string &name) override;
    virtual double apply(ParticleSet::WeightIterator set) override;

};
}

