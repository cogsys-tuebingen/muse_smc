#ifndef UPDATE_MODEL_HPP
#define UPDATE_MODEL_HPP

#include <muse_mcl_2d_gridmaps/mapping/log_odds.hpp>

namespace muse_mcl_2d_gridmaps {
namespace mapping {
class InverseModel
{
public:
    inline InverseModel(const double prob_prior,
                        const double prob_free,
                        const double prob_occupied) :
        l_prior_(LogOdds::to(prob_prior)),
        l_free_(LogOdds::to(prob_free)),
        l_occupied_(LogOdds::to(prob_occupied))
    {
    }

    inline double getProbPrior() const
    {
        return LogOdds::from(l_prior_);
    }

    inline double getProbFree() const
    {
        return LogOdds::from(l_free_);
    }

    inline double getProbOccupied() const
    {
        return LogOdds::from(l_occupied_);
    }

    inline double getLogOddsPrior() const
    {
        return l_prior_;
    }

    inline double getLogOddsFree() const
    {
        return l_free_;
    }

    inline double getLogOddsOccupied() const
    {
        return l_occupied_;
    }

    inline double updateFree(const double l_rec)
    {
        return l_free_ + l_rec - l_prior_;
    }

    inline double updateOccupied(const double l_rec)
    {
        return l_occupied_ + l_rec - l_prior_;
    }

private:
    const double l_prior_;
    const double l_free_;
    const double l_occupied_;
};
}
}
#endif // UPDATE_MODEL_HPP
