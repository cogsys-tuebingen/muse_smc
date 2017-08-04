#ifndef UPDATE_HPP
#define UPDATE_HPP

#include <muse_smc/update/update_model.hpp>
#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/time/time_frame.hpp>

namespace muse_smc {
template<typename sample_t>
class Update {
public:
    using Ptr = std::shared_ptr<Update>;
    using update_model_t = UpdateModel<sample_t>;
    using sample_set_t   = SampleSet<sample_t>;


    struct Less {
        bool operator()( const Update& lhs,
                         const Update& rhs ) const
        {
            return lhs.getStamp() < rhs.getStamp();
        }
        bool operator()( const Update::Ptr &lhs,
                         const Update::Ptr &rhs ) const
        {
            return lhs->getStamp() < rhs->getStamp();
        }
    };

    struct Greater {
        bool operator()( const Update& lhs,
                         const Update& rhs ) const
        {
            return lhs.getStamp() > rhs.getStamp();
        }
        bool operator()( const Update::Ptr &lhs,
                         const Update::Ptr &rhs ) const
        {
            return lhs->getStamp() > rhs->getStamp();
        }
    };

    Update(const Data::ConstPtr         &data,
           const StateSpace::ConstPtr   &state_space,
           const update_model_t::Ptr    &model) :
        data_(data),
        state_space(state_space),
        model_(model)
    {
    }

    virtual ~Update()
    {
    }

    inline void operator() (sample_set_t::weight_iterator_t weights)
    {
        model_->update(data_, state_space, weights);
    }

    inline void apply(sample_set_t::weight_iterator_t weights)
    {
        model_->update(data_, state_space, weights);
    }

    inline Time getStamp() const
    {
        return data_->getTimeFrame().end;
    }

    inline UpdateModel::Ptr getModel() const
    {
        return model_;
    }

    inline const std::string& getModelName() const
    {
        return model_->getName();
    }


private:
    const Data::ConstPtr        data_;
    const StateSpace::ConstPtr  state_space;
    update_model_t::Ptr         model_;
};
}

#endif /* UPDATE_HPP */
