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
    using state_space_t  = StateSpace<sample_t>;

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

    Update(const Data::ConstPtr                     &data,
           const typename state_space_t::ConstPtr   &state_space,
           const typename update_model_t::Ptr       &model) :
        data_(data),
        state_space(state_space),
        model_(model)
    {
    }

    virtual ~Update()
    {
    }

    inline void operator()
        (typename sample_set_t::weight_iterator_t weights)
    {
        model_->update(data_, state_space, weights);
    }

    inline void apply(typename sample_set_t::weight_iterator_t weights)
    {
        model_->apply(data_, state_space, weights);
    }

    inline Time const & getStamp() const
    {
        return data_->getTimeFrame().end;
    }

    inline typename update_model_t::Ptr getModel() const
    {
        return model_;
    }

    inline const std::string& getModelName() const
    {
        return model_->getName();
    }

    inline std::size_t getModelId() const
    {
        return model_->getId();
    }

private:
    const Data::ConstPtr                    data_;
    const typename state_space_t::ConstPtr  state_space;
    typename update_model_t::Ptr            model_;
};
}

#endif /* UPDATE_HPP */
