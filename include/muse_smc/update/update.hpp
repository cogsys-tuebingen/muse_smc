#ifndef UPDATE_HPP
#define UPDATE_HPP

#include <muse_smc/update/update_model.hpp>
#include <muse_smc/samples/sample_set.hpp>
#include <cslibs_time/time_frame.hpp>

namespace muse_smc {
template<typename state_space_description_t, typename data_t>
class Update {
public:
    using Ptr            = std::shared_ptr<Update>;
    using sample_t       = typename state_space_description_t::sample_t;
    using update_model_t = UpdateModel<state_space_description_t, data_t>;
    using sample_set_t   = SampleSet<state_space_description_t>;
    using state_space_t  = StateSpace<state_space_description_t>;

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
            const auto &lhs_stamp = lhs.getStamp();
            const auto &rhs_stamp = rhs.getStamp();
            return  lhs_stamp == rhs_stamp ? lhs.getStampReceived() > rhs.getStampReceived() :
                                             lhs_stamp > rhs_stamp;

        }
        bool operator()( const Update::Ptr &lhs,
                         const Update::Ptr &rhs ) const
        {
            const auto &lhs_stamp = lhs->getStamp();
            const auto &rhs_stamp = rhs->getStamp();
            return  lhs_stamp == rhs_stamp ? lhs->getStampReceived() > rhs->getStampReceived() :
                                             lhs_stamp > rhs_stamp;
        }
    };

    Update(const typename data_t::ConstPtr        &data,
           const typename state_space_t::ConstPtr &state_space,
           const typename update_model_t::Ptr     &model) :
        data_(data),
        state_space_(state_space),
        model_(model)
    {
    }

    virtual ~Update() = default;

    inline void operator()
        (typename sample_set_t::weight_iterator_t weights)
    {
        model_->update(data_, state_space_, weights);
    }

    inline void apply(typename sample_set_t::weight_iterator_t weights)
    {
        model_->apply(data_, state_space_, weights);
    }

    inline cslibs_time::Time const & getStamp() const
    {
        return data_->getTimeFrame().end;
    }

    inline cslibs_time::Time const & getStampReceived() const
    {
        return data_->getStampReceived();
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
    const typename data_t::ConstPtr        data_;
    const typename state_space_t::ConstPtr state_space_;
    typename update_model_t::Ptr           model_;
};
}

#endif // UPDATE_HPP
