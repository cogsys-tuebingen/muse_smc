#ifndef UPDATE_HPP
#define UPDATE_HPP

#include <muse_mcl/plugins/types/model_update.hpp>

#include "particle_set.hpp"

#include <muse_mcl/data_types/time_frame.hpp>

namespace muse_mcl {
class Update {
public:
    using Ptr = std::shared_ptr<Update>;

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

    Update(const Data::ConstPtr   &data,
           const Map::ConstPtr    &map,
           const ModelUpdate::Ptr &model) :
        data_(data),
        map_(map),
        model_(model)
    {
    }

    virtual ~Update()
    {
    }

    inline void operator() (ParticleSet::Weights weights)
    {
        model_->update(data_, map_, weights);
    }

    inline void apply(ParticleSet::Weights weights)
    {
        model_->update(data_, map_, weights);
    }

    inline ros::Time getStamp() const
    {
        return data_->getTimeFrame().end;
    }

    inline ModelUpdate::Ptr getModel() const
    {
        return model_;
    }

    inline const std::string& getModelName() const
    {
        return model_->getName();
    }


private:
    const Data::ConstPtr data_;
    const Map::ConstPtr  map_;
    ModelUpdate::Ptr     model_;
};
}

#endif /* UPDATE_HPP */
