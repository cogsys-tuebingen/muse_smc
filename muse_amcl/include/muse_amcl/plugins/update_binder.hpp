#pragma once

#include "types/update.hpp"
#include "types/map_provider.hpp"
#include "types/data_provider.hpp"
#include "update_queue.hpp"

namespace muse_amcl {
class UpdateBinder {
public:
    typedef std::shared_ptr<UpdateBinder> Ptr;

    UpdateBinder(Update::Ptr &update,
                 UpdateQueue::Ptr &update_queue) :
        update_(update),
        update_queue_(update_queue)
    {
    }

    virtual  ~UpdateBinder()
    {
    }

    void setup(muse_amcl::DataProvider::Ptr &data_provider,
               muse_amcl::MapProvider::Ptr  &map_provider)
    {
        data_connection_ = data_provider.connect(std::bind(UpdateBinder::dataCallback, this));
        map_provider_ = map_provider;
    }

protected:
    muse_amcl::DataProvider::DataConnection::Ptr data_connection_;
    MapProvider::Ptr                             map_provider_;
    Update::Ptr                                  update_;
    UpdateQueue::Ptr                             update_queue_;

    void dataCallback(const Data::ConstPtr &data)
    {


        /// bind all that suff
        /// build lambda expression and push to prio queue
        /// maybe model function can be exchanged ( no pointer )
    }

};
}
