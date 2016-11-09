#pragma once

#include "types/update.hpp"
#include "types/map_provider.hpp"
#include "types/data_provider.hpp"

namespace muse_amcl {
class UpdateManager {
public:
    typedef std::shared_ptr<UpdateManager> Ptr;

    UpdateManager(Update::Ptr &update
                  /*, prio queue */) :
        update_(update)
    {
    }

    virtual  ~UpdateManager()
    {
    }

    void setup(muse_amcl::DataProvider::Ptr &data_provider,
               muse_amcl::MapProvider::Ptr  &map_provider)
    {
        data_connection_ = data_provider.connect(std::bind(UpdateManager::dataCallback, this));
        map_provider_ = map_provider;
    }

protected:
    muse_amcl::DataProvider::DataConnection::Ptr data_connection_;
    MapProvider::Ptr                             map_provider_;
    Update::Ptr                                  update_;

    void dataCallback(const Data::ConstPtr &data)
    {
        /// build lambda expression and push to prio queue
        /// maybe model function can be exchanged ( no pointer )
    }

};
}
