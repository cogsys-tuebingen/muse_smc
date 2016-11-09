#pragma once

#include "types/update.hpp"
#include "types/map_provider.hpp"
#include "types/data_provider.hpp"

namespace muse_amcl {
class UpdateManager {
public:
    typedef std::shared_ptr<UpdateManager> Ptr;



private:
    muse_amcl::DataProvider::DataConnection::Ptr data_connection;
    MapProvider::Ptr                             map_provider;

};
}
