#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include <cslibs_indexed_storage/operations/clustering.hpp>

#include "indexation.hpp"
#include "clustering_data.hpp"

namespace muse_mcl {
namespace clustering {

namespace cis = cslibs_indexed_storage::operations::clustering;

template<std::size_t Dimension, typename StateT>
struct Clustering {
    using indexation_t = Indexation<Dimension, StateT>;
    using neighborhood_t =
        cis::GridNeighborhoodStatic<std::tuple<indexation_t::index_t::Base>::value,3>;
    using visitor_t = neighborhood_t::offset_t;
    using data_t = Data<StateT>;

    Clustering(indexation_t &indexation) :
        indexation(indexation)
    {
    }

    inline bool start(const indexation_t::index_t,
                      data_t &data)
    {
        if(data.cluster_id != -1)
            return false;
        ++cluster_id;
    }

    int          cluster_id = -1;
    indexation_t &indexation;


};
}
}
#endif // CLUSTERING_HPP
