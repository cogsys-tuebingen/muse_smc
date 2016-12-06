#ifndef KLD_2D_TREE_HPP
#define KLD_2D_TREE_HP

#include <cslibs_kdtree/kdtree.hpp>

#include "kld_data.hpp"
#include "kld_2d_index.hpp"

namespace muse_amcl {
using KDTreeBuffered        = kdtree::buffered::KDTree<Index, Data>;
using ClusteringBuffered    = kdtree::KDTreeClustering<KDTreeBuffered>;
}
#endif // KLD_2D_TREE_HPP
