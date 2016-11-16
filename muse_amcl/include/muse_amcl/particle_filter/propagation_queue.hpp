#pragma once

#include <muse_amcl/utils/synced_priority_queue.hpp>
#include "propagation_lambda.hpp"

namespace muse_amcl {
   typedef SyncedPriorityQueue<PropagationLambda, PropagationLambda::Less> PropagationQueue;
}
