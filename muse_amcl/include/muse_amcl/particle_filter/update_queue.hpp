#pragma once

#include <muse_amcl/utils/synced_priority_queue.hpp>
#include "update_lambda.hpp"

namespace muse_amcl {
   typedef SyncedPriorityQueue<UpdateLambda, UpdateLambda::Less> UpdateQueue;
}
