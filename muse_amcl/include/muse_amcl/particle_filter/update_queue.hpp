#ifndef UPDATE_QUEUE_HPP
#define UPDATE_QUEUE_HPP

#include <muse_amcl/utils/synced_priority_queue.hpp>
#include "update_lambda.hpp"

namespace muse_amcl {
   typedef SyncedPriorityQueue<UpdateLambda, UpdateLambda::Less> UpdateQueue;
}

#endif /* UPDATE_QUEUE_HPP */
