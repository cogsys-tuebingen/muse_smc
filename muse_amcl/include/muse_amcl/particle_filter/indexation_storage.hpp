#ifndef INDEXED_STORAGE_HPP
#define INDEXED_STORAGE_HPP

#include <memory>

#include "indexation.hpp"

/**
 * @brief The IndexationStorage class is used to index samples so the particle
 *        density can be estimated. It is also useful to build up and calculate
 *        the histogram size.
 */
namespace muse_amcl {
class IndexationStorage {
public:
    IndexationStorage()
    {
    }

    IndexationStorage(const Indexation &indexation)
    {
    }

    inline void trackIndex()
    {

    }





private:
    Indexation indexation;



};
}

#endif // INDEXED_STORAGE_HPP
