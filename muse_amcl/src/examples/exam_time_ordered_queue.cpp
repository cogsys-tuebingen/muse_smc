#include <muse_amcl/utils/sync_priority_queue.hpp>
#include <chrono>
#include <memory>

class Orderable {
public:
    typedef std::shared_ptr<Orderable> Ptr;

    std::chrono::time_point<std::chrono::system_clock> stamp() const
    {
        return stamp_;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> stamp_;
};


int main(int argc, char *argv[])
{
    auto comp1 = [] (Orderable &lhs, Orderable &rhs) {return lhs.stamp() > rhs.stamp();};
    auto comp2 = [] (Orderable::Ptr &lhs, Orderable::Ptr &rhs) {return lhs->stamp() > rhs->stamp();};

    muse_amcl::SyncPriorityQueue<Orderable, decltype(comp1)> tq1(comp1);
    muse_amcl::SyncPriorityQueue<Orderable::Ptr, decltype(comp2)> tq2(comp2);

    /// *** implement a test *** ///


    return 0;
}
