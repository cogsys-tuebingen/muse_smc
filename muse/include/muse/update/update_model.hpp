#ifndef UPDATE_MODEL_HPP
#define UPDATE_MODEL_HPP

#include <memory>

#include <muse/data/data.hpp>
#include <muse/state_space/state_space.hpp>
#include <muse/samples/sample_set.hpp>

namespace muse {
template<typename sample_t>
class UpdateModel {
public:
    using sample_set_t = SampleSet<sample_t>;
    using Ptr = std::shared_ptr<UpdateModel>;

    UpdateModel() = default;
    virtual ~UpdateModel() = default;

    inline const static string Type()
    {
        return "muse::UpdateModel";
    }

    inline const std::string& getName() const
    {
        return name_;
    }

    inline void setName(const std::string &name)
    {
        name_ = name;
    }

    inline std::size_t getId() const
    {
        return id_;
    }

    inline void setId(const std::size_t id)
    {
        id_ = id;
    }

    virtual void apply(const Data::ConstPtr &data,
                       const StateSpace<sample_t>::ConstPtr &state_space,
                       sample_set_t::weight_iterator_t weights) = 0;

protected:
    std::string name_;
    std::size_t id_;
};
}
#endif // UPDATE_MODEL_HPP
