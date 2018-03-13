#ifndef UPDATE_MODEL_HPP
#define UPDATE_MODEL_HPP

#include <memory>

#include <cslibs_plugins_data/data.hpp>
#include <muse_smc/state_space/state_space.hpp>
#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class UpdateModel {
public:
    using Ptr           = std::shared_ptr<UpdateModel>;
    using sample_t      = typename state_space_description_t::sample_t;
    using sample_set_t  = SampleSet<state_space_description_t>;
    using state_space_t = StateSpace<state_space_description_t>;
    using data_t        = cslibs_plugins_data::Data;

    UpdateModel()
    {
    }

    virtual ~UpdateModel() = default;

    inline const static std::string Type()
    {
        return "muse_smc::UpdateModel";
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

    virtual void apply(const data_t::ConstPtr &data,
                       const typename state_space_t::ConstPtr &state_space,
                       typename sample_set_t::weight_iterator_t weights) = 0;

protected:
    std::string          name_;
    std::size_t          id_;

};
}
#endif // UPDATE_MODEL_HPP
