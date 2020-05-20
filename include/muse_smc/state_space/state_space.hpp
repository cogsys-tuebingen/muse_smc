#ifndef STATE_SPACE_HPP
#define STATE_SPACE_HPP

#include <memory>
#include <chrono>

#include <cslibs_time/time.hpp>
#include <muse_smc/smc/traits/sample.hpp>
namespace muse_smc {
template<typename sample_t>
class StateSpace {
public:
    using Ptr                     = std::shared_ptr<StateSpace>;
    using ConstPtr                = std::shared_ptr<StateSpace const>;
    using state_t                 = typename traits::State<sample_t>::type;
    using state_space_transform_t = typename traits::Transform<sample_t>::type;
    using state_space_boundary_t  = typename traits::StateSpaceBoundary<sample_t>::type;

    StateSpace(const std::string &frame) :
        frame_(frame),
        stamp_(cslibs_time::Time::now())
    {
    }

    StateSpace(const std::string       &frame,
               const cslibs_time::Time &stamp) :
        frame_(frame),
        stamp_(stamp)
    {
    }

    virtual ~StateSpace() = default;

    virtual bool validate(const state_t&)       const = 0;
    virtual state_space_boundary_t  getMin()    const = 0;
    virtual state_space_boundary_t  getMax()    const = 0;
    virtual state_space_transform_t getOrigin() const = 0;

    inline std::string getFrame() const
    {
        return frame_;
    }

    inline cslibs_time::Time getStamp() const
    {
        return stamp_;
    }

    template<typename T>
    bool isType() const
    {
        const T *t = dynamic_cast<const T*>(this);
        return t != nullptr;
    }

    template<typename T>
    T const & as() const
    {
        return dynamic_cast<const T&>(*this);
    }

protected:
    StateSpace() = delete;

    std::string         frame_;
    cslibs_time::Time   stamp_;
};
}

#endif // STATE_SPACE_HPP
