#ifndef STATE_SPACE_HPP
#define STATE_SPACE_HPP

#include <memory>
#include <chrono>

#include <muse/time/time.hpp>

namespace muse {
template<typename sample_t>
class StateSpace {
public:
    using Ptr =  std::shared_ptr<StateSpace>;
    using ConstPtr = std::shared_ptr<StateSpace const>;
    using state_t = typename sample_t::state_t;


    StateSpace(const std::string &frame) :
        frame_(frame),
        stamp_(Time::now())
    {
    }

    StateSpace(const std::string &frame,
               const Time &stamp) :
        frame_(frame),
        stamp_(stamp)
    {
    }

    virtual ~StateSpace()
    {
    }

    virtual bool validate(const state_t &) const
    {
        return true;
    }

    virtual inline state_t getMin()    const = 0;
    virtual inline state_t getMax()    const = 0;
    virtual inline state_t getOrigin() const = 0;
    virtual inline bool isAvailable()     const = 0;

    inline std::string getFrame() const
    {
        return frame_;
    }

    inline Time getStamp() const
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

    std::string frame_;
    Time        stamp_;


#endif // STATE_SPACE_HPP
