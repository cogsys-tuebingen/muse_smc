
#include <class_loader/class_loader_register_macro.h>

#include <ros/time.h>

#include <muse_smc/state_space_samplers/normal.hpp>

#include <muse_mcl_2d/sampling/normal_2d.hpp>
#include <muse_mcl_2d/math/convert.hpp>

namespace muse_mcl_2d {
using Metric              = muse_smc::state_space_samplers::Metric;
using Radian              = muse_smc::state_space_samplers::Radian;
using RandomPoseGenerator = muse_smc::state_space_samplers::Normal<Metric, Metric, Radian>;


class Normal2D : public NormalSampling2D
{
public:
    virtual void update(const std::string &frame) override
    {
        const ros::Time   now   = ros::Time::now();

        math::Point2D min(std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max());
        math::Point2D max(std::numeric_limits<double>::lowest(),
                          std::numeric_limits<double>::lowest());

        for(auto &m : map_providers_) {
            tf::Transform tf_map_T_w;
            math::Transform2D map_t_w;
            Map2D::ConstPtr map = m->getStateSpace();
            if(!map) {
                throw std::runtime_error("[Normal2D] : map was null!");
            }

            if(tf_->lookupTransform(map->getFrame(), frame, now, tf_map_T_w, tf_timeout_)) {
                map_t_w = math::from(tf_map_T_w);

                maps_.emplace_back(map);
                maps_T_w_.emplace_back(map_t_w);

                math::Transform2D w_T_map = map_t_w.inverse();
                min = min.min(w_T_map * map->getMin());
                max = max.max(w_T_map * map->getMax());

            }
        }
    }

    virtual void apply(const math::Pose2D       &pose,
                       const math::Covariance2D &covariance,
                       sample_set_t &sample_set) override
    {
        update(sample_set.getFrame());

        RandomPoseGenerator::Ptr rng(new RandomPoseGenerator(pose.toEigen(), covariance.toEigen()));
        if(random_seed_ >= 0) {
            rng.reset(new RandomPoseGenerator(pose.toEigen(), covariance.toEigen(), random_seed_));
        }

        if(sample_size_ < sample_set.getMinimumSampleSize() &&
                sample_size_ > sample_set.getMaximumSampleSize()) {
            throw std::runtime_error("Initialization sample size invalid!");
        }

        sample_set_t::sample_insertion_t insertion = sample_set.getInsertion();

        const ros::Time sampling_start = ros::Time::now();
        const std::size_t map_count = maps_.size();

        Sample2D sample;
        sample.weight = 1.0 / static_cast<double>(sample_size_);
        for(std::size_t i = 0 ; i < sample_size_ ; ++i) {
            bool valid = false;
            while(!valid) {
                ros::Time now = ros::Time::now();
                if(sampling_start + sampling_timeout_ < now) {
                    return;
                }

                sample.state.setFrom(rng->get());
                valid = true;
                for(std::size_t i = 0 ; i < map_count ; ++i) {
                    valid &= maps_[i]->validate(maps_T_w_[i] * sample.state);
                }
            }
            insertion.insert(sample);
        }
    }

protected:

    int                             random_seed_;
    std::vector<Map2D::ConstPtr>    maps_;
    std::vector<math::Transform2D>  maps_T_w_;
    std::vector<MapProvider2D::Ptr> map_providers_;

    virtual void doSetup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
                         ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        random_seed_ = nh.param(param_name("seed"), -1);

        std::vector<std::string> map_provider_ids;
        nh.getParam(param_name("maps"), map_provider_ids);

        if(map_provider_ids.size() == 0) {
            throw std::runtime_error("[NormalSampling]: No map providers were found!");
        }

        std::string ms ="[";
        for(auto m : map_provider_ids) {

            if(map_providers.find(m) == map_providers.end())
                throw std::runtime_error("[NormalSampling]: Cannot find map provider '" + m + "'!");

            map_providers_.emplace_back(map_providers.at(m));
            ms += m + ",";
        }
        ms.back() = ']';
    }
};
}
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Normal2D, muse_mcl_2d::NormalSampling2D)
