#include "tf_provider.h"

using namespace muse_amcl::transforms;

TFProvider::TFProvider()
{
}

bool TFProvider::updateTransform(const std::string   &target,
                                 const std::string   &source,
                                 const ros::Time     &time,
                                 const ros::Duration &tolerance)
{
    if(!tfl_.waitForTransform(target, source, time, tolerance)) {
        return false;
    }

    tf::StampedTransform &transform = tfm_[TransformKey(target, source)];
    tfl_.lookupTransform(target, source, time, transform);

    return true;
}

bool TFProvider::updateTransform(const std::string    &target,
                                 const std::string    &source,
                                 const ros::Time      &time,
                                 const ros::Duration  &tolerance,
                                 tf::StampedTransform &transform)
{
    if(!tfl_.waitForTransform(target, source, time, tolerance)) {
        return false;
    }

    transform = tfm_[TransformKey(target, source)];
    tfl_.lookupTransform(target, source, time, transform);

    return true;
}

tf::StampedTransform TFProvider::getTransformation(const std::string &target,
                                                   const std::string &source)
{
    return tfm_[TransformKey(target, source)];
}

void TFProvider::getTransformation(const std::string &target,
                                   const std::string &source,
                                   tf::StampedTransform &transform)
{
    transform = tfm_[TransformKey(target, source)];
}
