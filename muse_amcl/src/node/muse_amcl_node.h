#ifndef MUSE_AMCL_NODE_H
#define MUSE_AMCL_NODE_H

#include "../../include/muse_amcl/particle_filter/particle_filter.hpp"

class MuseAMCLNode
{
public:
    MuseAMCLNode();
    virtual ~MuseAMCLNode();

    void setup();
    void start();

private:
    ros::NodeHandle nh_private_;
    ros::NodeHandle nh_public_;

};

#endif /* MUSE_AMCL_NODE_H */
