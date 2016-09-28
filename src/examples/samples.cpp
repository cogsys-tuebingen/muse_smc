#include "../particle_filter/sample.hpp"

void print(const tf::Pose &p)
{
    std::cout << "[ "
              << p.getOrigin().x() << " "
              << p.getOrigin().y() << " "
              << p.getOrigin().z() << " ]";
    std::cout << "[ "
              << p.getRotation().x() << " "
              << p.getRotation().y() << " "
              << p.getRotation().z() << " "
              << p.getRotation().w() << " ]"
              << std::endl;
}

int main(int argc, char *argv[])
{
    std::vector<muse_amcl::particle_filter::Sample> samples;
    for(std::size_t i = 0 ; i < 100 ; ++i) {
        samples.emplace_back(muse_amcl::particle_filter::Sample(tf::Pose(tf::Quaternion(0,0,0,1),
                                                                          tf::Vector3(i, 0, 0)), 0.01));

    }
    std::vector<muse_amcl::particle_filter::SamplePose*> samples_pose_access;
    std::vector<muse_amcl::particle_filter::SampleWeight*> samples_weight_access;
    for(auto &s : samples) {
        samples_pose_access.push_back(&s);
        samples_weight_access.push_back(&s);
    }

    tf::Transform t(tf::createQuaternionFromYaw(0), tf::Vector3(0,0,1));

    for(std::size_t i = 0 ; i < samples.size() ; ++i) {

        print(samples.at(i).getPose());
        samples_pose_access[i]->updatePose(t);
        print(samples.at(i).getPose());

        std::cout << samples.at(i).getWeight() << std::endl;
        // assert(&(samples.at(i)) == &(samples_weight_access[i]));
        samples_weight_access[i]->setWeight(0.5);
        std::cout << samples.at(i).getWeight() << std::endl;
    }


    return 0;
}
