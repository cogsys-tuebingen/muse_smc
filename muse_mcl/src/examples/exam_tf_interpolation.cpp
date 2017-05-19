#include <tf/tf.h>

int main(int argc, char *argv[])
{
    auto to_string = [](const tf::Transform &t) {
        std::cout << t.getOrigin().x() << " " << t.getOrigin().y() << " " << tf::getYaw(t.getRotation()) << std::endl;
    };

    tf::Transform w_T_b_1(tf::createQuaternionFromYaw(30.0 / 180.0 * M_PI), tf::Vector3(1,1,0));
    tf::Transform w_T_b_2(tf::createQuaternionFromYaw(60.0 / 180.0 * M_PI), tf::Vector3(2,2,0));
    tf::Transform b_1_T_b_2 = (w_T_b_1.inverse()) * w_T_b_2;
    std::cout << "w_T_b_1 ";
    to_string(w_T_b_1);
    std::cout << "w_T_b_2 ";
    to_string(w_T_b_2);
    std::cout << "b_1_T_b_2 ";
    to_string(b_1_T_b_2);

    double alpha = 0.25;
    tf::Vector3 trans_prime;
    trans_prime.setInterpolate3(w_T_b_1.getOrigin(), w_T_b_2.getOrigin(), alpha);
    tf::Quaternion rot_prime = tf::slerp(w_T_b_1.getRotation(), w_T_b_2.getRotation(), alpha);
    tf::Transform w_T_b_1_prime(rot_prime, trans_prime);
    std::cout << "w_T_b_1_prime ";
    to_string(w_T_b_1_prime);
    tf::Transform b_1_prime_T_b_2 = w_T_b_1_prime.inverse() * w_T_b_2;
    std::cout << "b_1_prime_T_b_2 ";
    to_string(b_1_prime_T_b_2);

    tf::Transform w_T_b_2_prime = w_T_b_1_prime * b_1_prime_T_b_2;
    std::cout << "w_T_b_2_prime ";
    to_string(w_T_b_2_prime);

    return 0;
}
