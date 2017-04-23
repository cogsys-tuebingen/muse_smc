#include <tf/tf.h>

int main(int argc, char *argv[])
{
    auto to_string = [](const tf::Transform &t) {
        std::cout << t.getOrigin().x() << " " << t.getOrigin().y() << " " << tf::getYaw(t.getRotation()) << std::endl;
    };

    tf::Transform w_T_b_1(tf::createQuaternionFromYaw(30.0 / 180.0 * M_PI), tf::Vector3(1,1,0));
    tf::Transform w_T_b_2(tf::createQuaternionFromYaw(60.0 / 180.0 * M_PI), tf::Vector3(2,2,0));
    tf::Transform delta = (w_T_b_1.inverse()) * w_T_b_2;
    std::cout << "w_T_b_1 ";
    to_string(w_T_b_1);
    std::cout << "w_T_b_2 ";
    to_string(w_T_b_2);
    std::cout << "delta ";
    to_string(delta);

    double alpha = 0.5;
    tf::Transform b_1_T_b_1_prime = tf::Transform(delta.getRotation() * alpha, delta.getOrigin() * alpha);
    tf::Transform b_1_prime_T_b_2 = tf::Transform(delta.getRotation() * (1.0 - alpha), delta.getOrigin() * (1.0 - alpha));
    std::cout << "b_1_T_b_1_prime ";
    to_string(b_1_T_b_1_prime);
    std::cout << "b_1_prime_T_b_2 ";
    to_string(b_1_prime_T_b_2);
    tf::Transform delta_prime = b_1_T_b_1_prime * b_1_prime_T_b_2;
    std::cout << "delta_prime ";
    to_string(delta_prime);


    //    tf::Transform w_T_b_1_prime = w_T_b_1 * tf::Transform(delta.getRotation() * alpha, delta.getOrigin() * alpha);
//    tf::Transform delta_prime = tf::Transform(delta.getRotation() * (1.0 - alpha), delta.getOrigin() * (1.0 - alpha));
//    std::cout << "w_T_b_1_prime: ";
//    to_string(w_T_b_1_prime);
//    tf::Transform w_T_b_2_prime = w_T_b_1_prime * delta_prime;
//    std::cout << "w_T_b_2: ";
//    to_string(w_T_b_2);
//    std::cout << "w_T_b_2_prime: ";
//    to_string(w_T_b_2_prime);

    return 0;
}
