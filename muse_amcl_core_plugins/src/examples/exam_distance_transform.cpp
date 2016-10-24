#include <opencv2/opencv.hpp>
#include <chrono>

#include <muse_amcl_core_plugins/maps/distance_transform.hpp>

int main(int argc, char *argv[])
{
    { // 3x3
        const muse::maps::distance_transform::Kernel k(3);
        cv::Mat kernel = cv::Mat(3,3, CV_32FC1, cv::Scalar());
        for(std::size_t i = 0 ; i < 3 ; ++i) {
            for(std::size_t j = 0 ; j < 3 ; ++j) {
                kernel.at<float>(i,j) = k.at(i,j);
            }
        }
        cv::Mat display;
        cv::normalize(kernel, display, 0.0, 1.0, cv::NORM_MINMAX);
        cv::resize(display, display, cv::Size(200, 200));
        std::cout << kernel << std::endl;
        cv::imshow("kernel", display);
        cv::waitKey();
    }
    { // 5x5
        const muse::maps::distance_transform::Kernel k(5);
        cv::Mat kernel = cv::Mat(5,5, CV_32FC1, cv::Scalar());
        for(std::size_t i = 0 ; i < 5 ; ++i) {
            for(std::size_t j = 0 ; j < 5 ; ++j) {
                kernel.at<float>(i,j) = k.at(i,j);
            }
        }
        cv::Mat display;
        cv::normalize(kernel, display, 0.0, 1.0, cv::NORM_MINMAX);
        cv::resize(display, display, cv::Size(200, 200));
        std::cout << kernel << std::endl;
        cv::imshow("kernel", display);
        cv::waitKey();
    }
    const std::size_t repetitions = 1;
    {
        cv::Mat test = cv::Mat(200, 200, CV_8UC1, cv::Scalar(1.f));
        cv::rectangle(test, cv::Point(50, 50), cv::Point(150,150), cv::Scalar(0.f));
        cv::Mat display;
        auto start = std::chrono::system_clock::now();
        for(std::size_t i = 0 ; i < repetitions ; ++i) {
            cv::distanceTransform(test, display, CV_DIST_L2, 5);
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << elapsed.count() / (double) repetitions << "[ms]" << '\n';

        cv::normalize(display, display, 0, 1, cv::NORM_MINMAX);
        cv::imshow("distance1", display);
        cv::waitKey();
    }
    {
        cv::Mat test = cv::Mat(200, 200, CV_32FC1, cv::Scalar(0.f));
        cv::rectangle(test, cv::Point(50, 50), cv::Point(150,150), cv::Scalar(1.f));
        cv::Mat display = cv::Mat(200, 200, CV_64FC1, cv::Scalar());

        auto start = std::chrono::system_clock::now();
        muse::maps::distance_transform::Borgefors<float> borge(test.rows, test.cols, 1.0, std::size_t(3), 1.f);
        for(std::size_t i = 0 ; i < repetitions ; ++i) {
            borge.apply(test.ptr<float>(), display.ptr<double>());
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << elapsed.count() / (double) repetitions << "[ms]" << '\n';

        cv::normalize(display, display, 0, 1, cv::NORM_MINMAX, CV_32F);
        cv::imshow("distance2", display);
        cv::waitKey();
    }
    {
        cv::Mat test = cv::Mat(2000, 2000, CV_32FC1, cv::Scalar(0.f));
        cv::rectangle(test, cv::Point(50, 50), cv::Point(150,150), cv::Scalar(1.f));
        cv::Mat display = cv::Mat(2000, 2000, CV_64FC1, cv::Scalar());

        auto start = std::chrono::system_clock::now();
        muse::maps::distance_transform::Borgefors<float> dijkstra(test.rows, test.cols, 1.0, std::size_t(3), 1.f);
        for(std::size_t i = 0 ; i < repetitions ; ++i) {
            dijkstra.apply(test.ptr<float>(), display.ptr<double>());
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << elapsed.count() / (double) repetitions << "[ms]" << '\n';

        cv::normalize(display, display, 0, 1, cv::NORM_MINMAX, CV_32F);
        cv::imshow("distance2", display);
        cv::waitKey();
    }



    return 0;
}
