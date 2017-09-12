#include <muse_mcl_2d_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
    muse_mcl_2d_gridmaps::algorithms::DistanceTransform<uint8_t> d(0.05, 2.0, 127);

    std::vector<uint8_t> data(200 * 200, 0);

    cv::Mat map(200, 200, CV_8UC1, data.data());
    cv::rectangle(map, cv::Rect(map.cols / 2 - 10, map.rows / 2 - 10, 20, 20), cv::Scalar::all(255), CV_FILLED);
    cv::imshow("map", map);
    cv::waitKey(0);

    std::vector<double> distances;
    d.apply(data, 200, distances);

    for(double d : distances) {
        if(d != 2)
        std::cout << d << "\n";
    }

    cv::Mat dis(200, 200, CV_64FC1, distances.data());
    cv::normalize(dis, dis, 0.0, 1.0, cv::NORM_MINMAX);
    cv::imshow("distances", dis);
    cv::waitKey(0);

    return 0;
}
