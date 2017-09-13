#include <muse_mcl_2d_gridmaps/dynamic_maps/gridmap.hpp>

#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{

    muse_mcl_2d_gridmaps::dynamic_maps::GridMap<uchar> map(0.0, 0.0, 0.0,
                                                           0.05,
                                                           1.0,
                                                           0,
                                                           std::string("taste"));
    auto render = [&map]()
    {
        const std::size_t width  = map.getWidth();
        const std::size_t height = map.getHeight() ;

        std::cout << "height : " << height << "\n";
        std::cout << "width  : " << width  << "\n";

        cv::Mat test(map.getHeight(), map.getWidth(), CV_8UC1, cv::Scalar());
        for(std::size_t i = 0 ; i < height ; ++i) {
            for(std::size_t j = 0 ; j < width ; ++j) {
                test.at<uchar>(i,j) = map.at(j,i);
            }
        }

        cv::resize(test, test, cv::Size(), 10., 10., CV_INTER_NN);
        cv::imshow("test", test);
        cv::waitKey(0);
    };


    /// these are the easy cases
    map.at(muse_mcl_2d::Point2D(0.0, 0.0))   = 255;
    render();
    map.at(muse_mcl_2d::Point2D(1.0, 0.0))   = 255;
    render();
    map.at(muse_mcl_2d::Point2D(0.0, 1.0))   = 255;
    render();
    map.at(muse_mcl_2d::Point2D(1.0, 1.0))   = 255;
    render();
//    std::cout << map.getOrigin() << std::endl;

    /// now to the more difficult cases
    map.at(muse_mcl_2d::Point2D(-1.0,  0.0))   = 127;
    render();
    std::cout << map.getOrigin() << std::endl;
    map.at(muse_mcl_2d::Point2D(-1.0,  1.0))   = 127;
    render();
    std::cout << map.getOrigin() << std::endl;
    map.at(muse_mcl_2d::Point2D(-1.0, -1.0))   = 127;
    render();
    std::cout << map.getOrigin() << std::endl;

    auto it = map.getLineIterator(std::array<int, 2>{0,0},
                                  std::array<int, 2>{59,59});
    do {
        *it = 70;
        render();
        ++it;
    } while(!it.done());

    *it = 70;
    render();

    return 0;
}
