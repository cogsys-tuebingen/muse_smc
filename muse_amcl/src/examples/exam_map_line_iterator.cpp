#include <opencv2/opencv.hpp>
#include <muse_amcl/maps/gridmap_line_iterator.hpp>

int main(int argc, char *argv[])
{
    cv::Mat display(200, 200, CV_8UC3, cv::Scalar());
    cv::Mat mask(200, 200, CV_8UC1, cv::Scalar());
    std::vector<cv::Point> start =
    {
        cv::Point(10,25),
        cv::Point(10,25),
        cv::Point(135, 151)
    };
    std::vector<cv::Point> end =
    {
        cv::Point(190, 190),
        cv::Point(120, 120),
        cv::Point(50, 50)
    };

    for(cv::Point s : start) {
        for(cv::Point e : end) {
            display.setTo(cv::Scalar());

            cv::circle(display, s, 2, cv::Scalar(255),CV_FILLED, CV_AA);
            cv::circle(display, e, 2, cv::Scalar(255,255),CV_FILLED, CV_AA);
            mask.at<uchar>(e.y, e.x) = 255;

            muse::maps::GridMapLineIterator<uchar> it({s.x, s.y},
                                                      {e.x,e.y},
                                                      mask.cols,
                                                      mask.ptr<uchar>() );
            while(!it.done()) {
                display.at<cv::Vec3b>(it.y(), it.x()) = cv::Vec3b(255,255,255);
                cv::waitKey(0);
                cv::imshow("display", display);
                cv::imshow("mask", mask);
                (*it) = 255;
                ++it;
            }

        }
    }



    return 0;
}
