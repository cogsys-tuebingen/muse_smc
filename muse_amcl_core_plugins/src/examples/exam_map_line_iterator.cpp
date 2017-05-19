#include <opencv2/opencv.hpp>
#include <muse_amcl_core_plugins/maps_2d/bresenham.hpp>
#include <muse_amcl_core_plugins/maps_2d/binary_gridmap.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char *argv[])
{
    cv::Mat display(200, 200, CV_8UC3, cv::Scalar());
    cv::Mat mask(200, 200, CV_8UC1, cv::Scalar());
    {
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

        int wait = 0;
        for(cv::Point s : start) {
            for(cv::Point e : end) {
                display.setTo(cv::Scalar());

                cv::circle(display, s, 2, cv::Scalar(255),CV_FILLED, CV_AA);
                cv::circle(display, e, 2, cv::Scalar(255,255),CV_FILLED, CV_AA);
                mask.at<uchar>(e.y, e.x) = 255;

                muse_mcl::maps::Bresenham<uchar> it({s.x, s.y},
                                                     {e.x,e.y},
                                                     {mask.cols, mask.rows},
                                                     mask.ptr<uchar>() );

                muse_mcl::maps::Bresenham<uchar> const_it({s.x, s.y},
                                                           {e.x,e.y},
                                                           {mask.cols, mask.rows},
                                                           mask.ptr<uchar>() );

                while(!it.done()) {
                    display.at<cv::Vec3b>(it.y(), it.x()) = cv::Vec3b(255,255,255);
                    cv::imshow("display", display);
                    cv::imshow("mask", mask);

                    std::cout << "before " << (int) *const_it << std::endl;
                    (*it) = 255;
                    std::cout << "after " << (int) *const_it << std::endl;
                    ++it;
                    ++const_it;

                    int key = cv::waitKey(wait) & 0xFF;
                    if(key == 27)
                        break;
                    if(key == 171)
                        wait = 19;
                }

            }
        }
    }
    display.setTo(cv::Scalar());
    mask.setTo(cv::Scalar());
    {
        /// build a grid map an iterate it with a rotated origin
        mask.at<uchar>(2,2) = 100;
        mask.at<uchar>(88,88) = 100;

    }



    return 0;
}
