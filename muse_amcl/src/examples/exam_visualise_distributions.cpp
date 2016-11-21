#include <muse_amcl/math/math.hpp>
#include <opencv2/opencv.hpp>

#include "../../include/muse_amcl/pose_generation/gaussian.hpp"
#include "../../include/muse_amcl/utils/eigen.hpp"



int main(int argc, char *argv[])
{
    const std::size_t     sample_count = 4000;
    const std::size_t     histogram_bins = 20;
    std::array<double, 2> histogram_range = {-10.0, 10.0};

    cv::Mat display(200, 200, CV_8UC3, cv::Scalar());
    cv::Mat canvas(display, cv::Rect(5,5, 190, 190));
    double width = canvas.cols / (double) histogram_bins;

    /// uniform 1D
    muse_amcl::math::random::Uniform<1> u1D(-10.0, 10.0);
    std::vector<double>            u1D_histogram(histogram_bins);
    std::size_t max = 0;
    for(std::size_t i = 0 ; i < sample_count ; ++i) {
        std::size_t idx = (u1D.get() - histogram_range[0]) /
                (histogram_range[1] - histogram_range[0]) *
                histogram_bins;
        ++u1D_histogram.at(idx);
        if(u1D_histogram.at(idx) > max)
            max = u1D_histogram.at(idx);
    }
    std::cout << max << std::endl;

    for(std::size_t i = 0 ; i < histogram_bins; ++i) {
        double percent = u1D_histogram[i] / (double) max;
        cv::Rect r(i * width, display.rows * (1.0 - percent), width, percent * display.rows);
        cv::rectangle(canvas, r, cv::Scalar(255), CV_FILLED);
        cv::rectangle(canvas, r, cv::Scalar(0,255));
    }
    cv::imshow("display", display);
    cv::waitKey(0);

    /// normally distributed
    display.setTo(cv::Scalar());

    muse_amcl::math::random::Normal<1> n1D(0.0, 5.0);
    std::vector<double> n1D_histogram(histogram_bins);
    max = 0;
    for(std::size_t i = 0 ; i < sample_count ; ++i) {
        double sample = n1D.get();
        if(sample < histogram_range[0] ||
                sample > histogram_range[1])
            continue;
        std::size_t idx = (sample - histogram_range[0]) /
                (histogram_range[1] - histogram_range[0]) *
                histogram_bins;

        ++n1D_histogram.at(idx);
        if(n1D_histogram.at(idx) > max)
            max = n1D_histogram.at(idx);
    }

    for(std::size_t i = 0 ; i < n1D_histogram.size(); ++i) {
        double percent = n1D_histogram[i] / (double) max;
        cv::Rect r(i * width, display.rows * (1.0 - percent), width, percent * display.rows);
        cv::rectangle(canvas, r, cv::Scalar(255), CV_FILLED);
        cv::rectangle(canvas, r, cv::Scalar(0,255));
    }
    cv::imshow("display", display);
    cv::waitKey(0);

    /// uniform distributed multivariate
    display.setTo(cv::Scalar());
    muse_amcl::math::random::Uniform<2> u2D(Eigen::Vector2d(0.0, 0.0),
                                            Eigen::Vector2d(1.0, 1.0));
    cv::Point dx(2,0);
    cv::Point dy(0,2);
    for(std::size_t i = 0 ; i < sample_count ; ++i) {
        Eigen::Vector2d sample = u2D.get();
        cv::Point pos(sample(0) * canvas.cols,
                      sample(1) * canvas.rows);
        cv::line(canvas, pos - dx, pos + dx, cv::Scalar(255));
        cv::line(canvas, pos - dy, pos + dy, cv::Scalar(255));
        cv::circle(canvas, pos, 1, cv::Scalar(0,255));
    }
    cv::imshow("display", display);
    cv::waitKey(0);

    /// normal distributed multivariate
    display.setTo(cv::Scalar());
    Eigen::Matrix2d n2D_rot =
            muse_amcl::getRotation(muse_amcl::math::angle::toRad(45.0));

    Eigen::Matrix2d n2D_cov = Eigen::Matrix2d::Zero();
    n2D_cov(0,0) = 0.0025;
    n2D_cov(1,1) = 0.025;
    n2D_cov = n2D_rot * n2D_cov * n2D_rot.transpose();

    Eigen::Vector2d n2D_mean = Eigen::Vector2d(0.5, 0.5);
    muse_amcl::math::random::Normal<2> n2D(n2D_mean,
                                           n2D_cov);

    for(std::size_t i = 0 ; i < sample_count ; ++i) {
        Eigen::Vector2d sample = n2D.get();
        cv::Point pos(sample(0) * canvas.cols,
                      sample(1) * canvas.rows);
        cv::line(canvas, pos - dx, pos + dx, cv::Scalar(255));
        cv::line(canvas, pos - dy, pos + dy, cv::Scalar(255));
        cv::circle(canvas, pos, 1, cv::Scalar(0,255));
    }
    cv::imshow("display", display);
    cv::waitKey(0);

    /// normal distributed multivariate pose generator
    using Metric = muse_amcl::pose_generation::Metric;
    using Radian = muse_amcl::pose_generation::Radian;
    Eigen::Vector3d n3d_mean = Eigen::Vector3d(0.5,0.5, 2 * M_PI);
    Eigen::Matrix3d n3d_cov = Eigen::Matrix3d::Zero();
    n3d_cov(0,0) = 0.0025;
    n3d_cov(1,1) = 0.025;
    n3d_cov(2,2) = 0.5;

    display.setTo(cv::Scalar());

    muse_amcl::pose_generation::Normal<Metric,Metric,Radian> n2Dp(n3d_mean, n3d_cov = Eigen::Matrix3d::Zero());

    for(std::size_t i = 0 ; i < sample_count ; ++i) {
        Eigen::Vector2d sample = n2D.get();
        cv::Point pos(sample(0) * canvas.cols,
                      sample(1) * canvas.rows);
        cv::line(canvas, pos - dx, pos + dx, cv::Scalar(255));
        cv::line(canvas, pos - dy, pos + dy, cv::Scalar(255));
        cv::circle(canvas, pos, 1, cv::Scalar(0,255));
    }
    cv::imshow("display", display);
    cv::waitKey(0);


    return 0;
}
