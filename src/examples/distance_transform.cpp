#include <opencv2/opencv.hpp>

struct DistanceTransform {

    template<std::size_t Dim>
    static cv::Mat createKernel()
    {
        static_assert((Dim % 2) != 0, "Has to be odd");

        cv::Mat kernel(Dim, Dim, CV_32FC1, cv::Scalar());
        int min_idx = -(int)Dim / 2;
        int max_idx =  Dim / 2;

        for(int i = min_idx ; i <= max_idx ; ++i) {
            for(int j = min_idx ; j <= max_idx ; ++j) {
                kernel.at<float>(i - min_idx, j - min_idx) = hypot(i,j);
            }
        }
        return kernel;
    }

    template<std::size_t Dim>
    static void apply(const cv::Mat &src,
                      cv::Mat &dst,
                      const float threshold = 1.f)
    {
        /// first preparation
        dst = cv::Mat(src.rows, src.cols, CV_32FC1, cv::Scalar(0));
        for(int i = 0 ; i < src.rows ; ++i) {
            for(int j = 0 ; j < src.cols ; ++j) {
                if(src.at<float>(i,j) < threshold) {
                    dst.at<float>(i,j) = std::numeric_limits<float>::max();
                } else {
                    dst.at<float>(i,j) = 0.f;
                }
            }
        }


        int margin = Dim / 2;
        int min_idx = -margin;
        int max_idx =  margin;
        cv::Mat kernel = createKernel<Dim>();

        int maxx = src.cols - 1;
        /// forward sweep
        for(int i = margin ; i < src.rows; ++i) {
            for(int j = 0 ; j < src.cols; ++j) {
                int kidy = 0;
                /// calc border distance

                for(int k = min_idx ; k < 0 ; ++k, ++kidy) {
                    int kidx = 0;
                    for(int l = min_idx ; l <= max_idx ; ++l, ++kidx) {
                        if(j + l < 0 ||
                                j + l > maxx)
                            continue;

                        float n = dst.at<float>(i + k, j + l);
                        if(n < std::numeric_limits<float>::max())
                            n+= kernel.at<float>(kidy, kidx);

                        if(n < dst.at<float>(i,j)) {
                            dst.at<float>(i,j) = n;
                        }
                    }

                }
                /// maybe kidy has to be incremented once again
                int kidx = 0;
                for(int l = min_idx ; l < 0 ; ++l, ++kidx) {
                    if(j + l < 0 ||
                            j + l > maxx)
                        continue;

                    float n = dst.at<float>(i, j + l);

                    if(n < std::numeric_limits<float>::max())
                        n += kernel.at<float>(kidy, kidx);

                    if(n < dst.at<float>(i,j)) {
                        dst.at<float>(i,j) = n;
                    }
                }
            }
        }

        /// backward sweep
        for(int i = src.rows - (margin + 1) ; i >= 0; --i) {
            for(int j = src.cols; j >= 0; --j) {
                int kidx = Dim / 2 + 1;
                int kidy = Dim / 2;
                for(int l = 1 ; l <= max_idx ; ++l, ++kidx) {
                    if(j + l < 0 ||
                            j + l > maxx)
                        continue;

                    float n = dst.at<float>(i, j + l);

                    if(n < std::numeric_limits<float>::max())
                        n+= kernel.at<float>(kidy, kidx);

                    if(n < dst.at<float>(i,j)) {
                        dst.at<float>(i,j) = n;
                    }
                }
                ++kidy;
                for(int k = 1 ; k <= max_idx ; ++k, ++kidy) {
                    int kidx = 0;
                    for(int l = min_idx ; l <= max_idx ; ++l, ++kidx) {
                        if(j + l < 0 ||
                                j + l > maxx)
                            continue;

                        float n = dst.at<float>(i + k, j + l);
                        if(n < std::numeric_limits<float>::max())
                            n+= kernel.at<float>(kidy, kidx);

                        if(n < dst.at<float>(i,j)) {
                            dst.at<float>(i,j) = n;
                        }
                    }
                }
            }
        }
    }
};




int main(int argc, char *argv[])
{
    { // 3x3
        cv::Mat kernel = DistanceTransform::createKernel<3>();
        cv::Mat display;
        cv::normalize(kernel, display, 0.0, 1.0, cv::NORM_MINMAX);
        cv::resize(display, display, cv::Size(200, 200));
        std::cout << kernel << std::endl;
        cv::imshow("kernel", display);
        cv::waitKey();
    }
    { // 5x5
        cv::Mat kernel = DistanceTransform::createKernel<5>();
        cv::Mat display;
        cv::normalize(kernel, display, 0.0, 1.0, cv::NORM_MINMAX);
        cv::resize(display, display, cv::Size(200, 200));
        std::cout << kernel << std::endl;
        cv::imshow("kernel", display);
        cv::waitKey();
    }
    {
        cv::Mat test = cv::Mat(200, 200, CV_8UC1, cv::Scalar(1.f));
        cv::rectangle(test, cv::Point(50, 50), cv::Point(150,150), cv::Scalar(0.f));
        cv::Mat display;
        cv::distanceTransform(test, display, CV_DIST_L2, 5);
        cv::normalize(display, display, 0, 1, cv::NORM_MINMAX);
        cv::imshow("distance1", display);
        cv::waitKey();
    }
    {
        cv::Mat test = cv::Mat(200, 200, CV_32FC1, cv::Scalar(0.f));
        cv::rectangle(test, cv::Point(50, 50), cv::Point(150,150), cv::Scalar(1.f));
        cv::Mat display;
        DistanceTransform::apply<5>(test, display);
        cv::normalize(display, display, 0, 1, cv::NORM_MINMAX);
        cv::imshow("distance2", display);
        cv::waitKey();
    }

    return 0;
}
