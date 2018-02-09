#ifndef PLOT_WEIGHTS_HPP
#define PLOT_WEIGHTS_HPP

#include <opencv2/opencv.hpp>

#include <muse_smc/samples/sample_set.hpp>
#include <muse_mcl_2d/state_space/state_space_description_2d.hpp>

#include <atomic>

namespace muse_mcl_2d {
class PlotWeights
{
public:
    using sample_set_t = muse_smc::SampleSet<StateSpaceDescription2D>;
    using Ptr = std::shared_ptr<PlotWeights>;

    PlotWeights(const std::size_t height,
                const std::size_t width,
                const std::string &window_name) :
        window_name_(window_name),
        canvas_(height, width, CV_8UC3, cv::Scalar::all(255)),
        stop_(false)
    {
    //     worker_ = std::thread([this](){loop();});
    }

    virtual ~PlotWeights()
    {
        stop_ = true;
        if(worker_.joinable())
            worker_.join();
    }

    void plot(const sample_set_t &sample_set)
    {
        std::cerr << "was here" << std::endl;

        /// wipe that
        canvas_.setTo(255);

        if(sample_set.getMaximumWeight() <= 0.0)
            return;

        const std::size_t size = sample_set.getSampleSize();
        const int width  = std::max(1, static_cast<int>(canvas_.cols/size));
        const double      height_scale = static_cast<double> (canvas_.rows) / sample_set.getMaximumWeight();
        const std::size_t step   = static_cast<std::size_t>(size / width);

        auto samples = sample_set.getSamples();
        for(std::size_t i = 0 ; i < size ; i += step) {
            std::vector<double> weights(step, -1.0);
            for(std::size_t j = i ; j < size ; ++j) {
                weights[j - i] = samples.at(j).weight;
            }
            std::sort(weights.begin(), weights.end(), std::greater<double>());
            for(std::size_t j = i ; j < size ; ++j) {
                int color = 255 * (1.0 - static_cast<double>(step - (j - i)) / static_cast<double>(step));
                cv::Rect r = cv::Rect(j,j+width,width, weights[j - i]*height_scale);
                cv::rectangle(canvas_, r, cv::Scalar(color), CV_FILLED);
            }
        }

        int y_mean = sample_set.getAverageWeight() * height_scale;
        cv::line(canvas_, cv::Point(0,y_mean), cv::Point(canvas_.cols - 1,y_mean), cv::Scalar(0,255));

        cv::flip(canvas_, canvas_, -1);

        std::unique_lock<std::mutex> l(mutex_display_);
        cv::copyMakeBorder(canvas_, display_, 5, 5, 5, 5, cv::BORDER_CONSTANT, cv::Scalar(127,127,127));
        cv::imshow(window_name_, display_);
        cv::waitKey(19);
    }

private:
    std::string         window_name_;
    cv::Mat             canvas_;

    std::thread         worker_;
    std::mutex          mutex_display_;
    cv::Mat             display_;
    std::atomic_bool    stop_;

    void loop()
    {
        cv::namedWindow(window_name_);
        while(!stop_) {
            if(!display_.empty()) {
                cv::imshow(window_name_, display_);
            }
            cv::waitKey(19);
        }
        cv::destroyWindow(window_name_);
    }


};
}

#endif // PLOT_WEIGHTS_HPP
