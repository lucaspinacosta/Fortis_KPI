#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <cmath>
#include <string>
#include <limits>
#include <ros/ros.h>

class MonitorArmLatency
{
public:
    MonitorArmLatency(int num_trials = 100, const std::string &output_csv = "arm_latency_log.csv")
        : num_trials_(num_trials), output_csv_(output_csv), trial_active_(false)
    {
        ros::NodeHandle nh;
        joints_sub_ = nh.subscribe("/ugv0/telehandler/joints", 10, &MonitorArmLatency::jointsCallback, this);
        telejoy_pub_ = nh.advertise<std_msgs::Int8MultiArray>("/ugv0/telehandler/telejoy", 10);
    }

    void measureLatency(const std::vector<int> &teleop_values = {1, 0, 0, 0, 0, 0})
    {
        ros::Duration(1.0).sleep(); // Allow some time for ROS topics to warm up
        ros::Rate rate(5);

        latencies_.clear();

        for (int t = 0; t < num_trials_; ++t)
        {
            start_position_.clear();
            trial_active_ = true;

            publishTelejoy(teleop_values);

            start_pub_time_ = ros::Time::now();

            ros::Time start_loop = ros::Time::now();
            while (trial_active_ && (ros::Time::now() - start_loop).toSec() < 1.0)
            {
                rate.sleep();
                ros::spinOnce();
            }
        }

        writeCSV();
        printStats();
    }

private:
    ros::Subscriber joints_sub_;
    ros::Publisher telejoy_pub_;
    int num_trials_;
    std::string output_csv_;
    std::vector<double> latencies_; // seconds (will convert to ms when writing/reporting)
    ros::Time start_pub_time_;
    bool trial_active_;
    std::vector<float> start_position_;

    void jointsCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        if (!trial_active_)
            return;

        if (start_position_.empty())
        {
            start_position_.assign(msg->data.begin(), msg->data.end());
            return;
        }

        std::vector<float> current(msg->data.begin(), msg->data.end());
        if (current != start_position_)
        {
            ros::Time t_sub = ros::Time::now();
            double latency_sec = (t_sub - start_pub_time_).toSec();
            latencies_.push_back(latency_sec);
            ROS_INFO("Trial %zu latency: %.4f s", latencies_.size(), latency_sec);
            trial_active_ = false;
        }
    }

    void publishTelejoy(const std::vector<int> &teleop_values)
    {
        start_pub_time_ = ros::Time::now();
        std_msgs::Int8MultiArray msg;
        msg.data = teleop_values;
        telejoy_pub_.publish(msg);
    }

    void writeCSV()
    {
        std::ofstream file(output_csv_);
        if (!file.is_open())
        {
            ROS_ERROR("Failed to open CSV file: %s", output_csv_.c_str());
            return;
        }
        file << "trial,latency_ms\n";
        for (size_t i = 0; i < latencies_.size(); ++i)
        {
            file << (i + 1) << "," << std::fixed << std::setprecision(3) << latencies_[i] * 1000.0 << "\n";
        }
        // Mean, max, stddev
        double mean = meanLatency() * 1000.0;
        double max = maxLatency() * 1000.0;
        double stddev = stdDevLatency() * 1000.0;
        file << "mean," << std::fixed << std::setprecision(3) << mean << "\n";
        file << "max," << std::fixed << std::setprecision(3) << max << "\n";
        file << "stddev," << std::fixed << std::setprecision(3) << stddev << "\n";
        file.close();
    }

    void printStats()
    {
        double mean = meanLatency() * 1000.0;
        double max = maxLatency() * 1000.0;
        double stddev = stdDevLatency() * 1000.0;
        ROS_INFO("\n--- Latency Results ---");
        ROS_INFO("Trials: %zu", latencies_.size());
        ROS_INFO("Mean: %.1f ms", mean);
        ROS_INFO("Max: %.1f ms", max);
        ROS_INFO("Std Dev: %.1f ms", stddev);
    }

    double meanLatency() const
    {
        if (latencies_.empty())
            return 0.0;
        return std::accumulate(latencies_.begin(), latencies_.end(), 0.0) / latencies_.size();
    }
    double maxLatency() const
    {
        if (latencies_.empty())
            return 0.0;
        return *std::max_element(latencies_.begin(), latencies_.end());
    }
    double stdDevLatency() const
    {
        if (latencies_.size() < 2)
            return 0.0;
        double mean = meanLatency();
        double accum = 0.0;
        for (const auto &lat : latencies_)
        {
            accum += (lat - mean) * (lat - mean);
        }
        return std::sqrt(accum / (latencies_.size() - 1));
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "monitor_arm_latency");
    MonitorArmLatency monitor(100, "arm_latency_log.csv");
    monitor.measureLatency(); // default teleop_values
    return 0;
}
