#ifndef PUBLISHER_H
#define PUBLISHER_H

#include "std_msgs/msg/string.hpp"
#include "lidar_log_reader.hpp"

namespace ros2_com
{

    class OdometryPublisher : public rclcpp::Node
    {
        using Storage = boost::interprocess::managed_shared_memory;

        struct OdomData
        {
            std::mutex mtx;
            int32_t front;
            int32_t back;
            bool newData{ false };
        };

        public:
            OdometryPublisher();
            ~OdometryPublisher();
        private:
            rclcpp::TimerBase::SharedPtr m_timer;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
            size_t const m_count;
            std::unique_ptr<ShmemConsumer<OdomData, Storage>> m_odomConsumer;

            void timerCallback();
            bool reconnect();
            //bool odomLogTask();

            OdomData m_data;
            LidarLogReader m_log;
    };
}

#endif //PUBLISHER_H
