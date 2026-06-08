#include <mc_rtc/logging.h>
#include <mutex>
#include <thread>
#include <mc_rtc_ros/ros.h>

#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

/**
 * @brief Describes data obtained by a subscriber, along with the time since it
 * was obtained
 *
 * @tparam Data Type of the data obtained by the subscriber
 */
template<typename Data>
struct SubscriberData
{
  bool isValid() const noexcept
  {
    return time_ <= maxTime_;
  }

  void operator=(const SubscriberData<Data> & data)
  {
    value_ = data.value_;
    time_ = data.time_;
    maxTime_ = data.maxTime_;
  }

  const Data & value() const noexcept
  {
    return value_;
  }

  void tick(double dt)
  {
    time_ += dt;
  }

  void maxTime(double t)
  {
    maxTime_ = t;
  }

  double time() const noexcept
  {
    return time_;
  }

  double maxTime() const noexcept
  {
    return maxTime_;
  }

  void value(const Data & data)
  {
    value_ = data;
    time_ = 0;
  }

private:
  Data value_;
  double time_ = std::numeric_limits<double>::max();
  double maxTime_ = std::numeric_limits<double>::max();
};

/**
 * @brief Simple interface for subscribing to data. It is assumed here that the
 * data will be acquired in a separate thread (e.g ROS spinner) and
 * setting/getting the data is thread-safe.
 * Don't forget to call tick(double dt) to update the time and value
 */
template<typename Data>
struct Subscriber
{
  /** Update time and value */
  void tick(double dt)
  {
    data_.tick(dt);
  }

  void maxTime(double t)
  {
    data_.maxTime(t);
  }

  const SubscriberData<Data> data() const noexcept
  {
    std::lock_guard<std::mutex> l(valueMutex_);
    return data_;
  }

protected:
  void value(const Data & data)
  {
    std::lock_guard<std::mutex> l(valueMutex_);
    data_.value(data);
  }

  void value(const Data && data)
  {
    std::lock_guard<std::mutex> l(valueMutex_);
    data_.value(data);
  }

private:
  SubscriberData<Data> data_;
  mutable std::mutex valueMutex_;
};

template<typename ROSMessageType, typename TargetType>
struct ROSSubscriber : public Subscriber<TargetType>
{
  template<typename ConverterFun>
  ROSSubscriber(ConverterFun && fun) : converter_(fun)
  {
  }

  void subscribe(std::shared_ptr<rclcpp::Node> & node, const std::string & topic, const unsigned bufferSize = 1)
  {
    sub_ = node->create_subscription<ROSMessageType>(topic, bufferSize, std::bind(&ROSSubscriber::callback, this, std::placeholders::_1));
  }

  std::string topic() const
  {
    return sub_->get_topic_name();
  }

  const rclcpp::Subscription<ROSMessageType> & subscriber() const
  {
    return sub_;
  }

protected:
  void callback(const std::shared_ptr<const ROSMessageType> & msg)
  {
    this->value(converter_(*msg));
  }

protected:
    typename rclcpp::Subscription<ROSMessageType>::SharedPtr sub_;
    std::function<TargetType(const ROSMessageType &)> converter_;
};

struct ROSFloatSubscriber : public ROSSubscriber<std_msgs::msg::Float64, double>
{
  ROSFloatSubscriber() : ROSSubscriber([](const std_msgs::msg::Float64 & msg) { return msg.data; }) {}
};