// MIT License

// Copyright (c) 2023  Miguel Ángel González Santamarta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef CONTACTILE_NODE_HPP_
#define CONTACTILE_NODE_HPP_

#include <memory>
#include <vector>

#include <contactile_msgs/msg/button_sensor_state.hpp>
#include <contactile_msgs/srv/bias_request.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef __unix__
typedef unsigned char byte;
#endif

#ifndef PTSDKCONSTANTS_H
#include <PTSDKConstants.h>
#endif
#ifndef PTSDKLISTENER_H
#include <PTSDKListener.h>
#endif
#ifndef PTSDKSENSOR_H
#include <PTSDKSensor.h>
#endif

namespace contactile_ros {

class ContactileNode : public rclcpp::Node {
public:
  ContactileNode();
  ~ContactileNode() { listener_.disconnect(); }

protected:
  void update_data();

private:
  int num_sensors_;
  int baud_rate_;
  int parity_;
  int byte_size_;
  std::string com_port_;

  PTSDKListener listener_;
  std::vector<std::unique_ptr<PTSDKSensor>> sensors_;

  std::vector<
      rclcpp::Publisher<contactile_msgs::msg::ButtonSensorState>::SharedPtr>
      sensor_pubs_;
  rclcpp::Service<contactile_msgs::srv::BiasRequest>::SharedPtr
      send_bias_request_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  void send_bias_request_cb(
      const std::shared_ptr<contactile_msgs::srv::BiasRequest::Request> req,
      std::shared_ptr<contactile_msgs::srv::BiasRequest::Response> res);
  void load_params();
};

} // namespace contactile_ros

#endif // CONTACTILE_NODE_HPP_
