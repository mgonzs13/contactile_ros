// MIT License

// Copyright (c) 2023 Miguel Ángel González Santamarta

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

#include <chrono>

#include <contactile_ros/contactile_node.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace contactile_ros;

ContactileNode::ContactileNode()
    : rclcpp::Node("contactile_node"), listener_(false) {

  RCLCPP_INFO(this->get_logger(), "Loading parameters...");
  this->load_params();

  // create sensors and add to listener
  RCLCPP_INFO(this->get_logger(), "Creating sensors...");

  for (int i = 0; i < num_sensors_; i++) {

    RCLCPP_INFO(this->get_logger(), "Creating sensors %d...", i);
    auto sensor = std::make_unique<PTSDKSensor>();

    RCLCPP_INFO(this->get_logger(), "Adding sensor %d to listener...", i);
    this->listener_.addSensor(sensor.get());

    RCLCPP_INFO(this->get_logger(), "Added sensor %d to listener!", i);
    this->sensors_.push_back(std::move(sensor));

    // publishers for sensor
    std::string topic = "sensor_" + std::to_string(i);
    auto sensor_pub =
        this->create_publisher<contactile_msgs::msg::ButtonSensorState>(topic,
                                                                        1);
    this->sensor_pubs_.push_back(sensor_pub);
  }

  // start listener
  RCLCPP_INFO(this->get_logger(), "Connecting to %s port...",
              this->com_port_.c_str());

  if (this->listener_.connect(this->com_port_.c_str(), this->baud_rate_,
                              this->parity_, char(this->byte_size_))) {
    RCLCPP_FATAL(this->get_logger(), "Failed to connect to port: %s",
                 this->com_port_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Connected to port: %s",
                this->com_port_.c_str());
  }

  // start services
  RCLCPP_INFO(this->get_logger(), "Starting services...");
  std::string service_name = "send_bias_request";
  this->send_bias_request_srv_ =
      this->create_service<contactile_msgs::srv::BiasRequest>(
          service_name,
          std::bind(&ContactileNode::send_bias_request_cb, this, _1, _2));
  RCLCPP_INFO(this->get_logger(), "Started %s service", service_name.c_str());

  // start timer
  this->timer_ = this->create_wall_timer(
      33ms, std::bind(&ContactileNode::update_data, this));
}

void ContactileNode::load_params() {

  this->declare_parameters<int>("", {
                                        {"hub_id", 0},
                                        {"num_sensors", 10},
                                        {"baud_rate", 9600},
                                        {"parity", 0},
                                        {"byte_size", 8},
                                    });
  this->declare_parameters<std::string>("", {{"com_port", "/dev/ttyACM0"}});

  this->get_parameter("num_sensors", this->num_sensors_);
  this->get_parameter("baud_rate", this->baud_rate_);
  this->get_parameter("parity", this->parity_);
  this->get_parameter("byte_size", this->byte_size_);
  this->get_parameter("com_port", this->com_port_);

  if (num_sensors_ > MAX_NSENSOR or num_sensors_ < 1) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Invalid number of sensors!  %d selected (must be no more than %d)",
        num_sensors_, MAX_NSENSOR);
  } else {
    RCLCPP_INFO(this->get_logger(), "Using %d sensor/s", num_sensors_);
  }

  RCLCPP_INFO(this->get_logger(), "Loaded parameters.\n");
}

void ContactileNode::update_data() {

  if (this->num_sensors_ == 0) {
    return;
  }

  // Read next sample from COM port
  this->listener_.readNextSample();

  for (size_t sensor_id = 0; sensor_id < this->sensors_.size(); sensor_id++) {

    contactile_msgs::msg::ButtonSensorState ss_msg;
    ss_msg.header.stamp = this->now();

    long timestamp_us = sensors_[sensor_id]->getTimestamp_us();
    ss_msg.tus = timestamp_us;

    double globalForce[NDIM];

    // Read global forces
    this->sensors_[sensor_id]->getGlobalForce(globalForce);
    ss_msg.gf_x = static_cast<float>(globalForce[X_IND]);
    ss_msg.gf_y = static_cast<float>(globalForce[Y_IND]);
    ss_msg.gf_z = static_cast<float>(globalForce[Z_IND]);

    // Publish ButtonSensorState message
    this->sensor_pubs_[sensor_id]->publish(ss_msg);
  }
}

void ContactileNode::send_bias_request_cb(
    const std::shared_ptr<contactile_msgs::srv::BiasRequest::Request> req,
    std::shared_ptr<contactile_msgs::srv::BiasRequest::Response> res) {

  RCLCPP_INFO(this->get_logger(), "sendBiasRequest callback");
  res->result = this->listener_.sendBiasRequest();
  rclcpp::sleep_for(std::chrono::seconds(1));
}
