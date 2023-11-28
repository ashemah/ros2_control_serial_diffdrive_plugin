#include "ros2_control_serial_diffdrive_plugin/serial_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

void SerialComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    sendMsg("z\r", false);
}

void SerialComms::activateMotors()
{
    std::string response = sendMsg("a\r", false);
}

void SerialComms::deactivateMotors()
{
    std::string response = sendMsg("d\r", false);
}

void SerialComms::readEncoderValues(int &val_1, int &val_2)
{
    std::string response = sendMsg("e\r", true);

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
}

void SerialComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

void SerialComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str(), false);
}

std::string SerialComms::sendMsg(const std::string &msg_to_send, bool wait_for_response)
{
    serial_conn_.write(msg_to_send);

    if (wait_for_response)
    {
        std::string response = serial_conn_.readline();
        return response;

        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }
    else
    {
        return "";
    }
}