/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "rplidar.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace rp::standalone::rplidar;

class RPlidarNode {
public:
    RPlidarNode();
    virtual ~RPlidarNode();
    void measure();
private:
    // methods
    void start();
    void stop();
    void publish_scan(
        rplidar_response_measurement_node_t *nodes,
        size_t node_count, ros::Time start,
        double scan_time, float angle_min, float angle_max);
    bool getRPLIDARDeviceInfo(RPlidarDriver * drv);
    bool checkRPLIDARHealth(RPlidarDriver * drv);
    bool stop_motor(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &res);
    bool start_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res);

    // attributes
    RPlidarDriver * drv;
    std::string serial_port;
    int serial_baudrate ;
    std::string frame_id;
    bool inverted;
    bool angle_compensate;
    ros::NodeHandle nh;
    // publications
    ros::Publisher scan_pub;
    // services
    ros::ServiceServer stop_motor_service;
    ros::ServiceServer start_motor_service;
};

RPlidarNode::RPlidarNode() :
    drv(NULL),
    serial_port(),
    serial_baudrate(),
    frame_id(),
    inverted(),
    angle_compensate(),
    nh("~"),
    // publications
    scan_pub(nh.advertise<sensor_msgs::LaserScan>("scan", 1000)),
    // services
    stop_motor_service(nh.advertiseService(
        "stop_motor", &RPlidarNode::stop_motor, this)),
    start_motor_service(nh.advertiseService(
        "start_motor", &RPlidarNode::start_motor, this))
{

    nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh.param<int>("serial_baudrate", serial_baudrate, 115200);
    nh.param<std::string>("frame_id", frame_id, "laser_frame");
    nh.param<bool>("inverted", inverted, false);
    nh.param<bool>("angle_compensate", angle_compensate, true);

    printf("RPLIDAR running on ROS package rplidar_ros\n"
           "SDK Version: "RPLIDAR_SDK_VERSION"\n");

    u_result     op_result;

    // create the driver instance
    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        throw std::runtime_error("Create Drive fail\n");
    }

    // make connection...
    if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
        char msg[50];
        snprintf(msg, 50, "Cannot bind to the specified port %s", serial_port.c_str());
        RPlidarDriver::DisposeDriver(drv);
        throw std::runtime_error(msg);
    }

    // get rplidar device info
    if (!getRPLIDARDeviceInfo(drv)) {
        RPlidarDriver::DisposeDriver(drv);
        throw std::runtime_error("Failed to get device info\n");
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        RPlidarDriver::DisposeDriver(drv);
        throw std::runtime_error("Health check failed\n");
    }

    // start scanning
    start();
}

void RPlidarNode::stop() {
    drv->stop();
    drv->stopMotor();
}

void RPlidarNode::start() {
    int motor_pwm = 0;
    nh.param<int>("motor_pwm", motor_pwm, 660);
    drv->startMotor();
    drv->setMotorPWM(motor_pwm);
    drv->startScan();
}

RPlidarNode::~RPlidarNode() {
    stop();
    RPlidarDriver::DisposeDriver(drv);
}

void RPlidarNode::publish_scan(
    rplidar_response_measurement_node_t *nodes,
    size_t node_count, ros::Time start,
    double scan_time, float angle_min, float angle_max)
{
    static int scan_count = 0;
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_count++;

    bool reversed = (angle_max > angle_min);
    if ( reversed ) {
      scan_msg.angle_min =  M_PI - angle_max;
      scan_msg.angle_max =  M_PI - angle_min;
    } else {
      scan_msg.angle_min =  M_PI - angle_min;
      scan_msg.angle_max =  M_PI - angle_max;
    }
    scan_msg.angle_increment =
        (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = 8.0;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);
    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
    if (!reverse_data) {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float) nodes[i].distance_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[i] = read_value;
            scan_msg.intensities[i] = (float) (nodes[i].sync_quality >> 2);
        }
    } else {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].distance_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[node_count-1-i] = read_value;
            scan_msg.intensities[node_count-1-i] = (float) (nodes[i].sync_quality >> 2);
        }
    }

    scan_pub.publish(scan_msg);
}

bool RPlidarNode::getRPLIDARDeviceInfo(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            fprintf(stderr, "Error, operation time out.\n");
        } else {
            fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
           , devinfo.firmware_version>>8
           , devinfo.firmware_version & 0xFF
           , (int)devinfo.hardware_version);
    return true;
}

bool RPlidarNode::checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { 
        printf("RPLidar health status : %d\n", healthinfo.status);
        
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected."
                            "Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve rplidar health code: %x\n", 
                        op_result);
        return false;
    }
}

bool RPlidarNode::stop_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
    ROS_DEBUG("Stop motor");
    stop();
    return true;
}

bool RPlidarNode::start_motor(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
    ROS_DEBUG("Start motor");
    start();
    return true;
}

void RPlidarNode::measure() {
    rplidar_response_measurement_node_t nodes[360*2];
    size_t   count = _countof(nodes);

    ros::Time start_scan_time = ros::Time::now();
    int op_result = drv->grabScanData(nodes, count);
    ros::Time end_scan_time = ros::Time::now();
    double scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

    if (op_result == RESULT_OK) {
        op_result = drv->ascendScanData(nodes, count);

        float angle_min = DEG2RAD(0.0f);
        float angle_max = DEG2RAD(359.0f);
        if (op_result == RESULT_OK) {
            if (angle_compensate) {
                const int angle_compensate_nodes_count = 360;
                const int angle_compensate_multiple = 1;
                int angle_compensate_offset = 0;
                rplidar_response_measurement_node_t angle_compensate_nodes[angle_compensate_nodes_count];
                memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_t));
                int i = 0, j = 0;
                for( ; i < count; i++ ) {
                    if (nodes[i].distance_q2 != 0) {
                        float angle = (float)((nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                        int angle_value = (int)(angle * angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                        for (j = 0; j < angle_compensate_multiple; j++) {
                            angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
                        }
                    }
                }

                publish_scan(angle_compensate_nodes, angle_compensate_nodes_count,
                         start_scan_time, scan_duration, angle_min, angle_max);
            } else {
                int start_node = 0, end_node = 0;
                int i = 0;
                // find the first valid node and last valid node
                while (nodes[i++].distance_q2 == 0);
                start_node = i-1;
                i = count -1;
                while (nodes[i--].distance_q2 == 0);
                end_node = i+1;

                angle_min = DEG2RAD((float)(nodes[start_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                angle_max = DEG2RAD((float)(nodes[end_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

                publish_scan(&nodes[start_node], end_node-start_node +1,
                         start_scan_time, scan_duration, angle_min, angle_max);
           }
        } else if (op_result == RESULT_OPERATION_FAIL) {
            // All the data is invalid, just publish them
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);

            publish_scan(nodes, count,
                         start_scan_time, scan_duration, angle_min, angle_max);
        }
    }
    ros::spinOnce();
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "rplidar_node");
    RPlidarNode node;
    while (ros::ok()) {
        node.measure();
    }
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
