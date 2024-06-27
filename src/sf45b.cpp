
#include "common.h"
#include "lwNx.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <math.h>

struct lwSf45Params {
    int32_t updateRate;
    int32_t cycleDelay;
    float lowAngleLimit;
    float highAngleLimit;
};

void validateParams(lwSf45Params *Params) {
    if (Params->updateRate < 1)
        Params->updateRate = 1;
    else if (Params->updateRate > 12)
        Params->updateRate = 12;

    if (Params->cycleDelay < 5)
        Params->cycleDelay = 5;
    else if (Params->cycleDelay > 2000)
        Params->cycleDelay = 2000;

    if (Params->lowAngleLimit < -160)
        Params->lowAngleLimit = -160;
    else if (Params->lowAngleLimit > -10)
        Params->lowAngleLimit = -10;

    if (Params->highAngleLimit < 10)
        Params->highAngleLimit = 10;
    else if (Params->highAngleLimit > 160)
        Params->highAngleLimit = 160;
}

int driverStart(lwSerialPort **Serial, const char *PortName, int32_t BaudRate) {
    platformInit();

    lwSerialPort *serial = platformCreateSerialPort();
    *Serial = serial;
    if (!serial->connect(PortName, BaudRate)) {
        ROS_ERROR("Could not establish serial connection on %s", PortName);
        return 1;
    };

    if (!lwnxCmdWriteUInt32(serial, 30, 0)) {
        return 1;
    }

    char modelName[16];
    if (!lwnxCmdReadString(serial, 0, modelName)) {
        return 1;
    }

    uint32_t hardwareVersion;
    if (!lwnxCmdReadUInt32(serial, 1, &hardwareVersion)) {
        return 1;
    }

    uint32_t firmwareVersion;
    if (!lwnxCmdReadUInt32(serial, 2, &firmwareVersion)) {
        return 1;
    }
    char firmwareVersionStr[16];
    lwnxConvertFirmwareVersionToStr(firmwareVersion, firmwareVersionStr);

    char serialNumber[16];
    if (!lwnxCmdReadString(serial, 3, serialNumber)) {
        return 1;
    }

    ROS_INFO("Model: %.16s", modelName);
    ROS_INFO("Hardware: %d", hardwareVersion);
    ROS_INFO("Firmware: %.16s (%d)", firmwareVersionStr, firmwareVersion);
    ROS_INFO("Serial: %.16s", serialNumber);

    return 0;
}

int driverScanStart(lwSerialPort *Serial, lwSf45Params *Params) {
    if (!lwnxCmdWriteUInt32(Serial, 27, 0x101)) {
        return 1;
    }

    if (!lwnxCmdWriteUInt8(Serial, 66, Params->updateRate)) {
        return 1;
    }

    if (!lwnxCmdWriteUInt16(Serial, 85, Params->cycleDelay)) {
        return 1;
    }

    if (!lwnxCmdWriteFloat(Serial, 98, Params->lowAngleLimit)) {
        return 1;
    }

    if (!lwnxCmdWriteFloat(Serial, 99, Params->highAngleLimit)) {
        return 1;
    }

    if (!lwnxCmdWriteUInt32(Serial, 30, 5)) {
        return 1;
    }

    return 0;
}

struct lwDistanceResult {
    float x;
    float y;
    float z;
};

int driverScan(lwSerialPort *Serial, lwDistanceResult *DistanceResult) {
    lwResponsePacket response;

    if (lwnxRecvPacket(Serial, 44, &response, 1000)) {
        int16_t distanceCm = (response.data[5] << 8) | response.data[4];
        int16_t angleHundredths = (response.data[7] << 8) | response.data[6];

        float distance = distanceCm / 100.0f;
        float angle = angleHundredths / 100.0f;
        float faceAngle = (angle + 180) * M_PI / 180.0;

        DistanceResult->x = distance * -cos(faceAngle);
        DistanceResult->y = distance * sin(faceAngle);
        DistanceResult->z = 0;

        return 1;
    }

    return 0;
}

void initializePointCloudMsg(sensor_msgs::PointCloud2 &msg, const std::string &frameId, int maxPointsPerMsg) {
    msg.header.frame_id = frameId;
    msg.height = 1;
    msg.width = maxPointsPerMsg;

    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = 7;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = 7;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = 7;
    msg.fields[2].count = 1;

    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = 12 * maxPointsPerMsg;
    msg.is_dense = true;

    msg.data = std::vector<uint8_t>(maxPointsPerMsg * 12);
}

void publishPointCloud(ros::Publisher &pointCloudPub, const std::vector<lwDistanceResult> &distanceResults, const std::string &frameId) {
    sensor_msgs::PointCloud2 pointCloudMsg;
    int numPoints = distanceResults.size();
    initializePointCloudMsg(pointCloudMsg, frameId, numPoints);

    for (int i = 0; i < numPoints; ++i) {
        memcpy(&pointCloudMsg.data[i * 12], &distanceResults[i], sizeof(float) * 3);
    }

    pointCloudMsg.header.stamp = ros::Time::now();
    pointCloudPub.publish(pointCloudMsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sf45b");

    ros::NodeHandle n;

    lwSerialPort *serial = 0;

    std::string portName;
    n.param(std::string("port_name"), portName, std::string("/dev/ttyACM0"));
    int32_t baudRate;
    n.param(std::string("baud_rate"), baudRate, 921600);
    std::string frameId;
    n.param(std::string("frame_id"), frameId, std::string("lidar_main_link"));
    std::string pclRawTopic;
    n.param(std::string("pcl_raw_topic"), pclRawTopic, std::string("/lidar_application/point_cloud_raw_v2"));
    float scanWindowTime;
    n.param(std::string("scan_window_time"), scanWindowTime, 0.5f);
    ros::Publisher pointCloudPub = n.advertise<sensor_msgs::PointCloud2>(pclRawTopic, 10);

    lwSf45Params params;
    n.param(std::string("updateRate"), params.updateRate, 5000);
    n.param(std::string("cycleDelay"), params.cycleDelay, 5);
    n.param(std::string("lowAngleLimit"), params.lowAngleLimit, -15.0f);
    n.param(std::string("highAngleLimit"), params.highAngleLimit, 15.0f);
    validateParams(&params);

    int maxPointsPerMsg;
    n.param(std::string("maxPoints"), maxPointsPerMsg, 100);
    if (maxPointsPerMsg < 1)
        maxPointsPerMsg = 1;

    ROS_INFO("Starting SF45B node");

    if (driverStart(&serial, portName.c_str(), baudRate) != 0) {
        ROS_ERROR("Failed to start driver");
        return 1;
    }

    if (driverScanStart(serial, &params) != 0) {
        ROS_ERROR("Failed to start scan");
        return 1;
    }

    std::vector<lwDistanceResult> distanceResults(maxPointsPerMsg);
    auto startTime = std::chrono::steady_clock::now();

    while (ros::ok()) {
        lwDistanceResult distanceResult;
        int status = driverScan(serial, &distanceResult);

        if (status != 0) {
            distanceResults.push_back(distanceResult);
        }

        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedTime = currentTime - startTime;

        if (elapsedTime.count() >= scanWindowTime) {
            if (!distanceResults.empty()) {
                publishPointCloud(pointCloudPub, distanceResults, frameId);
                distanceResults.clear();
            }
            startTime = std::chrono::steady_clock::now();
        }

        ros::spinOnce();
    }

    return 0;
}
