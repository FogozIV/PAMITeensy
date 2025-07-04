//
// Created by fogoz on 04/07/2025.
//

#ifndef OBSTACLEHANDLER_H
#define OBSTACLEHANDLER_H
#include "robot/BaseRobot.h"
#include <memory>

#include "lidar/rplidar_driver.h"

struct LidarPositionData {
    double x;
    double y;
    std::chrono::high_resolution_clock::time_point timestamp;
    LidarPositionData(double _x, double _y, std::chrono::high_resolution_clock::time_point time_point) : x(_x), y(_y), timestamp(time_point) {}
};

class ObstacleHandler {
    std::shared_ptr<RPLidar> lidar;
    std::shared_ptr<BaseRobot> baseRobot;
    std::vector<LidarPositionData> positions;
    bool scanning = false;
    rplidar_response_measurement_node_hq_t* nodes;
    size_t nodeCount;
public:
    ObstacleHandler(const std::shared_ptr<BaseRobot> &baseRobot, Stream& serial, size_t nodeCount=512);

    ~ObstacleHandler();

    void update();

    void startScanExpress();
};



#endif //OBSTACLEHANDLER_H
