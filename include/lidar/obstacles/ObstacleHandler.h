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
    LidarPositionData(double _x, double _y) : x(_x), y(_y) {}
};

class ObstacleHandler {
    std::shared_ptr<RPLidar> lidar;
    std::shared_ptr<BaseRobot> baseRobot;
    std::vector<LidarPositionData> positions;
    bool scanning = false;
    rplidar_response_measurement_node_hq_t* nodes;
    size_t nodeCount;
    uint8_t motorPin;
    bool express;
public:
    ObstacleHandler(const std::shared_ptr<BaseRobot> &baseRobot, Stream& serial, uint8_t motorPin, size_t nodeCount=512, bool express=false);

    ~ObstacleHandler();

    void update();

    void startScanExpress();

    void startScanNormal();

    void startScan();
};



#endif //OBSTACLEHANDLER_H
