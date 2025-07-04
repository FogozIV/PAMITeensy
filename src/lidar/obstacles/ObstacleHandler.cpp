//
// Created by fogoz on 04/07/2025.
//

#include <lidar/obstacles/ObstacleHandler.h>

ObstacleHandler::ObstacleHandler(const std::shared_ptr<BaseRobot> &baseRobot, Stream& stream, size_t nodeCount) {
    lidar = std::make_shared<RPLidar>(stream);
    this->baseRobot = baseRobot;
    lidar->begin();
    this->nodeCount = nodeCount;
    this->nodes = new rplidar_response_measurement_node_hq_t[this->nodeCount];
}

ObstacleHandler::~ObstacleHandler() {
    delete[] this->nodes;
}

void ObstacleHandler::update() {
    Position currentPosition = this->baseRobot->getCurrentPosition();
    auto r = this->lidar->loopScanExpressData();
    if (IS_FAIL(r)) {
        scanning = false;
        startScanExpress();
        return;
    }
    auto nodecount = this->nodeCount;
    auto result = this->lidar->grabScanExpressData(this->nodes, nodecount);
    if (IS_OK(result)) {
        for (size_t i = 0; i < nodecount; ++i) {
            double angleInDegrees = nodes[i].angle_z_q14 * 90.0 / (1<<14);
            double distance = static_cast<double>(nodes[i].dist_mm_q2) / (1<<2);
            streamSplitter.println(angleInDegrees);
            streamSplitter.println(distance);
            if(distance <100)
                continue;
            if(distance > 3.7)
                continue;
            auto pos = currentPosition.offsetRelative(distance, Angle::fromDegrees(angleInDegrees));
            this->positions.emplace_back(pos.getX(), pos.getY(), std::chrono::high_resolution_clock::now());
            streamSplitter.print(pos.getX());
            streamSplitter.print(", ");
            streamSplitter.print(pos.getY());
            streamSplitter.print(", ");
            uint64_t data = std::chrono::high_resolution_clock::now().time_since_epoch().count();
            streamSplitter.println(data);
        }
    }
}

void ObstacleHandler::startScanExpress() {
    if (!scanning) {
        auto result = this->lidar->startScanExpress(true, RPLIDAR_CONF_SCAN_COMMAND_EXPRESS);
        threads.delay(5000);
        streamSplitter.println(this->lidar->isScanning());
        streamSplitter.println(this->lidar->isConnected());
        streamSplitter.println(result);
        scanning = true;
    }
}
