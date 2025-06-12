//
// Created by fogoz on 12/06/2025.
//

#include <controller/calibration_methodo/ClothoidBenchmark.h>
void ClothoidBenchmark::save() {

}

void ClothoidBenchmark::printStatus(Stream &stream) {
    stream.printf("Benchmark In progress\r\n");
    stream.printf("Benchmark score : %f\r\n", error);
}

ClothoidBenchmark::ClothoidBenchmark(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<Mutex> &sdMutex) : CalibrationMethodo(robot, sdMutex) {
    assert(robot->getRobotType() == PAMIRobotType);
    this->robot = std::static_pointer_cast<PAMIRobot>(robot);
}

void ClothoidBenchmark::start() {
    CalibrationMethodo::start();
    streamSplitter.println("Starting benchmark of current controller");
    robot->clearTarget();
    robot->controllerClear();
    robot->resetTargetsCurvilinearAndAngular();
    this->openFile((String("BenchmarkClothoide") + String(rtc_get())+ ".bin").c_str(), 8192*2*2);
    buffer->write_raw((uint64_t)BinaryFileType::BENCHMARK_DISTANCE_V_0_1);
    buffer->write_raw(static_cast<uint8_t>(robot->getControllerDistance()->getType()));
    //curveTarget = std::make_shared<ContinuousCurveTarget<CalculatedQuadramp>>(robot, );
    benchmarkComputeHook = robot->addEndComputeHooks([this]() {
        if (!robot->isControlDisabled() && !robot->isDoneDistance()) {
            double current_error =0;
            error += current_error;
            dt += robot->getDT();
            lock_guard lg(bufferMutex);
            buffer->write_raw(current_error);
            buffer->write_raw(error);
            buffer->write_raw(dt);
            buffer->write_raw(robot->getDT());
            buffer->write_raw(robot->getTranslationalPosition());
            buffer->write_raw(robot->getTranslationalTarget());
            buffer->write_raw(robot->getLeftMotor()->getPWM());
            buffer->write_raw(robot->getRightMotor()->getPWM());
            if(robot->getControllerDistance()->getType() == BasicControllerType::PID){
                std::shared_ptr<PID> pid = std::static_pointer_cast<PID>(robot->getControllerDistance());
                buffer->write_raw(pid->getUp());
                buffer->write_raw(pid->getUi());
                buffer->write_raw(pid->getUd());
            }else if(robot->getControllerDistance()->getType() == BasicControllerType::PIDSpeedFeedForward){
                std::shared_ptr<PIDSpeedFeedForward> pid = std::static_pointer_cast<PIDSpeedFeedForward>(robot->getControllerDistance());
                buffer->write_raw(pid->getUp());
                buffer->write_raw(pid->getUi());
                buffer->write_raw(pid->getUd());
                buffer->write_raw(pid->getUff());
            }
            if(robot->getControllerDistanceAngle() ->getType() == BasicControllerType::PID){

            }else if(robot->getControllerDistance()->getType() == BasicControllerType::PIDSpeedFeedForward){

            }
            //buffer->printf("%f; %f; %f; %f\r\n", current_error, error, dt, robot->getDT());
        }
    });
}


void ClothoidBenchmark::stop() {
    CalibrationMethodo::stop();
}
