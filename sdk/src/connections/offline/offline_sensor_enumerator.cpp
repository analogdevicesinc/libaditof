#include "offline_sensor_enumerator.h"
#include "offline_depth_sensor.h"

OfflineSensorEnumerator::OfflineSensorEnumerator() {}

aditof::Status OfflineSensorEnumerator::getDepthSensors(
    std::vector<std::shared_ptr<aditof::DepthSensorInterface>> &depthSensors) {
    depthSensors.clear();

    depthSensors.emplace_back(std::make_shared<OfflineDepthSensor>());
    return aditof::Status::OK;
}

aditof::Status OfflineSensorEnumerator::searchSensors() {
    return aditof::Status::OK;
}

aditof::Status
OfflineSensorEnumerator::getUbootVersion(std::string &uBootVersion) const {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
OfflineSensorEnumerator::getKernelVersion(std::string &kernelVersion) const {
    return aditof::Status::UNAVAILABLE;
}

aditof::Status
OfflineSensorEnumerator::getSdVersion(std::string &sdVersion) const {
    return aditof::Status::UNAVAILABLE;
}
