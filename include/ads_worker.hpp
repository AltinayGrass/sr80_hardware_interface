#ifndef ADS_WORKER_HPP
#define ADS_WORKER_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <vector>
#include <array>
#include <tuple>

#include "AdsLib.h"
#include "AdsVariable.h"


class ADS_Worker
{
    public:

    ADS_Worker(const std::string& remote_ipv4, const std::string& remote_net_id);
    ~ADS_Worker();

    bool writeToADS(std::size_t num_joints, const std::vector<double>& pose, const std::vector<double>& vel);
    template<typename T>
    bool writeToADS(T var, const std::string& symbol_name);

    sensor_msgs::JointState readFromADS(const std::size_t num_joints);

    private:

    std::vector<double> m_JointPositions;

    AmsNetId m_RemoteNetID;

    std::string m_RemoteIpV4;

    AdsDevice* m_Route;

    AdsVariable<uint32_t>* m_AdsActTimestamp;

    std::vector<int> splitNetIdToIntegers(const std::string& remote_net_id);

};

#endif