#include "../include/ads_worker.hpp"

#include <boost/algorithm/string.hpp>

ADS_Worker::ADS_Worker(const std::string& remote_ipv4, const std::string& remote_net_id)
{

    auto ids = splitNetIdToIntegers(remote_net_id);
    
    m_RemoteNetID = AmsNetId(
        ids[0],
        ids[1],
        ids[2],
        ids[3],
        ids[4],
        ids[5]
    );    

    m_RemoteIpV4 = remote_ipv4;

    m_Route = new AdsDevice(
        m_RemoteIpV4,
        m_RemoteNetID,
        AMSPORT_R0_PLC_TC3
    );

    m_AdsActTimestamp = new AdsVariable<uint32_t>(
        *m_Route,
        "GVL.fActTimeStamp"
    );

    AdsVariable <uint32_t> AdsLastTimeStamp{*m_Route, "GVL.fLastTimeStamp"};
    AdsVariable<bool> AdsHalt{*m_Route, "GVL.state_HALT"};
    AdsVariable<bool> AdsTimeout{*m_Route, "GVL.state_TIMEOUT"};

    AdsLastTimeStamp = ros::Time::now().sec;
    *m_AdsActTimestamp = ros::Time::now().sec;

    AdsHalt = false;
    AdsTimeout = false;
}

ADS_Worker::~ADS_Worker()
{
    if(m_Route != nullptr)
    {
        delete m_Route;
    }
    if(m_AdsActTimestamp != nullptr)
    {
        delete m_AdsActTimestamp;
    }
}

sensor_msgs::JointState ADS_Worker::readFromADS(const std::size_t num_joints)
{
    sensor_msgs::JointState jointState;

    std::vector<double> positionArray;
    std::vector<double> velocityArray;

    try
    {
        AdsVariable<std::vector<double>> positionVar{*m_Route, "GVL.fActPose_Arm"};
        AdsVariable<std::vector<double>> velocityVar{*m_Route, "GVL.fActVel_Arm"};

        positionArray = positionVar;
        velocityArray = velocityVar;
        
        
    }catch(const AdsException& ex){
        std::cout << "Error Code: " << ex.errorCode << std::endl;
        std::cout << "AdsException message: " << ex.what() << std::endl;
    }catch(const std::runtime_error& ex){
        std::cout << ex.what() << std::endl;
    }

    int j = 0;
    while(jointState.position.size() != num_joints && jointState.velocity.size() != num_joints)
    {
        jointState.position.push_back(positionArray[j]);
        jointState.velocity.push_back(velocityArray[j]);
        j += 1;
    }

    return jointState;

}

bool ADS_Worker::writeToADS(std::size_t num_joints, const std::vector<double>& pose, const std::vector<double>& vel)
{
    try{

        AdsVariable<std::vector<double>> adsPosVar{*m_Route, "GVL.fGoalPos_Arm"};
        AdsVariable<std::vector<double>> adsVelVar{*m_Route, "GVL.fGoalVel_Arm"};

        adsPosVar = pose;
        adsVelVar = vel;

    }catch(const AdsException& ex){
        std::cout << "Error Code: " << ex.errorCode << std::endl;
        std::cout << "AdsException message: " << ex.what() << std::endl;
        return false;
    }catch(const std::system_error& ex){
        std::cout << ex.what() << std::endl;
        return false;
    }

    return true;
}

template<typename T>
bool ADS_Worker::writeToADS(T var, const std::string& symbol_name)
{
    try{

        AdsVariable<T> adsVar{*m_Route, symbol_name};
        adsVar = var; 

    }catch(const AdsException& ex){
        std::cout << "Error Code: " << ex.errorCode << std::endl;
        std::cout << "AdsException message: " << ex.what() << std::endl;
        return false;
    }catch(const std::system_error& ex){
        std::cout << ex.what() << std::endl;
        return false;
    }

    return true;
}

std::vector<int> ADS_Worker::splitNetIdToIntegers(const std::string& remote_net_id)
{
    std::vector<std::string> idsStr;

    boost::split(
        idsStr,
        remote_net_id,
        boost::is_any_of(".")
    );

    std::vector<int> ids;

    for(auto id : idsStr)
    {
        ids.push_back(std::stoi(id));
    }

    return ids;
}
