#include "../include/ads_worker.hpp"

#include <boost/algorithm/string.hpp>

ADS_Worker::ADS_Worker(const std::string& remote_ipv4, const std::string& remote_net_id)
{
    //ROS_INFO("Debug %s", "ADS worker crate");
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
    //ROS_INFO("Debug %s", "ADS worker read from ads1");
    std::array<double, 6> positionArray;
    std::array<double, 6> velocityArray;

    try
    {
        //ROS_INFO("Debug %s", "ADS worker read from ads2");
        AdsVariable<std::array<double, 6>> positionVar{*m_Route, "GVL.fActPose_Arm"};
        AdsVariable<std::array<double, 6>> velocityVar{*m_Route, "GVL.fActVel_Arm"};

        positionArray = positionVar;
        velocityArray = velocityVar;
        //ROS_INFO("Debug %s", "ADS worker read from ads2.5");
        
    }catch(const AdsException& ex){
        std::cout << "Error Code: " << ex.errorCode << std::endl;
        std::cout << "AdsException message: " << ex.what() << std::endl;
    }catch(const std::runtime_error& ex){
        std::cout << ex.what() << std::endl;
    }
    //ROS_INFO("Debug %s , %ld  , %ld , %ld", "ADS worker read from ads3",num_joints,jointState.position.size(),jointState.velocity.size());
    int j = 0;
    while(jointState.position.size() != num_joints && jointState.velocity.size() != num_joints)
    {
        jointState.position.push_back(positionArray[j]);
        jointState.velocity.push_back(velocityArray[j]);
        j += 1;
    }
    //ROS_INFO("Debug %s", "ADS worker read from ads4");
    return jointState;

}

bool ADS_Worker::writeToADS(std::size_t num_joints, const std::vector<double>& pose, const std::vector<double>& vel, const std::vector<double>& acc)
{
   
        std::array<double, 6> posToWrite;
        std::array<double, 6> velToWrite;
        std::array<double, 6> accToWrite;
    try{

        AdsVariable<std::array<double, 6>> adsPosVar{*m_Route, "GVL.fGoalPos_Arm"};
        AdsVariable<std::array<double, 6>> adsVelVar{*m_Route, "GVL.fGoalVel_Arm"};
        AdsVariable<std::array<double, 6>> adsAccVar{*m_Route, "GVL.fGoalAcc_Arm"};
        
        int j =0;

        while (j<6)
        {
            posToWrite[j] = pose.at(j);
            velToWrite[j] = vel.at(j);
            accToWrite[j] = acc.at(j);
            j+=1;
        }    

        adsPosVar=posToWrite;
        adsVelVar=velToWrite;
        adsAccVar=accToWrite;

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
