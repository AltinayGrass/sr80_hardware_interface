#include "../include/sr80_hw_interface.hpp"

namespace SR80
{
    HardwareInterface::HardwareInterface(ros::NodeHandle& nh)
        : m_NodeHandle(nh)
    {
        if(m_NodeHandle.hasParam("/SR80/hardware_interface/joints"))
        {    
            m_NodeHandle.getParam("/SR80/hardware_interface/joints", m_JointNames);

        }
        else
        {
            ROS_ERROR_NAMED("Joint Param Error", "No joint names found in the parameter server.");
            if(ros::ok())
                ros::shutdown();
        }

        m_NumJoints = m_JointNames.size();

        m_JointPositions.resize(m_NumJoints);
        m_JointVelocities.resize(m_NumJoints);
        m_JointEfforts.resize(m_NumJoints);

        m_PositionCommand.resize(m_NumJoints);
        m_VelocityCommand.resize(m_NumJoints);

        for(std::size_t i = 0; i < m_NumJoints; i++)
        {
            hardware_interface::JointStateHandle tempJSH(
                m_JointNames.at(i),
                &m_JointPositions[i],
                &m_JointVelocities[i],
                &m_JointEfforts[i]
            );

            m_JointStateInterface.registerHandle(tempJSH);

            hardware_interface::PosVelAccJointHandle posVelAccJointHandle(
                tempJSH,
                &m_PositionCommand.at(i),
                &m_VelocityCommand.at(i),
                &m_AccelCommand.at(i)
            );

            m_PosVelAccJointInterface.registerHandle(posVelAccJointHandle);

        }

        this->registerInterface(&m_JointStateInterface);
        this->registerInterface(&m_PosVelAccJointInterface);

        m_ControllerManager.reset(new controller_manager::ControllerManager(this, m_NodeHandle));

        std::string remoteIPV4;
        if(m_NodeHandle.hasParam("ads_config/remote_ipv4"))
        {
            m_NodeHandle.getParam("ads_config/remote_ipv4", remoteIPV4);
        }
        else
        {
            ROS_ERROR("Could not find Remote IPV4 parameter in the Parameter Server");
        }   

        std::string remoteNetId;
        if(m_NodeHandle.hasParam("ads_config/remote_net_id"))
        {
            m_NodeHandle.getParam("ads_config/remote_net_id", remoteNetId);
        }
        else
        {
            ROS_ERROR("Could not find Remote Net ID parameter in the Parameter Server");
        }

        bhf::ads::SetLocalAddress({192, 168, 6, 101, 1, 1});

        m_WorkerADS = new ADS_Worker(remoteIPV4, remoteNetId);

        m_NodeHandle.param("/SR80/hardware_interface/loop_hz", m_LoopFrequency, 0.1);

        ros::Duration updateFreq = ros::Duration(1.0 / m_LoopFrequency);

        m_Loop = m_NodeHandle.createTimer(updateFreq, &HardwareInterface::update, this);

    }

    HardwareInterface::~HardwareInterface()
    {
        
    }

    void HardwareInterface::read()
    {
        sensor_msgs::JointState actJointStates = m_WorkerADS->readFromADS(m_NumJoints);
        m_JointPositions = actJointStates.position;
        m_JointVelocities = actJointStates.velocity;
    }

    void HardwareInterface::write(const ros::Duration& elapsed_time)
    {
        m_WorkerADS->writeToADS(
            m_NumJoints,
            m_PositionCommand,
            m_VelocityCommand
        );
    }

    template<typename T>
    void HardwareInterface::write(const ros::Duration& elapsed_time, const std::string& symbol_name, T var)
    {
        m_WorkerADS->writeToADS(
            var,
            symbol_name
        );
    }

    void HardwareInterface::update(const ros::TimerEvent& timer_event)
    {
        m_ElapsedTime = ros::Duration(timer_event.current_real - timer_event.last_real);

        this->read();

        m_ControllerManager->update(timer_event.current_real, m_ElapsedTime);

        this->write(m_ElapsedTime);
        
        // Example of writing a single joint:
        // Fill the second parameter with the symbol name from Beckhoff
        /*
            this->write(m_ElapsedTime, "", m_PositionCommand[0]);
            this->write(m_ElapsedTime, "", m_VelocityCommand[0]);
        */
        
    }

}
