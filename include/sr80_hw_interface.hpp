#ifndef SR80_HW_INTERFACE
#define SR80_HW_INTERFACE

#include "sr80_hw.hpp"
#include "sr80_interface.hpp"
#include "ads_worker.hpp"

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <boost/shared_ptr.hpp>

namespace SR80
{

    class HardwareInterface : public Hardware
    {
        public:

        HardwareInterface(ros::NodeHandle &nh);
        ~HardwareInterface();

        void update(const ros::TimerEvent& timer_event);
        void read();
        void write(const ros::Duration& elapsed_time);
        template<typename T>
        void write(const ros::Duration& elapsed_time, const std::string& symbol_name, T var);

        private:
        
        Interface m_Interface;

        ADS_Worker* m_WorkerADS;

        boost::shared_ptr<controller_manager::ControllerManager> m_ControllerManager;

        ros::Timer m_Loop; // Non-Realtime

        //hardware_interface::PosVelAccJointInterface m_PosVelAccInterface;

        ros::NodeHandle m_NodeHandle;

        ros::Duration m_ControlPeriod;
        ros::Duration m_ElapsedTime;

        double m_LoopFrequency;


    };
}

#endif