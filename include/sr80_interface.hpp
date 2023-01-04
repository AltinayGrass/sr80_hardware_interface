#ifndef SR80_INTERFACE
#define SR80_INTERFACE


#include <sensor_msgs/JointState.h>

namespace SR80
{
    class Interface
    {
        public:

        private:

        std::vector<std::string> m_JointNames;
        
        std::vector<double> m_JointPositions;

    };

}

#endif