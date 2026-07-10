#pragma once

#include "marvin_ros2_control/tool/modbus_io.h"

namespace marvin_ros2_control
{
    /**
     * Thin wrappers around Marvin SDK On*ChDataA/B.
     *
     * COM1 (SDK ch=2, gripper Modbus) and COM2 (SDK ch=3, KWR75) are independent
     * logical buses on the 8-pin EE connector. No cross-COM locking: set *channel to
     * COM1_CHANNEL or COM2_CHANNEL before OnGetChData*; SDK routes by COM.
     */
    class MarvinRs485Bus
    {
    public:
        static Clear485Func clearA();
        static Send485Func sendA();
        static GetChDataFunc getA();
        static Clear485Func clearB();
        static Send485Func sendB();
        static GetChDataFunc getB();
    };
}  // namespace marvin_ros2_control
