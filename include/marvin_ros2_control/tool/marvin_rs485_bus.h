#pragma once

#include "marvin_ros2_control/tool/modbus_io.h"

namespace marvin_ros2_control
{
    /**
     * Thin wrappers around Marvin SDK On*ChDataA/B. No locks.
     *
     * COM1/COM2 are independent channels, but OnGetChDataA is not re-entrant:
     * only one thread per arm may call get/set (see tool_recv feeding KWR75).
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
