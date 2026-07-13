#include "marvin_ros2_control/tool/marvin_rs485_bus.h"

#include "MarvinSDK.h"

namespace marvin_ros2_control
{
namespace
{
    bool clearChDataA() { return OnClearChDataA(); }

    bool sendChDataA(uint8_t* data, long size, long channel)
    {
        return OnSetChDataA(data, size, channel);
    }

    long getChDataA(unsigned char* data, long* channel)
    {
        return OnGetChDataA(data, channel);
    }

    bool clearChDataB() { return OnClearChDataB(); }

    bool sendChDataB(uint8_t* data, long size, long channel)
    {
        return OnSetChDataB(data, size, channel);
    }

    long getChDataB(unsigned char* data, long* channel)
    {
        return OnGetChDataB(data, channel);
    }
}  // namespace

Clear485Func MarvinRs485Bus::clearA() { return clearChDataA; }
Send485Func MarvinRs485Bus::sendA() { return sendChDataA; }
GetChDataFunc MarvinRs485Bus::getA() { return getChDataA; }
Clear485Func MarvinRs485Bus::clearB() { return clearChDataB; }
Send485Func MarvinRs485Bus::sendB() { return sendChDataB; }
GetChDataFunc MarvinRs485Bus::getB() { return getChDataB; }

}  // namespace marvin_ros2_control
