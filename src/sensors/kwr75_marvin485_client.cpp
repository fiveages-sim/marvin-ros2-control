#include "marvin_ros2_control/sensors/kwr75_marvin485_client.h"

#include <algorithm>
#include <thread>

namespace marvin_ros2_control
{
    Kwr75Marvin485Client::Kwr75Marvin485Client(
        Send485Func send_485,
        long channel,
        Kwr75LockFreeSample* sample_slot,
        uint8_t command_code,
        bool convert_to_si,
        double gravity,
        int warmup_timeout_ms)
        : send_485_(send_485),
          channel_(channel),
          sample_slot_(sample_slot),
          command_code_(command_code),
          convert_to_si_(convert_to_si),
          gravity_(gravity),
          warmup_timeout_ms_(std::max(1, warmup_timeout_ms))
    {
    }

    bool Kwr75Marvin485Client::sendStartCommand()
    {
        const auto request = Kwr75Protocol::buildPollRequest(command_code_);
        return send_485_(
            const_cast<uint8_t*>(request.data()),
            static_cast<long>(request.size()),
            channel_);
    }

    bool Kwr75Marvin485Client::warmup()
    {
        if (!sample_slot_ || !sendStartCommand())
        {
            return false;
        }
        started_ = true;
        if (isPollMode())
        {
            return sample_slot_->waitHasAny(warmup_timeout_ms_);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return sample_slot_->waitHasAny(warmup_timeout_ms_);
    }

    bool Kwr75Marvin485Client::takeWrench(std::array<double, Kwr75Protocol::kAxisCount>& wrench_si)
    {
        if (!sample_slot_)
        {
            return false;
        }
        std::array<float, Kwr75Protocol::kAxisCount> raw {};
        if (!sample_slot_->tryConsume(raw))
        {
            return false;
        }
        Kwr75Protocol::rawToSi(raw, wrench_si, convert_to_si_, gravity_);
        return true;
    }

    bool Kwr75Marvin485Client::pollAndTakeWrench(
        const std::chrono::steady_clock::time_point deadline,
        std::array<double, Kwr75Protocol::kAxisCount>& wrench_si)
    {
        if (!sendStartCommand())
        {
            return false;
        }
        started_ = true;
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (takeWrench(wrench_si))
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }
}  // namespace marvin_ros2_control
