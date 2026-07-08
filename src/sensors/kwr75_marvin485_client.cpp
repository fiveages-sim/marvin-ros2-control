#include "marvin_ros2_control/sensors/kwr75_marvin485_client.h"

#include <algorithm>
#include <cstring>

namespace marvin_ros2_control
{
    Kwr75Marvin485Client::Kwr75Marvin485Client(
        Send485Func send_485,
        GetChDataFunc get_ch_data,
        long channel,
        uint8_t command_code,
        bool convert_to_si,
        double gravity,
        int response_timeout_ms)
        : send_485_(send_485),
          get_ch_data_(get_ch_data),
          channel_(channel),
          command_code_(command_code),
          convert_to_si_(convert_to_si),
          gravity_(gravity),
          response_timeout_ms_(response_timeout_ms)
    {
    }

    bool Kwr75Marvin485Client::warmup()
    {
        if (!sendPoll())
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (!sendPoll())
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::array<uint8_t, Kwr75Protocol::kFrameLength> frame {};
        if (!readResponseFrame(frame))
        {
            return false;
        }

        std::array<float, Kwr75Protocol::kAxisCount> raw {};
        return Kwr75Protocol::parseFrame(frame, raw);
    }

    bool Kwr75Marvin485Client::readWrench(std::array<double, Kwr75Protocol::kAxisCount>& wrench_si)
    {
        if (!sendPoll())
        {
            return false;
        }

        std::array<uint8_t, Kwr75Protocol::kFrameLength> frame {};
        if (!readResponseFrame(frame))
        {
            return false;
        }

        std::array<float, Kwr75Protocol::kAxisCount> raw {};
        if (!Kwr75Protocol::parseFrame(frame, raw))
        {
            return false;
        }

        Kwr75Protocol::rawToSi(raw, wrench_si, convert_to_si_, gravity_);
        return true;
    }

    bool Kwr75Marvin485Client::sendPoll()
    {
        const auto request = Kwr75Protocol::buildPollRequest(command_code_);
        return send_485_(const_cast<uint8_t*>(request.data()), static_cast<long>(request.size()), channel_);
    }

    bool Kwr75Marvin485Client::readResponseFrame(std::array<uint8_t, Kwr75Protocol::kFrameLength>& frame)
    {
        std::vector<uint8_t> buffer;
        buffer.reserve(Kwr75Protocol::kFrameLength * 2);
        const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(response_timeout_ms_);

        while (std::chrono::steady_clock::now() <= deadline)
        {
            unsigned char chunk[256] = {0};
            long channel = channel_;
            const long received = get_ch_data_(chunk, &channel);
            if (received > 0)
            {
                buffer.insert(
                    buffer.end(),
                    chunk,
                    chunk + std::min<long>(received, static_cast<long>(sizeof(chunk))));
                if (Kwr75Protocol::findFrameInBuffer(
                        buffer.data(), buffer.size(), command_code_, frame))
                {
                    return true;
                }
                if (buffer.size() > Kwr75Protocol::kFrameLength * 4)
                {
                    buffer.erase(
                        buffer.begin(),
                        buffer.end() - static_cast<std::ptrdiff_t>(Kwr75Protocol::kFrameLength));
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        return false;
    }
}  // namespace marvin_ros2_control
