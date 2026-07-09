#include "marvin_ros2_control/sensors/kwr75_marvin485_client.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <sstream>

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

    void Kwr75Marvin485Client::recordIoSample(const std::vector<uint8_t>& buffer, long rx_channel)
    {
        last_rx_channel_ = rx_channel;
        if (buffer.empty())
        {
            last_io_sample_hex_.clear();
            return;
        }
        const std::size_t sample_len = std::min(buffer.size(), std::size_t{32});
        std::ostringstream oss;
        oss << std::hex << std::setfill('0');
        for (std::size_t i = 0; i < sample_len; ++i)
        {
            if (i > 0)
            {
                oss << ' ';
            }
            oss << std::setw(2) << static_cast<int>(buffer[i]);
        }
        if (buffer.size() > sample_len)
        {
            oss << " ...";
        }
        last_io_sample_hex_ = oss.str();
    }

    bool Kwr75Marvin485Client::ensureStreaming()
    {
        if (streaming_started_)
        {
            return true;
        }
        if (!sendStartCommand())
        {
            return false;
        }
        streaming_started_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        return true;
    }

    bool Kwr75Marvin485Client::warmup()
    {
        last_io_sample_hex_.clear();
        last_rx_channel_ = -1;

        if (usesPollPerRead())
        {
            if (!sendStartCommand())
            {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (!sendStartCommand())
            {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        else if (!ensureStreaming())
        {
            return false;
        }

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
        if (usesPollPerRead())
        {
            if (!sendStartCommand())
            {
                return false;
            }
        }
        else if (!ensureStreaming())
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

    bool Kwr75Marvin485Client::usesPollPerRead() const
    {
        return command_code_ == 0x49;
    }

    bool Kwr75Marvin485Client::sendStartCommand()
    {
        const auto request = Kwr75Protocol::buildPollRequest(command_code_);
        return send_485_(const_cast<uint8_t*>(request.data()), static_cast<long>(request.size()), channel_);
    }

    bool Kwr75Marvin485Client::readResponseFrame(std::array<uint8_t, Kwr75Protocol::kFrameLength>& frame)
    {
        std::vector<uint8_t> buffer;
        buffer.reserve(Kwr75Protocol::kFrameLength * 2);
        long last_rx_channel = -1;
        const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(response_timeout_ms_);

        while (std::chrono::steady_clock::now() <= deadline)
        {
            unsigned char chunk[256] = {0};
            long rx_channel = channel_;
            const long received = get_ch_data_(chunk, &rx_channel);
            if (received > 0)
            {
                last_rx_channel = rx_channel;
                buffer.insert(
                    buffer.end(),
                    chunk,
                    chunk + std::min<long>(received, static_cast<long>(sizeof(chunk))));
                if (Kwr75Protocol::findFrameInBuffer(
                        buffer.data(), buffer.size(), command_code_, frame))
                {
                    last_io_sample_hex_.clear();
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

        recordIoSample(buffer, last_rx_channel);
        return false;
    }
}  // namespace marvin_ros2_control
