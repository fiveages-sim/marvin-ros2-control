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
        int response_timeout_ms,
        int recv_interval_ms,
        int warmup_timeout_ms,
        Kwr75RxLogFunc rx_log)
        : send_485_(send_485),
          get_ch_data_(get_ch_data),
          channel_(channel),
          command_code_(command_code),
          convert_to_si_(convert_to_si),
          gravity_(gravity),
          response_timeout_ms_(response_timeout_ms),
          recv_interval_ms_(std::max(1, recv_interval_ms)),
          warmup_timeout_ms_(std::max(response_timeout_ms, warmup_timeout_ms)),
          rx_log_(std::move(rx_log))
    {
    }

    void Kwr75Marvin485Client::logRxBytes(long received, long rx_channel) const
    {
        if (rx_log_)
        {
            rx_log_(received, rx_channel);
        }
    }

    void Kwr75Marvin485Client::stop()
    {
        stopRecvThread();
    }

    void Kwr75Marvin485Client::stopRecvThread()
    {
        recv_running_.store(false);
        if (recv_thread_.joinable())
        {
            recv_thread_.join();
        }
    }

    Kwr75Marvin485Client::~Kwr75Marvin485Client()
    {
        stop();
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

    bool Kwr75Marvin485Client::usesPollPerRead() const
    {
        return command_code_ == 0x49;
    }

    bool Kwr75Marvin485Client::usesStreaming() const
    {
        return !usesPollPerRead();
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

    void Kwr75Marvin485Client::startRecvThread()
    {
        if (recv_running_.exchange(true))
        {
            return;
        }
        recv_thread_ = std::thread(&Kwr75Marvin485Client::recvThreadFunc, this);
    }

    void Kwr75Marvin485Client::recvThreadFunc()
    {
        while (recv_running_.load())
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            unsigned char chunk[256] = {0};
            long rx_channel = channel_;
            const long received = get_ch_data_(chunk, &rx_channel);
            logRxBytes(received, rx_channel);
            if (Kwr75Protocol::isExactFrameRead(received, rx_channel, channel_))
            {
                last_rx_channel_ = rx_channel;
                std::array<float, Kwr75Protocol::kAxisCount> raw {};
                if (Kwr75Protocol::tryParseCompleteFrame(
                        chunk, static_cast<std::size_t>(received), command_code_, raw))
                {
                    {
                        std::lock_guard<std::mutex> lock(sample_mutex_);
                        latest_raw_ = raw;
                        has_sample_ = true;
                        has_pending_ = true;
                    }
                    last_io_sample_hex_.clear();
                }
                else
                {
                    recordIoSample(
                        std::vector<uint8_t>(chunk, chunk + received),
                        rx_channel);
                }
            }

            std::this_thread::sleep_until(
                cycle_start + std::chrono::milliseconds(recv_interval_ms_));
        }
    }

    bool Kwr75Marvin485Client::waitForFirstSample()
    {
        const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(warmup_timeout_ms_);
        while (std::chrono::steady_clock::now() <= deadline)
        {
            {
                std::lock_guard<std::mutex> lock(sample_mutex_);
                if (has_sample_)
                {
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }

    bool Kwr75Marvin485Client::takeNewRaw(std::array<float, Kwr75Protocol::kAxisCount>& raw_out)
    {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        if (!has_pending_)
        {
            return false;
        }
        raw_out = latest_raw_;
        has_pending_ = false;
        return true;
    }

    bool Kwr75Marvin485Client::warmup()
    {
        last_io_sample_hex_.clear();
        last_rx_channel_ = -1;

        if (usesPollPerRead())
        {
            return pollOnceTransaction(nullptr);
        }

        drainStaleCom2Rx();
        if (!ensureStreaming())
        {
            return false;
        }
        startRecvThread();
        return waitForFirstSample();
    }

    bool Kwr75Marvin485Client::readWrench(std::array<double, Kwr75Protocol::kAxisCount>& wrench_si)
    {
        std::array<float, Kwr75Protocol::kAxisCount> raw {};
        if (usesPollPerRead())
        {
            if (!pollOnceTransaction(&raw))
            {
                return false;
            }
        }
        else if (!takeNewRaw(raw))
        {
            return false;
        }

        Kwr75Protocol::rawToSi(raw, wrench_si, convert_to_si_, gravity_);
        return true;
    }

    bool Kwr75Marvin485Client::readWrenchUntil(
        const std::chrono::steady_clock::time_point deadline,
        std::array<double, Kwr75Protocol::kAxisCount>& wrench_si)
    {
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (readWrench(wrench_si))
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    }

    void Kwr75Marvin485Client::drainStaleCom2Rx()
    {
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            unsigned char chunk[256] = {0};
            long rx_channel = channel_;
            const long received = get_ch_data_(chunk, &rx_channel);
            if (received <= 0 || received > static_cast<long>(sizeof(chunk)) || rx_channel != channel_)
            {
                break;
            }
        }
    }

    bool Kwr75Marvin485Client::pollOnceTransaction(
        std::array<float, Kwr75Protocol::kAxisCount>* raw_out)
    {
        if (!sendStartCommand())
        {
            return false;
        }

        std::array<uint8_t, Kwr75Protocol::kFrameLength> frame {};
        if (!readPollResponseFrame(frame))
        {
            return false;
        }

        std::array<float, Kwr75Protocol::kAxisCount> raw {};
        if (!Kwr75Protocol::tryParseCompleteFrame(
                frame.data(), frame.size(), command_code_, raw))
        {
            recordIoSample(std::vector<uint8_t>(frame.begin(), frame.end()), channel_);
            return false;
        }

        if (raw_out != nullptr)
        {
            *raw_out = raw;
        }
        last_io_sample_hex_.clear();
        return true;
    }

    bool Kwr75Marvin485Client::sendStartCommand()
    {
        const auto request = Kwr75Protocol::buildPollRequest(command_code_);
        return send_485_(const_cast<uint8_t*>(request.data()), static_cast<long>(request.size()), channel_);
    }

    bool Kwr75Marvin485Client::readPollResponseFrame(
        std::array<uint8_t, Kwr75Protocol::kFrameLength>& frame)
    {
        long last_rx_channel = -1;
        std::vector<uint8_t> last_chunk;
        const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(response_timeout_ms_);

        while (std::chrono::steady_clock::now() <= deadline)
        {
            unsigned char chunk[256] = {0};
            long rx_channel = channel_;
            const long received = get_ch_data_(chunk, &rx_channel);
            if (!Kwr75Protocol::isExactFrameRead(received, rx_channel, channel_))
            {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
                continue;
            }

            last_rx_channel = rx_channel;
            last_chunk.assign(chunk, chunk + received);

            std::array<float, Kwr75Protocol::kAxisCount> raw {};
            if (Kwr75Protocol::tryParseCompleteFrame(
                    chunk, static_cast<std::size_t>(received), command_code_, raw))
            {
                std::copy_n(chunk, Kwr75Protocol::kFrameLength, frame.begin());
                last_io_sample_hex_.clear();
                return true;
            }
        }

        recordIoSample(last_chunk, last_rx_channel);
        return false;
    }
}  // namespace marvin_ros2_control
