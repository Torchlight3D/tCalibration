#include "utils.h"

#include <zmq.hpp>

#include "../soc/types.h"

namespace tl {
namespace zeromq {

ImuData parseMotionData(const zmq::message_t& msg)
{
    const auto data = msg.data<soc::imu_data_t>();
    return {.acc = {data->stamp, data->ax, data->ay, data->az},
            .gyro = {data->stamp, data->gx, data->gy, data->gz},
            .temperature = data->temp};
}

std::vector<ImuData> parseMotionDatas(const zmq::message_t& msg, size_t msgSize)
{
    const auto cnt = msgSize / sizeof(soc::imu_data_t);
    auto data = msg.data<soc::imu_data_t>();

    std::vector<ImuData> vals;
    vals.reserve(cnt);
    for (size_t i{0}; i < cnt; ++i) {
        vals.push_back({.acc = {data->stamp, data->ax, data->ay, data->az},
                        .gyro = {data->stamp, data->gx, data->gy, data->gz},
                        .temperature = data->temp});
        data++;
    }

    return vals;
}

} // namespace zeromq
} // namespace tl
