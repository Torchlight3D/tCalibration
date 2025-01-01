#pragma once

#include <zmq.hpp>

#include <AxImu/ImuData>

namespace tl {
namespace zeromq {

// The list version is almost the same
ImuData parseMotionData(const zmq::message_t& msg);

std::vector<ImuData> parseMotionDatas(const zmq::message_t& msg,
                                      size_t msgSize);
} // namespace zeromq
} // namespace tl
