
cmake_minimum_required(VERSION 3.16)

# Assume gRPC and all its dependencies are already installed.
set(gRPC_CMAKE_ROOT "$ENV{HOME}/Dependencies/grpc/1.56.0/lib/cmake")

# absl
set(absl_DIR ${gRPC_CMAKE_ROOT}/absl)
find_package(absl CONFIG REQUIRED)

# protobuf
option(protobuf_MODULE_COMPATIBLE TRUE)
set(protobuf_DIR ${gRPC_CMAKE_ROOT}/protobuf)
find_package(protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
if(CMAKE_CROSSCOMPILING)
    find_program(_PROTOBUF_PROTOC protoc)
else()
    set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
endif()

# gRPC
set(gRPC_DIR ${gRPC_CMAKE_ROOT}/grpc)
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

set(_GRPC_GRPCPP gRPC::grpc++)
if(CMAKE_CROSSCOMPILING)
    find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
else()
    set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
endif()