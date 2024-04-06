include(common.cmake)

# Proto file
set(proto_filename "device_router")
get_filename_component(proto "../protos/${proto_filename}.proto" ABSOLUTE)
get_filename_component(proto_path "${proto}" PATH)

# Generated sources' name
set(proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/${proto_filename}.pb.cc")
set(proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/${proto_filename}.pb.h")
set(grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/${proto_filename}.grpc.pb.cc")
set(grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/${proto_filename}.grpc.pb.h")

# Generate .h/.cc
add_custom_command(
      OUTPUT ${proto_srcs} ${proto_hdrs} ${grpc_srcs} ${grpc_hdrs}
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
           --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
           -I "${proto_path}"
           --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
           "${proto}"
      DEPENDS "${proto}"
)

add_library(grpc_proto
    ${grpc_srcs}
    ${grpc_hdrs}
    ${proto_srcs}
    ${proto_hdrs}
)

target_include_directories(grpc_proto
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
)
