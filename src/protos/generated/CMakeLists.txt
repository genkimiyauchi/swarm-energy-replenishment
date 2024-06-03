# Adapted from https://stackoverflow.com/questions/41430991/cmake-with-protobuf-file-in-subdirectory

## List generated sources files
FILE(GLOB HDRS "*.h")
FILE(GLOB SRCS "*.cc")

ADD_LIBRARY(protos ${HDRS} ${SRCS})
# ${PROTOBUF_LIBRARIES} should be defined by FIND_PACKAGE(Protobuf REQUIRED)
TARGET_LINK_LIBRARIES(protos ${PROTOBUF_LIBRARIES})