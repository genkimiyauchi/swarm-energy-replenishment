# Adapted from https://stackoverflow.com/questions/41430991/cmake-with-protobuf-file-in-subdirectory

FIND_PACKAGE(Protobuf REQUIRED)

FILE(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR} PROTOMODEL_PATH)
FILE(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/generated PROTOBINDING_PATH)

FILE(GLOB DATAMODEL_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

FOREACH(proto ${DATAMODEL_PROTOS})
   FILE(TO_NATIVE_PATH ${proto} proto_native)
   EXECUTE_PROCESS(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --proto_path=${PROTOMODEL_PATH} --cpp_out=${PROTOBINDING_PATH} ${proto_native} --python_out=${PROTOBINDING_PATH}
   RESULT_VARIABLE rv)
   # Optional, but that can show the user if something have gone wrong with the proto generation 
   IF(${rv})
        MESSAGE("Generation of data model returned ${rv} for proto ${proto_native}")
   ENDIF()
ENDFOREACH(proto)

add_subdirectory(generated)