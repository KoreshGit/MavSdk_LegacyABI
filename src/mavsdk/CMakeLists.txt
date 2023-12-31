add_library(mavsdk)

cmake_policy(SET CMP0076 NEW)

add_subdirectory(core)
add_subdirectory(plugins)

hunter_add_package(jsoncpp)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/build/default/third_party/install")
set(HUNTER_jsoncpp_VERSION "" CACHE STRING "Disable Hunter for jsoncpp" FORCE)
find_package(jsoncpp REQUIRED)

hunter_add_package(tinyxml2)
find_package(tinyxml2 REQUIRED)

target_link_libraries(mavsdk
    PRIVATE
    JsonCpp::JsonCpp
    tinyxml2::tinyxml2
)

if (NOT APPLE AND NOT ANDROID AND NOT MSVC)
    target_link_libraries(mavsdk
        PRIVATE
        stdc++fs
    )
endif()

set(UNIT_TEST_SOURCES ${UNIT_TEST_SOURCES} PARENT_SCOPE)
