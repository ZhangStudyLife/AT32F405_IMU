cmake_minimum_required(VERSION 3.24)
project(IMUStudio VERSION 1.0.2 LANGUAGES C CXX RC)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(IMUSTUDIO_REPOSITORY "ZhangStudyLife/AT32F405_IMU" CACHE STRING "GitHub repository for updates")
if(NOT IMUSTUDIO_REPOSITORY MATCHES "^[A-Za-z0-9-]+/[A-Za-z0-9_.-]+$")
    message(FATAL_ERROR "IMUSTUDIO_REPOSITORY must be owner/repo")
endif()
if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "IMUStudio requires Windows x64")
endif()

include(FetchContent)
FetchContent_Declare(sqlite
    URL https://sqlite.org/2026/sqlite-amalgamation-3540100.zip
    URL_HASH SHA3_256=92281c86e2957f9b9305890f9ac04204caa7b677cc868dc326d942689df28e30
)
FetchContent_MakeAvailable(sqlite)
add_library(sqlite3 STATIC "${sqlite_SOURCE_DIR}/sqlite3.c")
target_include_directories(sqlite3 SYSTEM PUBLIC "${sqlite_SOURCE_DIR}")
target_compile_definitions(sqlite3 PRIVATE SQLITE_OMIT_LOAD_EXTENSION SQLITE_DQS=0)

set(json_header "${CMAKE_CURRENT_BINARY_DIR}/include/nlohmann/json.hpp")
set(json_hash aaf127c04cb31c406e5b04a63f1ae89369fccde6d8fa7cdda1ed4f32dfc5de63)
file(DOWNLOAD https://raw.githubusercontent.com/nlohmann/json/v3.12.0/single_include/nlohmann/json.hpp "${json_header}"
    EXPECTED_HASH SHA256=${json_hash} TLS_VERIFY ON TIMEOUT 45)
configure_file(src/version.h.in version.h @ONLY)
configure_file(assets/app.rc.in app.rc @ONLY)
set_property(SOURCE "${CMAKE_CURRENT_BINARY_DIR}/app.rc" APPEND PROPERTY OBJECT_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/assets/board.bin;${CMAKE_CURRENT_SOURCE_DIR}/assets/app.ico;${CMAKE_CURRENT_SOURCE_DIR}/assets/app.manifest;${CMAKE_CURRENT_SOURCE_DIR}/assets/JSON-LICENSE.txt")

add_library(imustudio_core STATIC src/protocol.cpp src/model.cpp src/serial.cpp src/log.cpp src/update.cpp)
target_include_directories(imustudio_core PUBLIC src "${CMAKE_CURRENT_BINARY_DIR}")
target_include_directories(imustudio_core SYSTEM PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_compile_definitions(imustudio_core PUBLIC UNICODE _UNICODE NOMINMAX WIN32_LEAN_AND_MEAN _WIN32_WINNT=0x0A00)
target_link_libraries(imustudio_core PRIVATE sqlite3 setupapi bcrypt winhttp)

add_executable(IMUStudio WIN32 src/window.cpp src/draw.cpp "${CMAKE_CURRENT_BINARY_DIR}/app.rc")
target_link_libraries(IMUStudio PRIVATE imustudio_core comctl32 comdlg32 gdi32 msimg32 shell32 uxtheme ole32)
target_compile_options(imustudio_core PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4;/utf-8;/permissive->
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall;-Wextra;-Wpedantic>)
target_compile_options(IMUStudio PRIVATE
    $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/W4;/utf-8;/permissive->
    $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
if(MINGW)
    target_link_options(IMUStudio PRIVATE -municode -static $<$<CONFIG:Release>:-Wl,--strip-all>)
elseif(MSVC)
    target_link_options(IMUStudio PRIVATE /MANIFEST:NO)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(imustudio_test tests/test.cpp)
    target_link_libraries(imustudio_test PRIVATE imustudio_core)
    if(MSVC)
        target_compile_options(imustudio_test PRIVATE /utf-8)
    elseif(MINGW)
        target_link_options(imustudio_test PRIVATE -static)
    endif()
    add_test(NAME protocol_and_logs COMMAND imustudio_test)
    add_test(NAME window_startup COMMAND IMUStudio --smoke-test)
endif()
