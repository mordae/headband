cmake_minimum_required(VERSION 3.5)

set(ENV{PYTHON} "python3")
set(ENV{IDF_PATH} "$ENV{PWD}/vendor/esp-idf")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(headband)
