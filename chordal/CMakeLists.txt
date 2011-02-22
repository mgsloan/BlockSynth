cmake_minimum_required(VERSION 2.6)

PROJECT(chordal)

ADD_SUBDIRECTORY(nestk)
INCLUDE("${nestk_BINARY_DIR}/UseNestk.cmake")

ADD_EXECUTABLE(chordal main.cpp)
TARGET_LINK_LIBRARIES(chordal nestk ao)
