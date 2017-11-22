cmake_minimum_required(VERSION 2.8.3)

include_directories(lwpr/include misc learner)

set(HEADERS
	learner/base_learner.h
	learner/lwpr_learner.h
	misc/config.h
)

set(SOURCES 
	learner/base_learner.cc
	learner/lwpr_learner.cc
	misc/config.cpp
)

add_subdirectory(lwpr)

add_library(toolextension_lwpr_lib ${SOURCES} ${HEADERS})
target_link_libraries(toolextension_lwpr_lib lwpr)
