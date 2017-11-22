
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lwpr
#  CATKIN_DEPENDS utilities
#  DEPENDS system_lib
#)

#sl_package()

include_directories(include ${catkin_INCLUDE_DIRS})

set(HEADERS
	include/lwpr.h 
        include/lwpr_config.h 
        include/lwpr_config_def.h 
        include/lwpr_aux.h 
        include/lwpr_math.h 
        include/lwpr_binio.h 
        include/lwpr_mem.h 
        include/lwpr_xml.h 
        include/lwpr_matlab.h 
        include/lwpr.hh 
)

set(SOURCES 
	src/lwpr.c 
        src/lwpr_aux.c 
        src/lwpr_math.c 
        src/lwpr_binio.c 
        src/lwpr_mem.c 
        src/lwpr_xml.c
)

add_library(lwpr ${SOURCES} ${HEADERS})

target_link_libraries(lwpr ${utilities_LIBRARIES})

