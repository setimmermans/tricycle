#
#   Universite catholique de Louvain
#   CEREM : Centre for research in mechatronics
#   http://www.robotran.be  
#   Contact : info@robotran.be
#
#
# CMake for compiling a simple robotran project in C
# 


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                       PROJECT MAIN CONFIGURATIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# CMake minimum version
cmake_minimum_required(VERSION 2.8.7)

# project name
project (tricycle)

# project configuration
set( CMAKE_C_FLAGS_RELEASE   "-O3" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )

# Variable for storing the path to Robotran common files (should be adapted depending on the location of those source)
if(NOT DEFINED ROBOTRAN_SOURCE_DIR)
	set(TRIAL_PATHS_MBSYSC
		C:/Users/sebtims/.robotran/mbsysC/MBsysC/
		${PROJECT_SOURCE_DIR}/../mbsysCopy
	)
	find_path(ROBOTRAN_SOURCE_DIR mbs_common ${TRIAL_PATHS_MBSYSC} DOC "Path to the Robotran-MBsysC common files")
endif ( )

# message to display the project name
message(STATUS "Processing ${PROJECT_NAME}")
message("MBsysC path: ${ROBOTRAN_SOURCE_DIR}")

# for Unix: display all the warnings, except the ones related to /* -- */, to unused variables and to unknown warnings
if (UNIX)
    set( CMAKE_C_FLAGS "-g -Wall -Wno-unused-but-set-variable -Wno-unused-variable -Wno-comment -Wno-unknown-warning-option" )
endif ( )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         Windows libraries
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

if (WIN32)

## --- WIN32 or WIN64 DETECTION --- ##
	if( CMAKE_SIZEOF_VOID_P EQUAL 8 )   # 64bit Windows
		SET(WIN_LIB_DIRECTORY win64_include_lib)
	else( )  # 32bit Windows
		SET(WIN_LIB_DIRECTORY win32_include_lib)
	endif( )

## ---- WINDOWS DLL FILES ---- ##

# copy all the dll (except 'jvm.dll') used for Windows
# these dll files are copied in the Executable directory (Debug or Release)
    file(COPY ${ROBOTRAN_SOURCE_DIR}/${WIN_LIB_DIRECTORY}/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Debug)
    file(COPY ${ROBOTRAN_SOURCE_DIR}/${WIN_LIB_DIRECTORY}/dll/ DESTINATION ${CMAKE_BINARY_DIR}/Release)
endif ( )

SET(CMAKE_INSTALL_PREFIX  ${CMAKE_BINARY_DIR}/Debug)

set(CMAKE_MODULE_PATH ${ROBOTRAN_SOURCE_DIR}/conf)


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                    PROJECT SPECIFIC OPTIONS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# Link to MBSysC lib already compiled
option (FLAG_SEPARATE_BUILD "Link to MBSysC lib already compiled" OFF)

# Link to symbolic files already compiled
option (FLAG_SEPARATE_SYMBOLIC "Link to symbolic files already compiled (if FLAG_SEPARATE_BUILD is ON)" OFF)

# Link to user function files already compiled
option (FLAG_SEPARATE_USER_FCT "Link to user fonction files already compiled (if FLAG_SEPARATE_BUILD is ON)" OFF)


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         Additional CMakelists.txt
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# additional CMake functions
set(CMAKE_AUX ${ROBOTRAN_SOURCE_DIR}/cmake_aux)
set(CMAKE_AUX_BIN ${PROJECT_BINARY_DIR}/cmake_aux)

add_subdirectory ( ${CMAKE_AUX}/flags/     ${CMAKE_AUX_BIN}/flags/     )
add_subdirectory ( ${CMAKE_AUX}/listing/   ${CMAKE_AUX_BIN}/listing/   )
add_subdirectory ( ${CMAKE_AUX}/libraries/ ${CMAKE_AUX_BIN}/libraries/ )
add_subdirectory ( ${CMAKE_AUX}/make_opt/  ${CMAKE_AUX_BIN}/make_opt/  )

# uncomment to include the real-time libraries by default
option (FLAG_REAL_TIME "Real time" ON)

# CMake functions
release_debug()
make_options()

# MBsysC files to compile
if (NOT FLAG_SEPARATE_BUILD)
    add_subdirectory( ${ROBOTRAN_SOURCE_DIR}/mbs_common ${CMAKE_CURRENT_BINARY_DIR}/mbs_common )
    flags_check() #TODO move to mbsysC cmake
    definitions()
endif ( )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                           USER ABSOLUTE PATHS
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# name of the build folder (relative to main CMakelists.txt)
file(RELATIVE_PATH BUILD_PATH ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})

# configure a header file to pass some of the CMake settings to the source code
configure_file (
    "${ROBOTRAN_SOURCE_DIR}/conf/cmake_config.h.in"
    "${PROJECT_BINARY_DIR}/conf/cmake_config.h"
)
# 'cmake_config.h.in' is in the 'conf' folder
include_directories (${PROJECT_BINARY_DIR}/conf)


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                           LIST FILES
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 
# list source files to compile
init_src()

increment_src( ${PROJECT_SOURCE_DIR}/src )


# list include directories (to find headers)
init_include()

increment_include( ${PROJECT_SOURCE_DIR}/.. )

# SDL.h header
if (FLAG_PLOT)
    sdl_header_lib(project)
endif ( )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         EXECUTABLE COMPILATION
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

# include these directories
include_directories (${INCLUDE_DIR})

# name of the executable
set ( Executable exe_${PROJECT_NAME} )

# generate the executable
add_executable ( ${Executable} ${SOURCE_FILES} )


# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#                         EXECUTABLE LINKING
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

flags_clean()

if ( FLAG_SEPARATE_BUILD ) # find MBSysC dynamic libraries

    find_path(LibRobotranC_DIR LibRobotranCConfig.cmake "${ROBOTRAN_SOURCE_DIR}/build")
    find_package( LibRobotranC REQUIRED )
    target_link_libraries( ${Executable} ${LIB_MBSYSC_MODULES} ${LIB_MBSYSC_LOAD} ${LIB_MBSYSC_UTILITIES} )
    add_definitions(${LIB_MBSYSC_DEFINITIONS})
    
    if (WIN32)

        ## ---- WINDOWS DLL FILES ---- ##

        # copy all the dll used for Windows
        # these dll files are copied in the Executable directory (Debug)
        file(COPY ${ROBOTRAN_SOURCE_DIR}\\build\\mbs_module\\Debug\\MBsysC_module.dll DESTINATION ${CMAKE_BINARY_DIR}\\Debug)
        file(COPY ${ROBOTRAN_SOURCE_DIR}\\build\\mbs_load_xml\\Debug\\MBsysC_loadXML.dll DESTINATION ${CMAKE_BINARY_DIR}\\Debug)
        file(COPY ${ROBOTRAN_SOURCE_DIR}\\build\\mbs_utilities\\Debug\\MBsysC_utilities.dll DESTINATION ${CMAKE_BINARY_DIR}\\Debug)
        file(COPY ${ROBOTRAN_SOURCE_DIR}\\build\\mbs_realtime\\Debug\\MBsysC_realtime.dll DESTINATION ${CMAKE_BINARY_DIR}\\Debug)
    
    endif ( )
    
else ( ) 

    target_link_libraries( ${Executable} MBsysC_loadXML MBsysC_module )

    if (NOT FLAG_SHARED_LIB)  

        # find MBSysC static libraries
        target_link_libraries( ${Executable} MBsysC_struct MBsysC_numerics MBsysC_realtime MBsysC_utilities )
        
        #user functions library
        target_link_libraries ( ${Executable} Project_userfct )
        target_link_libraries ( ${Executable} Project_symbolic )
        
        #Libxml2 and GSL external libraries
        target_link_libraries ( ${Executable} ${LIBXML2_LIBRARIES} ${GSL_LIBRARIES} )
        
        # SDL external library
        if (FLAG_PLOT)
            target_link_libraries ( ${Executable} ${SDL2_LIBRARIES} ${SDL2TTF_LIBRARIES} )
        endif ( )
        
        # Java external library
        if (FLAG_VISU)
            target_link_libraries ( ${Executable} ${JNI_LIBRARIES} )
        endif ( )
        
        add_definitions( -DMBSYSC_MODULE_STATIC_DEFINE)
        
    endif ( )

endif ( )

include_directories(${LIB_MBSYSC_INCLUDE_DIRS})

# symbolic files
if ( FLAG_SEPARATE_SYMBOLIC )
    file(COPY ${PROJECT_SOURCE_DIR}/../symbolicR/build/libProject_symbolic.so DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/symbolicR)
else ( )
    add_subdirectory( ${PROJECT_SOURCE_DIR}/../symbolicR ${CMAKE_CURRENT_BINARY_DIR}/symbolicR)
endif ( )

# user fonction files
if ( FLAG_SEPARATE_USER_FCT )
    file(COPY ${PROJECT_SOURCE_DIR}/../userfctR/build/libProject_userfct.so DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/userfctR)
else ( )
    add_subdirectory( ${PROJECT_SOURCE_DIR}/../userfctR ${CMAKE_CURRENT_BINARY_DIR}/userfctR)
endif ( )

# dynamic linking library
if (UNIX)
    target_link_libraries ( ${Executable} dl )
endif ( )

# math external library (for Unix)
if (UNIX)
    target_link_libraries ( ${Executable} m )
endif ( )
