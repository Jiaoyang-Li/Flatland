# CMakeList.txt : CMake project for CBSH-rect-cmake, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Add source to this project's executable.
project(CBS-K)

IF(NOT CMAKE_BUILD_TYPE)
  #SET(CMAKE_BUILD_TYPE "DEBUG")
  SET(CMAKE_BUILD_TYPE "RELEASE")
  #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
  #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()


include_directories(../mapVisualiser/)

add_executable (CBS-K
		driver.cpp
		flat_map_loader.h
		flat_map_loader.cpp
        agents_loader.h
        compute_heuristic.h
		ICBSNode.h
		ICBSSearch.h
		LLNode.h
		map_loader.h
		MDD.h
		RectangleReasoning.h
		SingleAgentICBS.h
		agents_loader.cpp
        compute_heuristic.cpp
		ICBSNode.cpp
		ICBSSearch.cpp
		LLNode.cpp
		map_loader.cpp
		MDD.cpp
		RectangleReasoning.cpp
		SingleAgentICBS.cpp
		ReservationTable.h
		ReservationTable.cpp
		Conflict.h
		Conflict.cpp
		ConstraintTable.cpp
		ConstraintTable.h
		CorridorReasoning.cpp
		CorridorReasoning.h
		egraph_reader.h
		common.h
		common.cpp
		# ../mapVisualiser/Visualiser.h
		# ../mapVisualiser/Visualiser.cpp
		)






# Google's sparsehash
if (WIN32)
	cmake_policy(SET CMP0074 OLD)
#	set( "C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0/boost")

	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/sparsehash/src")
	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/sparsehash/src/windows")
	include_directories("C:/Users/czcz2/Anaconda3/include")
	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0")
	include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_71_0/stage/lib")
	include_directories("C:/Users/czcz2/Anaconda3/libs")
	include_directories("C:/Users/czcz2/Anaconda3/include")
	#include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/SFML/SFML-2.5.1-32/include")
	#include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/SFML/SFML-2.5.1-64/include")
	#include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/SFML/SFML-2.5.1-32/lib")
	#include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/SFML/SFML-2.5.1-64/lib")
	#include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/SFML/SFML-2.5.1-32/bin")
	#include_directories("C:/Users/czcz2/Google Drive/MIT/GCITR/SFML/SFML-2.5.1-64/bin")
	# set(SFML_DIR "C:/Users/czcz2/Google Drive/MIT/GCITR/SFML/SFML-2.5.1-64/lib/cmake/SFML")
	# set(SFML_INCLUDE_DIR "C:/Users/czcz2/Google Drive/MIT/GCITR/SFML/SFML-2.5.1-64/include")
	

	set(PYTHON_INCLUDE_DIRS "C:/Users/czcz2/Anaconda3/include")
	set(PYTHON_LIBRARIES "C:/Users/czcz2/Anaconda3/libs")
	find_package(PythonInterp 3.7 REQUIRED)
	# This goes after, since it uses PythonInterp as hint
	find_package(PythonLibs 3.7 REQUIRED)
	include_directories(${PYTHON_INCLUDE_DIRS})
	MESSAGE(${PYTHON_LIBRARIES} ${PYTHON_INCLUDE_DIRS})


	# Detect and add SFML
	#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
	#Find any version 2.X of SFML
	#See the FindSFML.cmake file for additional details and instructions
	# find_package(SFML 2.5.1 REQUIRED  graphics window system)
	# if(SFML_FOUND)
	# include_directories(${SFML_INCLUDE_DIR})
	# target_link_libraries(CBS-K ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
	# endif()
	
	SET(Boost_NO_BOOST_CMAKE ON)
	set(BOOST_ROOT "C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_61_0/")
	set(BOOST_LIBRARYDIR "C:/Users/czcz2/Google Drive/MIT/GCITR/boost_1_61_0/lib64-msvc-14.0")
	FIND_PACKAGE(Boost 1.61.0 COMPONENTS system program_options python)
	IF(Boost_FOUND)
		SET(Boost_USE_STATIC_LIBS OFF)
		SET(Boost_USE_MULTITHREADED ON)
		SET(Boost_USE_STATIC_RUNTIME OFF)
		include_directories(${Boost_INCLUDEDIR} )

		add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
		TARGET_LINK_LIBRARIES(CBS-K ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})


	ELSEIF(NOT Boost_FOUND)
	  MESSAGE(FATAL_ERROR "Unable to find all boost libs")
	ENDIF()
else()
	file(READ "/etc/issue" ETC_ISSUE)
	string(REGEX MATCH "Ubuntu|Manjaro" DIST ${ETC_ISSUE})
	if(DIST STREQUAL "Ubuntu")
		message(STATUS ">>>> Found Ubuntu <<<<")
		set(boostPython python-py36)
	elseif(DIST STREQUAL "Manjaro")
		message(STATUS ">>>> Found Manjaro <<<<")
		set(boostPython python37)
	else()
		message(STATUS ">>>> Found unknown distribution <<<<")
		set(boostPython python37)
	endif()

	find_path(SPARSEHASH_INCLUDE_DIRS sparsehash/dense_hash_map)
	if (NOT SPARSEHASH_INCLUDE_DIRS)
		message(FATAL_ERROR "Could not find <sparsehash/dense_hash_map>.")
	endif ()
	include_directories(SYSTEM ${SPARSEHASH_INCLUDE_DIRS})

	find_package(PythonInterp 3.6 REQUIRED)

	# This goes after, since it uses PythonInterp as hint
	find_package(PythonLibs 3.6 REQUIRED)
	include_directories(${PYTHON_INCLUDE_DIRS})

	set(Boost_USE_STATIC_LIBS OFF) 
	set(Boost_USE_MULTITHREADED ON)  
	set(Boost_USE_STATIC_RUNTIME OFF) 
	FIND_PACKAGE(Boost REQUIRED COMPONENTS system program_options ${boostPython})

	# Detect and add SFML
	# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
	#Find any version 2.X of SFML
	#See the FindSFML.cmake file for additional details and instructions
	# find_package(SFML 2 REQUIRED  graphics window system)
	# if(SFML_FOUND)
	# include_directories(${SFML_INCLUDE_DIR})
	# target_link_libraries(CBS-K ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
	# endif()

	if(Boost_FOUND)
		include_directories(${Boost_INCLUDE_DIRS})
		target_link_libraries(CBS-K ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
	else()
		 message(FATAL_ERROR "Could not find boost.")
	endif()
endif()
