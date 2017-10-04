CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)

set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

option(GEN_PYTHON_BINDINGS "Generate bindings for Python" ON)
if(${GEN_PYTHON_BINDINGS})
	set(PYTHON_INSTALL_PATH "/usr/local/lib/python2.7" CACHE STRING "")
endif()

PROJECT(wrench-cone-lib)

declare_PID_Package(
			AUTHOR 			Vincent Samy
			INSTITUTION		LIRMM
			ADDRESS			git@github.com:vsamy/wrench-cone-lib.git
			PUBLIC_ADDRESS	https://github.com/vsamy/wrench-cone-lib.git
			MAIL			vincent.samy@lirmm.fr
			YEAR 			2017
			LICENSE 		CeCILL-C
			DESCRIPTION 	"This library implements the Contact Wrench Cone as given [here](https://scaron.info/papers/journal/caron-tro-2016.pdf). It uses cdd for the polyhedron computation and Eigen for the matrix part. Python bindings are also available."
			README 			readme.md
)

set_PID_Package_Version(0 3 0)

#now finding packages
declare_PID_Package_Dependency(PACKAGE eigen EXTERNAL VERSION 3.3.4)
declare_PID_Package_Dependency(PACKAGE eigen-cdd NATIVE VERSION 0.4.0)
declare_PID_Package_Dependency(PACKAGE boost EXTERNAL VERSION 1.64.0)

build_PID_Package()
