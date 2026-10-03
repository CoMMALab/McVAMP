cmake_minimum_required(VERSION 3.16...3.22)
project(mcvamp_experiments LANGUAGES C CXX)

# ---------------------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------------------
option(EXPERIMENTS_USE_VAMP_SUBDIRECTORY
    "Build vamp from EXPERIMENTS_VAMP_SOURCE_DIR with add_subdirectory (OFF: use find_package(vamp))" ON)
set(EXPERIMENTS_VAMP_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../vamp" CACHE PATH
    "Source tree of vamp (the mcvamp branch) used when EXPERIMENTS_USE_VAMP_SUBDIRECTORY is ON")

option(EXPERIMENTS_BUILD_OMPL "Build the OMPL integration examples (needs OMPL + Boost)" OFF)
set(EXPERIMENTS_OMPL_PATH "" CACHE PATH "Extra search path for OMPL's omplConfig.cmake (or set ompl_DIR)")
option(EXPERIMENTS_BUILD_PINOCCHIO
    "Build the OMPL + pinocchio comparison example (needs EXPERIMENTS_BUILD_OMPL and pinocchio)" OFF)

option(EXPERIMENTS_FETCH_NLOHMANN_JSON
    "Download nlohmann_json when it is not installed (needed by the JSON-environment examples)" ON)

set(EXPERIMENTS_GENERATED_ROBOT_DIR "" CACHE PATH
    "Include root containing vamp/robots/<robot>.hh for robots mcvamp does not ship \
(g1_unitree.hh, bimanual_iiwa.hh), generated with cricket. Examples needing them are skipped if absent.")

option(EXPERIMENTS_LTO "Use LTO for the examples (slow to link with the large robot headers)." OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Same rule as the old branch's top-level CMakeLists.
add_compile_options(-Wno-unused-variable)

# ---------------------------------------------------------------------------------------
# Compiler settings. vamp's SIMD kernels need the same -march / fast-math flags vamp itself is
# configured with; those are set in vamp's cmake/CompilerSettings.cmake, which only applies to
# the directory that includes it, so it has to be included here for the executables.
# ---------------------------------------------------------------------------------------
set(VAMP_LTO ${EXPERIMENTS_LTO})
if(EXISTS "${EXPERIMENTS_VAMP_SOURCE_DIR}/cmake/CompilerSettings.cmake")
    include("${EXPERIMENTS_VAMP_SOURCE_DIR}/cmake/CompilerSettings.cmake")
else()
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FallbackCompilerSettings.cmake")
endif()

# ---------------------------------------------------------------------------------------
# vamp
# ---------------------------------------------------------------------------------------
if(EXPERIMENTS_USE_VAMP_SUBDIRECTORY)
    if(NOT EXISTS "${EXPERIMENTS_VAMP_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "vamp not found at ${EXPERIMENTS_VAMP_SOURCE_DIR}. Clone the mcvamp branch there, pass "
            "-DEXPERIMENTS_VAMP_SOURCE_DIR=<path>, or use -DEXPERIMENTS_USE_VAMP_SUBDIRECTORY=OFF "
            "with an installed vamp.")
    endif()

    # Only the header-only C++ library is needed: no bindings, JIT, WASM, install rules or demos.
    set(VAMP_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
    set(VAMP_BUILD_CPP_DEMO OFF CACHE BOOL "" FORCE)
    set(VAMP_BUILD_OMPL_DEMO OFF CACHE BOOL "" FORCE)
    set(VAMP_BUILD_JIT OFF CACHE BOOL "" FORCE)
    set(VAMP_BUILD_WASM OFF CACHE BOOL "" FORCE)
    set(VAMP_INSTALL_CPP_LIBRARY OFF CACHE BOOL "" FORCE)

    if(NOT TARGET vamp_cpp)
        add_subdirectory("${EXPERIMENTS_VAMP_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/vamp")
    endif()
    set(EXPERIMENTS_VAMP_TARGET vamp_cpp)
    set(_vamp_robot_dir "${EXPERIMENTS_VAMP_SOURCE_DIR}/src/impl/vamp/robots")
else()
    # vampConfig.cmake calls find_dependency() without including its macro module, and the
    # exported vamp target links pdqsort::pdqsort, which is installed as a bare targets file.
    include(CMakeFindDependencyMacro)
    find_file(_pdqsort_targets pdqsortTargets.cmake
        PATHS ${CMAKE_PREFIX_PATH} ${CMAKE_INSTALL_PREFIX} PATH_SUFFIXES lib/cmake/pdqsort lib64/cmake/pdqsort
        NO_DEFAULT_PATH)
    if(_pdqsort_targets AND NOT TARGET pdqsort::pdqsort)
        include("${_pdqsort_targets}")
    endif()
    find_package(vamp REQUIRED)
    if(TARGET vamp::vamp)
        set(EXPERIMENTS_VAMP_TARGET vamp::vamp)
    elseif(TARGET vamp::vamp_cpp)
        set(EXPERIMENTS_VAMP_TARGET vamp::vamp_cpp)
    else()
        set(EXPERIMENTS_VAMP_TARGET vamp_cpp)
    endif()
    get_target_property(_vamp_include_dirs ${EXPERIMENTS_VAMP_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    set(_vamp_robot_dir "")
    foreach(_dir IN LISTS _vamp_include_dirs)
        if(EXISTS "${_dir}/vamp/robots")
            set(_vamp_robot_dir "${_dir}/vamp/robots")
        elseif(EXISTS "${_dir}/robots")
            set(_vamp_robot_dir "${_dir}/robots")
        endif()
    endforeach()
endif()

# ---------------------------------------------------------------------------------------
# Optional dependencies
# ---------------------------------------------------------------------------------------
find_package(nlohmann_json 3 QUIET)
if(NOT nlohmann_json_FOUND AND EXPERIMENTS_FETCH_NLOHMANN_JSON)
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(nlohmann_json)
endif()

set(EXPERIMENTS_HAVE_OMPL FALSE)
if(EXPERIMENTS_BUILD_OMPL)
    find_package(ompl QUIET PATHS ${EXPERIMENTS_OMPL_PATH})
    if(ompl_FOUND)
        set(EXPERIMENTS_HAVE_OMPL TRUE)
    else()
        message(WARNING "EXPERIMENTS_BUILD_OMPL=ON but OMPL was not found (set ompl_DIR or "
                        "EXPERIMENTS_OMPL_PATH): skipping the OMPL examples.")
    endif()
endif()

set(EXPERIMENTS_HAVE_PINOCCHIO FALSE)
if(EXPERIMENTS_BUILD_PINOCCHIO)
    if(NOT EXPERIMENTS_HAVE_OMPL)
        message(WARNING "EXPERIMENTS_BUILD_PINOCCHIO needs OMPL: skipping the pinocchio example.")
    else()
        find_package(pinocchio REQUIRED)
        set(EXPERIMENTS_HAVE_PINOCCHIO TRUE)
    endif()
endif()

# ---------------------------------------------------------------------------------------
# Example registration
# ---------------------------------------------------------------------------------------
# experiments_add_example(<name> <source in cpp/> [LIBS ...] [OPTIONS ...])
function(experiments_add_example name source)
    cmake_parse_arguments(EXP "" "" "LIBS;OPTIONS" ${ARGN})
    add_executable(${name} "${CMAKE_CURRENT_SOURCE_DIR}/cpp/${source}")
    target_link_libraries(${name} PRIVATE ${EXPERIMENTS_VAMP_TARGET} ${EXP_LIBS})
    target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/cpp")
    target_compile_definitions(${name} PRIVATE
        EXPERIMENTS_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")

    # vamp_cpp propagates -fno-strict-aliasing (vamp::Vector aliases rows in place through
    # reinterpret_cast); repeat it here so it also holds for find_package(vamp) installs.
    target_compile_options(${name} PRIVATE -fno-strict-aliasing ${EXP_OPTIONS})

    # Same demo relaxations as the old branch.
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${name} PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
    endif()
endfunction()

# Robot headers that mcvamp does not ship: generate them with cricket (see README).
set(_generated_include "")
if(EXPERIMENTS_GENERATED_ROBOT_DIR)
    set(_generated_include "${EXPERIMENTS_GENERATED_ROBOT_DIR}")
endif()

function(experiments_robot_available robot_header out_var)
    set(_found FALSE)
    if(_vamp_robot_dir AND EXISTS "${_vamp_robot_dir}/${robot_header}")
        set(_found TRUE)
    elseif(_generated_include AND EXISTS "${_generated_include}/vamp/robots/${robot_header}")
        set(_found TRUE)
    endif()
    set(${out_var} ${_found} PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------------------
# Examples on robots that ship with mcvamp
# ---------------------------------------------------------------------------------------
experiments_add_example(rrtc_example rrtc_example.cc)
experiments_add_example(crrtc_spheres crrtc_spheres.cc)
experiments_add_example(crrtc_transport_mug crrtc_transport_mug.cc)
experiments_add_example(bimanual_panda_projection bimanual_panda_projection.cc)
experiments_add_example(bimanual_panda_planning bimanual_panda_planning.cc)
experiments_add_example(digit_transport digit_transport.cc)

if(TARGET nlohmann_json::nlohmann_json)
    experiments_add_example(crrtc_maze crrtc_maze.cc LIBS nlohmann_json::nlohmann_json)
    experiments_add_example(task_space_projection task_space_projection.cc LIBS nlohmann_json::nlohmann_json)
    experiments_add_example(benchmark_maze benchmark_maze.cc LIBS nlohmann_json::nlohmann_json)
    experiments_add_example(benchmark_line_plane benchmark_line_plane.cc LIBS nlohmann_json::nlohmann_json)
else()
    message(STATUS "nlohmann_json not found: skipping crrtc_maze, task_space_projection and the benchmarks "
                   "(install it or enable EXPERIMENTS_FETCH_NLOHMANN_JSON)")
endif()

# ---------------------------------------------------------------------------------------
# Examples on robots that mcvamp does not ship: built only if the generated header is found in
# the vamp source tree or under EXPERIMENTS_GENERATED_ROBOT_DIR (see README).
# ---------------------------------------------------------------------------------------
experiments_robot_available(g1_unitree.hh _have_g1)
if(_have_g1)
    set(_g1_include "")
    if(_generated_include)
        set(_g1_include "${_generated_include}")
    endif()
    # The CoM examples were built with -fno-finite-math-only on the old branch (vamp's Clang
    # flags include -fno-honor-nans, which the CoM hinge does not tolerate).
    foreach(_g1_example g1_bimanual_planning g1_com_planning g1_com_runner g1_com_projection)
        experiments_add_example(${_g1_example} ${_g1_example}.cc OPTIONS -fno-finite-math-only)
        if(_g1_include)
            target_include_directories(${_g1_example} PRIVATE "${_g1_include}")
        endif()
    endforeach()
else()
    message(STATUS "g1_unitree.hh not found: skipping the g1_* examples "
                   "(generate it with cricket, then set EXPERIMENTS_GENERATED_ROBOT_DIR)")
endif()

experiments_robot_available(bimanual_iiwa.hh _have_iiwa)
if(_have_iiwa)
    experiments_add_example(iiwa_bimanual_planning iiwa_bimanual_planning.cc)
    if(_generated_include)
        target_include_directories(iiwa_bimanual_planning PRIVATE "${_generated_include}")
    endif()
else()
    message(STATUS "bimanual_iiwa.hh not found: skipping iiwa_bimanual_planning "
                   "(generate it with cricket, then set EXPERIMENTS_GENERATED_ROBOT_DIR)")
endif()

# ---------------------------------------------------------------------------------------
# OMPL / pinocchio examples
# ---------------------------------------------------------------------------------------
if(EXPERIMENTS_HAVE_OMPL)
    experiments_add_example(ompl_unconstrained ompl_unconstrained.cc LIBS ompl::ompl)
    experiments_add_example(ompl_constrained_projected ompl_constrained_projected.cc LIBS ompl::ompl)

    if(TARGET nlohmann_json::nlohmann_json)
        experiments_add_example(ompl_constrained_statespace ompl_constrained_statespace.cc
            LIBS ompl::ompl nlohmann_json::nlohmann_json)
    endif()

    if(EXPERIMENTS_HAVE_PINOCCHIO)
        experiments_add_example(ompl_pinocchio ompl_pinocchio.cc LIBS ompl::ompl pinocchio::pinocchio)
        target_compile_definitions(ompl_pinocchio PRIVATE
            EXPERIMENTS_VAMP_ROOT="${EXPERIMENTS_VAMP_SOURCE_DIR}")
    endif()
endif()

message(STATUS "mcvamp experiments configuration:")
message(STATUS "  - vamp:            ${EXPERIMENTS_VAMP_TARGET} (subdirectory: ${EXPERIMENTS_USE_VAMP_SUBDIRECTORY})")
if(TARGET nlohmann_json::nlohmann_json)
    message(STATUS "  - nlohmann_json:   yes")
else()
    message(STATUS "  - nlohmann_json:   no")
endif()
message(STATUS "  - OMPL examples:   ${EXPERIMENTS_HAVE_OMPL}")
message(STATUS "  - pinocchio:       ${EXPERIMENTS_HAVE_PINOCCHIO}")
message(STATUS "  - g1 examples:     ${_have_g1}")
message(STATUS "  - iiwa examples:   ${_have_iiwa}")
