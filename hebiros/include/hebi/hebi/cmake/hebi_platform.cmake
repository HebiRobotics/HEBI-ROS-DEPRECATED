################################
# hebi_platform.cmake
# 
# You may modify this file to fit your needs.
# Clang, GCC and MSVC are the only _supported_ compilers here. Use others (icc, etc) at your own risk.
# 
# This file as is supports the following architectures for each platform:
# * Linux:
#   * aarch64  (armv8-a/64 bit ARM)
#   * armhf    (armv7 with hard float ABI)
#   * i686     (32 bit Intel/AMD [Intel Pentium 4 or newer])
#   * x86_64   (64 bit Intel/AMD [Intel Pentium 4 or newer])
# * OSX:
#   * amd64    (64 bit Intel)
# * Windows:
#   * x86      (32 bit Intel/AMD)
#   * x64      (64 bit Intel/AMD)
#   * ARM      (32 bit ARM)
#   * ARM64    (64 bit ARM)
# 
# Some additional experimental features include:
# * Cygwin support (Windows only)
# * Cross compiling support (Linux only)
# 
# These features are in beta and are subject to change without further notice.
# 
################################

cmake_minimum_required(VERSION 2.8)

################################
# Functions

function(hebi_set_cxx_version_latest)

  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER "3.1")
    set(CMAKE_CXX_STANDARD 11 PARENT_SCOPE)
    set(CMAKE_CXX_STANDARD_REQUIRED ON PARENT_SCOPE)
  elseif(HEBI_OSX_IS_APPLECLANG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" PARENT_SCOPE)
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" PARENT_SCOPE)
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" PARENT_SCOPE)
  elseif(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest" PARENT_SCOPE)
  endif()

endfunction(hebi_set_cxx_version_latest)

################
# Sets compilation flags
function(hebi_set_compile_flags)

  ####
  # Microsoft Visual C
  if(MSVC)
    set(HEBI_TARGET_CXX_FLAGS "${HEBI_CXX_FLAGS} /Zi" PARENT_SCOPE)
    set(HEBI_TARGET_CXX_FLAGS_DEBUG "/RTCs" PARENT_SCOPE)
    set(HEBI_TARGET_CXX_FLAGS_RELEASE "/O2xi" PARENT_SCOPE)
    set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "/O2xi" PARENT_SCOPE)

  ####
  # AppleClang
  elseif(HEBI_OSX_IS_APPLECLANG)
    set(HEBI_TARGET_CXX_FLAGS "${HEBI_CXX_FLAGS} -stdlib=libc++" PARENT_SCOPE)
    set(HEBI_TARGET_CXX_FLAGS_DEBUG "-O0 -g -stdlib=libc++" PARENT_SCOPE)
    set(HEBI_TARGET_CXX_FLAGS_RELEASE "-O3 -fno-signed-zeros -stdlib=libc++" PARENT_SCOPE)
    set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -fno-signed-zeros -stdlib=libc++" PARENT_SCOPE)

  ####
  # Clang
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(HEBI_TARGET_CXX_FLAGS "${HEBI_CXX_FLAGS}" PARENT_SCOPE)
    set(HEBI_TARGET_CXX_FLAGS_DEBUG "-O0 -g" PARENT_SCOPE)

    if("${HEBI_TARGET_ARCH}" STREQUAL "armhf")
      set(HEBI_TARGET_CXX_FLAGS_RELEASE "-O3 -fdenormal-fp-math=positive-zero -fno-signed-zeros -mcpu=${HEBI_TARGET_CPU_ARMHF} -mfpu=${HEBI_TARGET_FPU_ARMHF}" PARENT_SCOPE)
      set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -fdenormal-fp-math=positive-zero -fno-signed-zeros -mcpu=${HEBI_TARGET_CPU_ARMHF} -mfpu=${HEBI_TARGET_FPU_ARMHF}" PARENT_SCOPE)
    elseif("${HEBI_TARGET_ARCH}" STREQUAL "aarch64")
      set(HEBI_TARGET_CXX_FLAGS_RELEASE "-O3 -fdenormal-fp-math=positive-zero -fno-signed-zeros" PARENT_SCOPE)
      set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -fdenormal-fp-math=positive-zero -fno-signed-zeros" PARENT_SCOPE)
    elseif("${HEBI_TARGET_ARCH}" STREQUAL "i686")
      set(HEBI_TARGET_CXX_FLAGS_RELEASE "-O3 -march=${HEBI_TARGET_CPU_X86} -fdenormal-fp-math=positive-zero -fno-signed-zeros" PARENT_SCOPE)
      set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -march=${HEBI_TARGET_CPU_X86} -fdenormal-fp-math=positive-zero -fno-signed-zeros" PARENT_SCOPE)
    elseif("${HEBI_TARGET_ARCH}" STREQUAL "x86_64")
      set(HEBI_TARGET_CXX_FLAGS_RELEASE "-O3 -march=${HEBI_TARGET_CPU_X86_64} -fdenormal-fp-math=positive-zero -fno-signed-zeros" PARENT_SCOPE)
      set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -march=${HEBI_TARGET_CPU_X86_64} -fdenormal-fp-math=positive-zero -fno-signed-zeros" PARENT_SCOPE)
    endif()

  ####
  # GCC
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(HEBI_TARGET_CXX_FLAGS "${HEBI_CXX_FLAGS}" PARENT_SCOPE)
    set(HEBI_TARGET_CXX_FLAGS_DEBUG "-O0 -g" PARENT_SCOPE)

    if("${HEBI_TARGET_ARCH}" STREQUAL "armhf")
      set(HEBI_TARGET_CXX_FLAGS_RELEASE "-O3 -funsafe-math-optimizations -fno-signed-zeros -march=armv7-a -mfpu=${HEBI_TARGET_FPU_ARMHF} -mcpu=${HEBI_TARGET_CPU_ARMHF}" PARENT_SCOPE)
      set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -funsafe-math-optimizations -fno-signed-zeros -march=armv7-a -mfpu=${HEBI_TARGET_FPU_ARMHF} -mcpu=${HEBI_TARGET_CPU_ARMHF}" PARENT_SCOPE)
    elseif("${HEBI_TARGET_ARCH}" STREQUAL "aarch64")
      set(HEBI_TARGET_CXX_FLAGS_RELEASE "-O3 -march=armv8-a -mcpu=${HEBI_TARGET_CPU_AARCH64} -fno-signed-zeros" PARENT_SCOPE)
      set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -march=armv8-a -mcpu=${HEBI_TARGET_CPU_AARCH64} -fno-signed-zeros" PARENT_SCOPE)
    elseif("${HEBI_TARGET_ARCH}" STREQUAL "i686")
      set(HEBI_TARGET_CXX_FLAGS_RELEASE "-O3 -march=${HEBI_TARGET_CPU_X86} -fno-signed-zeros -mrecip=all" PARENT_SCOPE)
      set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -march=${HEBI_TARGET_CPU_X86} -fno-signed-zeros -mrecip=all" PARENT_SCOPE)
    elseif("${HEBI_TARGET_ARCH}" STREQUAL "x86_64")
      set(HEBI_TARGET_CXX_FLAGS_RELEASE "-O3 -march=${HEBI_TARGET_CPU_X86_64} -fno-signed-zeros -mrecip=all" PARENT_SCOPE)
      set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -march=${HEBI_TARGET_CPU_X86_64} -fno-signed-zeros -mrecip=all" PARENT_SCOPE)
    endif()
  endif()

endfunction(hebi_set_compile_flags)

################
# Sets up the build type flags
function(hebi_setup_build_type)

  hebi_set_compile_flags()

  set(CMAKE_CXX_FLAGS "${HEBI_TARGET_CXX_FLAGS}" PARENT_SCOPE)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${HEBI_TARGET_CXX_FLAGS_DEBUG}" PARENT_SCOPE)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${HEBI_TARGET_CXX_FLAGS_RELEASE}" PARENT_SCOPE)
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO}" PARENT_SCOPE)

  set(CMAKE_C_FLAGS "${HEBI_TARGET_CXX_FLAGS}" PARENT_SCOPE)
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${HEBI_TARGET_CXX_FLAGS_DEBUG}" PARENT_SCOPE)
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${HEBI_TARGET_CXX_FLAGS_RELEASE}" PARENT_SCOPE)
  set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO}" PARENT_SCOPE)

endfunction(hebi_setup_build_type)

################
# Sets up the toolchain
function(hebi_setup_toolchain)

  if(HEBI_CROSS_COMPILING)
    # Handle cross compiling
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
      set(CMAKE_C_COMPILER "${HEBI_TARGET_TRIPLET}-gcc" PARENT_SCOPE)
      set(CMAKE_CXX_COMPILER "${HEBI_TARGET_TRIPLET}-g++" PARENT_SCOPE)
    elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      set(HEBI_CXX_FLAGS "${HEBI_CXX_FLAGS} --target=${HEBI_TARGET_TRIPLET}" PARENT_SCOPE)
    endif()
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(HEBI_CXX_FLAGS "${HEBI_CXX_FLAGS} --target=${HEBI_TARGET_TRIPLET}" PARENT_SCOPE)
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    if("${HEBI_TARGET_ARCH}" STREQUAL "x86_64")
      set(HEBI_CXX_FLAGS "${HEBI_CXX_FLAGS} -m64" PARENT_SCOPE)
    elseif("${HEBI_TARGET_ARCH}" STREQUAL "i686")
      set(HEBI_CXX_FLAGS "${HEBI_CXX_FLAGS} -m32" PARENT_SCOPE)
    endif()
  endif()

endfunction(hebi_setup_toolchain)

################################
# Initial Setup

set(HEBI_HOST_LINUX FALSE)
set(HEBI_HOST_WINDOWS FALSE)
set(HEBI_HOST_OSX FALSE)
set(HEBI_OSX_IS_APPLECLANG FALSE)
set(HEBI_CROSS_COMPILING FALSE)
set(HEBI_HOST_ARCH "")

set(HEBI_CXX_FLAGS "")
set(HEBI_TARGET_CXX_FLAGS "")
set(HEBI_TARGET_CXX_FLAGS_DEBUG "")
set(HEBI_TARGET_CXX_FLAGS_RELEASE "")
set(HEBI_TARGET_CXX_FLAGS_RELWITHDEBINFO "")

################################
# Build Parameters
# You can modify these for your own builds, but you must
# make sure that your compiler supports the generated flags!
#
# Linux armhf distros require an FPU that satisfies, at a bare minimum,
# the specifications provided by the 'vfpv3-d16' class FPU.

# armhf specific
set(HEBI_TARGET_CPU_ARMHF "cortex-a8")
set(HEBI_TARGET_FPU_ARMHF "vfpv3-d16")

# aarch64 specific
set(HEBI_TARGET_CPU_AARCH64 "cortex-a53")

# x86_64 specific
set(HEBI_TARGET_CPU_X86_64 "sandybridge")

# i686 specific
set(HEBI_TARGET_CPU_X86 "prescott")

################################
# Host Architecture Initialization

string(TOLOWER "${CMAKE_HOST_SYSTEM_PROCESSOR}" HEBI_HOST_ARCH)

if("${HEBI_HOST_ARCH}" STREQUAL "aarch64")
  # arm 64 bit
  set(HEBI_HOST_ARM TRUE)
  set(HEBI_HOST_INTEL FALSE)
  set(HEBI_HOST_64BIT TRUE)
elseif("${HEBI_HOST_ARCH}" MATCHES "^arm" AND CMAKE_SIZEOF_VOID_P EQUAL 4)
  # arm 32 bit (armhf ABI)
  set(HEBI_HOST_ARCH "armhf")
  set(HEBI_HOST_ARM TRUE)
  set(HEBI_HOST_INTEL FALSE)
  set(HEBI_HOST_64BIT FALSE)
elseif("${HEBI_HOST_ARCH}" STREQUAL "i686" OR "${HEBI_HOST_ARCH}" STREQUAL "x86")
  # i686
  set(HEBI_HOST_ARCH "i686")
  set(HEBI_HOST_ARM FALSE)
  set(HEBI_HOST_INTEL TRUE)
  set(HEBI_HOST_64BIT FALSE)
elseif("${HEBI_HOST_ARCH}" STREQUAL "x86_64" OR "${HEBI_HOST_ARCH}" STREQUAL "amd64" OR "${HEBI_HOST_ARCH}" STREQUAL "x64")
  # x86_64
  set(HEBI_HOST_ARCH "x86_64")
  set(HEBI_HOST_ARM FALSE)
  set(HEBI_HOST_INTEL TRUE)
  set(HEBI_HOST_64BIT TRUE)
endif()

################################
# Host Platform Initialization

################
# Linux

if("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")

  if ("${ARCH}" STREQUAL "native")
    message("Setting ARCH from native to ${ARCH}")
    set(ARCH "${HEBI_HOST_ARCH}")
  endif()

  if(NOT DEFINED ARCH)
    MESSAGE(FATAL_ERROR "Must set processor architecture for Linux build: pass '-DARCH=<arch>' to cmake, where <arch> is armhf, aarch64, i686 or x86_64")
  elseif("${ARCH}" STREQUAL "armhf")
    set(HEBI_TARGET_TRIPLET "arm-linux-gnueabihf")
    set(HEBI_TARGET_ARCH "armhf")
    set(HEBI_TARGET_ARM TRUE)
    set(HEBI_TARGET_INTEL FALSE)
    set(HEBI_TARGET_64BIT FALSE)
  elseif("${ARCH}" STREQUAL "aarch64")
    set(HEBI_TARGET_TRIPLET "aarch64-linux-gnu")
    set(HEBI_TARGET_ARCH "aarch64")
    set(HEBI_TARGET_ARM TRUE)
    set(HEBI_TARGET_INTEL FALSE)
    set(HEBI_TARGET_64BIT TRUE)
  elseif("${ARCH}" STREQUAL "i686")
    set(HEBI_TARGET_TRIPLET "i686-linux-gnu")
    set(HEBI_TARGET_ARCH "i686")
    set(HEBI_TARGET_ARM FALSE)
    set(HEBI_TARGET_INTEL TRUE)
    set(HEBI_TARGET_64BIT FALSE)
  elseif("${ARCH}" STREQUAL "x86_64")
    set(HEBI_TARGET_TRIPLET "x86_64-linux-gnu")
    set(HEBI_TARGET_ARCH "x86_64")
    set(HEBI_TARGET_ARM FALSE)
    set(HEBI_TARGET_INTEL TRUE)
    set(HEBI_TARGET_64BIT TRUE)
  else()
    MESSAGE(WARNING "ERROR: Unknown architecture '${ARCH}'")
    MESSAGE(FATAL_ERROR "Possible 'ARCH' options: armhf, aarch64, i686 or x86_64")
  endif()

  # Check cross compiling
  if(
    "${HEBI_HOST_ARCH}" STREQUAL "${HEBI_TARGET_ARCH}" OR
  (HEBI_HOST_INTEL AND HEBI_TARGET_INTEL)
  )
    set(HEBI_CROSS_COMPILING FALSE)
  else()
    set(HEBI_CROSS_COMPILING TRUE)
    # Initialize cross compiling framework
    # By defining HEBI_TOOLCHAIN_ROOT_PATH, you can customize where the root path is for the toolchain
    if(DEFINED HEBI_TOOLCHAIN_ROOT_PATH)
      set(CMAKE_FIND_ROOT_PATH "${HEBI_TOOLCHAIN_ROOT_PATH}")
    else()
      # This is the default cross compile root path on Linux distros
      set(CMAKE_FIND_ROOT_PATH "/usr/${HEBI_TARGET_TRIPLET}")
    endif()
    # search for programs in the build host directories
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    # for libraries and headers in the target directories
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  endif()

  set(HEBI_HOST_LINUX TRUE)

################
# Windows

# Cygwin environment on Windows
elseif(CYGWIN)

  if ("${ARCH}" STREQUAL "native")
    message("Setting ARCH from native to ${ARCH}")
    set(ARCH "${HEBI_HOST_ARCH}")
  endif()

  if(NOT DEFINED ARCH)
    MESSAGE(FATAL_ERROR "Must set processor architecture for Cygwin build: pass '-DARCH=<arch>' to cmake, where <arch> is i686 or x86_64")
  elseif("${ARCH}" STREQUAL "i686")
    set(HEBI_TARGET_TRIPLET "i686-pc-cygwin")
    set(HEBI_TARGET_ARCH "i686")
    set(HEBI_TARGET_ARM FALSE)
    set(HEBI_TARGET_INTEL TRUE)
    set(HEBI_TARGET_64BIT FALSE)
  elseif("${ARCH}" STREQUAL "x86_64")
    set(HEBI_TARGET_TRIPLET "x86_64-pc-cygwin")
    set(HEBI_TARGET_ARCH "x86_64")
    set(HEBI_TARGET_ARM FALSE)
    set(HEBI_TARGET_INTEL TRUE)
    set(HEBI_TARGET_64BIT TRUE)
  else()
    MESSAGE(WARNING "ERROR: Unknown architecture '${ARCH}'")
    MESSAGE(FATAL_ERROR "Possible 'ARCH' options: i686 or x86_64")
  endif()

  set(HEBI_HOST_WINDOWS TRUE)

# Native Win32 environment
elseif(MSVC)

  if ("${ARCH}" STREQUAL "native")
    if("${HEBI_HOST_ARCH}" STREQUAL "i686")
      set(ARCH "x86")
      message("Setting ARCH from native to ${ARCH}")
    elseif("${HEBI_HOST_ARCH}" STREQUAL "x86_64")
      set(ARCH "x64")
      message("Setting ARCH from native to ${ARCH}")
    else()
      message(FATAL_ERROR "Please set processor architecture explicitly.")
    endif()
  endif()

  if(NOT DEFINED ARCH)
    MESSAGE(FATAL_ERROR "Must set processor architecture for MSVC build: pass '-DARCH=<arch>' to cmake, where <arch> is ARM, ARM64, x86 or x64")
  elseif("${ARCH}" STREQUAL "x86")
    set(HEBI_TARGET_TRIPLET "")
    set(HEBI_TARGET_ARCH "x86")
    set(HEBI_TARGET_ARM FALSE)
    set(HEBI_TARGET_INTEL TRUE)
    set(HEBI_TARGET_64BIT FALSE)
  elseif("${ARCH}" STREQUAL "x64")
    set(HEBI_TARGET_TRIPLET "")
    set(HEBI_TARGET_ARCH "x64")
    set(HEBI_TARGET_ARM FALSE)
    set(HEBI_TARGET_INTEL TRUE)
    set(HEBI_TARGET_64BIT TRUE)
  elseif("${ARCH}" STREQUAL "ARM")
    set(HEBI_TARGET_TRIPLET "")
    set(HEBI_TARGET_ARCH "ARM")
    set(HEBI_TARGET_ARM TRUE)
    set(HEBI_TARGET_INTEL FALSE)
    set(HEBI_TARGET_64BIT FALSE)
  elseif("${ARCH}" STREQUAL "ARM64")
    set(HEBI_TARGET_TRIPLET "")
    set(HEBI_TARGET_ARCH "ARM64")
    set(HEBI_TARGET_ARM TRUE)
    set(HEBI_TARGET_INTEL FALSE)
    set(HEBI_TARGET_64BIT TRUE)
  else()
    MESSAGE(WARNING "ERROR: Unknown architecture '${ARCH}'")
    MESSAGE(FATAL_ERROR "Possible 'ARCH' options: ARM, ARM64, x86 or x64")
  endif()

  set(HEBI_HOST_WINDOWS TRUE)

################
# OSX

elseif(APPLE)

  # Arch doesn't need to be defined, since the only option is amd64
  if(DEFINED ARCH)
    MESSAGE(WARNING "'ARCH' ignored for OSX. Targeting amd64")
  endif()

  set(HEBI_TARGET_ARCH "amd64")
  set(HEBI_TARGET_ARM FALSE)
  set(HEBI_TARGET_INTEL TRUE)
  set(HEBI_TARGET_64BIT TRUE)
  set(HEBI_HOST_OSX TRUE)

  # Work around CMP0025
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
    set(HEBI_OSX_IS_APPLECLANG TRUE)
  endif()

endif()

################################
# Target Type Initialization

hebi_setup_toolchain()
hebi_setup_build_type()
