/*! \mainpage The HEBI C++ API
 *
 * \section intro_sec Introduction
 *
 * The C++ API provides a more user-friendly interface to the HEBI C API.  For those familiar with the C API, the core concepts here are the same.
 *
 * The purpose of this API is to allow the user to communicate with \link hebi::Group groups \endlink of HEBI modules.  This is accomplished by sending \link hebi::Command commands \endlink to the modules or by requesting \link hebi::Feedback feedback \endlink and other \link hebi::Info info \endlink from the modules.
 *
 * A good place to start is figuring out how to create a group -- this is done through the \link hebi::Lookup lookup\endlink, which acts as a registry of modules.
 *
 * Then, look at the basic functionality of a \link hebi::Group group \endlink - how to send commands and receive feedback and other info.
 *
 * Finally, dig into the details of the \link hebi::Command command\endlink, \link hebi::Feedback feedback\endlink, and \link hebi::Info info\endlink structures to see all of the information that you can send to and receive from the modules.
 *
 * Along the way, there are plenty of examples to help you get started -- these are a great place to start for those just wanting to jump right in.
 *
 * \section install_sec Installation/Setup
 *
 * NOTE: the API is currently supported on Windows and Linux 64-bit (x64) architecture systems.
 *
 * The API uses CMake to easily generate platform-specific build files for the examples. However, CMake is not needed to use the API or even to build the examples -- see the \link api_outside section below \endlink for guidance on how to set up a custom build system.
 *
 * To get started, build and run the example scripts.  First unzip the provided API .zip or .tar.gz file.
 *
 * If you choose to use CMake, create a directory to contain your build files (we suggest creating a directory called "build" as a subdirectory in the extracted API folder).  From this file, run "cmake <path to API folder root>".
 *
 * After this, the process to build and run the program changes depending on the platform and resulting build system.  For example, using a standard Makefile, you can run "make examples" to build the example scripts.  On Windows with Visual Studio installed, you can open the .sln file and build the "examples" project.
 *
 * You can then run the scripts by running the generated example executables. Generally, the example scripts will only be useful in conjunction with modules accessible on the network. Depending on the example, you will have to either pass appropriate command line arguments to determine the module or group to use, or modify the hardcoded values in the example itself.
 *
 * The command line arguments needed for these examples will be described when running the example with no arguments; the 'lookup_helpers.cpp' file is responsible for the argument parsing for those who are interested.
 *
 * For a fast/easy start, copy one of the existing examples and start from there -- the existing examples are in the "examples" directory.
 *
 * \section api_outside Using the library and API in your project
 *
 * First, note that this library was designed to be used with the C++11 standard, and is not compatible with C++03.  Ensure that you have the proper compiler switches to enable this support (e.g., -std=c++11 for gcc).
 *
 * Integrating the library in your own project follows the basic approach taken by the examples (and looking through the CMakeLists.txt file is a good start).  The steps are:
 *
 * 1. Add the include/ and src/ directories to your own project (they can be renamed as necessary) to allow you to reference the library functions from your code.
 * 2. Add the library binary files to your project to allow you to link your executable against the library; ensure this path is referenced at runtime.
 * 3. Add the necessary elements to your build process to reference the include/src directories and link against the HEBI library (as well as pthreads when running on Linux).
 *
 * The third step is the tricky one -- for projects using CMake, this should be relatively straightforward by following the example of the CMakeLists.txt used for the example code.  Of course, your build system may be significantly different, even when using CMake.  Basically, the crucial elements are a few command line parameters during compilation and linking of your project (the arguments below assume you are using gcc or g++):
 *
 * 1. Add "-I<directory where you put the files from 'include'>" and "-I<directory where you put the files from 'src'>" during compilation to ensure the compiled code generates the correct library calls.
 * 2. Add "-L<directory where you put library> -lpthread -lhebi" during linking to ensure the calls to the library and its dependencies can be resolved. Remember that linking is done left to right, so these should be included after your source files on the command line.
 * 3. Also, ensure the files in 'src' are passed in as sources during the compilation/linking processes.
 *
 * \section api_notes API Design/Usage Notes
 *
 * Generally, users of the C++ API should restrict themselves to the documented public functions of the classes in the C++ API (those in the 'src' directory).  There are a handful of public functions which are marked as 'internal'; these must be public due to certain class interactions, but should not be used except by other classes in the API.
 *
 * Files in the C++ API are able to be modified by the user, but this is not recommended as (1) there may be unintended side effects and (2) future API updates may change these files arbitrarily.
 *
 * Similarly, users of the C++ API are expected to only access HEBI modules through the C++ API; direct use of the lower-level C API along with the C++ API is not recommended, as the C++ wrappers are designed to prevent memory management pitfalls.
 *
 */
