# The API reference.
#
# Off by default: Doxygen is not needed to build, test or consume the library,
# and a required tool that most contributors never invoke is a barrier for no
# return. CI turns it on, which is what keeps it honest -- see ADR-0009.

find_package(Doxygen REQUIRED)

set(DOXYGEN_INPUT_DIR  "${PROJECT_SOURCE_DIR}")
set(DOXYGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(DOXYFILE "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

# configure_file rather than a checked-in Doxyfile, so PROJECT_NUMBER is the
# CMake project version and cannot drift from it. A reference that reports the
# wrong version is worse than one that reports none.
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in" "${DOXYFILE}" @ONLY)

add_custom_target(docs
  COMMAND ${DOXYGEN_EXECUTABLE} "${DOXYFILE}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "Generating API reference (warnings are errors)"
  VERBATIM)
