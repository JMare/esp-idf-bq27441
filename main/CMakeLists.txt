cmake_minimum_required(VERSION 3.5)

file(GLOB SOURCES *.cpp)
idf_component_register(SRCS ${SOURCES}
	INCLUDE_DIRS "."
  REQUIRES esp-idf-bq27441)

target_compile_options(${COMPONENT_LIB} PRIVATE -fdiagnostics-color=always)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
