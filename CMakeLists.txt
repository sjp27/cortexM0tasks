cmake_minimum_required(VERSION 3.15)

project(cortexM0tasks LANGUAGES C CXX ASM)

find_bsp(
	ID com.sysprogs.arm.nxp_lpc
	VERSION 2.2
	MCU LPC1114FN28
	HWREGISTER_LIST_FILE LPC11xx/DeviceDefinitions/LPC11CXX.xml
	DISABLE_GNU_EXTENSIONS)

add_bsp_based_library(
	NAME cortexM0tasks
	SOURCES src/cortexM0tasks.c src/cortexM0tasks.h)
target_compile_definitions(cortexM0tasks PRIVATE)
