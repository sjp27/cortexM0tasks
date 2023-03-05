cmake_minimum_required(VERSION 3.15)

project(test LANGUAGES C CXX ASM)

find_bsp(
	ID com.sysprogs.arm.nxp_lpc
	VERSION 2.2
	MCU LPC1114FN28
	HWREGISTER_LIST_FILE LPC11xx/DeviceDefinitions/LPC11CXX.xml
	DISABLE_GNU_EXTENSIONS
	FRAMEWORKS com.sysprogs.arm.fixed_stack_and_heap com.sysprogs.embedded.semihosting_and_profiler
	FWCONFIGURATION
		com.sysprogs.bspoptions.stackheap.stacksize=256
		com.sysprogs.bspoptions.stackheap.heapsize=3072
		com.sysprogs.efp.semihosting.buffer_size=128
		com.sysprogs.efp.semihosting.blocking_mode=0)

add_bsp_based_executable(
	NAME test
	GENERATE_BIN
	GENERATE_MAP
	SOURCES test.c)
target_compile_definitions(test PRIVATE)
target_link_libraries(test PRIVATE -lcortexM0tasks -LC:/Home/c/arm_projects/cortexM0tasks/build/VisualGDB/Debug)
target_include_directories(test PRIVATE ../src)
