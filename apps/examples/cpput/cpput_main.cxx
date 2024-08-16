/*
 * Disable coverage testing by removing
 * (i)   "-lgcov" (Makefile-test, line 70);
 * (ii)  "-fprofile-arcs -ftest-coverage" (Makefile-test, line 143, 152);
 * (iii) "extern "C" void __gcov_flush();" (src-test/test-main.cpp, line 5);
 * (iv)  "__gcov_flush();" (src-test/test-main.cpp, line 14).
 */

#include <stdio.h>
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness_c.h"



#ifdef CONFIG_DEV_RANDOM
IMPORT_TEST_GROUP(RandomTestGroup);
#endif
#ifdef CONFIG_TIMER
IMPORT_TEST_GROUP(TimerTestGroup);
#endif
IMPORT_TEST_GROUP(UnameTestGroup);
#ifdef CONFIG_SYSTEM_FASTAPI
//IMPORT_TEST_GROUP(FastAnalogTestGroup);
//IMPORT_TEST_GROUP(FastLEDTestGroup);
//IMPORT_TEST_GROUP(FastGPIOexpTestGroup);
//IMPORT_TEST_GROUP(FastPowerManagementTestGroup);
//IMPORT_TEST_GROUP(FastConfigurationTestGroup);

#endif
extern "C"{
int cpput_main(int argc, char *argv[]) {
	//MemoryLeakWarningPlugin::turnOffNewDeleteOverloads(); // Disables memory leaks detection
	CommandLineTestRunner::RunAllTests(argc, argv); // Run All Tests
	return 0;
}
}
