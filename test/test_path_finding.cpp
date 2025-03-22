#include <unity.h>
#include <Arduino.h>
#include "PathPlanning/Mapping.h"

// TEST_ASSERT_X( {modifiers}, {expected}, actual, {size/count} )


void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
} 

void test_initialize_map(void) {
    Mapping::Initialize_Map(Team::Jaune);
}
void test_function_should_do(void) {
    // test stuff
    int8_t expected = 3;
    int8_t result = 3; 
    TEST_ASSERT_EQUAL_INT8(expected, result);
}

void test_function_true(void) {
    TEST_ASSERT_TRUE(true);
}

void test_function_fail(void) {
    TEST_ASSERT_TRUE(false);
}

// execute all test functions
int runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialize_map);
    RUN_TEST(test_function_should_do);
    RUN_TEST(test_function_fail);
    RUN_TEST(test_function_true);
    return UNITY_END();
  }

void setup() {
    // Wait ~2 seconds before the Unity test runner
    // establishes connection with a board Serial interface
    delay(2000);
  
    runUnityTests();
}

void loop() {}