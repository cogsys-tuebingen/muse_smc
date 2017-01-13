## use with gtest
enable_testing()
find_package(rostest REQUIRED)
# add_unit_test_rostest is a wrapper function for add_rostest_gtest
#   UNIT_TEST_NAME   : is the name for the test
#   UNIT_TEST_LAUNCH : is the launch file used to start the test cases
#   UNIT_TEST_SRCS   : a list of sources - make sure to wrap into quotes
#   UNIT_TEST_LIBS   : a list of libraries to link - make sure to wrap into quotes
#                      and use semicoli as delimiters.
function(add_unit_test_rostest UNIT_TEST_NAME UNIT_TEST_LAUNCH UNIT_TEST_SRCS UNIT_TEST_LIBS)
    add_rostest_gtest(${UNIT_TEST_NAME}
        ${UNIT_TEST_LAUNCH}
        ${UNIT_TEST_SRCS}
    )
    target_link_libraries(${UNIT_TEST_NAME}
        ${UNIT_TEST_LIBS}
        ${GTEST_LIBRARIES}
    )
endfunction()
