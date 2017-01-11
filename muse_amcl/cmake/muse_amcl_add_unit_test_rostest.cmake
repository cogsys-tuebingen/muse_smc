## use with gtest
find_package(rostest REQUIRED)
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
