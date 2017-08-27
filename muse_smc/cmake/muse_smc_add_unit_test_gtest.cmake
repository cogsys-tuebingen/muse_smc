## use with gtest
# add_unit_test_gtest is a wrapper function for catkin_add_gtest
#   UNIT_TEST_NAME : is the name for the test
#   UNIT_TEST_SRCS : a list of sources - make sure to wrap into quotes
#   UNIT_TEST_LIBS : a list of libraries to link - make sure to wrap into quotes
#                    and use semicoli as delimiters.
function(add_unit_test_gtest UNIT_TEST_NAME UNIT_TEST_SRCS UNIT_TEST_LIBS)
    find_package(Boost REQUIRED COMPONENTS system)

    catkin_add_gtest(${UNIT_TEST_NAME}
        ${UNIT_TEST_SRCS}
    )
    target_link_libraries(${UNIT_TEST_NAME}
        ${UNIT_TEST_LIBS}
        ${GTEST_LIBRARIES}
        ${Boost_LIBRARIES}
    )
endfunction()

## use with gtest
# add_unit_test_gtest is a wrapper function for catkin_add_gtest
#   UNIT_TEST_NAME : is the name for the test
#   UNIT_TEST_SRCS : a list of sources - make sure to wrap into quotes
function(add_unit_test_gtest UNIT_TEST_NAME UNIT_TEST_SRCS)
    find_package(Boost REQUIRED COMPONENTS system)

    catkin_add_gtest(${UNIT_TEST_NAME}
        ${UNIT_TEST_SRCS}
    )
    target_link_libraries(${UNIT_TEST_NAME}
        ${UNIT_TEST_LIBS}
        ${GTEST_LIBRARIES}
        ${Boost_LIBRARIES}
    )
endfunction()

