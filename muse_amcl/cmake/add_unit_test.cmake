## use with gtest
function(add_unit_test UNIT_TEST_NAME UNIT_TEST_SRCS UNIT_TEST_LIBS)
    enable_testing()

    add_executable(${UNIT_TEST_NAME}
        ${UNIT_TEST_SRCS}
    )

    target_link_libraries(${UNIT_TEST_NAME}
        ${UNIT_TEST_LIBS}
        ${GTEST_LIBRARIES}
    )

    add_custom_command(TARGET ${UNIT_TEST_NAME}
                       COMMENT "Run ${UNIT_TEST_NAME} ..."
                       POST_BUILD COMMAND ${UNIT_TEST_NAME}
                       WORKING_DIRECTORY  ${CMAKE_BINARY_DIR}
    )
endfunction()
