if(NOT GTest_FOUND)
    message(FATAL_ERROR "tests enabled but GTest not found")
endif()

add_library(test_common STATIC
    test_common/environment.cpp
)

target_include_directories(test_common PUBLIC
    ${PROJECT_SOURCE_DIR}/tests
)

target_link_libraries(test_common PUBLIC
    ${PROJECT_NAME}
    GTest::gtest
    GTest::gtest_main
)

function(create_test TEST_NAME)
    if(NOT ARGN)
        message(FATAL_ERROR "create_test(${TEST_NAME}) requires source files")
    endif()

    add_executable(${TEST_NAME} ${ARGN})

    target_link_libraries(${TEST_NAME} PRIVATE
        test_common
    )

    add_test(
        NAME ${PROJECT_NAME}.${TEST_NAME}
        COMMAND ${TEST_NAME}
    )

    set_tests_properties(${PROJECT_NAME}.${TEST_NAME}
        PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../"
            LABELS "unit"
    )
endfunction()

create_test(test_utils_yaml utils/test_yaml.cpp)