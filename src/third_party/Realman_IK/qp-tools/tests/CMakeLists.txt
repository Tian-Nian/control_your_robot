find_package(GTest)
find_package(Eigen3)

if(${GTest_FOUND} AND ${Eigen3_FOUND})
    # add_executable(tests)
    # target_sources(tests PRIVATE ${PROJECT_SOURCE_DIR}/tests/test.cpp ${PROJECT_SOURCE_DIR}/tests/parser.cpp)
    # target_include_directories(tests PRIVATE ${GTEST_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/tests/include)
    # target_link_libraries(tests PRIVATE ${GTEST_LIBRARIES} qpSWIFT-static m pthread)

    add_executable(stdaln)
    target_sources(stdaln PRIVATE ${PROJECT_SOURCE_DIR}/tests/stdaln.cpp)
    target_include_directories(stdaln PRIVATE ${GTEST_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/tests/include)
    target_link_libraries(stdaln PRIVATE ${GTEST_LIBRARIES} qpSWIFT-static m pthread)
    
    add_executable(stdalnT)
    target_sources(stdalnT PRIVATE ${PROJECT_SOURCE_DIR}/tests/stdalnT.cpp)
    target_include_directories(stdalnT PRIVATE ${GTEST_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/tests/include)
    target_link_libraries(stdalnT PRIVATE ${GTEST_LIBRARIES} qpSWIFT-static m pthread)

    # add_executable(stdalnTT)
    # target_sources(stdalnTT PRIVATE ${PROJECT_SOURCE_DIR}/tests/stdalnTT.cpp)
    # target_include_directories(stdalnTT PRIVATE ${GTEST_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/tests/include)
    # target_link_libraries(stdalnTT PRIVATE ${GTEST_LIBRARIES} qpSWIFT-static m pthread)


endif()