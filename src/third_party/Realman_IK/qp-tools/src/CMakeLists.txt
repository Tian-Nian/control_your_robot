set(
    qpSWIFT_src
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_1.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_aat.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_defaults.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_global.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_order.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_post_tree.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_valid.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_2.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_control.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_dump.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_info.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_postorder.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_preprocess.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/ldl.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/timer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Auxilary.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/qpSWIFT.c"
)


set(
  qpSWIFT_src
  "${qpSWIFT_src}"
  PARENT_SCOPE
)