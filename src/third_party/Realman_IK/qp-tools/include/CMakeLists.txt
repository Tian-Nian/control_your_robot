set(
    qpSWIFT_headers
    "${CMAKE_CURRENT_SOURCE_DIR}/amd.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/amd_internal.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/SuiteSparse_config.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/ldl.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/GlobalOptions.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/timer.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/Auxilary.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/qpSWIFT.h"
)


set(
  qpSWIFT_headers
  "${qpSWIFT_headers}"
  PARENT_SCOPE
)