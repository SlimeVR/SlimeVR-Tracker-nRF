if (CONFIG_SOC_NRF54L15_CPUAPP)
  board_runner_args(jlink "--device=nRF54L15_M33" "--speed=4000")
elseif (CONFIG_SOC_NRF54L15_CPUFLPR)
  board_runner_args(jlink "--device=nRF54L15_RV32" "--speed=4000")
endif()

include(${ZEPHYR_BASE}/boards/common/nrfutil.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)