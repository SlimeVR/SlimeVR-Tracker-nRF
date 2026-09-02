set(pm_static_dir ${CMAKE_CURRENT_LIST_DIR}/pm_static)

set(pm_static_candidates)

# (pm_static) + BOARD + QUALIFIERS
if(DEFINED SB_CONFIG_BOARD_QUALIFIERS AND NOT SB_CONFIG_BOARD_QUALIFIERS STREQUAL "")
  string(REPLACE "/" "_" pm_static_qualifiers ${SB_CONFIG_BOARD_QUALIFIERS})
  list(APPEND pm_static_candidates
    ${pm_static_dir}/pm_static_${SB_CONFIG_BOARD}_${pm_static_qualifiers}.yml
    ${pm_static_dir}/${SB_CONFIG_BOARD}_${pm_static_qualifiers}.yml
  )
endif()

# (pm_static) + BOARD
list(APPEND pm_static_candidates
  ${pm_static_dir}/pm_static_${SB_CONFIG_BOARD}.yml
  ${pm_static_dir}/${SB_CONFIG_BOARD}.yml
)

# (pm_static) + SOC + "uf2" in BOARD or QUALIFIERS
if((DEFINED SB_CONFIG_BOARD AND SB_CONFIG_BOARD MATCHES "uf2") OR (DEFINED SB_CONFIG_BOARD_QUALIFIERS AND SB_CONFIG_BOARD_QUALIFIERS MATCHES "uf2"))
  list(APPEND pm_static_candidates
    ${pm_static_dir}/pm_static_${SB_CONFIG_SOC}_uf2.yml
    ${pm_static_dir}/${SB_CONFIG_SOC}_uf2.yml
  )
endif()

#message("-- Partition manager static configuration search candidates: ${pm_static_candidates}")

foreach(pm_static_candidate ${pm_static_candidates})
  if(EXISTS ${pm_static_candidate})
    set(PM_STATIC_YML_FILE ${pm_static_candidate} CACHE INTERNAL "")
    break()
  endif()
endforeach()
