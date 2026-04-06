add_subdirectory(src)
zephyr_include_directories(include)
zephyr_linker_sources(DATA_SECTIONS include/linker/zaf-custom-events.ld)
