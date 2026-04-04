add_subdirectory(shell)
target_sources_ifdef(CONFIG_ZMK_ADAPTIVE_RGB app PRIVATE adaptive_rgb.c)
