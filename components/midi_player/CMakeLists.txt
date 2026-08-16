idf_component_register(SRCS "midi_player.cpp" "midi_parser.cpp"
        INCLUDE_DIRS "include"
        REQUIRES "foc_driver" "project_conf" "fast_trig" "esp_timer"
)
