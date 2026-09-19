idf_component_register(
    SRCS "main.c"
         "demo_koi.c"
    INCLUDE_DIRS "."
    REQUIRES bsp esp_timer
)
