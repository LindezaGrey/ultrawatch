idf_component_register(
    SRCS "bosch/bhy2.c" "bosch/bhy2_hif.c" "bosch/bhy2_parse.c" "bhi260ap_core.c"
    INCLUDE_DIRS "include"
    EMBED_FILES "BHI260AP.fw"
    PRIV_REQUIRES freertos twatch_board esp_driver_gpio
)
