idf_component_register(
    SRCS "bosch/bhy2.c" "bosch/bhy2_hif.c" "bosch/bhy2_parse.c" "bhi260ap_core.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES freertos twatch_board
)
