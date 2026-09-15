idf_component_register(
    SRCS "bosch/bhy2.c" "bosch/bhy2_hif.c" "bosch/bhy2_parse.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES freertos
)
