idf_component_register(SRCS "key_value_nvs_impl.cpp" "private/nvs_flash_manager.cpp"
    INCLUDE_DIRS "." REQUIRES brookesia_hal_interface
    PRIV_REQUIRES nvs_flash brookesia_lib_utils)
