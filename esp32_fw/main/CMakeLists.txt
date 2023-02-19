idf_component_register( SRCS "main.c" "wifi.c" "gps.c" 
                        INCLUDE_DIRS "" )

add_compile_definitions(PARSER_COUNT=1 ENABLE_GPGGA=1)