idf_component_register( SRCS "main.c" "wifi.c" "gps.c" "libnmea/nmea/nmea.c" "libnmea/nmea/parser_static.c" "libnmea/parsers/gpgga.c" "libnmea/parsers/parse.c"
                        INCLUDE_DIRS "" "libnmea/nmea" "libnmea/parsers" )

add_compile_definitions(PARSER_COUNT=1 ENABLE_GPGGA=1)