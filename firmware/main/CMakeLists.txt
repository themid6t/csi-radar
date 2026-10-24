idf_component_register(SRCS "main.c" "radar.c" "dsp.c" "web.c" "vorp.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "www/setup.html"
                       PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_driver_gpio esp_http_server
                                     esp-tls mbedtls json esp_timer esp_system)

# the page is served gzipped (~4x smaller); rebuilt whenever www/index.html changes
set(gz ${CMAKE_CURRENT_BINARY_DIR}/index.html.gz)
add_custom_command(OUTPUT ${gz}
                   COMMAND ${CMAKE_COMMAND} -E copy ${COMPONENT_DIR}/www/index.html ${CMAKE_CURRENT_BINARY_DIR}/index.html
                   COMMAND gzip -9 -n -f ${CMAKE_CURRENT_BINARY_DIR}/index.html
                   DEPENDS ${COMPONENT_DIR}/www/index.html VERBATIM)
add_custom_target(index_gz DEPENDS ${gz})
add_dependencies(${COMPONENT_LIB} index_gz)
target_add_binary_data(${COMPONENT_LIB} ${gz} BINARY)
