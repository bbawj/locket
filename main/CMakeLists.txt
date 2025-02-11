idf_component_register(
                    SRCS "main.c" "wg.c" "camera.c" "sync_time.c" "decode_image.c" "touch.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES test.jpg)

