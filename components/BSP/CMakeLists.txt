# Add Driver
set(src_dirs
    TFTLCD
    CAMERA
    SERVO
)

foreach(dir ${src_dirs})
    set(SRC_DIR ${APP_PATH}/components/BSP/${dir})
    aux_source_directory(${SRC_DIR} DRIVE_SRC)
    target_sources(${EXAMPLE_LIB}
        PRIVATE
            ${DRIVE_SRC}
    )
    target_include_directories(${EXAMPLE_LIB}
        PRIVATE
            ${SRC_DIR}
    )
endforeach()
