# Add Components
set(module_dirs
    Test
)

foreach(dir ${module_dirs})
    set(MODULE_SRC_DIR ${APP_PATH}/components/Middlewares/${dir})
    aux_source_directory(${MODULE_SRC_DIR} MODULE_SRC)
    target_sources(${EXAMPLE_LIB}
        PRIVATE
            ${MODULE_SRC}
    )
    target_include_directories(${EXAMPLE_LIB}
        PRIVATE
            ${MODULE_SRC_DIR}
    )
endforeach()