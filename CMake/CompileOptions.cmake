

function(compile_options TARGET_NAME)
    if(APPLE)
        target_compile_definitions(${TARGET_NAME} PUBLIC Darwin)
    endif()
    target_compile_options(${TARGET_NAME} PRIVATE -Wnon-virtual-dtor
                                                  -Wsign-promo
                                                  -Wvla
                                                  -fno-strict-aliasing
                                                  -Wall
                                                  -Wextra
                                                  -Winvalid-pch
                                                  -Winit-self
                                                  -Wno-unknown-pragmas
                                                  -Wshadow
                                                  -Werror
                                                  -O3)
endfunction()
