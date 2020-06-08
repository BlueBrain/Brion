

function(compile_options TARGET_NAME)
    if(APPLE)
        target_compile_definitions(${TARGET_NAME} PUBLIC Darwin)
    endif()
    set(COMPILE_OPTIONS)
    list(APPEND COMPILE_OPTIONS -Wsign-promo
                                -Wvla
                                -fno-strict-aliasing
                                -Wextra
                                -Winvalid-pch
                                -Winit-self
                                -Wno-unknown-pragmas
                                -Wshadow
                                -Werror
                                -fPIC
                                -O3)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
        list(APPEND COMPILE_OPTIONS -Wno-non-virtual-dtor -Wno-deprecated)
    else()
        list(APPEND COMPILE_OPTIONS -Wnon-virtual-dtor -Wno-deprecated)
    endif()
    target_compile_options(${TARGET_NAME} PRIVATE ${COMPILE_OPTIONS})
endfunction()
