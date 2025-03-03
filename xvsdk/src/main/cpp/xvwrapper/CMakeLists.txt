add_library(xv-wrapper SHARED xv_wrapper.cpp yuv_to_rgb.cpp customer/test.cpp)

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed")

set(LIBNAME "xv-wrapper" )

SET(LIBS ${LIBS} xslam-xv-sdk xslam-unity-wrapper)

set_target_properties(${LIBNAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_ANDROID_ARCH_ABI})


target_link_libraries(xv-wrapper log ${LIBS})

install(TARGETS xv-wrapper DESTINATION lib)
