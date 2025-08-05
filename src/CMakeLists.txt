include_directories(${scara_plotter_SOURCE_DIR}/include)

add_executable(scara_plotter main.c)

add_library(scara_plotter_lib
    curve.c
    data_structs.c
    fixed_point.c
    integration.c
    interpolation.c
    manipulator.c
    path.c
    polynomial.c
    )

target_link_libraries(scara_plotter scara_plotter_lib)
