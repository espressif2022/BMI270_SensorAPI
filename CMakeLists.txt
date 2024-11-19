idf_component_register(
    SRC_DIRS "." "./bmi270_examples/common/"
    INCLUDE_DIRS "." "./bmi270_examples/"
    REQUIRES "driver"
)

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
