set(deps
    driver
    heap
    nvs_flash
    vfs
    app_update
    spiffs
    OpenMRNIDF
)

idf_component_register(SRCS Esp32C3OlcbIO.cpp fs.cpp nvs_config.cpp
                       REQUIRES "${deps}")

# export the project version as a define for the SNIP data, note it must be
# truncated to 21 characters max.
idf_build_get_property(project_ver PROJECT_VER)
string(SUBSTRING "${project_ver}" 0 20 SNIP_SW_VERSION)
idf_build_get_property(project_name PROJECT_NAME)
string(SUBSTRING "${project_name}" 0 40 SNIP_PROJECT_NAME)

set(SNIP_HW_VERSION "1.0.0")
set(SNIP_PROJECT_PAGE "http://atanisoft.github.io/Esp32C3OlcbIO")
set(CDI_VERSION "0x0101")

set_source_files_properties(Esp32C3OlcbIO.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")
set_source_files_properties(Esp32C3OlcbIO.cpp PROPERTIES COMPILE_DEFINITIONS "SNIP_PROJECT_PAGE=\"${SNIP_PROJECT_PAGE}\"; SNIP_HW_VERSION=\"${SNIP_HW_VERSION}\"; SNIP_SW_VERSION=\"${SNIP_SW_VERSION}\"; SNIP_PROJECT_NAME=\"${SNIP_PROJECT_NAME}\"; CDI_VERSION=${CDI_VERSION}")
set_source_files_properties(nvs_config.cpp PROPERTIES COMPILE_FLAGS "-Wno-ignored-qualifiers")
