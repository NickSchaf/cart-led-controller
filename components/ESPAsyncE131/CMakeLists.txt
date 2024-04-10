set(e131_src
../../ESPAsyncE131/RingBuf.c
../../ESPAsyncE131/ESPAsyncE131.cpp
)

# if needed, private requires example:
# PRIV_REQUIRES console nvs_api AsyncUDP

idf_component_register(SRCS ${e131_src} INCLUDE_DIRS "../../ESPAsyncE131" "." REQUIRES AsyncUDP )

# Must place this after component register
# example extra definition:
#target_compile_options(${COMPONENT_LIB} PUBLIC -DMY_PRECOMPILE_DEFINE)

