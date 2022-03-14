set_global_assignment -entity "unique_id" -library "lib_unique_id" -name IP_TOOL_NAME "altchip_id"
set_global_assignment -entity "unique_id" -library "lib_unique_id" -name IP_TOOL_VERSION "16.1"
set_global_assignment -entity "unique_id" -library "lib_unique_id" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_unique_id" -name SPD_FILE [file join $::quartus(sip_path) "unique_id.spd"]

set_global_assignment -library "lib_unique_id" -name MISC_FILE [file join $::quartus(sip_path) "unique_id_sim/unique_id.v"]
set_global_assignment -library "lib_unique_id" -name MISC_FILE [file join $::quartus(sip_path) "unique_id_sim/altchip_id/altchip_id.v"]
