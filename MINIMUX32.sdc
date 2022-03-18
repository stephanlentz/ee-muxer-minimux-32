## mux32.sdc

#**************************************************************
# Time Information
#**************************************************************
set_time_format -unit ns -decimal_places 3

#**************************************************************
# Temporary false paths
# TODO: The following paths must be constrained when used
#**************************************************************


#**************************************************************
# Create Clocks
#**************************************************************

create_clock -period "20 MHz" -name {CLK} {CLK}
derive_pll_clocks 
derive_clock_uncertainty

set clk_100MHz  pll_inst|altpll_component|auto_generated|pll1|clk[0]
set clk_2MHz    pll_inst|altpll_component|auto_generated|pll1|clk[1]
set clk_20MHz   pll_inst|altpll_component|auto_generated|pll1|clk[2]


#***************************************************************************
# False paths to static or slow I/Os
#***************************************************************************
# Outputs
set_false_path -from * -to [get_ports *]
# Inputs
set_false_path -from [get_ports *] -to *

#**************************************************************
# Set false paths from parameters/unique_id (changing once)
#**************************************************************

set_false_path -from {i_data.bitlen_current*} -to {*}
set_false_path -from {local_data:local_data|unique_id:unique_id_inst|altchip_id:unique_id_inst|regout_wire} -to {*}
set_false_path -from {local_data:local_data|unique_id:unique_id_inst|altchip_id:unique_id_inst|output_reg*} -to {*}

set_false_path -from {*} -to {local_data:local_data|unique_id:unique_id_inst|altchip_id:unique_id_inst|*}

#**************************************************************
# Set false paths for pulse transfer
#**************************************************************

#set_false_path -from {pulse_transfer:pt_res|pulse_i_detected} -to {pulse_transfer:pt_res|pulse_o}
#set_false_path -from {pulse_transfer:pt_res|pulse_i_detected} -to {pulse_transfer:pt_res|pulse_o_set}
#set_false_path -from {pulse_transfer:pt_ack|pulse_o_set} -to {pulse_transfer:pt_ack|pulse_i_detected}

