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
# TODO: When using 50MHz as input clock change CLOCK freq
# and replace pll_main_200_inst by pll_main_50_inst
#**************************************************************

create_clock -period "20 MHz" -name {CLK} {CLK}
#create_clock -period "200 MHz" -name {CLOCK} {CLOCK}
derive_pll_clocks 
derive_clock_uncertainty

set clk_100MHz  pll_inst|altpll_component|auto_generated|pll1|clk[0]
set clk_2MHz    pll_inst|altpll_component|auto_generated|pll1|clk[1]
set clk_300kHz  pll_inst|altpll_component|auto_generated|pll1|clk[2]
set clk_20MHz   pll_inst|altpll_component|auto_generated|pll1|clk[3]


#***************************************************************************
# False paths to static or slow I/Os
#***************************************************************************
# Outputs

set_false_path -from {*} -to {EN_SND*}
set_false_path -from {*} -to {PD?}
set_false_path -from {*} -to {MUX1*}
set_false_path -from {*} -to {MUX2*}
set_false_path -from {*} -to {PREAMP*}
set_false_path -from {*} -to {MASTERSLAVE0}
set_false_path -from {*} -to {RS485*}
set_false_path -from {*} -to {Van_*}
set_false_path -from {*} -to {V5_EN}
set_false_path -from {*} -to {V3_SYNC}
set_false_path -from {*} -to {V6_SYNC}
set_false_path -from {*} -to {V6_EN}
set_false_path -from {*} -to {LED_*}
set_false_path -from {*} -to {TRG}

# Inputs
set_false_path -from {MASTERSLAVE1} -to {*}
set_false_path -from {RS485*} -to {*}
set_false_path -from {V3_PG} -to {*}
set_false_path -from {V5_PG} -to {*}
set_false_path -from {OVLD} -to {*}
set_false_path -from {CFG} -to {*}

# Unused. TODO: Constraint if used
set_false_path -from {Multi_IO} -to {*}
set_false_path -from {LVDS} -to {*}

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

