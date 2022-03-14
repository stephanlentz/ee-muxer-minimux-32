//----------------------------------------------------------------------
// BGEZ15N0100A 32 channel array multiplexer
// Two of this can be stacked, the lower one is the master 
// (Coded by MASTERSLAVE0/1)
//----------------------------------------------------------------------

// MUXER type must be defined in "Settings/Compiler Settings/Verilog HDL Input"
`ifndef BOX // 15N03000 DHW Verteilerbox
`ifndef HEAD // 15N02010 8 Channel Multiplexer for DHW Sensor Head with long flex for connector
`ifndef UNIVERSAL_8CH // 15N06000 Universal Multiplexer with 8 sensors, up to 4 stackable 
`ifndef ARRAY_32CH    // 15N01000 32 Channel Array Mux, up to 2 stackable
`ifndef MINIMUX_32CH    // 15N01000 32 Channel Array Mux, up to 2 stackable
    $error("No MUXER type defined in Settings/Compiler Settings/Verilog HDL Input");
`endif
`endif
`endif
`endif    
`endif    

// Defines

`define TYPE          8'd22 // 0x16: ARRAY_32CH
  
`ifdef AT_LEAST_WE_CHANGED_TO_DA_TYPE  
`define CFM_NUMBER    8'd1  // CONFIG_SEL is 1 => default boot is CFM 1, fall back (recovery) is CFM0
`else 
`define CFM_NUMBER    8'd0  // CFM0 (DC type has only one CFM)
`endif  

`define VERSION       8'd4  
`define SUBVERSION    8'd4

`define SERVERSION    8'd6  // UART 9600/enhanced

`define BITLEN_400k   250   // Min enhanced speed is 400kBit/s -> 2500ns    
`define TIMESCALE_NS   10   // 100MHz system/transceiver clock          
`define BITLEN_LIMIT   15   // 150 ns -> limit for head @20MHz (50ns)
`define HIERARCHY_LEVEL 0
`define N_CHILDS        0   // Two array muxer in parallel, no childs


/* History
4.4s6 from 27.9.2021:
- CHANGE: undef BASIC_ALWAYS_ENABLED, so switch to slow only on slow read of register 0x19
          Note: Though this is a change to the serial version we go on using s6 to 
                match the other multiplexers
- BUGFIX: Reset of hierarchy_id (write 0xFF) did not work
4.3.6 from 24.2.2021:
- BUGFIX: In serial_uart_receiver start_bit_detected was not cleared on receive timeout
          Effekt: When slow receiver detects a fake start bit (e.g. when PL600 is switched on)
			 it can hang in the start_bit_detected state, blocking the fast receiver by that.
			 -> Serial Version incremented to 6
- CHANGE: Re-defined BASIC_ALWAYS_ENABLED in transceiver.
4.2 from 22.2.2021:
- CHANGE: Undefined BASIC_ALWAYS_ENABLED in transceiver. Switch to slow by reading 0x19
4.1 from 22.2.2021:
- CHANGE: Reset signal for the business logic only released when inti_ready (after 3 sec)
4.0.5 from 17.9.2020:
- CHANGE: Revision C, Encoder inputs and external V6 supply on dedicated pins

Port of changes in serial communication:
- ADD: switch_to_clear_delay countdown triggered when switching receiver (monitor child)
       (not only when switching from send to receive)
- CHANGE: When basic_serial receives a valid start bit the enhanced_serial receiver is switched off 
         until the ready_pulse of the basic_serial comes
         
3.0 from 19.11.2019:
- CHANGE: Save status information of measured muxch in RAM. On mux message report 
          the latest measured state of the newly addressed muxch
- ADD: Persistent parameter switching_time during which OVLD/SFLT are ignored
2.3 from 1.8.2019:
- ADD: Signal SFLT/OVLD to A6
       NOTE: SFLT can not be detected due to hardware limitations, is always set to 0
2.2 from 14.3.2019:
- ADD: Encoder inputs usable as interface to external V6 converter (if CFG[0] bridged)
2.1 from 22.2.2019:
- ADD: Register voltage_sync (0x2C) to change the sync of the DC/DC converters
2.0 from 30.1.2019:
- ADD: Support for quad encoder
1.4 from 16.1.2019:
- Changed frequency for V3_SYNC from 2.33MHz to 1.538Mz
1.3 from 16.11.2018:
- flash csr status readable
- BUGFIX: CFM unprotect handling
1.2 from 15.11.2018:
- BUGFIX: Some fixes concerning field update
1.1 from 9.11.2018:
- Support of persistent data (Note: local data common source is located in MuxDistributor)
1.0 from 24.10.2017:
- First Version
*/

//----------------------------------------------------------------

module MINIMUX32
(
    output logic [32:1] EN_SND,
    
    output          MUX1P_EN, // Use muxer 1 for SNS 1..16
    output          MUX1P_A0,
    output          MUX1P_A1,
    output          MUX1P_A2,
    output          MUX1P_A3,

    output          MUX1N_EN,
    output          MUX1N_A0,
    output          MUX1N_A1,
    output          MUX1N_A2,
    output          MUX1N_A3,

    output          MUX2P_EN, // Use muxer 2 for SNS 17..32
    output          MUX2P_A0,
    output          MUX2P_A1,
    output          MUX2P_A2,
    output          MUX2P_A3,

    output          MUX2N_EN,
    output          MUX2N_A0,
    output          MUX2N_A1,
    output          MUX2N_A2,
    output          MUX2N_A3,

    output          PREAMP_EN,
    output          PREAMP_A0,
    output          PREAMP_A1,
    
    output          MASTERSLAVE0, // Top connector: Set this to 0
    input           MASTERSLAVE1, // Bottom connector: Use weak pull up, check for 0

    output          RS485_TX,
    output          RS485_DE,
    output          RS485_RE,
    input           RS485_RX,

    input           [3:0] CFG,    // 1 if open

    output          LED_RED,      
    output          LED_GRN,
    output          LED_BLU,

    output          V3_SYNC,
    input           V3_PG,
    output logic    V5_EN,
    input           V5_PG,

    input           LVDS,
    input           CLK,          // 20MHz
    input           MULTI_IO,
    output          TRG,

    input           [4:1] OVLD,
    
    // Dual purpose pins for either Quad Encoder if CFG[0]==1 (open) 
    //                     or external V6 supply if CFG[0]==0 (bridged)
                                   
    input           [0:0] HW_VERSION, // M6 0=RevB  1=RevC
    
    input           ENC_A_V6_PG,      // L15   V6_PG      
    inout           ENC_B_V6_SYNC,    // K15   V6_SYNC 
    inout           ENC_0_V6_EN,      // M15   V6_EN
    inout           V6_EN,            // C16   ENC_A      
    inout           V6_SYNC,          // D16   ENC_B       
    input           ENC_0             // (H15) ENC_0
);

logic ENC_B_V6_SYNC_dout;
logic ENC_B_V6_SYNC_din;
logic ENC_B_V6_SYNC_oe;

tristate_buffer tb_ENC_B_V6_SYNC(
		.dout      (ENC_B_V6_SYNC_din ),
		.din       (ENC_B_V6_SYNC_dout),
		.pad_io    (ENC_B_V6_SYNC     ),
		.oe        (ENC_B_V6_SYNC_oe  )
	);

logic ENC_0_V6_EN_dout;
logic ENC_0_V6_EN_din;
logic ENC_0_V6_EN_oe;

tristate_buffer tb_ENC_0_V6_EN(
		.dout      (ENC_0_V6_EN_din ),
		.din       (ENC_0_V6_EN_dout),
		.pad_io    (ENC_0_V6_EN     ),
		.oe        (ENC_0_V6_EN_oe  )
	);

logic V6_SYNC_dout;
logic V6_SYNC_din;
logic V6_SYNC_oe;

tristate_buffer tb_V6_SYNC(
		.dout      (V6_SYNC_din ),
		.din       (V6_SYNC_dout),
		.pad_io    (V6_SYNC     ),
		.oe        (V6_SYNC_oe  )
	);

logic V6_EN_dout;
logic V6_EN_din;
logic V6_EN_oe;

tristate_buffer tb_V6_EN(
		.dout      (V6_EN_din ),
		.din       (V6_EN_dout),
		.pad_io    (V6_EN     ),
		.oe        (V6_EN_oe  )
	);

assign TRG       = 1'b0; // INC signal to A6 not used
                       
// Master/ Slave handling

assign MASTERSLAVE0 = 1'b0; // Signal to the upper board that we are present
logic master;
assign master = MASTERSLAVE1; // If no lower board present, we are master

// LED: z=off, 0=on

logic led_red, led_green, led_blue;
assign LED_RED = led_red   ? 1'b0 : 1'bz;
assign LED_GRN = led_green ? 1'b0 : 1'bz;
assign LED_BLU = led_blue  ? 1'b0 : 1'bz;

// CFG: 1=open 0=closed (bridged)

(*keep=1*) logic [3:0] cfg;
assign cfg = ~CFG;

//----------------------------------------------------------------

table_if i_table();
data_if  i_data();
status_if i_status();
     
//---------------------------------------------------------------------------------------------------
// Voltage sync
//------------------------------------------------------------------------------                  

always@(*) begin
    case (i_data.v3_sync) 
        v_sync_clocked: V3_SYNC = clk2_22;
        v_sync_high_z:  V3_SYNC = 1'bz;
        v_sync_fixed_0: V3_SYNC = 1'b0;
        v_sync_fixed_1: V3_SYNC = 1'b1;
    endcase
end    

// V6 converter can be external in Rev.B, overlaying the Encoder pins
// Pin    Dir     Rev. B         Rev.C
//---------------------------------------
// L15    input   ENC_A_V6_PG    V6_PG
// K15    inOut   ENC_B_V6_SYNC  V6_SYNC
// M15    inOut   ENC_0_V6_EN    V6_EN
// D16    inOut   V6_SYNC        ENC_B
// C16    inOut   V6_EN          ENC_A
// H15    input   -              ENC_0

logic v6_sync, v6_en, v6_pg;
logic enc_a, enc_b, enc_0;
logic v6_ext_sync;

always@(*) begin
    if (HW_VERSION[0]==1) begin // Rev C
        // In Rev.C the V6 converter is always external using the former dual-purpose pins
        case (i_data.v6_sync) 
            v_sync_clocked: begin ENC_B_V6_SYNC_oe = 1'b1; ENC_B_V6_SYNC_dout = clk1_11; end
            v_sync_high_z:  begin ENC_B_V6_SYNC_oe = 1'b0; ENC_B_V6_SYNC_dout = 1'b0;    end
            v_sync_fixed_0: begin ENC_B_V6_SYNC_oe = 1'b1; ENC_B_V6_SYNC_dout = 1'b0;    end
            v_sync_fixed_1: begin ENC_B_V6_SYNC_oe = 1'b1; ENC_B_V6_SYNC_dout = 1'b1;    end
        endcase
        ENC_0_V6_EN_dout = 1'b1;
        ENC_0_V6_EN_oe   = v6_en;
        v6_pg = ENC_A_V6_PG;
        // The encoder uses the former pins for the internal V6 converter
        V6_SYNC_oe = 1'b0;
        V6_EN_oe   = 1'b0;
        V6_SYNC_dout = 1'b0;
        V6_EN_dout   = 1'b0;
        enc_a = V6_EN_din;
        enc_b = V6_SYNC_din;
        enc_0 = ENC_0;
    end
    else if (CFG[0]) begin
        // Config pin is open -> Use dual purpose pins for encoder
        ENC_B_V6_SYNC_dout = 1'b0;
        ENC_B_V6_SYNC_oe   = 1'b0;
        ENC_0_V6_EN_dout   = 1'b0;
        ENC_0_V6_EN_oe     = 1'b0;
        enc_a = ENC_A_V6_PG;
        enc_b = ENC_B_V6_SYNC_din;
        enc_0 = ENC_0_V6_EN_din;
        // Use internal V6 converter
        case (i_data.v6_sync) 
            v_sync_clocked: begin V6_SYNC_oe = 1'b1; V6_SYNC_dout = clk0_3; end
            v_sync_high_z:  begin V6_SYNC_oe = 1'b0; V6_SYNC_dout = 1'b0;   end
            v_sync_fixed_0: begin V6_SYNC_oe = 1'b1; V6_SYNC_dout = 1'b0;   end
            v_sync_fixed_1: begin V6_SYNC_oe = 1'b1; V6_SYNC_dout = 1'b1;   end
        endcase
        V6_EN_dout = 1'b1;
        V6_EN_oe   = v6_en;
        v6_pg   = 1'b1;   // Note: No PG input from internal V6 converter available
    end
    else begin
        // Config pin is bridged -> Use dual purpose pins for external V6 converter
        case (i_data.v6_sync) 
            v_sync_clocked: begin V6_SYNC_oe = 1'b1; V6_SYNC_dout = clk0_3; end
            v_sync_high_z:  begin V6_SYNC_oe = 1'b0; V6_SYNC_dout = 1'b0;   end
            v_sync_fixed_0: begin V6_SYNC_oe = 1'b1; V6_SYNC_dout = 1'b0;   end
            v_sync_fixed_1: begin V6_SYNC_oe = 1'b1; V6_SYNC_dout = 1'b1;   end
        endcase
        ENC_0_V6_EN_dout = 1'b1;
        ENC_0_V6_EN_oe   = v6_en;
        enc_a = 1'b0;
        enc_b = 1'b0;
        enc_0 = 1'b0;
        // Don't use internal V6 converter
        V6_SYNC_dout = 1'b0;
        V6_SYNC_oe   = 1'b0;
        V6_EN_dout   = 1'b0;
        V6_EN_oe     = 1'b0;
        v6_pg   = ENC_A_V6_PG;
    end
end




//----------------------------------------------------------------
// Clock/Reset
//----------------------------------------------------------------

logic clk100;
logic clk20;
logic clk2_22;
logic clk0_3;
logic pll_locked;
logic reset;
logic pre_reset;
logic n_reset;

pll pll_inst(
	.inclk0(CLK),
	.areset(1'b0),
	.c0(clk100),
	.c1(clk2_22), // 2.22MHz
	.c2(clk0_3), // 300kHz
	.c3(clk20),
	.locked(pll_locked)
    );
    
// Debounce the locked signal until the pll reaches a stable state
debouncer #(.nDebounceBits(16)) lock_debounce(
    .clk(CLK),
    .rst(1'b0), 
    .debounce_width(16'hFFFF),
    .signal_i(pll_locked),
    .signal_o(n_reset)
    );
	 
assign pre_reset=~n_reset;    

logic clk1_11=0;
always@(posedge clk2_22) begin
   clk1_11 = ~clk1_11;
end        

//----------------------------------------------------------------
// Init sequence:
// - Start voltage converters
// - Read non-volatile values from flash        
//----------------------------------------------------------------

logic [9:0] count_ns; // 0..990 in 10ns steps
logic [9:0] count_us; // 0..999 in us steps
logic [9:0] count_ms; // 0..999 in ms steps
logic [2:0] count_s;  // 0..7 in s

always @(posedge clk100 or posedge pre_reset) begin
    if (pre_reset) begin
        count_ns <= '0;
        count_us <= '0;
        count_ms <= '0;
        count_s  <= '0;
    end
    else begin
        if (count_ns >= 10'd1000 - `TIMESCALE_NS) begin
            count_ns <= '0;
            if (count_us == 10'd999) begin
                count_us <= '0;
                if (count_ms == 10'd999) begin
                    count_ms <= '0;
                    count_s <= count_s + 1'd1; 
                end
                else 
                    count_ms <= count_ms + 1'd1;
            end
            else begin
                count_us <= count_us + 1'd1;
            end
        end
        else begin
            count_ns <= count_ns + `TIMESCALE_NS;
        end
    end
end    

//reg initialized;
reg init_ready;
        
assign led_green = |{i_data.config_valid}; //initialized;
assign led_red   = |{OVLD};        
assign led_blue  = count_s[0];

always @(posedge clk100 or posedge pre_reset) begin
    if (pre_reset) begin
        v6_en     <= 1'b0;
        V5_EN     <= 1'bz;
        init_ready <= 1'b0;
    end
    else begin
        if (count_s == 3'd1) 
            v6_en <= 1'b1;
        
        if (count_s == 3'd2)  
            V5_EN <= 1'b1;
            
        if (count_s == 3'd3) 
            init_ready <= 1'b1;
    end
end

assign reset = ~init_ready;

//----------------------------------------------------------------
// Mux mapping to Sender/Receiver/PreAmp
//----------------------------------------------------------------

mux_hw_map mux_map(
    .clock(clk100),
    .reset,
    .i_table
    );
    
assign i_table.muxch = i_data.muxch[5:0];    

genvar i;
generate for (i=0; i<32; i++) begin: snd
    assign EN_SND[i+1] = i_table.en_snd[i] ? 1'b0 : 1'bz;  // LT6559 Enable: 30 ns latency
end
endgenerate

assign PREAMP_EN = 1'b1; //i_table.preamp[2];  // ADG1209 Enable:  115 ns max latency
assign PREAMP_A1 = i_table.preamp[1];          // Address to Data: 185 ns max latency
assign PREAMP_A0 = i_table.preamp[0];
                
assign MUX1P_EN = ~i_table.mux_p[4]; // ADG1206 Enable:  115 ns max latency
assign MUX1P_A3 = i_table.mux_p[3];  // Address to Data: 185 ns max latency
assign MUX1P_A2 = i_table.mux_p[2];
assign MUX1P_A1 = i_table.mux_p[1];
assign MUX1P_A0 = i_table.mux_p[0];

assign MUX2P_EN = i_table.mux_p[4];
assign MUX2P_A3 = i_table.mux_p[3];
assign MUX2P_A2 = i_table.mux_p[2];
assign MUX2P_A1 = i_table.mux_p[1];
assign MUX2P_A0 = i_table.mux_p[0];

assign MUX1N_EN = ~i_table.mux_n[4];
assign MUX1N_A3 = i_table.mux_n[3];
assign MUX1N_A2 = i_table.mux_n[2];
assign MUX1N_A1 = i_table.mux_n[1];
assign MUX1N_A0 = i_table.mux_n[0];

assign MUX2N_EN = i_table.mux_n[4];
assign MUX2N_A3 = i_table.mux_n[3];
assign MUX2N_A2 = i_table.mux_n[2];
assign MUX2N_A1 = i_table.mux_n[1];
assign MUX2N_A0 = i_table.mux_n[0];

//----------------------------------------------------------------
// Serial communication
//------------------------------------------------------------------------------                  

flash_if i_flash(.clk_dc(clk20));
switch_if i_switch(); 

transceiver_if i_trans(
	.tx_line(i_switch.tx_local2ext), 
	.rx_line(i_switch.tx_ext2local),
    .clock(clk100));
assign i_trans.bitlen_enhanced = i_data.bitlen_current;

assign i_trans.switch_to_clear_delay = i_data.switch_to_clear_delay;
assign i_trans.drvoff_to_rcvon_delay = i_data.drvoff_to_rcvon_delay;

assign i_trans.drven         = i_switch.drven_local;
   
assign RS485_TX    = i_switch.tx_box2elo;
assign RS485_DE    = i_switch.drven_box2elo;  // H-active
assign RS485_RE    = i_switch.drven_box2elo;  // L-active
   
assign i_switch.tx_elo2box   = RS485_RX;

protocol_slave   protocol( 
	.clock(clk100), 
	.reset, 
	.i_trans, 
	.i_switch, 
	.i_data 
	);
	
line_switch line_switcher( 
	.clock(clk100), 
	.reset,           
	.i_switch 
	);
	
transceiver #(.timescale_ns(`TIMESCALE_NS)) trans( 
	.clock(clk100), 
	.reset, 
	.i_trans
	);
    
    

//---------------------------------------------------------------------------------------------------
// Local data
//------------------------------------------------------------------------------                  

local_data #(
		.TYPE           (`TYPE),  
		.VERSION        (`VERSION),                        
		.SUBVERSION     (`SUBVERSION),
		.SERVERSION     (`SERVERSION),
        .CFM_NUMBER     (`CFM_NUMBER),
		.BITLEN_400k    (`BITLEN_400k),
        .TIMESCALE_NS   (`TIMESCALE_NS),
		.BITLEN_LIMIT   (`BITLEN_LIMIT),
        .HIERARCHY_LEVEL(`HIERARCHY_LEVEL),
		.N_CHILDS       (`N_CHILDS))            
	 local_data( 
		.clock(clk100), 
        .reset,                      
		.i_data, 
		.i_flash,
        .master,
        .i_table
		);
        
// TODO:        
//assign i_data.cfg = cfg;        
//assign i_data.v6_pg = v6_pg;        

//---------------------------------------------------------------------------------------------------
// Quad Encoder
//------------------------------------------------------------------------------                  

logic a_in, b_in, n_in;
debouncer #(.nDebounceBits(16)) debounce_a(
    .clk(clk100),
    .rst(reset), 
    .debounce_width(i_data.counter_debounce),
    .signal_i(enc_a),
    .signal_o(a_in)
    );
debouncer #(.nDebounceBits(16)) debounce_b(
    .clk(clk100),
    .rst(reset), 
    .debounce_width(i_data.counter_debounce),
    .signal_i(enc_b),
    .signal_o(b_in)
    );
debouncer #(.nDebounceBits(16)) debounce_n(
    .clk(clk100),
    .rst(reset), 
    .debounce_width(i_data.counter_debounce),
    .signal_i(enc_0),
    .signal_o(n_in)
    );

quad_encoder encoder(
    .clock(clk100),
    .reset,
    .a_in(a_in),
    .b_in(b_in),
    .n_in(n_in),
    .res_err_trigger(i_data.counter_err_read),
    // outputs
    .inc_pulse     (i_status.inc_pulse     ),
    .dec_pulse     (i_status.dec_pulse     ),
    .zero_pulse    (i_status.zero_pulse    ), 
    .counter_ab    (i_data.counter_ab    ),
    .counter_at_res(i_data.counter_at_res),
    .counter_err   (i_data.counter_err   )
    );

//---------------------------------------------------------------------------------------------------
// Status    
// S0: OVLD (on the just finished muxch,
// S1: SFLT  will be buffered by A6-demodulator)
// S2: INC
//------------------------------------------------------------------------------                  

logic sflt, ovld;

assign sflt = OVLD[2] & OVLD[3] & ~OVLD[1] & ~OVLD[4]; // Will always be 0 due to hardware limitations
assign ovld = |{OVLD};

assign i_status.sflt_trig = init_ready & i_data.switching_ready & sflt;
assign i_status.ovld_trig = init_ready & i_data.switching_ready & ovld & ~sflt;

// Forward status to protocol slave    
assign i_data.status       = i_status.status;
assign i_status.status_ack = i_data.status_ack;
assign i_status.muxch      = i_data.muxch;

status_engine status(
    .clock(clk100),
    .reset,
    .i_status
    );

//---------------------------------------------------------------------------------------------------
// Flash (UFM/CFM)
//------------------------------------------------------------------------------                  

flash_ctrl flash_ctrl_inst (
    .clk(clk100),  
    .reset,
    .i_flash
);

// TODO: Add analog block

endmodule

