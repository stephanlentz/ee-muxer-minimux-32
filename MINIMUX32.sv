//----------------------------------------------------------------------
// BGEZ15N07000 32 channel array mini-multiplexer
//----------------------------------------------------------------------

// MUXER type must be defined in "Settings/Compiler Settings/Verilog HDL Input"
`ifndef MINIMUX_32CH  // 15N07000 32 Channel Array MiniMux
    $error("Wrong MUXER type defined in Settings/Compiler Settings/Verilog HDL Input");
`endif

// Defines

`define TYPE          8'd24 // 0x18: MINIMUX_32CH
  
`ifdef AT_LEAST_WE_CHANGED_TO_DA_TYPE  
`define CFM_NUMBER    8'd1  // CONFIG_SEL is 1 => default boot is CFM 1, fall back (recovery) is CFM0
`else 
`define CFM_NUMBER    8'd0  // CFM0 (DC type has only one CFM)
`endif  

`define VERSION       8'd1
`define SUBVERSION    8'd1

`define SERVERSION    8'd6  // UART 9600/enhanced

`define BITLEN_400k   250   // Min enhanced speed is 400kBit/s -> 2500ns    
`define TIMESCALE_NS   10   // 100MHz system/transceiver clock          
`define BITLEN_LIMIT   15   // 150 ns -> limit for head @20MHz (50ns)
`define HIERARCHY_LEVEL 0
`define N_CHILDS        0   // Two array muxer in parallel, no childs


/* History
1.1. from 31.3.22:
- FIX: Table translation was wrong (copied from ARRAY_32CH)

1.0s6 from 17.3.22:
- First Version, port from ARRAY_32CH 4v4s6

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

    output          RS485_TX,   // P2
    output          RS485_DE,   // R2
    output          RS485_RE,   // P1
    input           RS485_RX,   // N1

    input     [3:0] CFG,        // CFG3 E14  1 if open
                                // CFG2 E11 
                                // CFG1 C15 
                                // CFG0 C14  

    output          LED_RED,    // N14  
    output          LED_GRN,    // M12
    output          LED_BLU,    // N15

    output          V3_SYNC,    // D12 (1-2.2MHz)
    input           V3_PG,      // B6
    
    output          V6_SYNC,    // E15 (1-2.2MHz)
    output          V6P_EN,     // A7
    output          V6N_EN,     // B7   
    
    input           V5P_PG,     // H11
    input           V5N_PG,     // H13
    
    input           CLK,        // J12 20MHz
    output          TRG_N,      // A3
    
    input     [4:1] OVLD,       // OVLD4 D2
                                // OVLD3 A2
                                // OVLD2 E10
                                // OVLD1 M11
    
    // Dual purpose pins for either Quad Encoder if CFG[0]==1 (open) 
    //                     or external V6 supply if CFG[0]==0 (bridged)
                                   
    input           [2:0] REV,  // REV2 K5
                                // REV1 L4
                                // REV0 L5
    
    inout           ENC_0,      // C1
    inout           ENC_A,      // E1
    inout           ENC_B       // B1
);

// LED: z=off, 0=on

logic led_red, led_green, led_blue;
assign LED_RED = led_red   ? 1'b0 : 1'bz;
assign LED_GRN = led_green ? 1'b0 : 1'bz;
assign LED_BLU = led_blue  ? 1'b0 : 1'bz;

// CFG: 1=open 0=closed (bridged)

(*keep=1*) logic [3:0] cfg;
assign cfg = ~CFG;

//----------------------------------------------------------------

table_if  i_table();
data_if   #(.N_CFG(4)) i_data(.cfg);
status_if i_status();
     
//----------------------------------------------------------------
// Clock/Reset
//----------------------------------------------------------------

logic clk100;
logic clk20;
logic clk2;
logic clk0_3;
logic pll_locked;
logic reset;
logic pre_reset;
logic n_reset;

pll pll_inst(
	.inclk0(CLK),
	.areset(1'b0),
	.c0(clk100),
	.c1(clk2), // 2MHz v6_sync/v3_sync
	.c2(clk20),   // Flash interface
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

logic v6_en
always @(posedge clk100 or posedge pre_reset) begin
    if (pre_reset) begin
        v6_en     <= 1'b0;
        init_ready <= 1'b0;
    end
    else begin
        if (count_s == 3'd1) 
            v6_en <= 1'b1;
        
        if (count_s == 3'd2) 
            init_ready <= 1'b1;
    end
end

assign reset = ~init_ready;

//---------------------------------------------------------------------------------------------------
// Voltage sync
//------------------------------------------------------------------------------                  

always@(*) begin
    case (i_data.v3_sync) 
        v_sync_clocked: V3_SYNC = clk2;
        v_sync_high_z:  V3_SYNC = 1'bz;
        v_sync_fixed_0: V3_SYNC = 1'b0;
        v_sync_fixed_1: V3_SYNC = 1'b1;
    endcase
end    

always@(*) begin
    case (i_data.v6_sync) 
        v_sync_clocked: V6_SYNC = clk2;
        v_sync_high_z:  V6_SYNC = 1'bz;
        v_sync_fixed_0: V6_SYNC = 1'b0;
        v_sync_fixed_1: V6_SYNC = 1'b1;
    endcase
end    

logic v5_pg;
assign V6N_EN = v6_en;
assign V6P_EN = v6_en;
assign v5_pg = V5N_PG & V5P_PG;

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
        .i_table
		);
        
assign i_data.hw_rev = REV;
assign i_data.power_state = {V3_PG, V5N_PG, V5P_PG};

//------------------------------------------------------------------------------
// Encoder/SPI
// The pins are either inputs of an encoder (A/B/0) or used as SPI interface
//------------------------------------------------------------------------------                  

logic USE_SPI;
assign USE_SPI = cfg[0]; // Resistor assembled 

// Tristate Buffer

logic ENC_0_SPI_NCS_din;
logic ENC_0_SPI_NCS_dout;
logic ENC_0_SPI_NCS_oe;

tristate_buffer  tb_ENC_0_SPI_NCS(
		.dout      (ENC_0_SPI_NCS_din ),
		.din       (ENC_0_SPI_NCS_dout),
		.pad_io    (ENC_0             ),
		.oe        (ENC_0_SPI_NCS_oe  )
	);
    
logic ENC_A_SPI_SCLK_din;
logic ENC_A_SPI_SCLK_dout;
logic ENC_A_SPI_SCLK_oe;

tristate_buffer  tb_ENC_A_SPI_SCLK(
		.dout      (ENC_A_SPI_SCLK_din ),
		.din       (ENC_A_SPI_SCLK_dout),
		.pad_io    (ENC_A              ),
		.oe        (ENC_A_SPI_SCLK_oe  )
	);
    
logic ENC_B_SPI_SDIO_din;
logic ENC_B_SPI_SDIO_dout;
logic ENC_B_SPI_SDIO_oe;

tristate_buffer  tb_ENC_B_SPI_SDIO(
		.dout      (ENC_B_SPI_SDIO_din ),
		.din       (ENC_B_SPI_SDIO_dout),
		.pad_io    (ENC_B              ),
		.oe        (ENC_B_SPI_SDIO_oe  )
	);

// Encoder in    
logic enc_a, enc_b, enc_n;
// SPI out
logic spi_send, spi_sclk, spi_ncs, spi_tx;
// SPI in
logic spi_rx;

always@(*) begin
    if (USE_SPI) begin
        ENC_0_SPI_NCS_oe  = 1'b1;
        ENC_A_SPI_SCLK_oe = 1'b1;
        ENC_B_SPI_SDIO_oe = spi_send;
        ENC_0_SPI_NCS_dout = spi_ncs;
        ENC_A_SPI_SCLK_dout = spi_sclk;
        ENC_B_SPI_SDIO_dout = spi_tx;
        spi_rx = ENC_B_SPI_SDIO_din;
        enc_n = 1'b0;
        enc_a = 1'b0;
        enc_b = 1'b0;
    end
    else begin // Encoder, all inputs
        ENC_0_SPI_NCS_oe  = 1'b0;
        ENC_A_SPI_SCLK_oe = 1'b0;
        ENC_B_SPI_SDIO_oe = 1'b0;
        ENC_0_SPI_NCS_dout  = 1'b0;
        ENC_A_SPI_SCLK_dout = 1'b0;
        ENC_B_SPI_SDIO_dout = 1'b0;
        spi_rx = 1'b0;
        enc_n = ENC_0_SPI_NCS_din;
        enc_a = ENC_A_SPI_SCLK_din;
        enc_b = ENC_B_SPI_SDIO_din;
    end
end
    
//------------------------------------------------------    
// Quad Encoder via a/b/0    
// Debounce inputs
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
    .signal_i(enc_n),
    .signal_o(n_in)
    );

quad_encoder quad_enc(
    .clock(clk100),
    .reset,
    .a_in(a_in),
    .b_in(b_in),
    .n_in(n_in),
    .res_err_trigger(i_data.counter_err_read),
    // outputs
    .inc_pulse     ( i_status.inc_pulse    ),
    .dec_pulse     ( i_status.dec_pulse    ),
    .zero_pulse    ( i_status.zero_pulse   ), 
    .counter_ab    ( i_data.counter_ab     ),
    .counter_at_res( i_data.counter_at_res ),
    .counter_err   ( i_data.counter_err    )
    );
// INC signal to A6 -> connect to ENC_0 signal    
assign TRG_N = ~i_status.zero_pulse;                        

//------------------------------------------------------
// PAT9125EL-TKMT via SPI

spi_if i_spi(
    .spi_ncs,
    .spi_sclk,
    .spi_tx,
    .spi_send,
    .spi_rx
    );
    
spi_engine spi_engine(
    .clock(clk100),
    .reset,
    .i_spi
    );
    
spi_encoder spi_enc(
    .clock(clk100),
    .reset,
    .i_spi,
    .i_data
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

