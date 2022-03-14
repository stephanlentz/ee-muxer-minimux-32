/* 
CSR: Control/Status register

*********************************
* Status Register  Address 0x00 *
*********************************

Bit     Field          Default  Description
    
1–0     busy            2'b00   2'b00 0=IDLE
                                2'b01 1=BUSY_ERASE
                                2'b10 2=BUSY_WRITE
                                2'b11 3=BUSY_READ
        
2       rs (read        1'b0    1'b0 Read failed
        success)                1'b1 Read successful
        
3       ws (write       1'b0    1'b0 Write failed
        success)                1'b1 Write successful
        
4       es (erase	    1'b0	1'b0 Erase failed
        success)                1'b1 Erase successful
    
5	    sp1 (Sector protect)    The IP core sets these bits based on the device,
6	    sp2 (Sector protect)    and configuration and access mode settings you specify
7	    sp3 (Sector protect)    during instantiation. 
8	    sp4 (Sector protect)    These settings are fixed. If the IP core sets one of these 
9	    sp5 (Sector protect)    bits, you cannot read or program on the specified sector.

31–10   dummy (padding)	        All of these bits are set to 1.

*********************************
* Control Register Address 0x01 *
*********************************

Bit     Field          Default  Description

19-0    pe (page       All 1's  Sets the page erase address to initiate a page erase operation. 
        erase address)          The IP core only accepts the page erase address when it is in IDLE state. 
                                Otherwise, the page address will be ignored. 
                                The legal value is any available address. 
                                The IP core erases the corresponding page of the given address.        

22–20   se (sector     3'b111   Sets the sector erase address to initiate a sector erase operation. 
        erase address)          The IP core only accepts the sector erase address when it is 
                                in IDLE state. Otherwise, the page address will be ignored.
                                3'b001	        Sector ID 1
                                3'b010	        Sector ID 2
                                3'b011	        Sector ID 3
                                3'b100	        Sector ID 4
                                3'b101	        Sector ID 5
                                Other values	Illegal address 
                                
                                Note: If you set both sector address and page address at the same time, 
                                the sector erase address gets the priority. The IP core accepts and 
                                executes the sector erase address and ignores the page erase address.

23	    wp sector 1     1       The IP core uses these bits to protect the sector from write 
24	    wp sector 2     1       and erase operation. You must clear the corresponding sector 
25	    wp sector 3     1       write protection bit before your program or erase the sector.
26	    wp sector 4     1       1'b0	Disable write protected mode
27	    wp sector 5     1       1'b1	Enable write protected mode
                                The IP core uses these bits to protect the sector 
                                from write and erase operation. 
                                You must clear the corresponding sector write protection 
                                bit before your program or erase the sector.
                                1'b0	Disable write protected mode
                                1'b1	Enable write protected mode

****************
* Data Sectors *
****************

Sector Usage          Access    Byte addresses      32-Bit Addresses (>>2)
                                   19 bit                  17 bit
1	    UFM            R/W      0x00000 - 0x03FFF    0x0000 - 0x0FFF
2	    UFM            R/W      0x04000 - 0x07FFF    0x0800 - 0x03FF
3	    CFM (Image 2)  R/W      0x08000 - 0x1C7FF    0x0400 - 0x0E3F
4	    CFM (Image 2)  R/W      0x1c800 - 0x2AFFF    0x0E40 - 0x157F
5	    CFM (Image 1)  R/W      0x2b000 - 0x4DFFF    0x1580 - 0x26FF 

1 page is 16kbit = 2kByte = 512 kDWORDS
e.g. UFM 0: 
Page 0:                         0x00000 - 0x007FF    0x0000 - 0x01FF
Page 1:                         0x00800 - 0x00FFF    0x0200 - 0x03FF
Page 2:                         0x01000 - 0x017FF    0x0400 - 0x05FF
Page 3:                         0x01800 - 0x01FFF    0x0600 - 0x07FF
Page 4:                         0x02000 - 0x027FF    0x0800 - 0x09FF
Page 5:                         0x02800 - 0x02FFF    0x0A00 - 0x0BFF
Page 6:                         0x03000 - 0x037FF    0x0C00 - 0x0DFF
Page 7:                         0x03800 - 0x03FFF    0x0E00 - 0x0FFF

Register adr in page has 9 bits

In total we can address 156 pages -> pagenr has 8 bit

*/
interface flash_if #(parameter PAGESIZE = 2048, // Byte
                               PAGENR_BITS = 8,
                               REGADR_BITS = 9) ();

    // First buffer page
    logic [PAGENR_BITS-1:0]  page;
    logic                    buffer_page; 
    logic                    sync_page;
    // Then access data in page   
    logic [REGADR_BITS-1:0]  reg_adr;  // of 32 bit register
    logic                    read;
    logic [31:0]             readdata;
    logic                    write;
    logic [31:0]             writedata;
                
    logic                    busy;
    logic                    csr_status;

endinterface

//----------------------------------------------------------------------------------------

module flash_ctrl #(parameter PAGESIZE = 2048, // Byte
                              PAGENR_BITS = 8,
                              REGADR_BITS = 9) (
    input    clk,                 // max 116 MHz
    input    reset,
    flash_if i_flash
    );

// Control/Status register    
logic        csr_address;          //    csr.address
logic        csr_read;             //       .read
logic [31:0] csr_readdata;         //       .readdata        ->
logic        csr_write;            //       .write
logic [31:0] csr_writedata;        //      .writedata
// Date sectors
logic [16:0] data_address;         //   data.address (of 32 bit data)
logic        data_read;            //       .read
logic [31:0] data_readdata;        //       .readdata        ->
logic        data_readdatavalid;   //       .readdatavalid   ->
logic        data_waitrequest;     //       .waitrequest     ->
logic [6:0]  data_burstcount;      //       .burstcount
logic        data_write;           //       .write
logic [31:0] data_writedata;       //       .writedata
/// Dual config control
logic [2:0]  dc_address;           // dual_config.address
logic        dc_read;              //            .read
logic [31:0] dc_writedata;         //            .writedata
logic        dc_write;             //            .write
logic [31:0] dc_readdata;          //            .readdata

// Control register assembly
logic [ 5:1] sector_protect;
logic [19:0] page_erase;
logic [ 2:0] sector_erase;

assign csr_writedata[19: 0] = page_erase    [19:0];
assign csr_writedata[22:20] = sector_erase  [ 2:0];
assign csr_writedata[27:23] = sector_protect[ 5:1];

// Before accessing a flash address read the corresponding page
// After writing you have to sync the page, which erases 
// and rewrites the content
// One page holds 2kByte=512 32 bit register

logic        page_in_UFM;
logic        page_in_CFM;
logic [16:0] flash_page_start_address; // in byte
logic [16:0] flash_page_limit_address; // in byte

assign page_in_UFM   = i_flash.page <  16;
assign page_in_CFM   = i_flash.page >= 16;
assign flash_page_start_address = {i_flash.page, 9'h000};
assign flash_page_limit_address = {i_flash.page, 9'h1FF};

logic [ 6:0]  buf_page;           // Which page is buffered right now
localparam INVALID_PAGE = 7'h7F;  // Reset value
 
logic [31:0]  buf_data;
logic [ 8:0]  buf_rdaddress;
logic [ 8:0]  buf_wraddress;
logic         buf_wren;
logic [31:0]  buf_q;

page_buffer page_buf_dp_512x32(
	.aclr(reset),
	.clock(clk),
	.data(buf_data),
	.wraddress(buf_wraddress),
	.wren(buf_wren),
	.rdaddress(buf_rdaddress),
	.q(buf_q)
    );


logic [4:0] step /* synthesis noprune */;
localparam  s_idle                            = 5'd0,             

            s_read_wait                       = 5'd1,
            s_read_wait_2                     = 5'd2,
            s_read_end                        = 5'd3,

            s_buffer_page_start               = 5'd4,
            s_buffer_wait                     = 5'd5,

            s_unprotect_sector_and_erase_page = 5'd6,
            s_page_erase_check_status         = 5'd7,
            s_page_erase_wait_erase_start     = 5'd8,
            s_page_erase_wait_erase_finished  = 5'd9,
            s_page_erase_finished             = 5'd10,
            
            s_sync_page_start                 = 5'd11,
            s_sync_page_wait_ram              = 5'd12,
            s_sync_page_wait_ram_2            = 5'd13,
            s_sync_page_write                 = 5'd14,
            s_sync_page_wait                  = 5'd15,
            
            s_protect_sector_start            = 5'd16,
            s_protect_sector_end              = 5'd17,
            
            s_verify_page_start               = 5'd18,
            s_verify_page_wait_ram            = 5'd19,
            s_verify_page_wait_ram_2          = 5'd20,
            s_verify_page_read                = 5'd21,
            s_verify_wait                     = 5'd22,

            s_error_erase                     = 5'd23,
            s_error_verify                    = 5'd24;

assign i_flash.busy = i_flash.write 
                    | i_flash.read 
                    | i_flash.buffer_page 
                    | i_flash.sync_page 
                    | (step != s_idle );

always @ (posedge clk or posedge reset) begin
    if ( reset ) begin
        step <= s_idle;
    end
    else begin
        csr_write       <= 1'b0;
        csr_read        <= 1'b0;
        
        data_write      <= 1'b0;
        data_read       <= 1'b0;

        sector_protect  <= '1;
        page_erase      <= '1;
        sector_erase    <= '1;

        data_burstcount <= 7'd1;
        buf_wren        <= 1'b0;
        
        case (step)
            s_idle: begin
                csr_address <= 1'b0;   // status register
                if (i_flash.buffer_page) begin
                    step <= s_buffer_page_start;
                end
                if (i_flash.sync_page) begin
                    step <= s_unprotect_sector_and_erase_page;
                end
                if (i_flash.read) begin
                    buf_rdaddress <= i_flash.reg_adr;
                    step <= s_read_wait;
                end
                if (i_flash.write) begin
                    buf_wraddress <= i_flash.reg_adr;
                    buf_data      <= i_flash.writedata;
                    buf_wren      <= 1'b1;
                    step          <= s_idle;
                 end
            end

            // --------------------------------------------------------------------
            // Wait 2 cycles until RAM output is valid
            
            s_read_wait:   step <= s_read_wait_2;
            s_read_wait_2: step <= s_read_end;
            
            s_read_end: begin
                i_flash.readdata <= buf_q;
                step <= s_idle;
            end

            // --------------------------------------------------------------------

            s_buffer_page_start: begin
                buf_wraddress <= '0;
                data_address  <= flash_page_start_address;
                data_read     <= 1'b1;
                step          <= s_buffer_wait;
            end
            
            s_buffer_wait: begin 
                // Wait for read accepted
                if (data_waitrequest) begin
                    data_read     <= 1'b1;
                    step <= step; // Stay here
                end
                // Wait for data available
                else if (data_readdatavalid) begin
                    // Copy data to buffer
                    buf_data <= data_readdata;
                    buf_wraddress <= data_address[REGADR_BITS-1:0];
                    buf_wren <= 1'b1;
                    // Address next value
                    if ( data_address[REGADR_BITS-1:0] != '1 ) begin
                        data_address  <= data_address + 1'd1;
                        data_read <= 1'b1;
                        step <= step; // Stay here
                    end
                    else begin
                        step <= s_idle;
                    end
                end
            end

            // --------------------------------------------------------------------

            s_unprotect_sector_and_erase_page: begin
                csr_address    <= 1'b1;     // control register
                sector_protect <= 5'b11100; // Unprotect UFM sectors
                page_erase     <= flash_page_start_address; // Erase page of this address
                csr_write      <= 1'b1;
                step           <= s_page_erase_check_status;
            end
            
            s_page_erase_check_status: begin
                csr_address <= 1'b0; // status register
                csr_read    <= 1'd1;
                step        <= s_page_erase_wait_erase_start;
            end
            
            s_page_erase_wait_erase_start: begin
                if (csr_readdata[1:0] == 2'b00) // IDLE
                    step <= step;
                else 
                    step <= s_page_erase_wait_erase_finished;
                csr_read    <= 1'd1;
            end
            
            s_page_erase_wait_erase_finished: begin
                if (csr_readdata[1:0] == 2'b00) // IDLE
                    step <= s_page_erase_finished;
                else 
                    step <= step;
                csr_read <= 1'd1;
            end

            s_page_erase_finished: begin
                if (csr_readdata[4])  // Erase successful
                    step <= s_sync_page_start;
                else
                    step <= s_error_erase;
            end
            
            // --------------------------------------------------------------------
            
            s_sync_page_start: begin
                buf_rdaddress <= '0;
                step          <= s_sync_page_wait_ram;
            end
            
            s_sync_page_wait_ram:   step <= s_sync_page_wait_ram_2;
            s_sync_page_wait_ram_2: step <= s_sync_page_write;
            
            s_sync_page_write: begin
                data_address   <= {i_flash.page, buf_rdaddress};
                data_writedata <= buf_q;
                data_write     <= 1'b1;
                step           <= s_sync_page_wait;
            end
            
            s_sync_page_wait: begin 
                // Wait for write finished (34..305us, typical 102us)
                if (data_waitrequest) begin
                    data_write     <= 1'b1;
                    step <= step; // Stay here
                end
                // data is written
                else begin
                    // Address next value
                    if ( buf_rdaddress != '1 ) begin
                        buf_rdaddress <= buf_rdaddress + 1'd1;
                        step <= s_sync_page_wait_ram;
                    end
                    else begin
                        step <= s_protect_sector_start;
                    end
                end
            end

            // --------------------------------------------------------------------
            
            s_protect_sector_start: begin
                csr_address    <= 1'b1; // control register
                sector_protect <= 5'b11111;
                page_erase     <= '1;
                csr_write      <= 1'b1;
                step           <= s_protect_sector_end;
            end

            s_protect_sector_end: begin
                step           <= s_verify_page_start;
            end

            // --------------------------------------------------------------------

            s_verify_page_start: begin
                buf_rdaddress <= '0;
                step          <= s_verify_page_wait_ram;
            end
            
            s_verify_page_wait_ram:   step <= s_verify_page_wait_ram_2;
            s_verify_page_wait_ram_2: step <= s_verify_page_read;
            
            s_verify_page_read: begin
                data_address <= {i_flash.page, buf_rdaddress};
                data_read    <= 1'b1;
                step         <= s_verify_wait;
            end
            
            s_verify_wait: begin 
                // Wait for read accepted
                if (data_waitrequest) begin
                    data_read     <= 1'b1;
                    step <= step; // Stay here
                end
                // Wait for data available
                else if (data_readdatavalid) begin
                    // Compare data to buffer
                    if (buf_q != data_readdata)
                        step <= s_error_verify;
                    // Address next value
                    else if ( buf_rdaddress != '1 ) begin
                        buf_rdaddress <= buf_rdaddress + 1'd1;
                        step <= s_verify_page_wait_ram;
                    end
                    else begin
                        step <= s_idle;
                    end
                end
            end

            // --------------------------------------------------------------------
            
            s_error_erase: begin
                // TODO: How to signal error?
                step <= s_idle;
            end
            
            s_error_verify: begin
                // TODO: How to signal error?
                step <= s_idle;
            end
            
            // --------------------------------------------------------------------
            default: begin 
                step <= s_idle;
            end
            
        endcase

    end
end

logic copy_csr_status;
always @ (posedge clk or posedge reset) begin
    if ( reset ) begin
        i_flash.csr_status <= '0;
        copy_csr_status <= 1'b0;
    end
    else begin
        copy_csr_status <= (csr_read & (csr_address=='0));
        // Data available in the next cycle after read
        if (copy_csr_status)
            i_flash.csr_status <= csr_readdata;
    end
end    

on_chip_flash on_chip_flash_inst (
    .clock                  ( clk                ),    //  clk.clk
    .reset_n                ( ~reset             ),    //  nreset.reset_n
    
    .avmm_csr_addr          ( csr_address        ),    //  csr.address
    .avmm_csr_read          ( csr_read           ),    //    .read
    .avmm_csr_writedata     ( csr_writedata      ),    //    .writedata
    .avmm_csr_write         ( csr_write          ),    //    .write
    .avmm_csr_readdata      ( csr_readdata       ),    //    .readdata
    .avmm_data_addr         ( data_address       ),    //  data.address
    .avmm_data_read         ( data_read          ),    //    .read
    .avmm_data_writedata    ( data_writedata     ),    //    .writedata
    .avmm_data_write        ( data_write         ),    //    .write
    .avmm_data_readdata     ( data_readdata      ),    //    .readdata
    .avmm_data_waitrequest  ( data_waitrequest   ),    //    .waitrequest
    .avmm_data_readdatavalid( data_readdatavalid ),    //    .readdatavalid
    .avmm_data_burstcount   ( data_burstcount    )     //    .burstcount
);


// TODO: This can operate up to 80 MHz max

dual_config dual_config_inst(
		.clk                (clk),           //    clk.clk
		.nreset             (~reset),        // nreset.reset_n
		.avmm_rcv_address   (dc_address  ),  // avalon.address
		.avmm_rcv_read      (dc_read     ),  //       .read
		.avmm_rcv_writedata (dc_writedata),  //       .writedata
		.avmm_rcv_write     (dc_write    ),  //       .write
		.avmm_rcv_readdata  (dc_readdata )   //       .readdata
);

endmodule: flash_ctrl