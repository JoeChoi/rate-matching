`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Rurisond, Inc.
// Engineer: J. Choi
// 
// Create Date: 11/14/2019 12:18:29 PM
// Design Name: 
// Module Name: rate_matching
// Project Name: OFDM_MODEM
// Target Devices: 
// Tool Versions: Vivado 2017.4
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module rate_matching (
input    wire        clk,
input    wire        reset,
input    wire [2:0]  s_axis_tdata,
input    wire        s_axis_tvalid,
input    wire        s_axis_tlast,
input    wire [63:0] s_axis_tuser,
input    wire        eob_in,
output   wire        s_axis_tready,

output   wire        m_axis_tdata,
output   wire        m_axis_tvalid,
output   wire        m_axis_tlast,
output   wire        eob_out,
input    wire        m_axis_tready,

output   wire [31:0] test_out0,                         //simuliation
output   wire [31:0] test_out1,                         //simuliation
output   wire [31:0] test_out2                          //simuliation
);
 
parameter C = 32;

//////////////////////////////////////////////////////////////////////////////////
//wire and register                                                                          
//////////////////////////////////////////////////////////////////////////////////
//
//AXIS stream interface
wire            resetn   = reset;
wire            d0       = s_axis_tdata[2];               //system bits
wire            d1       = s_axis_tdata[1];               //parity1 bits
wire            d2       = s_axis_tdata[0];               //parity2 bits
wire            tvalid   = s_axis_tvalid;
wire            tlast    = s_axis_tlast;
wire   [63:0]   tuser    = s_axis_tuser;
wire            eob      = eob_in;
assign          eob_out  = 1'b0;

wire            m_tvalid;
wire            m_tready = m_axis_tready;

reg    [7:0]    tvalid_delay;   
reg    [7:0]    tlast_delay;    
reg    [7:0]    eob_delay;      

//////////////////////////////////////////////////////////////////////////////////
//control and status 
//////////////////////////////////////////////////////////////////////////////////                                             
wire   [15:0]   code_block_size;       
wire   [15:0]   output_seq_len;
wire   [15:0]   ncb;
wire   [15:0]   k0;

reg    [15:0]   k_m0   ;                    //mem0, coded block size, K 
reg    [15:0]   e_m0   ;                    //mem0, output sequence size, E
reg    [15:0]   ncb_m0 ;                    //mem0, circular buffer szie, Ncb
reg    [15:0]   k0_m0  ;                    //mem0, dummy bit size, K0 
                       
reg    [15:0]   k_m1   ;                    //mem1, coded block size, K       
reg    [15:0]   e_m1   ;                    //mem1, output sequence size, E   
reg    [15:0]   ncb_m1 ;                    //mem1, circular buffer szie, Ncb 
reg    [15:0]   k0_m1  ;                    //mem1, dummy bit size, K0        

reg             ena_wr_int_mem0;
wire            ena_wr_int_mem1;
reg             mem0_full ;
reg             mem1_full ;              
wire            mem0_empty = ~mem0_full;
wire            mem1_empty = ~mem1_full;
wire            ena_int_mem_rd;
reg             ena_int_mem_rd_d1;
reg             ena_int_mem_rd_valid;


reg    [18:0]   cycle_cnt;
wire   [7:0]    R;                                 //no. of row in the matrix 
//reg    [7:0]    int_rd_addr = 8'd0;                   //simulation  

//                                                                   
reg  [15:0]     k;                                 //data bit index    
//wire [12:0]     K_pi = R * C;                    //block size ceiling

reg             ena_bit_selection_system_rd_valid;

//wire            ena_bit_collection;
reg             ena_bit_collection;                        
reg             ena_bit_collection_d1;                    
reg             ena_bit_collection_d2;
reg             ena_bit_collection_d3;      
wire            cir_buf_wren;       
wire            cir_buf_wr_done;    

//
wire            ena_bit_selection_rd;  
reg             ena_bit_selection_rd_d1;                     

reg             ena_bit_selection_system;                     
reg             ena_bit_selection_system_d1;                   
reg             ena_bit_selection_system_d2;                   
reg  [15:0]     kk;                             //data bit index  
                                                           
reg             ena_bit_selection_parity;                     
reg             ena_bit_selection_parity_d1;                   
reg             ena_bit_selection_parity_d2;                   
reg             ena_bit_selection_parity_d3;                   
reg  [15:0]     kkk;                            //data bit index  

reg             ena_bit_selection_parity_1; 

wire            cir_buf_rd_system_done_early;
wire            parity_cir_buf_rd_done;
wire            cir_buf_rd_start_early;
wire            parity_cir_buf_rd_done_; 

//sub-block interleaver 
reg    [31:0]   int_shifter0; 
reg    [31:0]   int_shifter1; 
reg    [31:0]   int_shifter2; 
reg    [4:0]    int_shift_cnt;
reg    [7:0]    int_wr_addr;               
//            
wire   [31:0]   v00;
wire   [31:0]   v10;
wire   [31:0]   v20;
//
wire   [31:0]   v01;
wire   [31:0]   v11;
wire   [31:0]   v21;
//                              
wire   [31:0]   v0;       
wire   [31:0]   v1;       
wire   [31:0]   v2; 
//
wire   [31:0]   w0; 
wire   [31:0]   w1; 
wire   [31:0]   w2; 
                
//assign          test_out = v0;  
//assign          test_out = v1;  
//assign          test_out = v2;  
assign          test_out0 = w0; 
assign          test_out1 = w1;  
assign          test_out2 = w2;                                  
                                   
//bit collection 
wire [7:0]  int_rd_addr;
wire        int_mem_rd_done;
wire [4:0]  permutation_pattern; 
wire [4:0]  col_addr;
reg  [4:0]  col_addr_delay1; 
reg  [4:0]  col_addr_delay2;
                     
reg  [31:0] cir_shifter0;
reg  [31:0] cir_shifter1;
reg  [31:0] cir_shifter2;

reg  [4:0]  cir_shift_cnt;
reg  [7:0]  cir_buf_wr_addr;

reg  [7:0]  system_cir_buf_rd_addr;
reg  [7:0]  parity1_cir_buf_rd_addr;
reg  [7:0]  parity2_cir_buf_rd_addr;

wire [31:0] system_bit;
wire [31:0] parity1_bit;
wire [31:0] parity2_bit;

//bit selection 
reg  [31:0] sel_shifter0;   
reg  [31:0] sel_shifter1;   
reg  [31:0] sel_shifter2; 
  
reg  [4:0]  system_sel_shift_cnt; 
reg  [4:0]  parity1_sel_shift_cnt;
reg  [4:0]  parity2_sel_shift_cnt; 

wire        sel_shift_cnt_mod32;
wire        sel_shift_cnt_mod0;
wire        system_sel_shifter_load ;
wire        parity1_sel_shifter_load;
wire        parity2_sel_shifter_load;

reg         ena_parity;                                             
wire        ena_parity1;  
wire        ena_parity2;  

wire        puncture;
wire        repetition;
wire        end_of_bit_stream;
wire        end_of_bit_stream_early;

//output control
reg  [15:0] out_seq_bit_cnt;

/////////////////////////////////////////////////////////////////////////////////      
//main                                       
//////////////////////////////////////////////////////////////////////////////////      
//                                                                                         
////////////////////////////////////////////////////////////////////////////////// 
//axis input 
////////////////////////////////////////////////////////////////////////////////// 
always @(posedge clk or negedge resetn)
   if (~resetn) begin
      tvalid_delay   <= 8'd0;
      tlast_delay    <= 8'd0;
      eob_delay      <= 8'd0;
      end
   else begin
      tvalid_delay   <= {tvalid_delay[6:0], tvalid};
      tlast_delay    <= {tlast_delay[6:0] , tlast};
      eob_delay      <= {eob_delay[6:0]   , eob};
      end
      
//////////////////////////////////////////////////////////////////////////////////       
//control and status                                         
//////////////////////////////////////////////////////////////////////////////////   
always @(posedge clk or negedge resetn)                                        
   if (~resetn) begin                                                          
      k_m0   <= 16'd0;                                                         
      e_m0   <= 16'd0;                                                         
      ncb_m0 <= 16'd0;                                                         
      k0_m0  <= 16'd0;                                                         
      k_m1   <= 16'd0;                                                         
      e_m1   <= 16'd0;                                                         
      ncb_m1 <= 16'd0;                                                         
      k0_m1  <= 16'd0;                                                         
      end         
   else if (~tvalid_delay[0] & tvalid & ena_wr_int_mem0) begin                 
      k_m0   <= tuser[15:0];                                                   
      e_m0   <= tuser[31:16];                                                  
      ncb_m0 <= tuser[47:32];                                                  
      k0_m0  <= tuser[63:48];                                                  
      end                                                                      
   else if (~tvalid_delay[0] & tvalid & ena_wr_int_mem1) begin                 
      k_m1   <= tuser[15:0];                                                   
      e_m1   <= tuser[31:16];                                                  
      ncb_m1 <= tuser[47:32];                                                  
      k0_m1  <= tuser[63:48];                                                  
      end                                                                      

//   
always @(posedge clk or negedge resetn)
   if (~resetn)
      cycle_cnt <= 13'd0;
   else if (end_of_bit_stream)
      cycle_cnt <= 13'd0;
   else if (cycle_cnt == code_block_size-1) 
      cycle_cnt <= 13'd0;
   else if (tvalid_delay[1])
      cycle_cnt <= cycle_cnt + 13'd1;
   
//
always @(posedge clk or negedge resetn)    
   if (~resetn)                            
      ena_wr_int_mem0 <= 1'b1;      
   else if (tlast_delay[0])                
      ena_wr_int_mem0 <= ~ena_wr_int_mem0;       
                                           
assign ena_wr_int_mem1 = ~ena_wr_int_mem0;   

reg rst0;
always @(posedge clk or negedge resetn)     
   if (~resetn)                             
      rst0 <= 1'b1;              
   //else if (parity_cir_buf_rd_done & ena_parity2) 
   else if (end_of_bit_stream & ena_parity2)                  
      rst0 <= ~rst0;  
                                            
wire rst1 = ~rst0;  

//sub-block interleaver DP RAM write control
always @(posedge clk or negedge resetn)
   if (~resetn) 
      mem0_full <= 1'b0;
   else if (ena_wr_int_mem0 & tlast_delay[0])
      mem0_full <= 1'b1;     
   //else if (rst0 & parity_cir_buf_rd_done) 
   else if (rst0 & end_of_bit_stream)  
      mem0_full <= 1'b0;
     
always @(posedge clk or negedge resetn)                  
   if (~resetn)                                                                    
      mem1_full <= 1'b0;                                                             
   else if (ena_wr_int_mem1 & tlast_delay[0])                        
      mem1_full <= 1'b1;                                   
   //else if (rst1 & parity_cir_buf_rd_done) 
   else if (rst1 & end_of_bit_stream)
      mem1_full <= 1'b0;                                 

assign s_axis_tready  = mem0_empty | mem1_empty;
assign ena_int_mem_rd = mem0_full  | mem1_full;

//  
assign code_block_size = (({16{mem0_full}} & k_m0) | 
                          ({16{mem1_full}} & k_m1)); 
                                    
assign output_seq_len  = (({16{mem0_full}} & e_m0) | 
                          ({16{mem1_full}} & e_m1));
                                     
assign ncb             = (({16{mem0_full}} & ncb_m0) | 
                          ({16{mem1_full}} & ncb_m1));
                                     
assign k0              = (({16{mem0_full}} & k0_m0) | 
                          ({16{mem1_full}} & k0_m1));                                
                               
assign R = ceil_32(code_block_size);  

   
////////////////////////////////////////////////////////////////////////////////// 
//write sub-block interleaver - input d0, d1, d2                                   
////////////////////////////////////////////////////////////////////////////////// 
always @(posedge clk or negedge resetn)
   if (~resetn) begin 
      int_shifter0 <= 32'd0;
      int_shifter1 <= 32'd0;
      int_shifter2 <= 32'd0;     
      end  
   else if (tvalid) begin
      int_shifter0[31:0] <= {d0, int_shifter0[31:1]};
      int_shifter1[31:0] <= {d1, int_shifter1[31:1]};
      int_shifter2[31:0] <= {d2, int_shifter2[31:1]};     
      end
   else if (int_wr_addr > (R-1)) begin
      int_shifter0[31:0] <= 32'd0; 
      int_shifter1[31:0] <= 32'd0; 
      int_shifter2[31:0] <= 32'd0;
      end  
   else begin                                      
      int_shifter0[31:0] <= int_shifter0[31:0];
      int_shifter1[31:0] <= int_shifter1[31:0];
      int_shifter2[31:0] <= int_shifter2[31:0];     
      end   
         
//
always @(posedge clk or negedge resetn)
   if (~resetn)
      int_shift_cnt <= 5'b00000;
   else if (cycle_cnt == code_block_size-1)
      int_shift_cnt <= 5'b00000;
   else if (tvalid_delay[0])   
      int_shift_cnt <= int_shift_cnt + 1'b1;
   else
      int_shift_cnt <= int_shift_cnt;
      
wire int_wren = ((int_shift_cnt == 5'd31) && tvalid_delay[0]);  

//
always @(posedge clk or negedge resetn)
   if (~resetn)
      int_wr_addr <= 8'd0;
   else if (int_shift_cnt == 5'd31 && tvalid_delay[0])
      int_wr_addr <= int_wr_addr + 1'b1;
   else if (int_wr_addr > (R-1))
      int_wr_addr <= 8'd0;
   else
      int_wr_addr <= int_wr_addr;
                                                       
////////////////////////////////////////////////////////////////////////////////// 
//sub-block interleaver memory 0
//////////////////////////////////////////////////////////////////////////////////  
ram_2port system_bit_0 (
  .clka(clk),    
  .ena(ena_wr_int_mem0), 
  .wea(int_wren & ena_wr_int_mem0),     
  .addra(int_wr_addr),   
  .dia(int_shifter0[31:0]),  
  .doa(),
    
  .clkb(clk),    
  .enb(~ena_wr_int_mem0 & ena_int_mem_rd),  
  .web(1'b0),                 
  .addrb(int_rd_addr), 
  .dib(32'd0),          
  .dob(v00)  
);

ram_2port parity1_bit_0 (
  .clka(clk),    
  .ena(ena_wr_int_mem0),
  .wea(int_wren & ena_wr_int_mem0),      
  .addra(int_wr_addr),  
  .dia(int_shifter1[31:0]),
  .doa(),  
    
  .clkb(clk),    
  .enb(~ena_wr_int_mem0 & ena_int_mem_rd), 
  .web(1'b0),      
  .addrb(int_rd_addr), 
  .dib(32'd0),      
  .dob(v10) 
);

ram_2port parity2_bit_0 (
  .clka(clk),    
  .ena(ena_wr_int_mem0), 
  .wea(int_wren & ena_wr_int_mem0),     
  .addra(int_wr_addr),  
  .dia(int_shifter2[31:0]), 
  .doa(),
     
  .clkb(clk),    
  .enb(~ena_wr_int_mem0 & ena_int_mem_rd), 
  .web(1'b0),   
  .addrb(int_rd_addr), 
  .dib(32'd0),      
  .dob(v20)   
);

//////////////////////////////////////////////////////////////////////////////////
//sub-block interleaver memory 1 
//////////////////////////////////////////////////////////////////////////////////
ram_2port system_bit_1 (
  .clka(clk),    
  .ena(ena_wr_int_mem1),      
  .wea(int_wren & ena_wr_int_mem1),     
  .addra(int_wr_addr),  
  .dia(int_shifter0[31:0]), 
  .doa(),
     
  .clkb(clk),    
  .enb(~ena_wr_int_mem1 & ena_int_mem_rd),
  .web(1'b0),   
  .addrb(int_rd_addr), 
  .dib(32'd0),          
  .dob(v01) 
); 

ram_2port parity1_bit_1 (
  .clka(clk),    
  .ena(ena_wr_int_mem1),      
  .wea(int_wren & ena_wr_int_mem1),      
  .addra(int_wr_addr),  
  .dia(int_shifter1[31:0]), 
  .doa(),
     
  .clkb(clk),    
  .enb(~ena_wr_int_mem1 & ena_int_mem_rd),
  .web(1'b0),    
  .addrb(int_rd_addr),  
  .dib(32'd0),         
  .dob(v11) 
);  

ram_2port parity2_bit_1 (
  .clka(clk),    
  .ena(ena_wr_int_mem1),      
  .wea(int_wren & ena_wr_int_mem1),      
  .addra(int_wr_addr),  
  .dia(int_shifter2[31:0]),
  .doa(), 
     
  .clkb(clk),    
  .enb(~ena_wr_int_mem1 & ena_int_mem_rd),
  .web(1'b0),    
  .addrb(int_rd_addr), 
  .dib(32'd0),           
  .dob(v21) 
);     
 
//                                                         
assign v0 = v00 & {32{~ena_wr_int_mem0}} | v01 & {32{~ena_wr_int_mem1}};     
assign v1 = v10 & {32{~ena_wr_int_mem0}} | v11 & {32{~ena_wr_int_mem1}};     
assign v2 = v20 & {32{~ena_wr_int_mem0}} | v21 & {32{~ena_wr_int_mem1}};     
 
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////       
//sub-block interleaver to bit collection
////////////////////////////////////////////////////////////////////////////////// 
 
//////////////////////////////////////////////////////////////////////////////////
//read sub-block interleaver buffer
//inter-column permutation
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////       
//bit collection
////////////////////////////////////////////////////////////////////////////////// 
always @(posedge clk or negedge resetn)
   if (~resetn)
      ena_int_mem_rd_d1 <= 1'b0;
   else    
      ena_int_mem_rd_d1 <= ena_int_mem_rd;

//sub-block interleave memory ready to read
//sub-block interleave memory read after output bit sequence transmitted                                                                                          
wire ena_int_mem_rd_start = (~ena_int_mem_rd_d1 & ena_int_mem_rd) | end_of_bit_stream;
                                   
always @(posedge clk or negedge resetn)         
   if (~resetn)                                 
      ena_bit_collection <= 1'b0;             
   else if (ena_int_mem_rd_start)               
      ena_bit_collection <= 1'b1;             
   else if (k > code_block_size-1)              
      ena_bit_collection <= 1'b0;   
                                                                 
always @(posedge clk or negedge resetn)                       
   if (~resetn) begin                                         
      ena_bit_collection_d1 <= 1'b0;                          
      ena_bit_collection_d2 <= 1'b0;  
      ena_bit_collection_d3 <= 1'b0;                         
      end                                                     
   else begin                                                 
      ena_bit_collection_d1 <= ena_bit_collection;            
      ena_bit_collection_d2 <= ena_bit_collection_d1; 
      ena_bit_collection_d3 <= ena_bit_collection_d2;          
      end                                                     
              
//generate running index for data bit - k = 0, 1, 2, ..., 6143.          
always @(posedge clk or negedge resetn)                                  
   if (~resetn)                                                          
      k <= 16'd0;  
   else if (end_of_bit_stream)        
      k <= 16'd0;                                                      
   else if (ena_bit_collection_d1 && m_tready)                            
      k <= k + 16'd1;                                                    
   else if (~ena_bit_collection_d1)                                      
      k <= 16'd0;                                                        
   else                                                                  
      k <= k;                                                            
                                                                         
assign int_mem_rd_done = (k == code_block_size) && ena_bit_collection_d1; 
                                                                                                                                                                                         
                      
//compute sub-block interleaver read address                                                                                                   
assign int_rd_addr = n_mod_m(k, R); 
                                                                               
//compute read column address - col_addr = 0, 1, 2, ..., 31.                                                                                     
assign permutation_pattern = (int_rd_addr == R) ? (k/R + 5'd1) : (k/R); 

//inter-column permutation                                                                                                                                                                            
//assign col_addr = column(permutation_pattern);                             
assign col_addr = (int_rd_addr == R) ? (k/R + 1'b1) : (k/R);   //simulation  

//
always @(posedge clk or negedge resetn)               
   if (~resetn) begin                                   
      col_addr_delay1 <= 5'd0;                    
      col_addr_delay2 <= 5'd0;                    
      end                                               
   else begin                                           
      col_addr_delay1 <= col_addr;      
      col_addr_delay2 <= col_addr_delay1;   
      end                                                               
//////////////////////////////////////////////////////////////////////////////////
//write circular buffer - input v0, v1, v2
//////////////////////////////////////////////////////////////////////////////////
always @(posedge clk or negedge resetn)
   if (~resetn) begin 
     cir_shifter0 <= 32'd0; 
     cir_shifter1 <= 32'd0; 
     cir_shifter2 <= 32'd0;
     end  
   else if (end_of_bit_stream) begin         
     cir_shifter0 <= 32'd0;    
     cir_shifter1 <= 32'd0;    
     cir_shifter2 <= 32'd0;    
     end                                
   else if (ena_bit_collection_d1 && m_tready) begin  
     cir_shifter0[31:0] <= {v0[col_addr_delay2], cir_shifter0[31:1]}; //v0 - read column 
     cir_shifter1[31:0] <= {v1[col_addr_delay2], cir_shifter1[31:1]}; //v1 - read column 
     cir_shifter2[31:0] <= {v2[col_addr_delay2], cir_shifter2[31:1]}; //v2 - read column 
     end
   else if (k > code_block_size) begin
     cir_shifter0[31:0] <= 32'd0;
     cir_shifter1[31:0] <= 32'd0;
     cir_shifter2[31:0] <= 32'd0;
     end 
     
//
always @(posedge clk or negedge resetn)
   if (~resetn)
      cir_shift_cnt <= 5'd0;    
   else if (end_of_bit_stream)            
      cir_shift_cnt <= 5'd0; 
   else if (ena_bit_collection_d3 && m_tready)  
      cir_shift_cnt <= cir_shift_cnt + 1'b1;
      
assign cir_buf_wren = (cir_shift_cnt == 5'd31 && ena_bit_collection_d3); 
 
//
always @(posedge clk or negedge resetn)
   if (~resetn)
      cir_buf_wr_addr <= 8'd0;     
   else if (end_of_bit_stream)           
      cir_buf_wr_addr <= 8'd0; 
   else if (cir_shift_cnt == 5'd31 & ena_bit_collection_d2 && m_tready)   
      cir_buf_wr_addr <= cir_buf_wr_addr + 1'b1;
   else if (cir_buf_wr_addr > (R-1))
      cir_buf_wr_addr <= 8'd0;
       
assign cir_buf_wr_done = (cir_buf_wr_addr == (R-1)) && cir_buf_wren; 
 
////////////////////////////////////////////////////////////////////////////////// 
//circular_buffer memory 
//////////////////////////////////////////////////////////////////////////////////  
ram_2port circ_system_bit_inst (
  .clka(clk),    
  .ena(ena_bit_collection |
       ena_bit_collection_d1),    
  .wea(cir_buf_wren),    
  .addra(cir_buf_wr_addr), 
  .dia(cir_shifter0[31:0]),
  .doa(),  
    
  .clkb(clk),     
  .enb(ena_bit_selection_system),   
  .web(1'b0),
  .addrb(system_cir_buf_rd_addr), 
  .dib(32'd0),
  .dob(system_bit)  
);

ram_2port circ_parity1_bit_inst (
  .clka(clk),    
  .ena(ena_bit_collection |
       ena_bit_collection_d1),    
  .wea(cir_buf_wren),     
  .addra(cir_buf_wr_addr), 
  .dia(cir_shifter1[31:0]), 
  .doa(),  
  
  .clkb(clk),    
  .enb(ena_bit_selection_parity), 
  .web(1'b0),     
  .addrb(parity1_cir_buf_rd_addr), 
  .dib(32'd0),
  .dob(parity1_bit)  
);

ram_2port circ_parity2_bit_inst (
  .clka(clk),    
  .ena(ena_bit_collection |
       ena_bit_collection_d1),    
  .wea(cir_buf_wren),     
  .addra(cir_buf_wr_addr), 
  .dia(cir_shifter1[31:0]), 
  .doa(),  
  
  .clkb(clk),     
  .enb(ena_bit_selection_parity), 
  .web(1'b0),  
  .addrb(parity2_cir_buf_rd_addr),
  .dib(32'd0), 
  .dob(parity2_bit)  
);

//////////////////////////////////////////////////////////////////////////////////       
//bit selection
//////////////////////////////////////////////////////////////////////////////////    

//////////////////////////////////////////////////////////////////////////////////    
//read system bit                                                                                                     
//////////////////////////////////////////////////////////////////////////////////                                                                                                                                                                                                                                                                                                                          
always @(posedge clk or negedge resetn)                                           
   if (~resetn)                                                                   
      ena_bit_selection_system <= 1'b0; 
   else if (end_of_bit_stream)
      ena_bit_selection_system <= 1'b0;                                                                       
   else if (cir_buf_wr_done |                                //need attention 010920
      (cir_buf_rd_start_early & ~end_of_bit_stream_early))   //early start                            
      ena_bit_selection_system <= 1'b1;                  
   else if (kk == code_block_size-1)                                              
      ena_bit_selection_system <= 1'b0;                                                 
                                                                                  
//generate running index for data bit - k = 0, 1, 2, ..., 6143.                   
always @(posedge clk or negedge resetn)                                           
   if (~resetn)                                                                   
      kk <= 16'd0;    
   else if (end_of_bit_stream)                        //maybe puncture  010920 
      kk <= 16'd0;                                                      
   else if (ena_bit_selection_system_d2 && m_tready)  //start when data loads in shifter                                  
      kk <= kk + 16'd1;                                                             
   else if (~ena_bit_selection_system)                                                  
      kk <= 16'd0;                                                                 
                                                                                                                                                                                                                                                                                                            
//                                                                                                                
always @(posedge clk or negedge resetn)                            
   if (~resetn) begin                                              
      ena_bit_selection_system_d1 <= 1'b0;                         
      ena_bit_selection_system_d2 <= 1'b0;                             
      end  
   else if (end_of_bit_stream) begin
      ena_bit_selection_system_d1 <= 1'b0; 
      ena_bit_selection_system_d2 <= 1'b0; 
      end                                                     
   else begin                                                      
      ena_bit_selection_system_d1 <= ena_bit_selection_system;     
      ena_bit_selection_system_d2 <= ena_bit_selection_system_d1; 
      end                                                          

//
always @(posedge clk or negedge resetn)                                                                
   if (~resetn)                                                                                        
      system_sel_shift_cnt <= 5'd0;  
   else if (cir_buf_rd_start_early | end_of_bit_stream)   //010920              
      system_sel_shift_cnt <= 5'd0;
   else if (ena_bit_selection_system_d1 && m_tready)                                                                                                                                                                                                                                                   
      system_sel_shift_cnt <= system_sel_shift_cnt + 5'd1;                                                                                                                                                                                                                                                      
  
assign system_sel_shifter_load = system_sel_shift_cnt == 5'd0;                                  

//                                                         
always @(posedge clk or negedge resetn)                         
   if (~resetn)                                                 
      system_cir_buf_rd_addr <= 8'd0; 
   else if (cir_buf_rd_start_early | end_of_bit_stream)   //010920              
      system_cir_buf_rd_addr <= 8'd0;                                          
   else if (system_sel_shift_cnt == 5'd31 && ena_bit_selection_system && m_tready)    
      system_cir_buf_rd_addr <= system_cir_buf_rd_addr + 8'd1;                
   else if (system_cir_buf_rd_addr > (R-1))                             
      system_cir_buf_rd_addr <= 8'd0;                                                                                                                                                                                                                                                                                       
                                             
//                                                                                
always @(posedge clk or negedge resetn)                                         
   if (~resetn)                                                           
      sel_shifter0[31:0] <= 32'd0;                                                                                                                                 
   else if (cir_buf_rd_start_early | end_of_bit_stream)     //010920              
      sel_shifter0[31:0] <= 32'd0;                                                                    
   else if (system_sel_shifter_load && m_tready)            //load                                      
      sel_shifter0[31:0] <= system_bit;                                                                                                                                                                                                                       
   else if (ena_bit_selection_system_d1 && m_tready)                                             
      sel_shifter0[31:0] <= {1'b0, sel_shifter0[31:1]};     //shift-right                                                                                     
                                                                                                                                                                                                                                       
//
wire cir_buf_rd_system_done = (system_cir_buf_rd_addr > (R-1)) && 
                               system_sel_shifter_load;
                               
//
assign cir_buf_rd_system_done_early = kk == (code_block_size-3);   
                              
///////////////////////////////////////////////////////////////////////////////// 
//read parity bit                                                                 
//////////////////////////////////////////////////////////////////////////////////  
//contiguous with last system bit
//early start 
always @(posedge clk or negedge resetn)                                           
   if (~resetn)                                                                   
      ena_bit_selection_parity <= 1'b0;     
   else if (end_of_bit_stream)           
      ena_bit_selection_parity <= 1'b0;                                              
   else if (cir_buf_rd_system_done_early)   //early start                                             
      ena_bit_selection_parity <= 1'b1;                                                 
   else if (kkk > 2*code_block_size)                                                
      ena_bit_selection_parity <= 1'b0; 
      
//enable kkk counter
//generate running index for data bit - kk = 0, 1, 2, ..., 6143.   
always @(posedge clk or negedge resetn)                            
   if (~resetn)                                                    
      kkk <= 16'd0;      
   else if (end_of_bit_stream)           
      kkk <= 16'd0;                                              
   else if (ena_bit_selection_parity && m_tready)                              
      kkk <= kkk + 16'd1;                                          
   else if (kkk > 2*code_block_size)                                 
      kkk <= 16'd0;                                                
                                                                                                                        
//                                                                                            
always @(posedge clk or negedge resetn)                                                       
   if (~resetn) begin                                                                         
      ena_bit_selection_parity_d1 <= 1'b0;                                                    
      ena_bit_selection_parity_d2 <= 1'b0; 
      ena_bit_selection_parity_d3 <= 1'b0;                                                   
      end
   else if (end_of_bit_stream) begin
      ena_bit_selection_parity_d1 <= 1'b0;
      ena_bit_selection_parity_d2 <= 1'b0;
      ena_bit_selection_parity_d3 <= 1'b0;
      end                                                                                                                    
   else begin                                                                                 
      ena_bit_selection_parity_d1 <= ena_bit_selection_parity;                                
      ena_bit_selection_parity_d2 <= ena_bit_selection_parity_d1; 
      ena_bit_selection_parity_d3 <= ena_bit_selection_parity_d2;                            
      end                                                                                     

//parity1 bit and parity2 bit control
always @(posedge clk or negedge resetn)
   if (~resetn)
      ena_parity <= 1'b0;    
   else if (end_of_bit_stream)            
      ena_parity <= 1'b0;  
   else if (ena_bit_selection_parity)
      ena_parity <= ~ena_parity;
      
assign ena_parity1 = ena_bit_selection_parity & ~ena_parity;
assign ena_parity2 = ena_bit_selection_parity &  ena_parity;
                                               
////////////////////////////////////////////////////////////////////////////////// 
//parity1  
//////////////////////////////////////////////////////////////////////////////////                                                                                                    
always @(posedge clk or negedge resetn)                                                               
   if (~resetn)                                                                                       
      parity1_sel_shift_cnt <= 5'd0;  
   else if (parity_cir_buf_rd_done | end_of_bit_stream)           
      parity1_sel_shift_cnt <= 5'd0;                                                                  
   else if ((ena_bit_selection_parity & ena_bit_selection_parity_d1) &&   
            ~ena_parity && m_tready)   //not leading edge detect
                                       //define enable signal duration                                               
      parity1_sel_shift_cnt <= parity1_sel_shift_cnt + 5'd1;
                                                                                                                                                     
assign parity1_sel_shifter_load =  parity1_sel_shift_cnt == 5'd0 & 
                                  ena_parity;                  
 
//                                                   
always @(posedge clk or negedge resetn)                         
   if (~resetn)                                                 
      parity1_cir_buf_rd_addr <= 8'd0;   
   else if (parity_cir_buf_rd_done | end_of_bit_stream)           
      parity1_cir_buf_rd_addr <= 8'd0;                                        
   else if (parity1_sel_shift_cnt == 5'd31 && ena_bit_selection_parity && 
            ena_parity && m_tready)        
      parity1_cir_buf_rd_addr <= parity1_cir_buf_rd_addr + 8'd1;                
   else if (parity1_cir_buf_rd_addr > (R-1))                            
      parity1_cir_buf_rd_addr <= 8'd0;   
                                                                                                                   
//                                                                                 
always @(posedge clk or negedge resetn)                                            
   if (~resetn)                                                             
      sel_shifter1[31:0] <= 32'd0;                                                     
   else if (parity_cir_buf_rd_done | end_of_bit_stream)           
      sel_shifter1[31:0] <= 32'd0;                                                                        
   else if (parity1_sel_shifter_load && ena_parity && m_tready)      //load                 
      sel_shifter1[31:0] <= parity1_bit;                                                                                                                    
   else if (ena_bit_selection_parity_d1 && ena_parity && m_tready)   //shift-right                                      
      sel_shifter1[31:0] <= {1'b0, sel_shifter1[31:1]};                                                                                             
                               
//////////////////////////////////////////////////////////////////////////////////                                                                                                                              
//parity2
//////////////////////////////////////////////////////////////////////////////////                                                                                                  
always @(posedge clk or negedge resetn)                                                               
   if (~resetn)                                                                                       
      parity2_sel_shift_cnt <= 5'd0;     
   else if (parity_cir_buf_rd_done | end_of_bit_stream)           
      parity2_sel_shift_cnt <= 5'd0;                                                              
   else if ((ena_bit_selection_parity & ena_bit_selection_parity_d1) &&   
            ~ena_parity && m_tready)   //not leading edge detect
                                       //define enable signal duration                                                
      parity2_sel_shift_cnt <= parity2_sel_shift_cnt + 5'd1;                                          

assign parity2_sel_shifter_load = parity2_sel_shift_cnt == 5'd0 & 
                                  ena_parity;                 
                             
//
always @(posedge clk or negedge resetn)                         
   if (~resetn)                                                 
      parity2_cir_buf_rd_addr <= 8'd0;
   else if (parity_cir_buf_rd_done | end_of_bit_stream)  
      parity2_cir_buf_rd_addr <= 8'd0;                                          
   else if (parity2_sel_shift_cnt == 5'd31 && ena_bit_selection_parity && 
            ena_parity && m_tready)     
      parity2_cir_buf_rd_addr <= parity2_cir_buf_rd_addr + 1'b1;                 
                                                                          
//                                                                              
always @(posedge clk or negedge resetn)                                         
   if (~resetn)                                                           
      sel_shifter2[31:0] <= 32'd0;                                              
   else if (parity_cir_buf_rd_done | end_of_bit_stream)       
      sel_shifter2[31:0] <= 32'd0;                                                                      
   else if (parity2_sel_shifter_load && ena_parity && m_tready)   //load             
      sel_shifter2[31:0] <= parity2_bit;                                                                                                              
   else if (ena_bit_selection_parity_d1 && ena_parity)            //shift-right                                  
      sel_shifter2[31:0] <= {1'b0, sel_shifter2[31:1]};                                                                                  
                           

assign parity_cir_buf_rd_done = (kkk == 2*code_block_size+1);                                                                          


////////////////////////////////////////////////////////////////////////////////// 
//output control                                                                   
//////////////////////////////////////////////////////////////////////////////////                                                                                                                                                                                                                                                                    
//m_axis outputs
wire e0_system  = sel_shifter0[0];
wire e0_parity1 = sel_shifter1[0];
wire e0_parity2 = sel_shifter2[0];

//splicing system bit with parity bit, interlacing parity1 bit and parity2 bit    
wire e0 = (e0_system  & (ena_bit_selection_system & ena_bit_selection_system_d2)) |
          (e0_parity1 & ena_parity1)                                              |
          (e0_parity2 & ena_parity2);
      
assign m_tvalid      = ena_bit_selection_system_d2 |          
                       ena_bit_selection_parity;
 
//                       
assign m_axis_tvalid = m_tvalid;
assign m_axis_tdata  = e0;
                                             
//                                                              
always @(posedge clk or negedge resetn)                                       
   if (~resetn)                                                               
      out_seq_bit_cnt <= 16'd0;                                                                                                  
   else if (m_tvalid && m_tready)                                             
      out_seq_bit_cnt <= out_seq_bit_cnt + 16'd1;  
   else if (out_seq_bit_cnt >= output_seq_len-1)
      out_seq_bit_cnt <= 16'd0;                           
    
//repetition read early start from system bit
assign cir_buf_rd_start_early = (kkk == 2*code_block_size-1);
                                                    
//use it to release memory buffer, terminate process, and reset registers                                                   
assign end_of_bit_stream       = (out_seq_bit_cnt == output_seq_len-1); 
assign end_of_bit_stream_early = (out_seq_bit_cnt == output_seq_len-3);

//m_axis_tlast
assign m_axis_tlast = end_of_bit_stream;          

//puncture
assign puncture = out_seq_bit_cnt > output_seq_len-1;                              
                  
                   
//////////////////////////////////////////////////////////////////////////////////    
//simulation - bit marker                                                             
//////////////////////////////////////////////////////////////////////////////////    
//                                                                                    
reg  [15:0]  bit_marker_system;                                                        
reg  [15:0]  bit_marker_parity1;                                                       
reg  [15:0]  bit_marker_parity2;                                                       
                                                                                      
//                                                                                    
always @(posedge clk or negedge resetn)                                               
   if (~resetn) begin                                                                 
      bit_marker_system <= 16'd0;                                                      
      end                                                                             
   else if (ena_bit_selection_system_d2)                                              
      bit_marker_system <= bit_marker_system + 1'b1;                                  
   else if (bit_marker_system == code_block_size-1)                                                                              
      bit_marker_system <= 16'd0;                                         
                                                                                      
//                                                                                    
always @(posedge clk or negedge resetn)                                               
   if (~resetn) begin                                                                 
      bit_marker_parity1 <= 16'd0;                                                     
      end                                                                             
   else if (ena_bit_selection_parity_d2)                                              
      bit_marker_parity1 <= bit_marker_parity1 + 1'b1;                                
   else if (bit_marker_parity1 == 2*code_block_size-1)                                                                                
      bit_marker_parity1 <= 16'd0;                                       
                                                                                      
//                                                                                    
always @(posedge clk or negedge resetn)                                               
   if (~resetn) begin                                                                 
      bit_marker_parity2 <= 16'd0;                                                     
      end                                                                             
   else if (ena_bit_selection_parity_d2)                                              
      bit_marker_parity2 <= bit_marker_parity2 + 1'b1;                                
   else if (bit_marker_parity2 == 2*code_block_size-1)                                       
      bit_marker_parity2 <= 16'd0;                                                           



//////////////////////////////////////////////////////////////////////////////////
//simulation - observation 
//////////////////////////////////////////////////////////////////////////////////                                                           
wire e0_system_valid = ena_bit_selection_system & ena_bit_selection_system_d2;     
wire e0_system_out   = e0_system;                                                  
wire e0_parity1_out  = e0_parity1 & ena_parity1;                                   
wire e0_parity2_out  = e0_parity2 & ena_parity2;                                                                                                                                                                                                                                      
                                                                                                                                                                                                                                                                                                                 
//////////////////////////////////////////////////////////////////////////////////       
//functions
//////////////////////////////////////////////////////////////////////////////////     

//////////////////////////////////////////////////////////////////////////////////   
//function ceil_32 - the ceiling of n with modulo 32        
//f(n) = ceil_32(n)                         
//////////////////////////////////////////////////////////////////////////////////   
function automatic [15:0] ceil_32;                                                   
input [15:0] n;                                                                      
begin                                                                                
   ceil_32 = (n[4:0] == 5'd0) ? { {5{1'b0}}, n[15:5] } :                             
                                { {5{1'b0}},(n[15:5] + 1'b1) };                       
end                                                                                  
endfunction                                                                          

//////////////////////////////////////////////////////////////////////////////////  
//function n_mod_m - n modulo m  m <= n < (2*m - 1)  
//f(n,m) = n_mod_m(n,m)                    
//////////////////////////////////////////////////////////////////////////////////  
function automatic [15:0] n_mod_m;
input [15:0] n;
input [15:0] m;
begin
//   n_mod_m = ( (n >= m) && (n < 2*m) ) ? (n - m) : n ;
   n_mod_m = n - (n/m) * m;  
end
endfunction

//////////////////////////////////////////////////////////////////////////////////  
//function inter-column permutation pattern (LUT) 
//f(inter_column_permutation_pattern) = column(inter_column_permutation_pattern)                    
//////////////////////////////////////////////////////////////////////////////////  
function automatic [4:0] column;
input [4:0] inter_column_permutation_pattern;
 
    case (inter_column_permutation_pattern)   //LUT
       5'd0 : column = 5'd0 ;
       5'd1 : column = 5'd16;
       5'd2 : column = 5'd8 ;      
       5'd3 : column = 5'd24;      
       5'd4 : column = 5'd4;        
       5'd5 : column = 5'd20;        
       5'd6 : column = 5'd12;        
       5'd7 : column = 5'd28;        
       5'd8 : column = 5'd2 ;     
       5'd9 : column = 5'd18;     
       5'd10: column = 5'd10;     
       5'd11: column = 5'd26;           
       5'd12: column = 5'd6 ;       
       5'd13: column = 5'd22;       
       5'd14: column = 5'd14;       
       5'd15: column = 5'd30;  
       5'd16: column = 5'd1 ;       
       5'd17: column = 5'd17;       
       5'd18: column = 5'd9 ;       
       5'd19: column = 5'd25;       
       5'd20: column = 5'd5 ;       
       5'd21: column = 5'd21;       
       5'd22: column = 5'd13;       
       5'd23: column = 5'd29;       
       5'd24: column = 5'd3 ;       
       5'd25: column = 5'd19;       
       5'd26: column = 5'd11;       
       5'd27: column = 5'd27;       
       5'd28: column = 5'd7 ;       
       5'd29: column = 5'd23;       
       5'd30: column = 5'd15;       
       5'd31: column = 5'd31; 
    endcase  
 endfunction
                                                                                         
endmodule
