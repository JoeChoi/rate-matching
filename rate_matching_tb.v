`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Rurisond, Inc.
// Engineer: J. Choi
// 
// Create Date: 11/14/2019 08:58:18 PM
// Design Name: 
// Module Name: rate_matching_tb
// Project Name: 
// Target Devices: 
// Tool Versions: Xilinx Vivado 2017.4 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module rate_matching_tb();

reg          clk       =  1'b0;
reg          resetn    =  1'b0;
reg  [18:0]  cycle_tb  = 19'd0;
wire         mod32     = cycle_tb == 5'b11111;
reg  [12:0]  bit_cnt   = 13'd0;
reg  [4:0]   mod32_cnt =  5'b0;  
reg  [31:0]  data      = 32'd0;
reg  [31:0]  shifter   = 32'd0;

wire         d0;
wire         d1;
wire         d2;

wire  [2:0]  tdata_in;     //input to rate matching block
reg          tvalid_in;    //input to rate matching block
reg          tlast_in;     //input to rate matching block
wire  [63:0] tuser_in;     //input to rate matching block
wire         eob_in;       //input to rate matching block
wire         s_tready;     //output from rate matching block
  
wire         tdata_out;    //output from rate matching block 
wire         tvalid_out;   //output from rate matching block
wire         tlast_out;    //output from rate matching block
wire         eob_out;      //output from rate matching block
wire         m_tready;     //input to rate matching block

wire  [31:0] v0;  
wire  [31:0] v1;
wire  [31:0] v2; 

wire  [31:0] w0; 
wire  [31:0] w1; 
wire  [31:0] w2; 


////////////////////////////////////////////////////////////////////////////////// 
//clock and reset                                                             
////////////////////////////////////////////////////////////////////////////////// 
initial
   begin
      clk = 1'b0;
      forever
         begin
            clk = 1'b1;
            #(5/2);
            clk = 1'b0;
            #(5/2);
         end
   end
         
initial
   begin
      #20 resetn = 1'b0;
      #41 resetn = 1'b1;
   end
   
   
//test bench 19K  
always @(posedge clk or negedge resetn)
   if (~resetn)
      cycle_tb <= 19'd0;
   else if (s_tready)
      cycle_tb <= cycle_tb + 1'b1;  
   else 
      cycle_tb <= cycle_tb; 

//test bench 16K  
always @(posedge clk or negedge resetn)
      if (~resetn)
         bit_cnt <= 13'd0;
      else if (s_tready && (bit_cnt > 13'd6207))
         bit_cnt <= 14'd0;
      else if (s_tready)
         bit_cnt <= bit_cnt + 1'b1;
      else
         bit_cnt <= bit_cnt;


////////////////////////////////////////////////////////////////////////////////// 
//interface signals                                                                  
//////////////////////////////////////////////////////////////////////////////////  
wire bit_gen;
wire bit_last;
assign bit_gen  = bit_cnt >= (14'd0    ) &&         
                  bit_cnt < (14'd6144 ) && resetn;                  
assign bit_last = bit_cnt == (14'd6143 );      

always @(posedge clk) begin
   tvalid_in <= bit_gen;
   tlast_in  <= bit_last;
   end     


wire [15:0] k    =   16'd6144; 
//wire [15:0] e    = 3*16'd6144;
wire [15:0] e    = 9*16'd6144;
//wire [15:0] e    = 3*16'd6144 + 16'd1536;   //19968
//wire [15:0] e    = 16'd3072;
wire [15:0] ncb  = 3*16'd6144;
wire [15:0] ko   =   16'd16;
assign tuser_in  = {ko, ncb, e, k};

assign eob_in    = 1'b0;
assign m_tready  = 1'b1;    //down stream block ready to accept date 
                            //from rate matching
                              
//////////////////////////////////////////////////////////////////////////////////
//data generator
//////////////////////////////////////////////////////////////////////////////////
always @(posedge clk or negedge resetn)
   if (~resetn)
      mod32_cnt <= 5'd0;
   //else if (bit_cnt == 13'd0)
   //   mod32_cnt <= 5'd0;
   else if (s_tready && bit_gen)
      mod32_cnt <= mod32_cnt + 1'b1;
   else 
      mod32_cnt <= mod32_cnt;
    
wire shifter_load = (mod32_cnt == 5'd0);
      
//      
always @(posedge clk or negedge resetn)
   if (~resetn) 
      //data <= 32'd0; 
      data <= 32'd4294967104;
   else if (bit_cnt == 13'd0)
      data <= 32'd4294967104;
   else if (s_tready && mod32_cnt == 5'd31)
      data <= data + 32'd1;
   else 
      data <= data;

//     
always @(posedge clk or negedge resetn)
   if (~resetn)
      //shifter[31:0] <= 32'd0;
      shifter[31:0] <= 32'd4294967104;
   else if (bit_cnt == 13'd0)
      //shifter[31:0] <= 32'd0;
      shifter[31:0] <= 32'd4294967104;
   else if (s_tready && shifter_load)             //load  
      shifter[31:0] <= data;
   else if (s_tready && bit_gen)
      shifter[31:0] <= {1'b0, shifter[31:1]};     //shift-right
   else
      shifter[31:0] <= shifter[31:0];
                               
//bit stream input to device under test
assign d0        = shifter[0];   
assign d1        = shifter[0];   
assign d2        = shifter[0];   
assign tdata_in  = {d0,d1,d2};  
 
//////////////////////////////////////////////////////////////////////////////////                      
//device under test
//////////////////////////////////////////////////////////////////////////////////
rate_matching rate_matching_inst (
   .clk           (clk),               
   .reset         (resetn),         
   .s_axis_tdata  (tdata_in),              
   .s_axis_tvalid (tvalid_in),              
   .s_axis_tlast  (tlast_in),
   .s_axis_tuser  (tuser_in),              
   .eob_in        (eob_in),              
   .s_axis_tready (s_tready),   //TRUE: RM drive this bit               
                 
   .m_axis_tdata  (tdata_out),              
   .m_axis_tvalid (tvalid_out),              
   .m_axis_tlast  (tlast_out),              
   .eob_out       (eob_out),
   .m_axis_tready (m_tready),    //TRUE: RM receive this bit                  
  
//   .test_out0       (v0), 
//   .test_out1       (v1),
//   .test_out2       (v2)   
   .test_out0       (w0),    
   .test_out1       (w1),    
   .test_out2       (w2)               
);

endmodule
