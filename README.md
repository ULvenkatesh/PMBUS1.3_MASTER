PMBUS1.3_MASTER
===============

Verilog implemetation of AVS_BUS MASTER
//Name: Venkatesh karra
//Organization:University of Limerick
//Supervisor: Dr.Karl Rinne
//MASTER THESIS PM BUS SPECIFICATION 1.3
//AVS MASTER 
//REFERENCES:[1]PM.BUS 1.3 spcification
//				 [2] Advanced digital design by michale D cileleti
				 

//////////////////////////////////////////////////////////////////////////////////
module AVS_mater_statemachinesrln#(
  // the following parameters can be overridden at module instantiation (see p. 180 of [1])
  parameter		NOC_HIGH=5,		// number of clock cycles during clk high period
  parameter		NOC_LOW=5
)(
/////////////////////////////////////////////////////////////////////////////////
//INPUTS AND OUTPUTS//
/////////////////////////////////////////////////////////////////////////////////
	
    input clk,
	 input rst,
	 input async_in_slave,//data that comes from the slave
	 input [10:0] avs_cmddata,
	 output reg avs_clk,
	 output reg avs_mdata ,
	 output reg avs_sdata_sample_display,
    output reg ledperfect,//glows when the computed crc for the recieved bits is zero
	 output reg lederror  , //glows when the computed crc for the recieved bits is not zero
	 
	 output wire avs_sdata_sample_async,
	 output wire avs_sdata_sample_sync
	 
    );

///////////////////////////////////////////////////////////////////////////////////////
syncroniser N4(.sync_out(avs_sdata_sample_sync),.async_in(avs_sdata_sample_async),.clk(clk),.rst(rst));	

///////////////////////////////////////////////////////////////////////////////////////				
//PARAMETER USED FOR 1MILLI SECOND TRIGGERING
//////////////////////////////////////////////////////////////////////////////////////
parameter		COUNT_1M  = 49999;
////////////////////////////////////////////////////////////////////////////////////////////////////////
//generation of 1ms triggeR//
//////////////////////////////////////////////////////////////////////////////////////////
wire event_1m;							//I MILLI SEC INPUT TRIGGER
reg [16:0] count_1m;
//////////////////////////////////////////////////////////////////////////////////////////
always@(posedge clk)
begin
if(rst == 1)
begin
count_1m <= 0;
end
else
if(count_1m == COUNT_1M)
begin
count_1m <= 0;
end
else
count_1m <= count_1m + 1;
end
assign event_1m=(count_1m==0);


//////////////////////////////////////////////////////////////////////////////////////
//INTERNAL REGISTERS
//////////////////////////////////////////////////////////////////////////////////////
reg [28:0] transmit_reg;				
wire [28:0] transmit_subframe;
reg load_transmit_reg;
reg shift_transmit_reg;
reg check;
wire [15:0] avs_cmd_data_master;
assign transmit_subframe = {2'b01,2'b00,1'b0,8'b0,5'b0,avs_cmddata};
assign avs_cmd_data_master = {5'b0,avs_cmddata};
//////////////////////////////////////////////////////////////////////////////////////////


//****************************************************************************************
//transmit_reg//
//****************************************************************************************
always @(posedge clk) begin
  if (rst) transmit_reg<=0;					// synchronous reset, according to [2]
  else if (load_transmit_reg)transmit_reg<=transmit_subframe;
  else if (shift_transmit_reg)begin transmit_reg<={transmit_reg[27:0],1'b0};end 	// shift left
end
assign transmit_reg_dout=transmit_reg[28];				// pointing at current bit to be applied to AVS_MDATA
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ************************************************************************************************
// management of outputs avs_clk and avs_mdata
// ************************************************************************************************
reg				avs_clk_clr;
reg				avs_clk_set;
reg				avs_mdata_clr;
reg				avs_mdata_set;
reg				avs_mdata_clr_recieve;
reg				avs_mdata_set_recieve;
always @(posedge clk) begin
  if (rst) avs_clk<=0;
  else if (avs_clk_set) avs_clk<=1;
  else if (avs_clk_clr) avs_clk<=0;
end
always @(posedge clk) begin
  if (rst) avs_mdata<=1;
  else if (avs_mdata_set) avs_mdata<=1;
  else if (avs_mdata_clr) avs_mdata<=0;
 end
 
always @(posedge clk) begin
	if(rst) avs_sdata_sample_display<=1;
  else if (avs_mdata_set_recieve) avs_sdata_sample_display<=1;
  else if (avs_mdata_clr_recieve) avs_sdata_sample_display<=0;	
end
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ************************************************************************************************
// management of counter_bits (bit counter)
// ************************************************************************************************
reg [5:0]			counter_bits;
reg [5:0]			counter_bits_value;
reg				counter_bits_load;
reg				counter_bits_dec;	// dec command
wire				counter_bits_z;		// flag that counter_bits has reached a value of zero
always @(posedge clk) begin
  if (rst) counter_bits<=0;
  else if (counter_bits_load) counter_bits<=counter_bits_value;
  else if (counter_bits_dec && (~counter_bits_z)) counter_bits<=counter_bits-1;
end
assign counter_bits_z=(counter_bits==0); 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ************************************************************************************************
// management of counter_clks (clk cycle counter)
// ************************************************************************************************
reg [3:0]			counter_clks;
reg [3:0]			counter_clks_value;
reg				counter_clks_load;
wire				counter_clks_z;		// flag that counter_clks has reached a value of zero
always @(posedge clk) begin
  if (rst) counter_clks<=0;
  else if (counter_clks_load) counter_clks<=counter_clks_value;
  else if (~counter_clks_z) counter_clks<=counter_clks-1;	// if counter is not already zero, decrement it
end
assign counter_clks_z=(counter_clks==0); 	
//***************************************************************************
//data revieve from the slave
//***************************************************************************
reg [31:0] rx; //stores 32 bits that comes from the slave
reg recieve;	//used as the control signal every assertion shifts in data

always@(posedge clk )
	if(rst == 1) begin
		rx<=0;
		end

	else if(recieve)
		rx<={rx[30:0],avs_sdata_sample_sync};
	assign rx_din = avs_sdata_sample_sync;	//used for the display of the data
//*********************************************************************************************************
//MANAGEMENT OF CRC
//*********************************************************************************************************
reg [2:0]crc;
reg [2:0] crctransmit;
reg compute;
reg shiftcrc;
reg clearcrc;
wire [2:0] crcout;

always@(posedge clk)
if(rst == 1) begin
	crc<=0;
	crctransmit<=0;
end
else if(clearcrc)
crc<=0;

else if(compute)
begin

	crctransmit<=crcout;
end
else if(check)	begin	//useful to check the value of crc after the reception of 32 bits
	crctransmit<=crcout;
		end
else if(shiftcrc)  begin
		
		crctransmit<={crctransmit[1:0],1'b0};
		end
	
	
else if(shift_transmit_reg | load_transmit_reg )   
	begin	
		
			     crc[0] <=transmit_reg_dout ^ crc[2];
                       crc[1] <= crc[0]^(crc[2]^transmit_reg_dout);
                       crc[2] <= crc[1];
				end			  
else if(recieve) //switching the crc engine to calculate the crc of the recieved bits
		begin
			 crc[0] <=rx_din ^ crc[2];
                       crc[1] <= crc[0]^(crc[2]^rx_din );
                       crc[2] <= crc[1];
				end
else
		crc<=crc;
assign crcout = crc;
assign crc_dout = crctransmit[2];	
//////////////////////////////////////////////////////////////////////////////////
//controlling the LED's
//////////////////////////////////////////////////////////////////////////////////

always @(posedge clk)
	if(rst == 1)
		begin
			ledperfect<=0;
			lederror<=0;
		end
		else if(check == 1) //check transfers thr crc into crc transmit after 32 bits 
							//of recieved data
			begin
				if(crctransmit == 0)//if the computed crc is zero then the led perfect
										  //glows else led error glows.
					begin
							ledperfect<=1;
							lederror<=0;
					end
				else
					begin
						ledperfect<=0;
							lederror<=1;
					end
end


// ************************************************************************************************
// state machine (Moore, type 3, see [4]
// ************************************************************************************************
  // states
  localparam		STATE_COUNT=4;		// number of bits to encode states
  localparam		IDLE=0;
  localparam		TX_LOAD=1;
  localparam		TX_CLKH=2;
  localparam		TX_CLKL=3;
  localparam		TX_CRC_CLKH=4;
  localparam		TX_CRC_CLKL=5;
  localparam		RX_avs_sdata_sample_sync_CLK_HIGH = 6;//state for the recievd data clk high signal
  localparam		DELAY = 7;
  localparam		RX_avs_sdata_sample_sync_CLK_LOW = 8;//state for the recievd data clk low signal
  //used to maintain 32 complete clock cycles because
										//reception of data takes place during falling edge
										//of the AVS_CLK.So this state helps to maintain the 
										//remaining high state for 100ms thereby completing 
										//32 complete clocks.
  
  
  reg [STATE_COUNT-1:0]	currentstate, next_state;	// pcm master state machine
/////////////////////////////////////////////////////////////////////////////////
//state register//
/////////////////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
	if(rst == 1)
		currentstate<=IDLE;
	else
		currentstate<=next_state;
end
////////////////////////////////////////////////////////////////////////////////
// SM next state logic combined with output logic
always @(*) begin
  // default outputs
  next_state=currentstate;
  counter_bits_value=0; counter_bits_load=0; counter_bits_dec=0;
  counter_clks_value=0; counter_clks_load=0;
  avs_clk_clr=0; avs_clk_set=0;
  avs_mdata_clr=0; avs_mdata_set=0; avs_mdata_clr_recieve=0; avs_mdata_set_recieve=0;
  load_transmit_reg=0; shift_transmit_reg=0;
  compute = 0; shiftcrc = 0; clearcrc = 0;
  recieve = 0;  check = 0;
  case(currentstate)
	
		IDLE: begin
					avs_clk_clr = 1;
					avs_mdata_set = 1;
					avs_mdata_set_recieve = 1;
//					clearcrc = 1;
				if(event_1m)
					begin
						load_transmit_reg = 1;
						next_state = TX_LOAD;
					end
				end
	TX_LOAD: begin
					if (transmit_reg_dout) avs_mdata_set=1; else avs_mdata_clr=1;
						
						avs_clk_set=1;
						counter_bits_load=1; counter_bits_value=29;
						counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
						next_state=TX_CLKH;
				end		
	TX_CLKH: begin
					if (counter_clks_z) begin
							avs_clk_clr=1;
							
							shift_transmit_reg=1; counter_bits_dec=1;
							counter_clks_load=1; counter_clks_value=(NOC_LOW-1);
							next_state=TX_CLKL;
							end
							
							
					
			    end
	TX_CLKL: begin
					if (counter_bits_z) compute = 1;
					if (counter_clks_z) begin
							if (counter_bits_z) begin
									
							
							
								counter_bits_load=1; counter_bits_value=3;
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
								if(crc_dout)  avs_mdata_set=1; else avs_mdata_clr = 1 ;		// NB: CRC is not implemented here in this example, 0's will be transmitted
								avs_clk_set=1; 
								
								next_state=TX_CRC_CLKH;
								end 
								
							else begin
							if (transmit_reg_dout) avs_mdata_set=1; else avs_mdata_clr=1;
								avs_clk_set=1;
								
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
								next_state=TX_CLKH;
							end
					end
				end
	TX_CRC_CLKH: 
						begin
							if (counter_clks_z) begin
								avs_clk_clr=1;
								counter_bits_dec=1;
								counter_clks_load=1; 
								counter_clks_value=(NOC_LOW-1);
								shiftcrc = 1;	
								next_state=TX_CRC_CLKL;
      end
    end
 TX_CRC_CLKL: begin
 if (counter_bits_z)  ;
						if (counter_clks_z) begin
							if (counter_bits_z) begin
							
								counter_bits_load=1; counter_bits_value=32;//loasding with 32 bits
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
							  avs_mdata_set =1;
								avs_clk_set = 1; 
								clearcrc = 1;
								next_state= RX_avs_sdata_sample_sync_CLK_HIGH;
								
							end 
							
							else begin
								
								avs_clk_set=1;//		
																	
								if(crc_dout) avs_mdata_set=1; else avs_mdata_clr = 1 ;	
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
								next_state=TX_CRC_CLKH;
							end
						end
					 end
 RX_avs_sdata_sample_sync_CLK_HIGH: begin

 
										if (counter_clks_z) begin
										avs_clk_clr=1;
										if(rx_din)avs_mdata_set_recieve = 1; else avs_mdata_clr_recieve=1;
										counter_bits_dec=1;
										counter_clks_load=1; 
													counter_clks_value=(NOC_LOW-1);
															recieve = 1;	
															next_state= RX_avs_sdata_sample_sync_CLK_LOW;
									end
    end
	//data is shifted in during this state									
 RX_avs_sdata_sample_sync_CLK_LOW: 	begin  	
						if (counter_bits_z) check = 1;
						if (counter_clks_z) begin
							if (counter_bits_z) begin
							clearcrc = 1;
							next_state=IDLE;
								
							end 
							else begin
								
								avs_clk_set=1;
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
								next_state= RX_avs_sdata_sample_sync_CLK_HIGH;
							end
						end
	
				 end
		 

							
 default: begin
	next_state=IDLE;
    end
  endcase
end
