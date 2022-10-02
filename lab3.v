module lab3		#(parameter N=16,parameter W=9) //ANSI style of port declaration
				(input CLOCK_50,		//clock from the SOC
				 input [9:0] SW,		//switches for the SW[8:0]=DATA INPUT and the SW[9]=mode
				 input [3:0] KEY,		//keys for [3:0]=WRITE, READ, CLEAR_OV AND RESET
				 output [9:0] LEDR,	//LED's to be either used for LEDR[9]=mode or for displaying output data/write_index, rd_index
				 output [6:0] HEX0);
	
	reg [N-1:0]data_in;
	wire [N-1:0]data_out;


	reg mode;	
	wire empty, full;
	wire overflow;
	wire [$clog2(N)-1:0] wr_index; 
	wire [$clog2(N)-1:0] rd_index;
	
	wire rd,wr;
	
	wire [3:0]hexaout;
	
	//handle debouncing of the keys
	reg pre_reset,reset;
	reg pre_clear,clear;
	reg pre_read,read;
	reg pre_write,write;
	always @ (posedge(CLOCK_50))
	begin 
		pre_reset <= !KEY[0]; //keys are by default 1 and are connected to pull up resistors
		reset	    <= pre_reset; //handle input metastability for reset
		pre_clear <= !KEY[1];
		clear	    <= pre_clear; //handle input metastability for clear_overflow_req
//		pre_read  <= !KEY[2];
//		read	    <= pre_read;  //handle input metastability for read req
//		pre_write <= !KEY[3];
//		write	    <= pre_write; //handle input metastability for write req
		data_in	 <= SW[8:0];
		mode		 <= SW[9];	
	end
	
	
	//handle the edge detections for the read and write
	edge_detector read_edge(!KEY[2],CLOCK_50,rd);
	edge_detector write_edge(!KEY[3],CLOCK_50,wr);
	
	always @ (posedge(CLOCK_50) or posedge(rd)  or posedge(wr))
	begin
	pre_read  <= rd;
	read	    <= pre_read;  //handle input metastability for read req
	pre_write <= wr;
	write	    <= pre_write; //handle input metastability for write req
	end
	
	FF fifo_1(CLOCK_50,reset,data_in,write,data_out,read,empty,full, overflow,clear,wr_index,rd_index);
	
	
	assign LEDR[8:0]	= (mode==1'b0) ? data_out : {wr_index,1'b0,rd_index};
	assign LEDR[9]		= (mode==1'b0) ? 1'b0 : 1'b1;
	
	assign hexaout={1'd0,full,empty,overflow};

	decTo7 hexa(hexaout,HEX0);

endmodule

module FF	 #(parameter N=16,parameter W=9)
				(input clk, 
				input reset, 
				input[W-1:0] wr_data, 
				input wr_request, 
				output reg [W-1:0] rd_data, 
				input rd_request,
				output empty, 
				output full, 
				output reg overflow, 
				input clear_overflow_request,
				output reg [$clog2(N)-1:0] wr_index, 
				output reg [$clog2(N)-1:0] rd_index);

	
	reg [W-1:0] stack[N-1:0]; //16 X 9 bit buffer
	reg [4:0]counter;
	assign full  = (counter==5'd16) && !overflow ? 1'b1:1'b0;
	assign empty = (counter==1'd0)? 1'b1:1'b0;
	
	always @(posedge(clk))
	begin
		if(reset)
		begin
			wr_index<=0;
			rd_index<=0;
			overflow<=0;
			counter<=0;
			rd_data<=0;
		end
		else if(clear_overflow_request && overflow)
		begin
			counter<=counter-2'd2;
			overflow<=0; //clear the overflow
		end
		else
		begin
			if(wr_request)
			begin
				if(!full)
				begin
					stack[wr_index]<= wr_data;
					counter<=counter+1'b1;
					wr_index<=wr_index+1'b1;
					overflow<=0;
				end
				else
				begin
					counter<=5'd17;
					overflow<=1;			
				end
			end
			else if(rd_request && !empty)
			begin
				if(overflow)
				begin
					rd_data<=stack[rd_index];
					rd_index<=rd_index+1'b1;
					counter<=counter-2'b10;		
					overflow<=0;
				end
				else
				begin
					rd_data<=stack[rd_index];
					rd_index<=rd_index+1'b1;
					counter<=counter-1'b1;
				end				
			end
		end
	end
		

endmodule




module edge_detector(input in,
							input clk,
							output out);
		reg del_in;
		
		always @(posedge clk)
		begin
				del_in<=in;
		end	
		assign out = in & ~del_in;		
//		always @ (posedge del_in)
//		begin
//			out<= (in & ~del_in);
//		end						
endmodule



module decTo7(input [3:0] in, output reg [6:0] out);		
	always @ (in)
	begin
		case (in)
			4'b0010: out=7'b0001000; //print A saying empty
			4'b0100: out=7'b0000010; //print G saying Full
			4'b0001: out=7'b1000000; //print D saying overflow
			default: out=7'b1111111; //dont print anything if something else is given as input
		endcase  
	end	
endmodule