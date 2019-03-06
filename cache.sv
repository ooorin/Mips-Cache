`timescale 1ns / 1ps

module dcache_one_line(
	input logic 			clk, rst, enable, compare, read,
	input logic [31:0] 		address_in, data_in,
	input logic [255:0] 	data_line_in,

	output logic 			hit, dirty, valid,
	output logic [31:0] 	address_out,
	output logic [31:0] 	data_out,
	output logic [255:0] 	data_line_out
	);

	logic [255:0] 	mem [63:0];
	logic [20:0] 	tag [63:0];
	logic 			validate_bit [63:0];
	logic 			dirty_bit [63:0];

	logic [20:0]	addr_tag;
	logic [5:0]		addr_index;
	logic [2:0]		addr_in_block;

	assign addr_tag = address_in[31:11];
	assign addr_index = address_in[10:5];
	assign addr_in_block = address_in[4:2];

	assign hit = compare == 1 ? (addr_tag == tag[addr_index] ? 1 : 0) : 0;
	assign dirty = dirty_bit[addr_index];
	assign valid = validate_bit[addr_index];

	assign address_out = {tag[addr_index], addr_index, 5'b0};

	integer i;
	always_ff @(negedge clk) // posedge/negdege ?
	begin
		if (rst)
		begin
			for (i = 0; i < 64; i = i + 1)
			begin
				validate_bit[i] = 1'b0;
       			dirty_bit[i] = 1'b0;
       			tag[i] = 21'b0;
			end
		end
		else if (enable)
		begin	
			if (read)
			begin
				case(addr_in_block)
					0: data_out <= mem[addr_index][31:0];
					1: data_out	<= mem[addr_index][63:32];
					2: data_out	<= mem[addr_index][95:64];
					3: data_out	<= mem[addr_index][127:96];
					4: data_out	<= mem[addr_index][159:128];
					5: data_out	<= mem[addr_index][191:160];
					6: data_out	<= mem[addr_index][223:192];
					7: data_out	<= mem[addr_index][255:224];
				endcase
				data_line_out <= mem[addr_index];
			end
			else if (~read)
			begin
				if (~compare)
				begin
					dirty_bit[addr_index] <= 1'b0;
					mem[addr_index] <= data_line_in;
					tag[addr_index] <= addr_tag;
					validate_bit[addr_index] <= 1'b1;
				end
				else if (compare & hit)
				begin
					dirty_bit[addr_index] <= 1'b1;
					//validate_bit[addr_index] <= 1'b1;
					case(addr_in_block)
						0: mem[addr_index][31:0] <= data_in[31:0];
						1: mem[addr_index][63:32] <= data_in[31:0];
						2: mem[addr_index][95:64] <= data_in[31:0];
						3: mem[addr_index][127:96] <= data_in[31:0];
						4: mem[addr_index][159:128] <= data_in[31:0];
						5: mem[addr_index][191:160] <= data_in[31:0];
						6: mem[addr_index][223:192] <= data_in[31:0];
						7: mem[addr_index][255:224] <= data_in[31:0];
					endcase
				end
			end
		end
	end

endmodule

module dcache_two_way_group(
	input logic 			clk, rst, enable, compare, read,
	input logic [31:0] 		address_in, data_in,
	input logic [255:0] 	data_line_in,

	output logic 			hit, dirty, valid,
	output logic [31:0]		data_out,
	output logic [31:0]		address_out,
	output logic [255:0]	data_line_out
	);

	logic 			enable0, enable1;
	logic 			hit0, hit1, dirty0, dirty1, valid0, valid1;
	logic [31:0] 	data_out0, data_out1;
	logic [31:0] 	address_out0, address_out1;
	logic [255:0] 	data_line_out0, data_line_out1;

	logic sel; // select

	always @(enable or read or compare)
	begin
		if (rst)
		begin
			sel = 1'b0;
		end
		else if (enable)
		begin
			if (read)
			begin
				if (compare)
				begin
					sel = ~sel;
				end
			end
		end
	end

	assign hit = hit0 | hit1;
	assign dirty = (hit0 & hit1) ? (sel ? dirty1 : dirty0) : (hit0 ? dirty0 : (hit1 ? dirty1 : 0));
	assign valid = (hit0 & hit1) ? (sel ? valid1 : valid0) : (hit0 ? valid0 : (hit1 ? valid1 : 0));
	assign address_out = (enable0 & enable1) ? (sel ? address_out1 : address_out0) : (enable0 ? address_out0 : (enable1 ? address_out1 : 0));
	assign data_line_out = (hit0 & hit1) ? (sel ? data_line_out1 : data_line_out0) : (hit0 ? data_line_out0 : (hit1 ? data_line_out1 : 0));
	assign data_out = (hit0 & hit1) ? (sel ? data_out1 : data_out0) : (hit0 ? data_out0 : (hit1 ? data_out1 : 0));
	assign enable0 = enable & (compare | ~read & (~valid0 & valid1 | ~sel));
	assign enable1 = enable & (compare | ~read & (~valid1 & valid0 |  sel));

	dcache_one_line dcache_one_line0(
						clk, rst, enable0, compare, read,
						address_in, data_in,
						data_line_in,

						hit0, dirty0, valid0,
						address_out0,
						data_out0,
						data_line_out0
					);

	dcache_one_line dcache_one_line1(
						clk, rst, enable1, compare, read,
						address_in, data_in,
						data_line_in,

						hit1, dirty1, valid1,
						address_out1,
						data_out1,
						data_line_out1
					);

endmodule

module cache_control(
	input logic 	clk, rst,
	input logic 	d_cache_read, d_cache_write,
	input logic 	response_ram_to_cache,

	input logic 	d_cache_miss,
	input logic 	d_cache_dirty,
	input logic 	d_cache_hit,
	
	output logic 	d_cache_enable,
	output logic 	d_cache_compare,
	output logic 	d_cache_read_o,

	output logic 	enable_cache_to_ram,
	output logic 	write_cache_to_ram,
	output logic 	response_data_cache_to_core,

	output logic 	[3:0] stateOut
	);

	logic [3:0] state;

	always_ff @(posedge clk)
	begin
		if (rst)
		begin
			state <= 0;
		end
		else
		begin
			case (state)
				0: 
				begin
					if(d_cache_read)
					begin
						state <= 1;
					end
					else if(d_cache_write)
					begin
						state <= 2;
					end	
				end
				1: 
				begin
					if(d_cache_hit)
						state <= 3;
					else if(d_cache_miss && ~d_cache_dirty)
						state <= 5;	
					else if(d_cache_miss && d_cache_dirty)
						state <= 4;
				end
				2: 
				begin
					if(d_cache_hit)
						state <= 3;
					else if(d_cache_miss && ~d_cache_dirty)
						state <= 5;
					else if(d_cache_miss && d_cache_dirty)
						state <= 4;
				end
				3: 
				begin
					state <= 0;
				end
				4: 
				begin
					if(response_ram_to_cache)
						state <= 5;
				end
				5: 
				begin
					if(response_ram_to_cache)
						state <= 6;
				end
				6: 
				begin
					if(d_cache_read)
						state <= 1;
					else if(d_cache_write)
						state <= 2;
				end
				default: state <= 0;
			endcase
		end
	end

	always_comb
	begin
		case (state)
			0:
			begin
				d_cache_enable				= 0;
				d_cache_compare				= 0;
				d_cache_read_o				= 0;
				enable_cache_to_ram			= 0;
				write_cache_to_ram			= 0;
				response_data_cache_to_core = 0;
			end
			1:
			begin
				d_cache_enable				= 1;
				d_cache_compare				= 1;
				d_cache_read_o				= 1;
				enable_cache_to_ram			= 0;
				write_cache_to_ram			= 0;
				response_data_cache_to_core = 0;
			end
			2:
			begin
				d_cache_enable				= 1;
				d_cache_compare				= 1;
				d_cache_read_o				= 0;
				enable_cache_to_ram			= 0;
				write_cache_to_ram			= 0;
				response_data_cache_to_core = 0;
			end
			3:
			begin
				d_cache_enable				= 0;
				d_cache_compare				= 1;
				d_cache_read_o				= 0;
				enable_cache_to_ram			= 0;
				write_cache_to_ram			= 0;
				response_data_cache_to_core = 1;
			end
			4:
			begin
				d_cache_enable				= 0;
				d_cache_compare				= 0;
				d_cache_read_o				= 0;
				enable_cache_to_ram			= 1;
				write_cache_to_ram			= 1;
				response_data_cache_to_core = 0;
			end
			5:
			begin
				d_cache_enable				= 1;
				d_cache_compare				= 0;
				d_cache_read_o				= 0;
				enable_cache_to_ram			= 1;
				write_cache_to_ram			= 0;
				response_data_cache_to_core	= 0;
			end
			6:
			begin
				d_cache_enable				= 1;
				d_cache_compare				= 0;
				d_cache_read_o				= 0;
				enable_cache_to_ram			= 0;
				write_cache_to_ram			= 0;
				response_data_cache_to_core	= 0;
			end
		endcase
	end

	assign stateOut = state;

endmodule

module dcache_top(
	input logic clk, rst,
	input logic d_cache_read, d_cache_write,
	input logic response_ram_to_cache,
	input logic [31:0] data_core_to_dcache, address_core_to_dcache,
	input logic [255:0] data_ram_to_cache,

	output logic response_data_cache_to_core, 
	output logic enable_cache_to_ram, write_cache_to_ram,
	output logic [31:0] data_cache_to_core, address_cache_to_ram,
	output logic [255:0] data_cache_to_ram,

	output logic [3:0] stateOut,
	output logic hitOut
	);

	logic d_cache_miss, d_cache_dirty, d_cache_hit;
	logic d_cache_enable, d_cache_compare, d_cache_read_o;

	cache_control cache_control(
					clk, rst,
					d_cache_read, d_cache_write,
					response_ram_to_cache,

					d_cache_miss,
					d_cache_dirty,
					d_cache_hit,
					
					d_cache_enable,
					d_cache_compare,
					d_cache_read_o,

					enable_cache_to_ram,
					write_cache_to_ram,
					response_data_cache_to_core,

					stateOut
		);

	assign d_cache_miss = ~d_cache_hit;

	logic d_cache_valid; // ?
	dcache_two_way_group dcache_two_way_group(
							.clk(clk),
							.rst(rst),
							.enable(d_cache_enable),
							.compare(d_cache_compare),
							.read(d_cache_read_o),
							.address_in(address_core_to_dcache),
							.data_in(data_core_to_dcache),
							.data_line_in(data_ram_to_cache),
							
							.hit(d_cache_hit),
							.dirty(d_cache_dirty),
							.valid(d_cache_valid),
							.data_out(data_cache_to_core),
							.address_out(address_cache_to_ram),
							.data_line_out(data_cache_to_ram)
		);

	assign hitOut = d_cache_hit;

endmodule

module cache_manage(
	input logic 		clk, rst,
	input logic 		dc_read_en,
	input logic 		dc_write_en,
	input logic 		ram_ready,
	input logic [31:0] 	dc_addr,
	input logic [31:0]	dc_data,
	input logic [255:0] block_from_ram,

	output logic		mem_stall,
	output logic [31:0] dc_data_out,

	output logic 		ram_en,
	output logic 		ram_write_en,
	output logic [31:0] ram_addr,
	output logic [255:0] ram_data,

	output logic [3:0] stateOut,
	output logic hitOut
	);

	logic response_data_cache_to_core;

	dcache_top  dcache_top(
				.clk(clk),
				.rst(rst),
				.d_cache_read(dc_read_en),
				.d_cache_write(dc_write_en),
				.response_ram_to_cache(ram_ready),
				.data_core_to_dcache(dc_data),
				.address_core_to_dcache(dc_addr),
				.data_ram_to_cache(block_from_ram),

				.response_data_cache_to_core(response_data_cache_to_core),
				.enable_cache_to_ram(ram_en),
				.write_cache_to_ram(ram_write_en),
				.data_cache_to_core(dc_data_out),
				.address_cache_to_ram(ram_addr),
				.data_cache_to_ram(ram_data),

				.stateOut(stateOut),
				.hitOut(hitOut)
            );

	assign mem_stall = (dc_read_en | dc_write_en) ? (response_data_cache_to_core ? 0 : 1) : 0;

endmodule
