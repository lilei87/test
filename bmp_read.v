`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//================================================================================
//  Revision History:
//  Date          By            Revision    Change Description
//--------------------------------------------------------------------------------
//      
//*******************************************************************************/
module bmp_read
(
	input                       clk,
	input                       rst,
	input   wire                transfer_start_flag,         //sdram突发写，写完一帧数据后反馈信号
	output wire[31:0]           udp_num,                     //一帧数据的udp包数，传递到“image_data.v”
	input                       find,
	input                       sd_init_done,                //SD card initialization completed
	output reg[3:0]             state_code,                  //state indication coding,
															               // 0:SD card is initializing,
															               // 1:wait for the button to press//按键可以改为计数器延时拉低电平
															               // 2:looking for the bmp file
															               // 3:reading SD card
	output reg[31:0]            bmp_len_cnt,                 // 引出测试
	input[31:0]                 bmp_width,                   //search the width of bmp
	output reg                  write_req,                   //start writing request
	input                       write_req_ack,               //write request response
	output reg                  sd_sec_read,                 //SD card sector read
	output reg[31:0]            sd_sec_read_addr,            //SD card sector read address
	output reg[31:0]            sd_sec_read_addr_reg1,       //引出测试，S_READ状态机结束，一帧读取完毕，将扇区地址赋值给此寄存器
	output reg[31:0]            frame_end,                   //引出测试，当前帧数据有效的结束（域数据段的结束）
	output reg[31:0]            frame_len,                   //帧长度
	input[7:0]                  sd_sec_read_data,            //SD card sector read data
	input                       sd_sec_read_data_valid,      //SD card sector read data valid
	input                       sd_sec_read_end,             //SD card sector read end
	output reg                  bmp_data_wr_en,              //bmp image data write enable
	output reg[15:0]            bmp_data,                    //bmp image data
	output reg[1:0]             write_addr_index,            //sdram写地址
	output reg[1:0]             read_addr_index,             //sdram读地址
	output wire[31:0]           frame_cnt,                   //已写sdram的帧的计数器,表示当前帧序号,从1开始
	output wire[31:0]           frame_interval_wire          //一帧数据从开始发送到发送完成的时钟周期数量计算  //引出测试
);
localparam S_IDLE         = 0;
localparam S_FIND         = 1;
localparam S_FIND2        = 2;
localparam S_READ_WAIT    = 3;
localparam S_READ         = 4;
localparam S_END          = 5;

localparam INTERVAL_40MS  = 32'd4_000_000;  //单位：clk，多少个时钟周期，此程序暂以固定帧率25帧、单帧数据98个域以内
localparam BASE           = 32'd24_000;
localparam CONSTANT       = 32'd40_000;

reg              transfer_start_flag_dly0;   // transfer_start_flag打一拍
reg              transfer_start_flag_dly1;   // transfer_start_flag打两拍
wire             transfer_start;             // transfer_start_flag_dly1的下降沿产生该信号，sdram开始写下一帧

reg[31:0]        frame_start;                //当前帧数据有效的起始（域数据段的起始）
reg[31:0]        frame_cnt0;                 //已写sdram的帧的计数器,表示当前帧序号,从1开始
wire[31:0]       frame_act_len;              //灯效文件一帧的长度+帧头8字节
wire[31:0]       sd_sec_num;                 //当前帧（含之前所有帧）数据所占的扇区数量（实际从灯效文件第一字节算起），是变量
reg[31:0]        sd_sec_read_addr_reg0;      //找到灯效文件时，将扇区地址赋值给此寄存器，这是灯效文件的起始扇区地址
reg[31:0]        sd_sec_read_addr_reg2;      //寄存计算得到的扇区地址，是当前帧读取有效数据的扇区起始地址(*keep="true"*)
wire[11:0]       diff_size;                  //差值大小，是变量，以协助确定当前帧的有效域数据段的扇区地址

reg[31:0]        frame_rate;                 //帧率：一秒钟发送多少帧数据,范围：0~100
wire[31:0]       ini_value;                  //frame_interval计数器的初始值,wire型,用以判断
wire[31:0]       ini_value0;                 //用以赋值
reg[31:0]        frame_num;                  //灯效文件的帧数量
reg[31:0]        file_in_len; 
reg[31:0]        universe_num;               //（一帧数据的）域数量
wire[63:0]       product;                    //计算CONSTANT*universe_num
reg              found2;                     //第二帧及之后的帧，搜索到标志信号:暂由计数器rd_frame_cnt间接控制
reg[31:0]        rd_frame_cnt;               //帧间隔等待稳定的计数器，暂定511自减到1的时间段，认为下一帧需要的各个参数能计算完毕
reg[14:0]        wait_cnt;                   //frame_cnt0在收到transfer_start信号后延迟一段时间，确保状态机在frame_end变化之前能够跳出S_READ状态
reg[31:0]        frame_interval;             //一帧数据从开始发送到发送完成的时钟周期数量计算

reg[3:0]         state;
reg[9:0]         rd_cnt;                     //sector read length counter
reg[7:0]         header_0;
reg[7:0]         header_1;
reg[7:0]         header_2;
reg[7:0]         header_3;
reg[31:0]        width;
reg              found;
wire             bmp_data_valid;             //bmp image data valid
reg              bmp_len_cnt_tmp;            //bmp RGB counter 0 1 2

//灯效文件一帧的长度+帧头8字节
assign frame_act_len = 32'd8 + frame_len;
                         
//除以512并取整，表示当前帧(及之前帧)数据所占的扇区数量（从灯效文件第一字节算起），是变量//排除第一帧这个特殊情况
assign sd_sec_num    = (frame_cnt0 > 0 )? (sd_sec_read_addr_reg1 - sd_sec_read_addr_reg0 + 32'd1): 1'b0;                            
//差值大小，是变量，以协助确定当前帧的有效域数据段的扇区地址
assign diff_size     = (frame_cnt0 > 0 )? ((sd_sec_num << 9) - file_in_len): 1'b0;   
     
assign udp_num       = universe_num;
assign frame_cnt     = frame_cnt0;
assign frame_interval_wire = frame_interval;
assign ini_value     = INTERVAL_40MS - BASE - product;//frame_interval计数器的初始值
assign ini_value0    = (ini_value > 32'd99)? ini_value : 32'd99;
           
//remove the first N bytes of the file，按帧来发送，一次发送1帧 //去掉文件头N字节之后，之后的数据为有效数据
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1 && bmp_len_cnt >= frame_start && bmp_len_cnt < frame_end);
                                                                                               
//wait_cnt:frame_cnt0在收到transfer_start信号后延迟一段时间，确保状态机在frame_end变化之前能够跳出S_READ状态
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)	  
		wait_cnt <= 15'd0;		
	else if(transfer_start == 1'b1) 
	  begin
      if(state != S_READ)begin
		wait_cnt   <= 15'd1; 
		end
		else begin
		wait_cnt   <= 15'd32000;     //读SD卡一个扇区需要（100M时钟下）16384个clk，可修改32000试试
	   end
	  end		                         
	else if(state == S_READ && wait_cnt >= 15'd2) //只要还在S_READ状态，wait_cnt就自减计数
	   wait_cnt <= wait_cnt - 1'b1;		
   else if(state != S_READ && wait_cnt >= 15'd2)
	   wait_cnt <= 15'd1;		
	else
		wait_cnt <= 15'd0;
end

//frame_cnt0:已写sdram的帧的计数器(与计数器wait_cnt有关)
always@(posedge clk or posedge rst)
begin
    if(rst == 1'b1)
	  begin
		frame_cnt0     <= 32'd0;                                                             //第1帧数据
	   file_in_len    <= 32'd0;
	  end
	 else if((wait_cnt == 15'd1) && (frame_cnt0 < frame_num) && (frame_cnt0 != 32'd0))      //第3帧及之后的帧//最终"frame_cnt0 == frame_num"
	  begin
		frame_cnt0     <= frame_cnt0 + 32'd1;                                                //倒数第2帧写完sdram后，frame_cnt0最终值为frame_num
	   file_in_len    <= file_in_len + frame_act_len;
	  end
	 else if((wait_cnt == 15'd1) && (frame_cnt0 < frame_num) && (frame_cnt0 == 32'd0))      //第2帧
	  begin
		frame_cnt0     <= frame_cnt0 + 32'd1;                       
	   file_in_len    <= 32'd64 + frame_act_len;
	  end	 
	 else
	  begin
		frame_cnt0     <= frame_cnt0;
		file_in_len    <= file_in_len;
	  end
end

//frame_interval:一帧数据从开始发送到发送完成的时钟周期数量计算//测试目的，chipscope可看
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		frame_interval <= 32'd0;
	else if(state == S_READ && frame_cnt0 == 32'd9)	
		frame_interval <= frame_interval + 32'd1;
	else if(state == S_READ_WAIT && frame_cnt0 == 32'd9)
		frame_interval <= 32'd0;
	else
	   frame_interval <= frame_interval;
end

//transfer_start_flag打2拍
always@(posedge clk or posedge rst)
    if(rst == 1'b1)
       begin	 
		  transfer_start_flag_dly0 <= 0;
		  transfer_start_flag_dly1 <= 0;
		 end
    else begin
        transfer_start_flag_dly0 <= transfer_start_flag;
		  transfer_start_flag_dly1 <= transfer_start_flag_dly0;
	     end

//transfer_start:transfer_start_flag_dly1信号下降沿//换用上升沿？
assign  transfer_start = ((transfer_start_flag_dly1 == 1'b0) && (transfer_start_flag_dly0 == 1'b1));

//计算当前帧的起始扇区地址，当前帧的有效域数据范围：frame_start、frame_end
always@(posedge clk or posedge rst)
begin  
	 if(rst == 1'b1)                                             
	   begin
		frame_start  <= 0;
	   frame_end    <= 0;
		sd_sec_read_addr_reg2 <= 0;                               
		end
	 else if(frame_cnt0 == 32'd0)
	   begin
		frame_start  <= 32'd84;
	   frame_end    <= 32'd64 + frame_act_len;
		sd_sec_read_addr_reg2 <= 0;                                                 //第一帧数据的扇区地址已在“搜索灯效文件头信息”中给出
		end
	 else if((diff_size > 32'd20)&&( frame_cnt0> 32'd0)&& (frame_cnt0< frame_num)&& (state != S_READ)) //第二帧及之后的帧//diff_size范围22~512,246种可能（偶数才有可能性）	
	   begin
	   frame_start  <= 32'd532 - diff_size;
	   frame_end    <= 32'd512 + frame_act_len - diff_size;
		sd_sec_read_addr_reg2 <= sd_sec_read_addr_reg1 ;                            //sd_sec_read_addr_reg表示起始扇区地址，本身地址已含1个扇区，注意 
	   end
	 else if((diff_size <= 32'd20)&&( frame_cnt0> 32'd0)&&(frame_cnt0< frame_num)&& (state != S_READ)) //diff_size范围2~20,10种可能（偶数才有可能性）
      begin
		frame_start  <= 32'd20 - diff_size;
	   frame_end    <= frame_act_len - diff_size;
		sd_sec_read_addr_reg2 <= sd_sec_read_addr_reg1 + 32'd1; 		
		end
    else
	   begin
		frame_start  <= frame_start;
	   frame_end    <= frame_end;
		sd_sec_read_addr_reg2 <= sd_sec_read_addr_reg2; 		
		end		
end

//rd_frame_cnt:帧切换等待参数计算完成，等待计数器,保证每秒25帧的帧率
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		rd_frame_cnt <= 32'd0;
	else if((wait_cnt == 15'd1) && (frame_cnt0 < frame_num))
	   rd_frame_cnt <= ini_value0;
	else if(rd_frame_cnt >= 32'd2 && (state == S_IDLE))
	   rd_frame_cnt <= rd_frame_cnt - 1'b1;		
	else if(state == S_FIND2)
	   rd_frame_cnt <= 32'd0;
	else
		rd_frame_cnt <= rd_frame_cnt;
end

//状态机进入S_FIND2，found2赋值为1
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		found2 <= 1'b0;
	else if(state == S_FIND2 && ( frame_cnt0> 32'd0) && (frame_cnt0< frame_num))
	   found2 <= 1'b1;
	else if(state != S_FIND2)
		found2 <= 1'b0;
end

//生成write_addr_index、	read_addr_index，写完一帧读一帧，读比写晚一帧	 
always@(posedge clk or posedge rst)
begin
     if(rst == 1'b1) 
	    begin
       write_addr_index <= 2'd0;
		 read_addr_index  <= 2'd0;
		 end
	  else if((transfer_start == 1'b1)&&(frame_cnt0< frame_num))//确保生成最后一帧的读地址
		 begin
		 write_addr_index <= write_addr_index + 2'd1;
		 read_addr_index  <= write_addr_index       ;//read_addr_index要晚write_addr_index一帧
		 end	  
	  else
	    begin
		 write_addr_index <= write_addr_index ;
		 read_addr_index  <= read_addr_index  ;
		 end
end

//搜索文件头计数器
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		rd_cnt <= 10'd0;
	else if(state == S_FIND)//进入S_FIND状态开始计数
	begin
		if(sd_sec_read_data_valid == 1'b1)//并且扇区数据读信号有效
			rd_cnt <= rd_cnt + 10'd1;
		else if(sd_sec_read_end == 1'b1)
			rd_cnt <= 10'd0;
	end
	else
		rd_cnt <= 10'd0;
end

//搜索灯效文件头信息
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		header_0 <= 8'd0;
		header_1 <= 8'd0;
      header_2 <= 8'd0;
		header_3 <= 8'd0;
		frame_len <= 32'd0;
		frame_num <= 32'd0;
		frame_rate   <= 32'd0;
		universe_num <= 32'd0;
		found <= 1'b0;
		sd_sec_read_addr_reg0<= 32'd0;
	end
	else if(state == S_FIND && sd_sec_read_data_valid == 1'b1)
	begin
		//file header
		if(rd_cnt == 10'd0)
			header_0 <= sd_sec_read_data;
		if(rd_cnt == 10'd1)
			header_1 <= sd_sec_read_data;
      if(rd_cnt == 10'd2)
			header_2 <= sd_sec_read_data;
		if(rd_cnt == 10'd3)
			header_3 <= sd_sec_read_data; 
		if(rd_cnt == 10'd4)
			width[7:0] <= sd_sec_read_data;
		if(rd_cnt == 10'd5)
			width[15:8] <= sd_sec_read_data;
		if(rd_cnt == 10'd6)
			width[23:16] <= sd_sec_read_data;
		if(rd_cnt == 10'd7)
			width[31:24] <= sd_sec_read_data;
		if(rd_cnt == 10'd56)
		   frame_num[7:0]   <= sd_sec_read_data;
		if(rd_cnt == 10'd57)
		   frame_num[15:8]  <= sd_sec_read_data;
		if(rd_cnt == 10'd58)
		   frame_num[23:16] <= sd_sec_read_data;
		if(rd_cnt == 10'd59)
		   frame_num[31:24] <= sd_sec_read_data;
	   if(rd_cnt == 10'd68)
		   frame_len[7:0]   <= sd_sec_read_data;
		if(rd_cnt == 10'd69)
		   frame_len[15:8]  <= sd_sec_read_data;
		if(rd_cnt == 10'd70)
		   frame_len[23:16] <= sd_sec_read_data;
		if(rd_cnt == 10'd71)
		   frame_len[31:24] <= sd_sec_read_data;				
		if(rd_cnt == 10'd76)
		   frame_rate[7:0]   <= sd_sec_read_data;
		if(rd_cnt == 10'd77)
		   frame_rate[15:8]  <= sd_sec_read_data;
		if(rd_cnt == 10'd78)
		   frame_rate[23:16] <= sd_sec_read_data;
		if(rd_cnt == 10'd79)
		   frame_rate[31:24] <= sd_sec_read_data;		
		if(rd_cnt == 10'd80)
		   universe_num[7:0]   <= sd_sec_read_data;
		if(rd_cnt == 10'd81)
		   universe_num[15:8]  <= sd_sec_read_data;
		if(rd_cnt == 10'd82)
		   universe_num[23:16] <= sd_sec_read_data;
		if(rd_cnt == 10'd83)
		   universe_num[31:24] <= sd_sec_read_data;			
		//check the width of the image and file header  after the end of the file header
		if(rd_cnt == 10'd499 && (header_0 == 8'hB2) && (header_1 == 8'h76 )&& (header_2 == 8'hAA) && (header_3 == 8'h89 )&& width == bmp_width)//修改计数值
			begin
			found <= 1'b1;
			sd_sec_read_addr_reg0 <= sd_sec_read_addr;   //灯效文件的起始扇区地址
			end
	end
	else if(state != S_FIND)
		found <= 1'b0;
end

//bmp file length counter
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		bmp_len_cnt <= 32'd0;
	else if(state == S_READ)                 //进入S_READ状态才开始计数
	begin
		if(sd_sec_read_data_valid == 1'b1)    //并且扇区数据读信号有效    
			bmp_len_cnt <= bmp_len_cnt + 32'd1;
	end
	else if(state == S_END)
		bmp_len_cnt <= 32'd0;
end

//bmp_len_cnt_tmp = 1 时，才相当于数据的有效标志信号，bmp_data_valid还不算实质意义上的数据有效标志信号
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		bmp_len_cnt_tmp <= 1'b0;
	else if(state == S_READ)
	begin
		if(bmp_data_valid == 1'b1)		
		  bmp_len_cnt_tmp <= ~bmp_len_cnt_tmp;			
	end
	else if(state == S_END)
		bmp_len_cnt_tmp <= 1'b0;
end

always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	 begin
		bmp_data_wr_en <= 1'b0;
		bmp_data <= 16'd0;
	 end
	else if(state == S_READ)
	 begin
      if(bmp_len_cnt_tmp == 1'b1 && bmp_data_valid == 1'b1) 
		begin
			bmp_data_wr_en <= 1'b1;
			bmp_data[15:8] <= sd_sec_read_data;
		end
		else if(bmp_len_cnt_tmp == 1'b0 && bmp_data_valid == 1'b1) 
		begin
			bmp_data_wr_en <= 1'b0;
			bmp_data[7:0] <= sd_sec_read_data;
		end		
     else begin
		bmp_data_wr_en <= 1'b0;     //将write_en置为0
	   end
	end
end

//状态机：状态转换
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		sd_sec_read <= 1'b0;
		sd_sec_read_addr <= 32'd600;//初始搜索扇区地址
		sd_sec_read_addr_reg1<= 32'd0;
		write_req <= 1'b0;
		state_code <= 4'd0;
	end
	else if(sd_init_done == 1'b0)
	begin
		state <= S_IDLE;
	end
	else
		case(state)
			S_IDLE:
			begin
				state_code  <= 4'd1;
				sd_sec_read <= 1'b0;
				begin
				if((rd_frame_cnt == 32'd1) && (frame_cnt0 < frame_num))//写完最后一帧，不会再进入S_FIND2
				   begin
					state <= S_FIND2;
				   sd_sec_read_addr <= sd_sec_read_addr_reg2;
					end
				else if(find == 1'b1)  //一直检测到find变为1
					begin
					state <= S_FIND;
				   sd_sec_read_addr <= {sd_sec_read_addr[31:3],3'd0};//address 8 aligned,作用是？只为保证低3位是000
					end
			   end
			end
			S_FIND2:
			begin
			   state_code <= 4'd2;
			   if(sd_sec_read_end == 1'b1)
				begin
			     if(found2 == 1'b1)//触发读开始，进入 S_READ_WAIT 的条件
					 begin
						state <= S_READ_WAIT;
						sd_sec_read <= 1'b0;
						write_req <= 1'b1;
						sd_sec_read_addr <= sd_sec_read_addr_reg2;
					 end
              else begin
				      sd_sec_read_addr <= sd_sec_read_addr_reg2;
				    end     				 
			   end
				else begin
					sd_sec_read <= 1'b1;
				end
			end			
			S_FIND:
			begin
				state_code <= 4'd2;
				if(sd_sec_read_end == 1'b1)
				begin
					if(found == 1'b1)//触发读开始，进入 S_READ_WAIT 的条件
					begin
						state <= S_READ_WAIT;
						sd_sec_read <= 1'b0;
						write_req <= 1'b1;//start writing data 
					end								
					else
					begin
						//search every 8 sectors(4K)，每次搜索8个扇区
						sd_sec_read_addr <= sd_sec_read_addr + 32'd8;
					end
				end
				else
				begin
					sd_sec_read <= 1'b1;
				end
			end			
		  S_READ_WAIT:
			begin
				if(write_req_ack == 1'b1)//write data response 
				begin
					state <= S_READ;      //read SD card data
					write_req <= 1'b0;
				end
			end
			S_READ:
			begin
				state_code <= 4'd3;
				if(sd_sec_read_end == 1'b1)
				begin
					sd_sec_read_addr <= sd_sec_read_addr + 32'd1;//开始读下一个扇区					
					sd_sec_read <= 1'b0;
					if(bmp_len_cnt >= frame_end)                 //读结束的条件
					begin
						state <= S_END;
						sd_sec_read <= 1'b0;
						sd_sec_read_addr_reg1 <= sd_sec_read_addr;//将扇区地址赋值给寄存器,一帧发送完毕
					end
				end
				else
				begin
					sd_sec_read <= 1'b1;                         //读sd卡开始
				end
			end
			S_END:
			  begin
				state <= S_IDLE;
				sd_sec_read <= 1'b0;
			  end
			default:
			  begin
				state <= S_IDLE;
				sd_sec_read <= 1'b0;
			  end
		endcase
end

//例化乘法器，计算 CONSTANT*universe_num
point_mul u_point_mul
(
    .clk   (clk         ),  
    .a     (CONSTANT    ),  
    .b     (universe_num),     
    .ce    (  ),  
	 .sclr  (  ),
	 .p     (product     )
);

endmodule