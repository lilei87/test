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
	input   wire                transfer_start_flag,         //sdramͻ��д��д��һ֡���ݺ����ź�
	output wire[31:0]           udp_num,                     //һ֡���ݵ�udp���������ݵ���image_data.v��
	input                       find,
	input                       sd_init_done,                //SD card initialization completed
	output reg[3:0]             state_code,                  //state indication coding,
															               // 0:SD card is initializing,
															               // 1:wait for the button to press//�������Ը�Ϊ��������ʱ���͵�ƽ
															               // 2:looking for the bmp file
															               // 3:reading SD card
	output reg[31:0]            bmp_len_cnt,                 // ��������
	input[31:0]                 bmp_width,                   //search the width of bmp
	output reg                  write_req,                   //start writing request
	input                       write_req_ack,               //write request response
	output reg                  sd_sec_read,                 //SD card sector read
	output reg[31:0]            sd_sec_read_addr,            //SD card sector read address
	output reg[31:0]            sd_sec_read_addr_reg1,       //�������ԣ�S_READ״̬��������һ֡��ȡ��ϣ���������ַ��ֵ���˼Ĵ���
	output reg[31:0]            frame_end,                   //�������ԣ���ǰ֡������Ч�Ľ����������ݶεĽ�����
	output reg[31:0]            frame_len,                   //֡����
	input[7:0]                  sd_sec_read_data,            //SD card sector read data
	input                       sd_sec_read_data_valid,      //SD card sector read data valid
	input                       sd_sec_read_end,             //SD card sector read end
	output reg                  bmp_data_wr_en,              //bmp image data write enable
	output reg[15:0]            bmp_data,                    //bmp image data
	output reg[1:0]             write_addr_index,            //sdramд��ַ
	output reg[1:0]             read_addr_index,             //sdram����ַ
	output wire[31:0]           frame_cnt,                   //��дsdram��֡�ļ�����,��ʾ��ǰ֡���,��1��ʼ
	output wire[31:0]           frame_interval_wire          //һ֡���ݴӿ�ʼ���͵�������ɵ�ʱ��������������  //��������
);
localparam S_IDLE         = 0;
localparam S_FIND         = 1;
localparam S_FIND2        = 2;
localparam S_READ_WAIT    = 3;
localparam S_READ         = 4;
localparam S_END          = 5;

localparam INTERVAL_40MS  = 32'd4_000_000;  //��λ��clk�����ٸ�ʱ�����ڣ��˳������Թ̶�֡��25֡����֡����98��������
localparam BASE           = 32'd24_000;
localparam CONSTANT       = 32'd40_000;

reg              transfer_start_flag_dly0;   // transfer_start_flag��һ��
reg              transfer_start_flag_dly1;   // transfer_start_flag������
wire             transfer_start;             // transfer_start_flag_dly1���½��ز������źţ�sdram��ʼд��һ֡

reg[31:0]        frame_start;                //��ǰ֡������Ч����ʼ�������ݶε���ʼ��
reg[31:0]        frame_cnt0;                 //��дsdram��֡�ļ�����,��ʾ��ǰ֡���,��1��ʼ
wire[31:0]       frame_act_len;              //��Ч�ļ�һ֡�ĳ���+֡ͷ8�ֽ�
wire[31:0]       sd_sec_num;                 //��ǰ֡����֮ǰ����֡��������ռ������������ʵ�ʴӵ�Ч�ļ���һ�ֽ����𣩣��Ǳ���
reg[31:0]        sd_sec_read_addr_reg0;      //�ҵ���Ч�ļ�ʱ����������ַ��ֵ���˼Ĵ��������ǵ�Ч�ļ�����ʼ������ַ
reg[31:0]        sd_sec_read_addr_reg2;      //�Ĵ����õ���������ַ���ǵ�ǰ֡��ȡ��Ч���ݵ�������ʼ��ַ(*keep="true"*)
wire[11:0]       diff_size;                  //��ֵ��С���Ǳ�������Э��ȷ����ǰ֡����Ч�����ݶε�������ַ

reg[31:0]        frame_rate;                 //֡�ʣ�һ���ӷ��Ͷ���֡����,��Χ��0~100
wire[31:0]       ini_value;                  //frame_interval�������ĳ�ʼֵ,wire��,�����ж�
wire[31:0]       ini_value0;                 //���Ը�ֵ
reg[31:0]        frame_num;                  //��Ч�ļ���֡����
reg[31:0]        file_in_len; 
reg[31:0]        universe_num;               //��һ֡���ݵģ�������
wire[63:0]       product;                    //����CONSTANT*universe_num
reg              found2;                     //�ڶ�֡��֮���֡����������־�ź�:���ɼ�����rd_frame_cnt��ӿ���
reg[31:0]        rd_frame_cnt;               //֡����ȴ��ȶ��ļ��������ݶ�511�Լ���1��ʱ��Σ���Ϊ��һ֡��Ҫ�ĸ��������ܼ������
reg[14:0]        wait_cnt;                   //frame_cnt0���յ�transfer_start�źź��ӳ�һ��ʱ�䣬ȷ��״̬����frame_end�仯֮ǰ�ܹ�����S_READ״̬
reg[31:0]        frame_interval;             //һ֡���ݴӿ�ʼ���͵�������ɵ�ʱ��������������

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

//��Ч�ļ�һ֡�ĳ���+֡ͷ8�ֽ�
assign frame_act_len = 32'd8 + frame_len;
                         
//����512��ȡ������ʾ��ǰ֡(��֮ǰ֡)������ռ�������������ӵ�Ч�ļ���һ�ֽ����𣩣��Ǳ���//�ų���һ֡����������
assign sd_sec_num    = (frame_cnt0 > 0 )? (sd_sec_read_addr_reg1 - sd_sec_read_addr_reg0 + 32'd1): 1'b0;                            
//��ֵ��С���Ǳ�������Э��ȷ����ǰ֡����Ч�����ݶε�������ַ
assign diff_size     = (frame_cnt0 > 0 )? ((sd_sec_num << 9) - file_in_len): 1'b0;   
     
assign udp_num       = universe_num;
assign frame_cnt     = frame_cnt0;
assign frame_interval_wire = frame_interval;
assign ini_value     = INTERVAL_40MS - BASE - product;//frame_interval�������ĳ�ʼֵ
assign ini_value0    = (ini_value > 32'd99)? ini_value : 32'd99;
           
//remove the first N bytes of the file����֡�����ͣ�һ�η���1֡ //ȥ���ļ�ͷN�ֽ�֮��֮�������Ϊ��Ч����
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1 && bmp_len_cnt >= frame_start && bmp_len_cnt < frame_end);
                                                                                               
//wait_cnt:frame_cnt0���յ�transfer_start�źź��ӳ�һ��ʱ�䣬ȷ��״̬����frame_end�仯֮ǰ�ܹ�����S_READ״̬
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
		wait_cnt   <= 15'd32000;     //��SD��һ��������Ҫ��100Mʱ���£�16384��clk�����޸�32000����
	   end
	  end		                         
	else if(state == S_READ && wait_cnt >= 15'd2) //ֻҪ����S_READ״̬��wait_cnt���Լ�����
	   wait_cnt <= wait_cnt - 1'b1;		
   else if(state != S_READ && wait_cnt >= 15'd2)
	   wait_cnt <= 15'd1;		
	else
		wait_cnt <= 15'd0;
end

//frame_cnt0:��дsdram��֡�ļ�����(�������wait_cnt�й�)
always@(posedge clk or posedge rst)
begin
    if(rst == 1'b1)
	  begin
		frame_cnt0     <= 32'd0;                                                             //��1֡����
	   file_in_len    <= 32'd0;
	  end
	 else if((wait_cnt == 15'd1) && (frame_cnt0 < frame_num) && (frame_cnt0 != 32'd0))      //��3֡��֮���֡//����"frame_cnt0 == frame_num"
	  begin
		frame_cnt0     <= frame_cnt0 + 32'd1;                                                //������2֡д��sdram��frame_cnt0����ֵΪframe_num
	   file_in_len    <= file_in_len + frame_act_len;
	  end
	 else if((wait_cnt == 15'd1) && (frame_cnt0 < frame_num) && (frame_cnt0 == 32'd0))      //��2֡
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

//frame_interval:һ֡���ݴӿ�ʼ���͵�������ɵ�ʱ��������������//����Ŀ�ģ�chipscope�ɿ�
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

//transfer_start_flag��2��
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

//transfer_start:transfer_start_flag_dly1�ź��½���//���������أ�
assign  transfer_start = ((transfer_start_flag_dly1 == 1'b0) && (transfer_start_flag_dly0 == 1'b1));

//���㵱ǰ֡����ʼ������ַ����ǰ֡����Ч�����ݷ�Χ��frame_start��frame_end
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
		sd_sec_read_addr_reg2 <= 0;                                                 //��һ֡���ݵ�������ַ���ڡ�������Ч�ļ�ͷ��Ϣ���и���
		end
	 else if((diff_size > 32'd20)&&( frame_cnt0> 32'd0)&& (frame_cnt0< frame_num)&& (state != S_READ)) //�ڶ�֡��֮���֡//diff_size��Χ22~512,246�ֿ��ܣ�ż�����п����ԣ�	
	   begin
	   frame_start  <= 32'd532 - diff_size;
	   frame_end    <= 32'd512 + frame_act_len - diff_size;
		sd_sec_read_addr_reg2 <= sd_sec_read_addr_reg1 ;                            //sd_sec_read_addr_reg��ʾ��ʼ������ַ�������ַ�Ѻ�1��������ע�� 
	   end
	 else if((diff_size <= 32'd20)&&( frame_cnt0> 32'd0)&&(frame_cnt0< frame_num)&& (state != S_READ)) //diff_size��Χ2~20,10�ֿ��ܣ�ż�����п����ԣ�
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

//rd_frame_cnt:֡�л��ȴ�����������ɣ��ȴ�������,��֤ÿ��25֡��֡��
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

//״̬������S_FIND2��found2��ֵΪ1
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		found2 <= 1'b0;
	else if(state == S_FIND2 && ( frame_cnt0> 32'd0) && (frame_cnt0< frame_num))
	   found2 <= 1'b1;
	else if(state != S_FIND2)
		found2 <= 1'b0;
end

//����write_addr_index��	read_addr_index��д��һ֡��һ֡������д��һ֡	 
always@(posedge clk or posedge rst)
begin
     if(rst == 1'b1) 
	    begin
       write_addr_index <= 2'd0;
		 read_addr_index  <= 2'd0;
		 end
	  else if((transfer_start == 1'b1)&&(frame_cnt0< frame_num))//ȷ���������һ֡�Ķ���ַ
		 begin
		 write_addr_index <= write_addr_index + 2'd1;
		 read_addr_index  <= write_addr_index       ;//read_addr_indexҪ��write_addr_indexһ֡
		 end	  
	  else
	    begin
		 write_addr_index <= write_addr_index ;
		 read_addr_index  <= read_addr_index  ;
		 end
end

//�����ļ�ͷ������
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		rd_cnt <= 10'd0;
	else if(state == S_FIND)//����S_FIND״̬��ʼ����
	begin
		if(sd_sec_read_data_valid == 1'b1)//�����������ݶ��ź���Ч
			rd_cnt <= rd_cnt + 10'd1;
		else if(sd_sec_read_end == 1'b1)
			rd_cnt <= 10'd0;
	end
	else
		rd_cnt <= 10'd0;
end

//������Ч�ļ�ͷ��Ϣ
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
		if(rd_cnt == 10'd499 && (header_0 == 8'hB2) && (header_1 == 8'h76 )&& (header_2 == 8'hAA) && (header_3 == 8'h89 )&& width == bmp_width)//�޸ļ���ֵ
			begin
			found <= 1'b1;
			sd_sec_read_addr_reg0 <= sd_sec_read_addr;   //��Ч�ļ�����ʼ������ַ
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
	else if(state == S_READ)                 //����S_READ״̬�ſ�ʼ����
	begin
		if(sd_sec_read_data_valid == 1'b1)    //�����������ݶ��ź���Ч    
			bmp_len_cnt <= bmp_len_cnt + 32'd1;
	end
	else if(state == S_END)
		bmp_len_cnt <= 32'd0;
end

//bmp_len_cnt_tmp = 1 ʱ�����൱�����ݵ���Ч��־�źţ�bmp_data_valid������ʵ�������ϵ�������Ч��־�ź�
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
		bmp_data_wr_en <= 1'b0;     //��write_en��Ϊ0
	   end
	end
end

//״̬����״̬ת��
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		sd_sec_read <= 1'b0;
		sd_sec_read_addr <= 32'd600;//��ʼ����������ַ
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
				if((rd_frame_cnt == 32'd1) && (frame_cnt0 < frame_num))//д�����һ֡�������ٽ���S_FIND2
				   begin
					state <= S_FIND2;
				   sd_sec_read_addr <= sd_sec_read_addr_reg2;
					end
				else if(find == 1'b1)  //һֱ��⵽find��Ϊ1
					begin
					state <= S_FIND;
				   sd_sec_read_addr <= {sd_sec_read_addr[31:3],3'd0};//address 8 aligned,�����ǣ�ֻΪ��֤��3λ��000
					end
			   end
			end
			S_FIND2:
			begin
			   state_code <= 4'd2;
			   if(sd_sec_read_end == 1'b1)
				begin
			     if(found2 == 1'b1)//��������ʼ������ S_READ_WAIT ������
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
					if(found == 1'b1)//��������ʼ������ S_READ_WAIT ������
					begin
						state <= S_READ_WAIT;
						sd_sec_read <= 1'b0;
						write_req <= 1'b1;//start writing data 
					end								
					else
					begin
						//search every 8 sectors(4K)��ÿ������8������
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
					sd_sec_read_addr <= sd_sec_read_addr + 32'd1;//��ʼ����һ������					
					sd_sec_read <= 1'b0;
					if(bmp_len_cnt >= frame_end)                 //������������
					begin
						state <= S_END;
						sd_sec_read <= 1'b0;
						sd_sec_read_addr_reg1 <= sd_sec_read_addr;//��������ַ��ֵ���Ĵ���,һ֡�������
					end
				end
				else
				begin
					sd_sec_read <= 1'b1;                         //��sd����ʼ
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

//�����˷��������� CONSTANT*universe_num
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