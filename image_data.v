`timescale  1ns/1ns
////////////////////////////////////////////////////////////////////////
// Author        : lee@lingluren
// Create Date   : 2021/07/24
// Module Name   : image_data
// Project Name  : sd_sdram_udp
// Target Devices: Xilinx XC6SLX16
// Tool Versions : ISE 14.7
// Description   : 数据包发送
// 
// Revision      : V1.0
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////

module  image_data
#(
    parameter   H_PIXEL =   11'd520             ,      //UDP包大小：520字节        
    parameter   CNT_FRAME_WAIT = 27'h06_00      ,      //单帧图像等待时间计数 24'h50_FF_FF // 27'h05_F5_E1_00：帧间隔1秒
    parameter   CNT_IDLE_WAIT  = 24'h06_00             //单包数据等待时间计数 24'h00_10_99 //24'h00_22_99
)
(
    input   wire            sys_clk         ,    //系统时钟
    input   wire            sys_rst_n       ,    //复位信号,低电平有效
	 input   wire    [31:0]  udp_num         ,    //bmp_read.v计算而来
	 input   wire            transfer_start_flag, //首包数据开始传输前的计数器计数指示标志，sdram突发写已经完成
    input   wire    [8:0]   fifo_rd_data_count,  //sdram read_buf fifo 已写入数据数量，以16bit为1个数据为单位
	 input   wire    [31:0]  image_data      ,    //自SDRAM中读取的32位图像数据
	 input   wire            eth_tx_req      ,    //以太网发送数据请求信号
    input   wire            eth_tx_done     ,    //以太网发送数据完成信号
    output  reg     [5:0]   state           ,    //状态机状态,引出测试
	 
    input   wire             almost_empty_flag,  // sdram读fifo的几乎空标志
	 input   wire             empty_flag,         // sdram读fifo的空标志 //此程序实际并不需要使用此参数
	 output  reg              read_req,           // Start reading a frame of data     
	 input                    read_req_ack,       // Read request response
	 output  reg              read_en,            // Read data enable,取代此程序原来的data_rd_req
	 output  wire             video_data_valid,   // read_data数据线上数据有效信号
    
	 output  reg     [10:0]  cnt_h           ,    // 引出测试
	 output  reg             almost_empty_flag_dv,// 引出测试
//  output  reg             data_rd_req     ,    // 图像数据请求信号 
    output  reg             eth_tx_start    ,    // 以太网发送数据开始信号
    output  wire    [31:0]  eth_tx_data     ,    // 以太网发送数据
    output  reg     [15:0]  eth_tx_data_num      // 以太网单包数据有效字节数
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
parameter   IDLE        =   6'b000_001, //初始状态
            FIRST_BAG   =   6'b000_010, //发送第一包数据(包含包头)
            COM_BAG     =   6'b000_100, //发送普通包数据
            LAST_BAG    =   6'b001_000, //发送最后一包数据(包含CRC-16);必须要CRC-16吗？
            BAG_WAIT    =   6'b010_000, //单包数据发送完成等待
            FRAME_END   =   6'b100_000; //一帧图像发送完成等待
//				LAST_BAG_WAIT = 6'b1_000_000; //最后一包发完，cnt_h必须清零

//wire          video_data_valid ;  //sdram 读fifo（afifo） 的输出数据有效
reg             read_en_dly;        //read_en延一拍生成video_data_valid信号
reg     [8:0]   cnt_dv           ;  //read_req拉高之后计数
//wire  define
wire            fifo_empty      ;   //FIFO读空信号（fifo_image）
wire            fifo_empty_fall ;   //FIFO读空信号下降沿（fifo_image）
//wire          rd_data_count   ;   //FIFO读侧，可以读的数据数量
reg             transfer_start_flag_dly0;     // transfer_start_flag打一拍
reg             transfer_start_flag_dly1;     // transfer_start_flag打两拍
reg             transfer_start;               // transfer_start_flag_dly1的下降沿产生该信号，网口开始传输

//reg define
reg             data_valid      ;   //图像数据有效信号
//reg     [5:0]   state           ;   //状态机状态
reg     [23:0]  cnt_idle_wait   ;   //初始状态即单包间隔等待时间计数
//reg   [10:0]  cnt_h           ;   //单包数据包含像素个数计数(一行图像)
reg             data_rd_req     ;   
reg             wr_fifo_en      ;   //FIFO写使能
reg     [15:0]  cnt_wr_data     ;   //写入FIFO数据个数(单位由2变成4字节)
reg     [31:0]  wr_fifo_data    ;   //写入FIFO数据(单位由2变成4字节)
reg             fifo_empty_reg  ;   //fifo读空信号打一拍
reg     [11:0]  cnt_v           ;   //一帧图像发送包的个数(一帧图像行数)      
reg     [26:0]  cnt_frame_wait  ;   //单帧图像等待时间计数                    

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
//state:状态机状态变量
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        state   <=  IDLE;
    else    case(state)
        IDLE:
            if(cnt_idle_wait == CNT_IDLE_WAIT)
                state   <=  FIRST_BAG;
            else
                state   <=  IDLE;
        FIRST_BAG:
            if(eth_tx_done == 1'b1)
                state   <=  BAG_WAIT;
            else
                state   <=  FIRST_BAG;
        BAG_WAIT:
            if((cnt_v < udp_num - 12'd1) &&    
                (cnt_idle_wait == CNT_IDLE_WAIT))
                state   <=  COM_BAG;
            else    if((cnt_v == udp_num - 12'd1) && 
                        (cnt_idle_wait == CNT_IDLE_WAIT))
                state   <=  LAST_BAG;
            else    if((cnt_v > udp_num - 12'd1) &&           
                        (cnt_idle_wait == CNT_IDLE_WAIT))
				   state    <=  FRAME_END;
				else
                state   <=  BAG_WAIT;
        COM_BAG:
            if(eth_tx_done == 1'b1)
                state   <=  BAG_WAIT;
            else
                state   <=  COM_BAG;
        LAST_BAG:
            if(eth_tx_done == 1'b1)
                state   <=  FRAME_END;
            else
                state   <=  LAST_BAG;				
        FRAME_END:
            if(cnt_frame_wait == CNT_FRAME_WAIT)
                state   <=  IDLE;
            else
                state   <=  FRAME_END;
        default:state   <=  IDLE;
    endcase

//transfer_start_flag打2拍
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
       begin	 
		  transfer_start_flag_dly0 <= 0;
		  transfer_start_flag_dly1 <= 0;
		 end
    else begin
        transfer_start_flag_dly0 <= transfer_start_flag;
		  transfer_start_flag_dly1 <= transfer_start_flag_dly0;    
	     end

//transfer_start_flag_dly1的下降沿产生该信号transfer_start，网口开始传输
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)	 
		  transfer_start <= 0;
    else if((transfer_start_flag_dly1 == 1'b1)&&(transfer_start_flag_dly0 == 1'b0))
        transfer_start <= 1'b1;		     
	 else if(state == FRAME_END)
	     transfer_start <= 0;   
	 else
	     transfer_start <= transfer_start;
	    
//cnt_idle_wait:单包间隔等待时间计数//首包开始，普通包、最后一包的包间隔计数器
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0) begin
        cnt_idle_wait   <=  24'd0;
        end
    else if(((state == IDLE)||(state == BAG_WAIT)) 
	        && (cnt_idle_wait < CNT_IDLE_WAIT) && (transfer_start == 1'b1)) //20210913
	     begin
        cnt_idle_wait   <=  cnt_idle_wait + 1'b1;
        end
    else begin
        cnt_idle_wait   <=  24'd0;
        end

//cnt_h:单包数据包含像素个数计数(一行图像)
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cnt_h   <=  11'd0;
    else if(cnt_h == 11'd0)
        if((state == IDLE)||(state == BAG_WAIT)||(state == FRAME_END))		  
            cnt_h   <=  11'd131; //cnt_h   <=  H_PIXEL;//H_PIXEL =   11'd512
        else
            cnt_h   <=  cnt_h;
    else if((cnt_h == 11'd131) && (fifo_rd_data_count[8] == 1'b1) && 
	        ((state == FIRST_BAG)||(state == COM_BAG)||(state == LAST_BAG)))
        cnt_h   <=  cnt_h - 1'b1;
	 else if((cnt_h != 11'd0) && (cnt_h <= 11'd130)&& (!almost_empty_flag_dv)) //sdram 读fifo（afifo）非空时，cnt_h减1
        cnt_h   <=  cnt_h - 1'b1;
	 else
        cnt_h   <=  cnt_h;

//read_en是sdram的数据 读使能 信号，是本模块的输出信号，产生过程如下：
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        read_en     <=  1'b0;
    else if((cnt_h != 11'd0) && (cnt_h <= 11'd130)&& (!almost_empty_flag_dv))
        read_en     <=  1'b1;
	 else
        read_en     <=  1'b0;

//read_req 与 read_req_ack
always@(posedge sys_clk or negedge sys_rst_n)    
  begin
	if(sys_rst_n == 1'b0)
		read_req <= 1'b0;	
	else if( state == IDLE  && (cnt_idle_wait == CNT_IDLE_WAIT ) ) 
		read_req <= 1'b1;
	else if(read_req_ack)
		read_req <= 1'b0;
  end

//reg almost_empty_flag_dv;//almost_empty_flag有效信号
//当read_req拉高以后开始计数，约15个clk之后，将almost_empty_flag的寄存器值赋给almost_empty_flag_dv；在此之前，almost_empty_flag_dv直接赋值为1
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        almost_empty_flag_dv   <=  1'b1;		  
    else if( cnt_dv == 9'd1  )
        almost_empty_flag_dv   <=  almost_empty_flag;     
	 else
	     almost_empty_flag_dv   <=  almost_empty_flag_dv;

always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cnt_dv   <=  9'd0;
    else if(read_req_ack == 1'b1)
        cnt_dv   <=  9'd15;
    else if(cnt_dv >= 9'd2) 
        cnt_dv   <=  cnt_dv - 1'b1;
	 else if(cnt_dv == 9'd1)
        cnt_dv   <=  cnt_dv;
	 else
	     cnt_dv   <=  cnt_dv;

assign video_data_valid = read_en_dly;

//read_en打一拍,生成video_data_valid信号
always@(posedge sys_clk or negedge sys_rst_n)    
  begin
	if(sys_rst_n == 1'b0)
	begin
		read_en_dly <= 1'b0;	
	end
	else
	begin
		//delay 1 clock cycles
		read_en_dly <= read_en;		
	end
  end

//wr_fifo_en:FIFO写使能
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        wr_fifo_en   <=  1'b0;
    else    if(video_data_valid == 1'b1)
        wr_fifo_en   <=  1'b1;
    else
        wr_fifo_en   <=  1'b0;

//wr_fifo_data:写入FIFO数据                                                
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        wr_fifo_data    <=  32'h0;
     else if(state == FIRST_BAG || state == COM_BAG || state == LAST_BAG)	
		  wr_fifo_data    <=  image_data;  
     else
        wr_fifo_data    <=  32'h0;

//fifo_empty:FIFO读空信号
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        fifo_empty_reg   <=  1'b1;
    else
        fifo_empty_reg   <=  fifo_empty;

//fifo_empty_fall:FIFO读空信号下降沿//fifo中有数据则为0，无数据则为1，初始值为1
assign  fifo_empty_fall = ((fifo_empty_reg == 1'b1) && (fifo_empty == 1'b0));

//eth_tx_start:以太网发送数据开始信号
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        eth_tx_start    <=  1'b0;
    else    if(fifo_empty_fall == 1'b1) 
        eth_tx_start    <=  1'b1;  
    else
        eth_tx_start    <=  1'b0;

//eth_tx_data_num:以太网单包数据有效字节数,一个字节8bit
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        eth_tx_data_num     <=  16'd0;
    else    if(state == FIRST_BAG || state == COM_BAG || state == LAST_BAG)
        eth_tx_data_num       <=   H_PIXEL;      //520字节
    else
        eth_tx_data_num     <=  eth_tx_data_num;

//cnt_v:一帧图像发送包个数(一帧图像行数)
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cnt_v   <=  12'd0;
    else    if(state == IDLE)
        cnt_v   <=  12'd0;
    else    if(eth_tx_done == 1'b1)
        cnt_v   <=  cnt_v + 1'b1;
    else
        cnt_v   <=  cnt_v;

//cnt_frame_wait:单帧图像等待时间计数//frame_start此参数程序中未用到
reg frame_start;
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0) begin
        cnt_frame_wait  <=  27'd0;
        frame_start<=1'b0;
        end
    else    if((state == FRAME_END) && (cnt_frame_wait < CNT_FRAME_WAIT)) begin
        cnt_frame_wait  <=  cnt_frame_wait + 1'b1;
        if(cnt_frame_wait==27'd1) begin
        frame_start<=1'b1;
        end
        else begin
            frame_start<=1'b0;
        end
    end
    else begin
        cnt_frame_wait  <=  27'd0;
        frame_start<=1'b0;
        end

//********************************************************************//
//*************************** Instantiation **************************//
//********************************************************************//


fifo_image fifo_image_inst (
  .clk      (sys_clk        ), // input clk
  .rst      (~sys_rst_n     ), // input rst
  .din      (wr_fifo_data   ), // input [31 : 0] din
  .wr_en    (wr_fifo_en     ), // input wr_en
  .rd_en    (eth_tx_req     ), // input rd_en
  .dout     (eth_tx_data    ), // output [31 : 0] dout
  .full     (               ), // output full
  .empty    (fifo_empty     )  // output empty   
);

endmodule
