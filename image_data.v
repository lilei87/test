`timescale  1ns/1ns
////////////////////////////////////////////////////////////////////////
// Author        : lee@lingluren
// Create Date   : 2021/07/24
// Module Name   : image_data
// Project Name  : sd_sdram_udp
// Target Devices: Xilinx XC6SLX16
// Tool Versions : ISE 14.7
// Description   : ���ݰ�����
// 
// Revision      : V1.0
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////

module  image_data
#(
    parameter   H_PIXEL =   11'd520             ,      //UDP����С��520�ֽ�        
    parameter   CNT_FRAME_WAIT = 27'h06_00      ,      //��֡ͼ��ȴ�ʱ����� 24'h50_FF_FF // 27'h05_F5_E1_00��֡���1��
    parameter   CNT_IDLE_WAIT  = 24'h06_00             //�������ݵȴ�ʱ����� 24'h00_10_99 //24'h00_22_99
)
(
    input   wire            sys_clk         ,    //ϵͳʱ��
    input   wire            sys_rst_n       ,    //��λ�ź�,�͵�ƽ��Ч
	 input   wire    [31:0]  udp_num         ,    //bmp_read.v�������
	 input   wire            transfer_start_flag, //�װ����ݿ�ʼ����ǰ�ļ���������ָʾ��־��sdramͻ��д�Ѿ����
    input   wire    [8:0]   fifo_rd_data_count,  //sdram read_buf fifo ��д��������������16bitΪ1������Ϊ��λ
	 input   wire    [31:0]  image_data      ,    //��SDRAM�ж�ȡ��32λͼ������
	 input   wire            eth_tx_req      ,    //��̫���������������ź�
    input   wire            eth_tx_done     ,    //��̫��������������ź�
    output  reg     [5:0]   state           ,    //״̬��״̬,��������
	 
    input   wire             almost_empty_flag,  // sdram��fifo�ļ����ձ�־
	 input   wire             empty_flag,         // sdram��fifo�Ŀձ�־ //�˳���ʵ�ʲ�����Ҫʹ�ô˲���
	 output  reg              read_req,           // Start reading a frame of data     
	 input                    read_req_ack,       // Read request response
	 output  reg              read_en,            // Read data enable,ȡ���˳���ԭ����data_rd_req
	 output  wire             video_data_valid,   // read_data��������������Ч�ź�
    
	 output  reg     [10:0]  cnt_h           ,    // ��������
	 output  reg             almost_empty_flag_dv,// ��������
//  output  reg             data_rd_req     ,    // ͼ�����������ź� 
    output  reg             eth_tx_start    ,    // ��̫���������ݿ�ʼ�ź�
    output  wire    [31:0]  eth_tx_data     ,    // ��̫����������
    output  reg     [15:0]  eth_tx_data_num      // ��̫������������Ч�ֽ���
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//parameter define
parameter   IDLE        =   6'b000_001, //��ʼ״̬
            FIRST_BAG   =   6'b000_010, //���͵�һ������(������ͷ)
            COM_BAG     =   6'b000_100, //������ͨ������
            LAST_BAG    =   6'b001_000, //�������һ������(����CRC-16);����ҪCRC-16��
            BAG_WAIT    =   6'b010_000, //�������ݷ�����ɵȴ�
            FRAME_END   =   6'b100_000; //һ֡ͼ������ɵȴ�
//				LAST_BAG_WAIT = 6'b1_000_000; //���һ�����꣬cnt_h��������

//wire          video_data_valid ;  //sdram ��fifo��afifo�� �����������Ч
reg             read_en_dly;        //read_en��һ������video_data_valid�ź�
reg     [8:0]   cnt_dv           ;  //read_req����֮�����
//wire  define
wire            fifo_empty      ;   //FIFO�����źţ�fifo_image��
wire            fifo_empty_fall ;   //FIFO�����ź��½��أ�fifo_image��
//wire          rd_data_count   ;   //FIFO���࣬���Զ�����������
reg             transfer_start_flag_dly0;     // transfer_start_flag��һ��
reg             transfer_start_flag_dly1;     // transfer_start_flag������
reg             transfer_start;               // transfer_start_flag_dly1���½��ز������źţ����ڿ�ʼ����

//reg define
reg             data_valid      ;   //ͼ��������Ч�ź�
//reg     [5:0]   state           ;   //״̬��״̬
reg     [23:0]  cnt_idle_wait   ;   //��ʼ״̬����������ȴ�ʱ�����
//reg   [10:0]  cnt_h           ;   //�������ݰ������ظ�������(һ��ͼ��)
reg             data_rd_req     ;   
reg             wr_fifo_en      ;   //FIFOдʹ��
reg     [15:0]  cnt_wr_data     ;   //д��FIFO���ݸ���(��λ��2���4�ֽ�)
reg     [31:0]  wr_fifo_data    ;   //д��FIFO����(��λ��2���4�ֽ�)
reg             fifo_empty_reg  ;   //fifo�����źŴ�һ��
reg     [11:0]  cnt_v           ;   //һ֡ͼ���Ͱ��ĸ���(һ֡ͼ������)      
reg     [26:0]  cnt_frame_wait  ;   //��֡ͼ��ȴ�ʱ�����                    

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
//state:״̬��״̬����
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

//transfer_start_flag��2��
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

//transfer_start_flag_dly1���½��ز������ź�transfer_start�����ڿ�ʼ����
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)	 
		  transfer_start <= 0;
    else if((transfer_start_flag_dly1 == 1'b1)&&(transfer_start_flag_dly0 == 1'b0))
        transfer_start <= 1'b1;		     
	 else if(state == FRAME_END)
	     transfer_start <= 0;   
	 else
	     transfer_start <= transfer_start;
	    
//cnt_idle_wait:��������ȴ�ʱ�����//�װ���ʼ����ͨ�������һ���İ����������
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

//cnt_h:�������ݰ������ظ�������(һ��ͼ��)
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
	 else if((cnt_h != 11'd0) && (cnt_h <= 11'd130)&& (!almost_empty_flag_dv)) //sdram ��fifo��afifo���ǿ�ʱ��cnt_h��1
        cnt_h   <=  cnt_h - 1'b1;
	 else
        cnt_h   <=  cnt_h;

//read_en��sdram������ ��ʹ�� �źţ��Ǳ�ģ�������źţ������������£�
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        read_en     <=  1'b0;
    else if((cnt_h != 11'd0) && (cnt_h <= 11'd130)&& (!almost_empty_flag_dv))
        read_en     <=  1'b1;
	 else
        read_en     <=  1'b0;

//read_req �� read_req_ack
always@(posedge sys_clk or negedge sys_rst_n)    
  begin
	if(sys_rst_n == 1'b0)
		read_req <= 1'b0;	
	else if( state == IDLE  && (cnt_idle_wait == CNT_IDLE_WAIT ) ) 
		read_req <= 1'b1;
	else if(read_req_ack)
		read_req <= 1'b0;
  end

//reg almost_empty_flag_dv;//almost_empty_flag��Ч�ź�
//��read_req�����Ժ�ʼ������Լ15��clk֮�󣬽�almost_empty_flag�ļĴ���ֵ����almost_empty_flag_dv���ڴ�֮ǰ��almost_empty_flag_dvֱ�Ӹ�ֵΪ1
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

//read_en��һ��,����video_data_valid�ź�
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

//wr_fifo_en:FIFOдʹ��
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        wr_fifo_en   <=  1'b0;
    else    if(video_data_valid == 1'b1)
        wr_fifo_en   <=  1'b1;
    else
        wr_fifo_en   <=  1'b0;

//wr_fifo_data:д��FIFO����                                                
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        wr_fifo_data    <=  32'h0;
     else if(state == FIRST_BAG || state == COM_BAG || state == LAST_BAG)	
		  wr_fifo_data    <=  image_data;  
     else
        wr_fifo_data    <=  32'h0;

//fifo_empty:FIFO�����ź�
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        fifo_empty_reg   <=  1'b1;
    else
        fifo_empty_reg   <=  fifo_empty;

//fifo_empty_fall:FIFO�����ź��½���//fifo����������Ϊ0����������Ϊ1����ʼֵΪ1
assign  fifo_empty_fall = ((fifo_empty_reg == 1'b1) && (fifo_empty == 1'b0));

//eth_tx_start:��̫���������ݿ�ʼ�ź�
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        eth_tx_start    <=  1'b0;
    else    if(fifo_empty_fall == 1'b1) 
        eth_tx_start    <=  1'b1;  
    else
        eth_tx_start    <=  1'b0;

//eth_tx_data_num:��̫������������Ч�ֽ���,һ���ֽ�8bit
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        eth_tx_data_num     <=  16'd0;
    else    if(state == FIRST_BAG || state == COM_BAG || state == LAST_BAG)
        eth_tx_data_num       <=   H_PIXEL;      //520�ֽ�
    else
        eth_tx_data_num     <=  eth_tx_data_num;

//cnt_v:һ֡ͼ���Ͱ�����(һ֡ͼ������)
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        cnt_v   <=  12'd0;
    else    if(state == IDLE)
        cnt_v   <=  12'd0;
    else    if(eth_tx_done == 1'b1)
        cnt_v   <=  cnt_v + 1'b1;
    else
        cnt_v   <=  cnt_v;

//cnt_frame_wait:��֡ͼ��ȴ�ʱ�����//frame_start�˲���������δ�õ�
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
