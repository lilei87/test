//****************************************Copyright (c)***********************************//
//ԭ�Ӹ����߽�ѧƽ̨��www.yuanzige.com
//����֧�֣�www.openedv.com
//�Ա����̣�http://openedv.taobao.com
//��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡZYNQ & FPGA & STM32 & LINUX���ϡ�
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           fifo_wr
// Last modified Date:  2019/05/10 13:38:04
// Last Version:        V1.0
// Descriptions:        ��FIFOģ��
//----------------------------------------------------------------------------------------
// Created by:          ����ԭ��
// Created date:        2019/05/10 13:38:14
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module fifo_rd(
    //system clock
    input               clk ,        // ʱ���ź�
    input               rst_n ,      // ��λ�ź�
    //FIFO interface
    input        [7:0]  fifo_dout ,  // ��FIFO����������
    input               almost_full ,// FIFO�����ź�
    input               almost_empty,// FIFO�����ź�
    output  reg         fifo_rd_en   // FIFO��ʹ��
);

//reg define
reg  [1:0]  state           ;  // ����״̬
reg         almost_full_d0  ;  // fifo_full �ӳ�һ��
reg  		almost_full_syn ;  // fifo_full �ӳ�����
reg  [3:0]  dly_cnt         ;  //�ӳټ�����

//*****************************************************
//**                    main code
//*****************************************************

//��Ϊ fifo_full �ź�������FIFOдʱ�����
//����Ҫ����ͬ������ʱ������
always@( posedge clk ) begin
	if( !rst_n ) begin
		almost_full_d0  <= 1'b0 ;
		almost_full_syn <= 1'b0 ;
	end
	else begin
		almost_full_d0  <= almost_full ;
		almost_full_syn <= almost_full_d0 ;
	end
end

//����FIFO������
always @(posedge clk ) begin
    if(!rst_n) begin
        fifo_rd_en <= 1'b0;
		state      <= 2'd0;
		dly_cnt    <= 4'd0;
    end
    else begin
        case(state)
            2'd0: begin
                if(almost_full_syn)      //�����⵽FIFO����д��
                    state <= 2'd1;       //�ͽ�����ʱ״̬
                else
                    state <= state;
            end 
			2'd1: begin
                if(dly_cnt == 4'd10) begin  //��ʱ10��
											//ԭ����FIFO IP���ڲ�״̬�źŵĸ��´�����ʱ
											//�ӳ�10���Եȴ�״̬�źŸ������
                    dly_cnt <= 4'd0;
					state   <= 2'd2;        //��ʼ������
				end
				else
					dly_cnt <= dly_cnt + 4'd1;
            end
		    2'd2: begin
                if(almost_empty) begin     //�ȴ�FIFO��������
                    fifo_rd_en <= 1'b0;    //�رն�ʹ��
                    state      <= 2'd0;    //�ص���һ��״̬
                end
                else                       //���FIFOû�б�����
                    fifo_rd_en <= 1'b1;    //������򿪶�ʹ��
            end 
			default : state <= 2'd0;
        endcase
    end
end

endmodule