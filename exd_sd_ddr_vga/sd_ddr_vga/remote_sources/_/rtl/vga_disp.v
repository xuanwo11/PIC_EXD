`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name:    vga_test 
//
//////////////////////////////////////////////////////////////////////////////////
module vga_disp(
			input vga_clk,
	      input vga_rst,                 
	      input [127:0] ddr_data,        //DDR�е�ͼ������			

			output vga_hsync,
			output vga_vsync,
			output [4:0] vga_r,
			output [5:0] vga_g,
			output [4:0] vga_b,
         output reg ddr_addr_set,      //ddr����ַ��λ�ź�
	      output reg ddr_rden,
			output reg [4:0] ddr_r_reg,
			output reg [5:0] ddr_g_reg,
			output reg [4:0] ddr_b_reg,
			output reg ddr_rd_cmd,
			output reg hsync_de
    );
//-----------------------------------------------------------//
// ˮƽɨ��������趨1024*768 60Hz VGA
//-----------------------------------------------------------//
parameter LinePeriod =1344;            //��������
parameter H_SyncPulse=136;             //��ͬ�����壨Sync a��
parameter H_BackPorch=160;             //��ʾ���أ�Back porch b��
parameter H_ActivePix=1024;            //��ʾʱ��Σ�Display interval c��
parameter H_FrontPorch=24;             //��ʾǰ�أ�Front porch d��
parameter Hde_start=296;
parameter Hde_end=1320;

//-----------------------------------------------------------//
// ��ֱɨ��������趨1024*768 60Hz VGA
//-----------------------------------------------------------//
parameter FramePeriod =806;           //��������
parameter V_SyncPulse=6;              //��ͬ�����壨Sync o��
parameter V_BackPorch=29;             //��ʾ���أ�Back porch p��
parameter V_ActivePix=768;            //��ʾʱ��Σ�Display interval q��
parameter V_FrontPorch=3;             //��ʾǰ�أ�Front porch r��
parameter Vde_start=35;
parameter Vde_end=803;

//-----------------------------------------------------------//
// ˮƽɨ��������趨800*600 VGA
//-----------------------------------------------------------//
//parameter LinePeriod =1056;           //��������
//parameter H_SyncPulse=128;            //��ͬ�����壨Sync a��
//parameter H_BackPorch=88;             //��ʾ���أ�Back porch b��
//parameter H_ActivePix=800;            //��ʾʱ��Σ�Display interval c��
//parameter H_FrontPorch=40;            //��ʾǰ�أ�Front porch d��

//-----------------------------------------------------------//
// ��ֱɨ��������趨800*600 VGA
//-----------------------------------------------------------//
//parameter FramePeriod =628;           //��������
//parameter V_SyncPulse=4;              //��ͬ�����壨Sync o��
//parameter V_BackPorch=23;             //��ʾ���أ�Back porch p��
//parameter V_ActivePix=600;            //��ʾʱ��Σ�Display interval q��
//parameter V_FrontPorch=1;             //��ʾǰ�أ�Front porch r��


  reg[10 : 0] x_cnt;
  reg[9 : 0]  y_cnt;
  reg[4 : 0] vga_r_reg;
  reg[5 : 0] vga_g_reg;
  reg[4 : 0] vga_b_reg;  

  
  reg hsync_r;
  reg vsync_r; 
  //reg hsync_de;
  reg vsync_de;
  
 
  reg [127:0] ddr_data_reg;               //ddr���������ݴ洢
  reg [3:0] num_counter;       
		  
  reg vsync_buf1;
  reg vsync_buf2; 
  

//----------------------------------------------------------------
////////// ˮƽɨ�����
//----------------------------------------------------------------
always @ (posedge vga_clk)
       if(1'b0)    x_cnt <= 1;
       else if(x_cnt == LinePeriod) x_cnt <= 1;
       else x_cnt <= x_cnt+ 1;
		 
//----------------------------------------------------------------
////////// ˮƽɨ���ź�hsync,hsync_de����
//----------------------------------------------------------------
always @ (posedge vga_clk)
   begin
       if(1'b0) hsync_r <= 1'b1;
       else if(x_cnt == 1) hsync_r <= 1'b0;             //����hsync�ź�
       else if(x_cnt == H_SyncPulse) hsync_r <= 1'b1;
		 
		 		 
	    if(1'b0) hsync_de <= 1'b0;
       else if(x_cnt == Hde_start) hsync_de <= 1'b1;    //����hsync_de�ź�
       else if(x_cnt == Hde_end) hsync_de <= 1'b0;	
		 
	    if(vsync_de) begin
             if((x_cnt == 100) | (x_cnt == 750))        //�������ط���������DDR busrt������
     				  ddr_rd_cmd <= 1'b1;    
				 else
     				  ddr_rd_cmd <= 1'b0; 
       end 				     
       else 
     			 ddr_rd_cmd <= 1'b0;  			 
				 
 
	end

//----------------------------------------------------------------
////////// ��ֱɨ�����
//----------------------------------------------------------------
always @ (posedge vga_clk)
       if(1'b0) y_cnt <= 1;
       else if(y_cnt == FramePeriod) y_cnt <= 1;
       else if(x_cnt == LinePeriod) y_cnt <= y_cnt+1;

//----------------------------------------------------------------
////////// ��ֱɨ���ź�vsync, vsync_de����
//----------------------------------------------------------------
always @ (posedge vga_clk)
  begin
       if(1'b0) vsync_r <= 1'b1;
       else if(y_cnt == 1) vsync_r <= 1'b0;    //����vsync�ź�
       else if(y_cnt == V_SyncPulse) vsync_r <= 1'b1;
		 
	    if(1'b0) vsync_de <= 1'b0;
       else if(y_cnt == Vde_start) vsync_de <= 1'b1;    //����vsync_de�ź�
       else if(y_cnt == Vde_end) vsync_de <= 1'b0;	 
  end
		 

//----------------------------------------------------------------
////////// ddr��ַ��λ�������
//---------------------------------------------------------------- 
always @(posedge vga_clk)
begin
   if (vga_rst) begin
	    vsync_buf1<=1'b0;
		 vsync_buf2<=1'b0;
	    ddr_addr_set<=1'b0;
     end
   else begin
		 vsync_buf1<=vsync_r;
		 vsync_buf2<=vsync_buf1;
       if (vsync_buf2&~vsync_buf1)      //���vsync���½���,ddr�ĵ�ַ��λ
		   ddr_addr_set<=1'b1;
		 else
		   ddr_addr_set<=1'b0;		   
	   end
end

//----------------------------------------------------------------
////////// ddr�������źŲ�������	, 128bit��DDR����ת��8���������
//---------------------------------------------------------------- 
 always @(negedge vga_clk)
 begin
   if (vga_rst) begin
		 ddr_data_reg<=128'd0;
		 vga_r_reg<=5'd0;
		 vga_g_reg<=6'd0;
		 vga_b_reg<=5'd0;
		 num_counter<=4'b0000;
		 ddr_rden<=1'b0;   
   end
   else begin
    if (hsync_de && vsync_de) begin             //���LCD�����Ч��ͼ������
		       case(num_counter)
			    4'b0000:begin 
                  vga_r_reg<=ddr_data_reg[127:123]; //��N��������1��9....)
                  vga_g_reg<=ddr_data_reg[122:117];
                  vga_b_reg<=ddr_data_reg[116:112];	 
						num_counter<=4'b0001;
						ddr_rden<=1'b1;                           //ddr����������
						end
			    4'b0001:begin                                  //��N+1��������2��10....)
				      vga_r_reg<=ddr_data_reg[111:107];
                  vga_g_reg<=ddr_data_reg[106:101];
                  vga_b_reg<=ddr_data_reg[100:96];		 
						num_counter<=4'b0010;
						ddr_rden<=1'b0; 
                  end						
			    4'b0010:begin                                 //��N+2��������3��11....)
                  vga_r_reg<=ddr_data_reg[95:91];
                  vga_g_reg<=ddr_data_reg[90:85];
                  vga_b_reg<=ddr_data_reg[84:80];
						num_counter<=4'b0011;	
						ddr_rden<=1'b0;
						end
			    4'b0011:begin                                //��N+3��������4��12....)
                  vga_r_reg<=ddr_data_reg[79:75];
                  vga_g_reg<=ddr_data_reg[74:69];
                  vga_b_reg<=ddr_data_reg[68:64];		 
						num_counter<=4'b0100;	
						ddr_rden<=1'b0;	
                  end						
			    4'b0100:begin                                //��N+5��������5��13....)
                  vga_r_reg<=ddr_data_reg[63:59];
                  vga_g_reg<=ddr_data_reg[58:53];
                  vga_b_reg<=ddr_data_reg[52:48];
						num_counter<=4'b0101;	
						ddr_rden<=1'b0;
                  end		
			    4'b0101:begin                                //��N+6��������6��14....)
                  vga_r_reg<=ddr_data_reg[47:43];
                  vga_g_reg<=ddr_data_reg[42:37];
                  vga_b_reg<=ddr_data_reg[36:32];
						num_counter<=4'b0110;	
						ddr_rden<=1'b0;
                  end	
			    4'b0110:begin                                //��N+7��������7��15....)
                  vga_r_reg<=ddr_data_reg[31:27];
                  vga_g_reg<=ddr_data_reg[26:21];
                  vga_b_reg<=ddr_data_reg[20:16];
						num_counter<=4'b0111;	
						ddr_rden<=1'b0;
                  end	
			    4'b0111:begin                                //��N+8��������8��16....)
                  vga_r_reg<=ddr_data_reg[15:11];          
                  vga_g_reg<=ddr_data_reg[10:5];
                  vga_b_reg<=ddr_data_reg[4:0];	
			         ddr_data_reg<=ddr_data;                 //ddr���ݸı�						
						num_counter<=4'b0000;
						ddr_rden<=1'b0;
                  end					
            default:begin
					  vga_r_reg<=5'd0;                    
                 vga_g_reg<=6'd0;
                 vga_b_reg<=5'd0;
					  num_counter<=4'b0000;	
					  ddr_rden<=1'b0;
					  end
				endcase;
		end
		else begin
				vga_r_reg<=5'd0;                    
            vga_g_reg<=6'd0;
            vga_b_reg<=5'd0;
				num_counter<=4'b0000;	
				ddr_rden<=1'b0;
			   ddr_data_reg<=ddr_data;                     //ddr���ݸı�
		end
	 end
end



  assign vga_hsync = hsync_r;
  assign vga_vsync = vsync_r;  
  assign vga_r = (hsync_de & vsync_de)?vga_r_reg:5'b00000;
  assign vga_g = (hsync_de & vsync_de)?vga_g_reg:6'b000000;
  assign vga_b = (hsync_de & vsync_de)?vga_b_reg:5'b00000;

	 
	  

endmodule
