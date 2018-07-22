`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name:    ddr_rw 
//////////////////////////////////////////////////////////////////////////////////
module ddr_rw #
(

   parameter C3_NUM_DQ_PINS          = 16,    
                                       // External memory data width
   parameter C3_MEM_ADDR_WIDTH       = 14,       
                                       // External memory address width
   parameter C3_MEM_BANKADDR_WIDTH   = 3        
                                       // External memory bank address width
)
    
(			  
   input clk_50M,
	input reset_n,
	output spi_clk,
	output vga_clk,

	//sd 读模块接口信号
                           
	input [7:0] mydata,                   //从flash读出的数据
   input myvalid,                        //字节有效信号
	input pic_read_done,	                 //读SD的图像数据完成信号	
	
	//VGA 模块接口信号
	input ddr_rd_cmd,
	input ddr_addr_set,                   //ddr 的地址复位
   input ddr_rden,                       //vga读ddr图像数据请求
	output reg [127:0] ddr_data,          //ddr图像数据输出	
   
	output reg lbp_finish,
	output reg lbp_wrend_flag,
   //MIG39 控制器输出状态信号
   output c3_p0_wr_underrun,          
	output c3_p0_rd_overflow,
	output c3_p0_wr_full,
	output c3_p0_cmd_full, 
   //DDR的接口信号
   inout  [C3_NUM_DQ_PINS-1:0]                      mcb3_dram_dq,     
   output [C3_MEM_ADDR_WIDTH-1:0]                   mcb3_dram_a,      
   output [C3_MEM_BANKADDR_WIDTH-1:0]               mcb3_dram_ba,
   output                                           mcb3_dram_ras_n,
   output                                           mcb3_dram_cas_n,
   output                                           mcb3_dram_we_n,
   output                                           mcb3_dram_odt,
   output                                           mcb3_dram_cke,
   output                                           mcb3_dram_reset_n,
   output                                           mcb3_dram_dm,
   inout                                            mcb3_dram_udqs,
   inout                                            mcb3_dram_udqs_n,
   inout                                            mcb3_rzq,
   inout                                            mcb3_zio,
   output                                           mcb3_dram_udm,
   input                                            c3_sys_clk,
   output                                           c3_rst0,
   inout                                            mcb3_dram_dqs,
   inout                                            mcb3_dram_dqs_n,
   output                                           mcb3_dram_ck,
   output                                           mcb3_dram_ck_n,
   output reg [5:0]                                 ddr_read_state,                         //for chipscope debug
   output reg [29:0]		                            c3_p0_cmd_byte_addr,                    //for chipscope debug
   output reg			           	                   c3_p0_cmd_en,                           //for chipscope debug
   output reg [2:0]			                         c3_p0_cmd_instr,                        //for chipscope debug
   output reg [5:0]			                         c3_p0_cmd_bl,                           //for chipscope debug
   output   				                            c3_p0_rd_en,                            //for chipscope debug
	output     [127:0]	                            c3_p0_rd_data,
	output                                        	 c3_clk0
    );


wire c3_calib_done;
wire pic_store_done; 
assign pic_store_done=(pic_read_done)&(!ddr_write_busy);

 
//寄存器用于存储从sd读出的数据
reg [127:0] sd_data_reg;
reg [127:0] ddr_wdata_reg;

reg [9:0] counter;
reg [7:0] counter1;

reg ddr_write_busy;
reg ddr_rd_busy;

reg ddr_wren;
reg ddr_wr_req;
reg ddr_wren_reg1;
reg ddr_wren_reg2;

reg ddr_rden_req;
reg ddr_rden_reg1;
reg ddr_rden_reg2;

reg ddr_rd_cmd_req;
reg ddr_rd_cmd_reg1;
reg ddr_rd_cmd_reg2;

reg first_read;

//DDR读写的状态寄存器
reg [5:0] ddr_write_state;


parameter write_idle=6'b000000;
parameter write_fifo=6'b000001;
parameter write_data_done=6'b000010;
parameter write_cmd_start=6'b000011;
parameter write_cmd=6'b000100;
parameter write_done=6'b000101;

parameter read_idle=6'b000110;
parameter read_cmd_start=6'b000111;
parameter read_cmd=6'b001000;
parameter read_wait=6'b001001;
parameter read_data=6'b001010;
parameter read_done=6'b001011;

parameter write_exd_idle=6'b100000;
parameter write_exd_fifo=6'b100001;
parameter write_exd_done=6'b100010;
parameter write_exdcmd_start=6'b100011;
parameter write_exdcmd=6'b100100;
parameter write_exdcmd_done=6'b100101;
parameter write_exd_stay=6'b101101;

parameter read_exd_idle=6'b100110;
parameter read_exdcmd_start=6'b100111;
parameter read_exdcmd=6'b101000;
parameter read_exd_wait=6'b101001;
parameter read_exd_data=6'b101010;
parameter read_exd_done=6'b101011;
parameter read_exd_stay=6'b101100;

//ddr user interface

wire				c3_p0_cmd_empty;
wire				c3_p0_cmd_full;

reg				c3_p0_wr_en;
reg[15:0]	   c3_p0_wr_mask;
reg[127:0]	   c3_p0_wr_data;
wire				c3_p0_wr_full;
wire				c3_p0_wr_empty;
wire[6:0]		c3_p0_wr_count;
wire				c3_p0_wr_underrun;
wire				c3_p0_wr_error;



wire				c3_p0_rd_full;
wire				c3_p0_rd_empty;
wire[6:0]		c3_p0_rd_count;
wire				c3_p0_rd_overflow;
wire				c3_p0_rd_error;

reg            c3_p0_lbprd_en;
/*****************************************************************************/
////////////////////////LBP
/*****************************************************************************/
parameter exd_n = 15;//2048,2048/16-1=127
parameter exd_m = 126;//768,512
//parameter lbp_addr = 1572864;

reg [31:0] exd_i;
reg [31:0] exd_j;

reg exd_rd_on;
reg exd_rd_off;
reg exd_wr_on;
reg exd_wr_off;
reg exd_rd_flag;
reg exd_rd_flag1;
reg exd_rd_flag2;
reg exd_wr_flag;
reg exd_wr_flag1;
reg exd_wr_flag2;

reg [29:0] exd_ddr_addr [3:0];
reg [127:0] exd_ddr_data [3:0];
reg [4:0] exd_code_data [17:0];
reg [5:0] exd_code_datwo [17:0];
reg [4:0] exd_code_dathree [17:0];

reg [15:0] exd_code_cnt;
reg exd_wr_data_sign;
reg [7:0] exd_cnt;
reg exd_cal_sign;
reg [3:0] exd_wr_cnt;

reg exd_rd_state;
reg exd_wr_state;
reg exd_wr_data_on;
reg exd_wr_data_off;

reg [4:0] i_B;
reg [5:0] i_G;
reg [4:0] i_R;
reg [4:0] j_B;
reg [5:0] j_G;
reg [4:0] j_R;
reg [4:0] u_B;
reg [5:0] u_G;
reg [4:0] u_R;
		
reg [31:0] exd_addr;
reg [15:0] exdt_code_cnt;
reg [127:0] exd_line [3:0];
reg [127:0] exd_line_one;
reg [127:0]	exd_line_two;
reg [127:0] exd_line_three;
reg [127:0] exd_line_four;
reg [127:0] exdt_ddr_addr [3:0];

/*****************************************************************************/
//读取16个字节(8个像素)的sd数据转换为128bit的数据存在data寄存器中
/*****************************************************************************/
always @(negedge spi_clk)
begin
	if(c3_rst0 || !c3_calib_done)	begin
			counter1<=10'd0;
			ddr_wren<=1'b0;
			sd_data_reg<=0;
			ddr_wdata_reg<=0;
	end		
   else if (myvalid)
      begin		  
			  if(counter1<15) begin                             //读取前15个sd数据	  
					 sd_data_reg <= {sd_data_reg[119:0],mydata};    //119
					 counter1 <= counter1+1;
					 ddr_wren <= 1'b0;
			  end
			  else begin                                        //读取第16个sd数据
					 ddr_wdata_reg <= {sd_data_reg[119:0],mydata};//{sd_data_reg[119:0],mydata};
					 sd_data_reg <= 0;
					 counter1 <= 0;
					 ddr_wren <= 1'b1;                        //接收到16个bytes数据,产生ddr写信号
			  end
		end
   else 
      ddr_wren<=1'b0;
	  
end
	  
/*****************************************************************************/
//脉宽转化,ddr_wren--->ddr_wr_req
/*****************************************************************************/
always @(negedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done)  begin
	    ddr_wren_reg1<=1'b0;
	    ddr_wren_reg2<=1'b0;
	    ddr_wr_req<=1'b0;
	end
   else begin
	  	 ddr_wren_reg1<=ddr_wren;
	    ddr_wren_reg2<=ddr_wren_reg1;   
	    if(ddr_wren_reg1 && !ddr_wren_reg2)           //如果检测到ddr_wren的上升沿,产生ddr写请求
		   ddr_wr_req<=1'b1;
		 else
		   ddr_wr_req<=1'b0;
	end
end
 
/*****************************************************************************/
//DDR读数据请求信号脉宽处理程序: ddr_rden->ddr_rden_req
/*****************************************************************************/
always @(posedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done) begin
     ddr_rden_reg1<=1'b0;
     ddr_rden_reg2<=1'b0; 
	  ddr_rden_req<=1'b0;	
   end
   else begin
     ddr_rden_reg1<=ddr_rden;
     ddr_rden_reg2<=ddr_rden_reg1;
     if(ddr_rden_reg1 && !ddr_rden_reg2)           //如果检测到ddr_rden的上升沿,产生ddr读数据请求
		   ddr_rden_req<=1'b1;
	  else
		   ddr_rden_req<=1'b0;	
    end
end	 

assign c3_p0_rd_en = (lbp_finish == 1'b1)?ddr_rden_req:c3_p0_lbprd_en;

/*****************************************************************************/
//DDR burst读命令请求信号脉宽处理程序: ddr_rd_cmd->ddr_rd_cmd_req
/*****************************************************************************/
always @(negedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done) begin
     ddr_rd_cmd_reg1<=1'b0;
     ddr_rd_cmd_reg2<=1'b0; 
	  ddr_rd_cmd_req<=1'b0;	
   end
   else begin
     ddr_rd_cmd_reg1<=ddr_rd_cmd;
     ddr_rd_cmd_reg2<=ddr_rd_cmd_reg1;
     if(ddr_rd_cmd_reg1 && !ddr_rd_cmd_reg2)           //ddr_rd_cmd,产生ddr burst读命令请求
		   ddr_rd_cmd_req<=1'b1;
	  else
		   ddr_rd_cmd_req<=1'b0;	
    end
end
 
/*****************************************************************************/
//把Ram寄存器的16字节的数据写入ddr中
/*****************************************************************************/
reg [7:0] delay_cnt;
always @(posedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done) begin  			 
     c3_p0_wr_en<=1'b0;
	  c3_p0_wr_mask<=16'd0;
	  c3_p0_wr_data<=128'd0;
	  ddr_write_busy <=1'b0;
     c3_p0_cmd_en<=1'b0;
     c3_p0_cmd_instr<=3'd0;
     c3_p0_cmd_bl<=6'd0;
     c3_p0_cmd_byte_addr<=30'd0;
     ddr_write_state<=write_idle;
     ddr_rd_busy <=1'b0;
     
	  exd_ddr_data[0] <= 128'd0;
	  exd_ddr_data[1] <= 128'd0;
	  exd_ddr_data[2] <= 128'd0;
	  exd_ddr_data[3] <= 128'd0;
	  exd_rd_off <= 1'b0;
	  
	  exd_cnt <= 8'd0;
	  exd_wr_cnt <= 4'd0;
     exd_wr_off <= 1'b0;
	  
	  c3_p0_lbprd_en <= 0;
	  lbp_wrend_flag <= 1'b0;
	  delay_cnt <= 8'd0;
    end
  else begin
  case(ddr_write_state)
		write_idle:begin			  
            c3_p0_wr_en<=1'b0;
	         c3_p0_wr_mask<=16'd0;
				if(ddr_wr_req) begin                             //如果写DDR请求信号为高				
					   ddr_write_busy<=1'b1;                //ddr写数据忙标志
						ddr_write_state<=write_fifo;
						c3_p0_wr_data<=ddr_wdata_reg;        //准备写入DDR的数据
				end
				else begin
				 if(pic_store_done) 
				 begin
				    if(lbp_finish == 1'b0)
					 begin
					     if(exd_wr_on == 1'b1)
				        begin
						    delay_cnt <= 0;
				          exd_wr_off <= 1'b0;
				          ddr_write_state<=write_exd_idle;
							 
						  end
						  else if(exd_rd_on == 1'b1)
				        begin
						    delay_cnt <= 0;
				          exd_rd_off <= 1'b0;
				          ddr_write_state<=read_exd_idle;
				        end
					 end
					 else begin
						if(ddr_addr_set==1'b1) begin             
							c3_p0_cmd_byte_addr<=30'd16;	           //ddr的地址置位, 初始值为地址16 
						end
						else begin
							if(ddr_rd_cmd_req==1'b1) begin              //如果有ddr读命令请求
							ddr_write_state<=read_cmd_start;
							ddr_rd_busy <=1'b1;
							end
						end
					 end
				 end
				end
	   end
		write_fifo:begin	  
	         if(!c3_p0_wr_full) begin                      //如p0写fifo数据不满				
						c3_p0_wr_en<=1'b1;    
				      ddr_write_state<=write_data_done;
				end			   
		end
      write_data_done:begin
			  	c3_p0_wr_en<=1'b0;
			   ddr_write_state<=write_cmd_start;
      end
		write_cmd_start:begin
            c3_p0_cmd_en<=1'b0;                    
            c3_p0_cmd_instr<=3'b010;                  //010为写命令
            c3_p0_cmd_bl<=6'd0;                       //burst length为1个128bit数据
            c3_p0_cmd_byte_addr<=c3_p0_cmd_byte_addr+16;	   //地址加16
				ddr_write_state<=write_cmd;
      end
      write_cmd:begin
			   if (!c3_p0_cmd_full) begin                        //如果命令FIFO不满				  
                 c3_p0_cmd_en<=1'b1;                   //写命令使能
				     ddr_write_state<=write_done;
				end
      end
      write_done:begin
            c3_p0_cmd_en<=1'b0;
            ddr_write_state<=write_idle;
            ddr_write_busy<=1'b0;
      end
		
		read_cmd_start:begin
				c3_p0_cmd_en<=1'b0;
				c3_p0_cmd_instr<=3'b001;               //命令字为读
				c3_p0_cmd_bl<=6'd63;                   //64个数据读
				ddr_write_state<=read_cmd; 
		end						 
		read_cmd:begin			
				c3_p0_cmd_en<=1'b1;                    //ddr读命令使能
				ddr_write_state<=read_done;
		end
		read_done:begin
				c3_p0_cmd_en<=1'b0; 
				ddr_rd_busy <=1'b0;
				c3_p0_cmd_byte_addr<=c3_p0_cmd_byte_addr+1024;    //ddr的读地址加1024 (64*128bit/8)
				first_read<=1'b0;
				ddr_write_state<=write_idle;
		end
		
      //////////////////////////////////////////////////LBP
      read_exd_idle:begin
			   c3_p0_cmd_en<=1'b0;
            c3_p0_cmd_instr<=3'd0;
            c3_p0_cmd_bl<=6'd0;
				c3_p0_lbprd_en <= 0;
            //c3_p0_cmd_byte_addr_lbpr<=30'd16;
				//if(delay_cnt <= 5)
				//begin
				//   delay_cnt <= delay_cnt + 1;
				//end
				//else begin
				   if(exd_cnt <= 3)
				   begin
			      ddr_write_state<=read_exdcmd_start;
               delay_cnt <= 0;
				   end
				   else if(exd_cnt > 3)
				   begin
				   exd_rd_off <= 1'b1;
				   exd_cnt <= 0;
					ddr_write_state<=write_idle;
					delay_cnt <= 0;
				   end
				//end
	   end
		read_exdcmd_start:begin
		      c3_p0_cmd_byte_addr <= 30'd16 + exd_ddr_addr[exd_cnt];
			   c3_p0_cmd_en<=1'b0;
				c3_p0_cmd_instr<=3'b001;              //命令字为读
				c3_p0_cmd_bl<=6'd0;                   //4个数据读
				ddr_write_state<=read_exdcmd;
		end
		read_exdcmd:begin	
            ///if (!c3_p0_cmd_full)
            ///begin	
            exd_cnt <= exd_cnt + 1;				
			   c3_p0_cmd_en<=1'b1;                    //ddr读命令使能
				ddr_write_state<=read_exd_done;
				///end
		end
		/*read_exd_stay:begin
		      c3_p0_cmd_en<=1'b0;                    //ddr读命令使能
				if(!c3_p0_rd_empty)
				begin
				ddr_write_state<=read_exd_done;
				end
		end*/
		read_exd_done:begin
		      c3_p0_lbprd_en <= 1;
				c3_p0_cmd_en<=1'b0;			
				//c3_p0_cmd_byte_addr_lbpr<=c3_p0_cmd_byte_addr_lbpr+1024;    //ddr的读地址加1024 (64*128bit/8)
				ddr_write_state<=read_exd_stay;
		end
		read_exd_stay:begin
		      c3_p0_lbprd_en <= 0;
				c3_p0_cmd_en<=1'b0;
				//if (c3_p0_rd_empty)
				//begin
				exd_ddr_data[exd_cnt-1] <= c3_p0_rd_data;	
				ddr_write_state<=read_exd_idle;
				//end
		end
		
		//////////////////////////////////////////////////LBP
      write_exd_idle:begin			  
            c3_p0_wr_en<=1'b0;
	         c3_p0_wr_mask<=16'd0;
				c3_p0_cmd_en <= 0;
				c3_p0_cmd_instr <= 0;
				c3_p0_cmd_bl <= 0;
				c3_p0_cmd_byte_addr <=0;
				if(delay_cnt <= 2)
				begin
				   delay_cnt <= delay_cnt + 1;
				end
				else begin
					if(exd_wr_cnt < 4)
					begin
					   delay_cnt <= 0;
						ddr_write_state <= write_exd_fifo;
					end
					else begin
					   delay_cnt <= 0;
						exd_wr_off <= 1'b1;
						exd_wr_cnt <= 0;
						ddr_write_state<=write_idle;
					end
			   end
	   end
		write_exd_fifo:begin			
	         if(!c3_p0_wr_full) begin                      //如p0写fifo数据不满	
                  c3_p0_wr_en<=1'b1;				
						c3_p0_wr_data <= exd_line[exd_wr_cnt];//lbp_ddr_wrdata;//准备写入DDR的数据
						c3_p0_cmd_instr<=3'b010;                  //010为写命令
                  c3_p0_cmd_bl<=6'd0;                       //burst length为1个128bit数据
                  c3_p0_cmd_byte_addr<= exdt_ddr_addr[exd_wr_cnt];	   //地址加16
				      ddr_write_state <= write_exdcmd;
				end			   
		end
      /*write_exd_done:begin
			  	c3_p0_wr_en<=1'b0;			
			   ddr_write_state<=write_exdcmd_start;
      end
		write_exdcmd_start:begin
            c3_p0_cmd_en<=1'b0;                    
            c3_p0_cmd_instr<=3'b010;                  //010为写命令
            c3_p0_cmd_bl<=6'd0;                       //burst length为1个128bit数据
            c3_p0_cmd_byte_addr<= exd_ddr_addr[exd_wr_cnt-1];	   //地址加16	
				ddr_write_state<=write_exdcmd;
				
      end*/
      write_exdcmd:begin
			   c3_p0_wr_en<=1'b0;
				if (!c3_p0_cmd_full)
				begin                        //如果命令FIFO不满  	                    					
			   c3_p0_cmd_en<=1'b1;
				///if (c3_p0_wr_empty)
			   ///begin      				
				   ddr_write_state<=write_exd_done;
					if(exd_i == 1)
				   begin
				   lbp_wrend_flag <= 1;//
				   end
				end
				///end
      end
      write_exd_done:begin
            c3_p0_cmd_en<=1'b0;
				//lbp_wrend_flag <= (exd_i == 1)?1'b1:1'b0;
            ddr_write_state<=write_exd_idle;            
				exd_wr_cnt <= exd_wr_cnt + 1;
      end
		
      //////////////////////////////////////////////////LBP		
		default:begin		
		      c3_p0_wr_en<=1'b0;
            c3_p0_cmd_en<=1'b0;
            c3_p0_cmd_instr<=3'd0; 
            c3_p0_cmd_bl<=6'd0;
            ddr_write_state<=write_idle;
      end				  
      endcase;			
   end
end  

/*****************************************************************************/
///////////////////////////////LBP
/*****************************************************************************/
always @(posedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done) 
	begin
		
		exd_i <= 32'd0;//768
		exd_j <= 32'd0;//2048
		
		i_B <= 5'd0;
		i_G <= 6'd0;
		i_R <= 5'd0;
		j_B <= 5'd0;
		j_G <= 6'd0;
		j_R <= 5'd0;
		u_B <= 5'd0;
		u_G <= 6'd0;
		u_R <= 5'd0;
		
		exd_addr <= 266512;
		exdt_code_cnt <= 9;
		exd_line[0] <= 0;
		exd_line[1] <= 0;
		exd_line[2] <= 0;
		exd_line[3] <= 0;
		exd_line_one <= 0;
		exd_line_two <= 0;
		exd_line_three <= 0;
		exd_line_four <= 0;
		exd_ddr_addr[0] <= 0;
		exd_ddr_addr[1] <= 0;
		exd_ddr_addr[2] <= 0;
		exd_ddr_addr[3] <= 0;

		exd_rd_on <= 1'b0;
		exd_rd_flag <= 1'b0;
		exd_rd_flag1 <= 1'b0;
		exd_rd_flag2 <= 1'b0;
		exd_wr_flag <= 1'b1;
		exd_wr_flag1 <= 1'b0;
		exd_wr_flag2 <= 1'b0;
		
		exd_ddr_addr[0] <= 30'd0;
		exd_ddr_addr[1] <= 30'd0;
		exd_ddr_addr[2] <= 30'd0;
		exd_ddr_addr[3] <= 30'd0;
		
		exd_code_cnt <= 16'd0;
		exd_wr_data_on <= 1'b0;	
		exd_wr_data_off <= 1'b0;
		exd_cal_sign <= 1'b0;
		exd_wr_on <= 1'b0;
		lbp_finish <= 1'b0;
		exd_rd_state <= 1'b0;
		exd_wr_state <= 1'b0;		
		
		//lbp_wrend_flag <= 0;//
	end
	else begin
	   if(pic_store_done) 
		begin
		   if(exd_i <= exd_m && exd_wr_flag == 1'b1 && exd_j < exd_n)
			begin 
				exd_ddr_addr[2] <= (exd_i<<11) + (exd_j<<4);
				exd_ddr_addr[3] <= (exd_i<<11) + ((exd_j+1)<<4);
				exd_ddr_addr[0] <= ((exd_i+1)<<11) + (exd_j<<4);
				exd_ddr_addr[1] <= ((exd_i+1)<<11) + ((exd_j+1)<<4);
				exd_rd_on <= 1'b1;
				exd_rd_state <= 1'b1;
				exd_wr_flag <= 1'b0;
				
			end
			else if(exd_i <= exd_m && exd_wr_flag == 1'b1 && exd_j == exd_n)
			begin
			   exd_ddr_addr[2] <= (exd_i<<11) + (exd_j<<4);
				exd_ddr_addr[3] <= (exd_i<<11) + ((exd_j)<<4);
				exd_ddr_addr[0] <= ((exd_i+1)<<11) + (exd_j<<4);
				exd_ddr_addr[1] <= ((exd_i+1)<<11) + ((exd_j)<<4);
				exd_rd_on <= 1'b1;
				exd_rd_state <= 1'b1;
				exd_wr_flag <= 1'b0;
			end
			else if(exd_i > exd_m && exd_wr_flag == 1'b1 && exd_j <= exd_n)
			begin
			   //lbp_finish <= 1'b1;
				exd_j <= exd_j + 1;
				exd_i <= 32'd0;
				exd_wr_flag <= 1'b1;
				
				//lbp_wrend_flag <= 1;//
				exd_addr <= 266512 + ((exd_j+1)<<5);
				exd_code_cnt <= 16'd0;
				exdt_code_cnt <= 9;
				
				exd_line[0] <= 0;
				exd_line[1] <= 0;
				exd_line[2] <= 0;
				exd_line[3] <= 0;
				exd_line_one <= 0;
				exd_line_two <= 0;
				exd_line_three <= 0;
				exd_line_four <= 0;
				exdt_ddr_addr[0] <= 0;
				exdt_ddr_addr[1] <= 0;
				exdt_ddr_addr[2] <= 0;
				exdt_ddr_addr[3] <= 0;
				i_B <= 5'd0;
		      i_G <= 6'd0;
		      i_R <= 5'd0;
		      j_B <= 5'd0;
		      j_G <= 6'd0;
		      j_R <= 5'd0;
				u_B <= 5'd0;
				u_G <= 6'd0;
				u_R <= 5'd0;
            exd_ddr_addr[0] <= 30'd0;
		      exd_ddr_addr[1] <= 30'd0;
		      exd_ddr_addr[2] <= 30'd0;
		      exd_ddr_addr[3] <= 30'd0;
			end
			else if(exd_j > exd_n)
			begin
			   lbp_finish <= 1'b1;
				
			end
			if(exd_cnt == 1)
			begin
			   exd_rd_on <= 1'b0;
			end
			if(exd_wr_cnt == 2)
			begin
			   exd_wr_on <= 1'b0;
			end
         exd_rd_flag1 <= exd_rd_off;
			//exd_rd_flag2 <= exd_rd_flag1;
			if(!exd_rd_flag1 && exd_rd_off)//上升沿检测
			begin
			   
				exd_rd_flag <= 1'b1;
				exd_rd_state <= 1'b0;
				exd_code_data[0] <= (exd_ddr_data[0]>>112);//5 bit
				exd_code_data[1] <= (exd_ddr_data[0]>>96);
				exd_code_data[2] <= (exd_ddr_data[0]>>80);
				exd_code_data[3] <= (exd_ddr_data[0]>>64);
				exd_code_data[4] <= (exd_ddr_data[0]>>48);
				exd_code_data[5] <= (exd_ddr_data[0]>>32);
				exd_code_data[6] <= (exd_ddr_data[0]>>16);
				exd_code_data[7] <= exd_ddr_data[0];
				exd_code_data[8] <= (exd_ddr_data[1]>>112);
				exd_code_data[9] <= (exd_ddr_data[2]>>112);
				exd_code_data[10] <= (exd_ddr_data[2]>>96);
				exd_code_data[11] <= (exd_ddr_data[2]>>80);
				exd_code_data[12] <= (exd_ddr_data[2]>>64);
				exd_code_data[13] <= (exd_ddr_data[2]>>48);
				exd_code_data[14] <= (exd_ddr_data[2]>>32);
				exd_code_data[15] <= (exd_ddr_data[2]>>16);
				exd_code_data[16] <= exd_ddr_data[2];
				exd_code_data[17] <= (exd_ddr_data[3]>>112);
				
				exd_code_datwo[0] <= (exd_ddr_data[0]>>117);//6 bit
				exd_code_datwo[1] <= (exd_ddr_data[0]>>101);
				exd_code_datwo[2] <= (exd_ddr_data[0]>>85);
				exd_code_datwo[3] <= (exd_ddr_data[0]>>69);
				exd_code_datwo[4] <= (exd_ddr_data[0]>>53);
				exd_code_datwo[5] <= (exd_ddr_data[0]>>37);
				exd_code_datwo[6] <= (exd_ddr_data[0]>>21);
				exd_code_datwo[7] <= (exd_ddr_data[0]>>5);
				exd_code_datwo[8] <= (exd_ddr_data[1]>>117);
				exd_code_datwo[9] <= (exd_ddr_data[2]>>117);
				exd_code_datwo[10] <= (exd_ddr_data[2]>>101);
				exd_code_datwo[11] <= (exd_ddr_data[2]>>85);
				exd_code_datwo[12] <= (exd_ddr_data[2]>>69);
				exd_code_datwo[13] <= (exd_ddr_data[2]>>53);
				exd_code_datwo[14] <= (exd_ddr_data[2]>>37);
				exd_code_datwo[15] <= (exd_ddr_data[2]>>21);
				exd_code_datwo[16] <= (exd_ddr_data[2]>>5);
				exd_code_datwo[17] <= (exd_ddr_data[3]>>117);
				
				exd_code_dathree[0] <= (exd_ddr_data[0]>>123);//5 bit
				exd_code_dathree[1] <= (exd_ddr_data[0]>>107);
				exd_code_dathree[2] <= (exd_ddr_data[0]>>91);
				exd_code_dathree[3] <= (exd_ddr_data[0]>>75);
				exd_code_dathree[4] <= (exd_ddr_data[0]>>59);
				exd_code_dathree[5] <= (exd_ddr_data[0]>>43);
				exd_code_dathree[6] <= (exd_ddr_data[0]>>27);
				exd_code_dathree[7] <= (exd_ddr_data[0]>>11);
				exd_code_dathree[8] <= (exd_ddr_data[1]>>123);
				exd_code_dathree[9] <= (exd_ddr_data[2]>>123);
				exd_code_dathree[10] <= (exd_ddr_data[2]>>107);
				exd_code_dathree[11] <= (exd_ddr_data[2]>>91);
				exd_code_dathree[12] <= (exd_ddr_data[2]>>75);
				exd_code_dathree[13] <= (exd_ddr_data[2]>>59);
				exd_code_dathree[14] <= (exd_ddr_data[2]>>43);
				exd_code_dathree[15] <= (exd_ddr_data[2]>>27);
				exd_code_dathree[16] <= (exd_ddr_data[2]>>11);
				exd_code_dathree[17] <= (exd_ddr_data[3]>>123);
				
			end
			if(exd_rd_flag == 1'b1)
			begin
			   if(exd_code_cnt <= 7)
			   begin
					i_B <= (exd_code_data[exd_code_cnt]>>1) + (exd_code_data[exdt_code_cnt]>>1);
					i_G <= (exd_code_datwo[exd_code_cnt]>>1) + (exd_code_datwo[exdt_code_cnt]>>1);
					i_R <= (exd_code_dathree[exd_code_cnt]>>1) + (exd_code_dathree[exdt_code_cnt]>>1);
					
					j_B <= (exd_code_data[exd_code_cnt]>>2) + (exd_code_data[exd_code_cnt+1]>>2) + (exd_code_data[exdt_code_cnt]>>2) + (exd_code_data[exdt_code_cnt+1]>>2);
					j_G <= (exd_code_datwo[exd_code_cnt]>>2) + (exd_code_datwo[exd_code_cnt+1]>>2) + (exd_code_datwo[exdt_code_cnt]>>2) + (exd_code_datwo[exdt_code_cnt+1]>>2);
					j_R <= (exd_code_dathree[exd_code_cnt]>>2) + (exd_code_dathree[exd_code_cnt+1]>>2) + (exd_code_dathree[exdt_code_cnt]>>2) + (exd_code_dathree[exdt_code_cnt+1]>>2);
					
					u_B <= (exd_code_data[exdt_code_cnt]>>1) + (exd_code_data[exdt_code_cnt+1]>>1);//8
               u_G <= (exd_code_datwo[exdt_code_cnt]>>1) + (exd_code_datwo[exdt_code_cnt+1]>>1);					
					u_R <= (exd_code_dathree[exdt_code_cnt]>>1) + (exd_code_dathree[exdt_code_cnt+1]>>1);
					
					exd_rd_flag <= 1'b0;
					exd_cal_sign <= 1'b1;		
				end
				else begin
				   exd_code_cnt <= 0;
					exdt_code_cnt <= 9;
				   exd_wr_on <= 1'b1;
					exd_wr_state <= 1'b1;
					exd_line[0] <= exd_line_one;
					exd_line[1] <= exd_line_two;
					exd_line[2] <= exd_line_three;
					exd_line[3] <= exd_line_four;
					exdt_ddr_addr[0] <= exd_addr;//266256
				   exdt_ddr_addr[1] <= exd_addr + 16;
				   exdt_ddr_addr[2] <= exd_addr + 2048;
				   exdt_ddr_addr[3] <= exd_addr + 2064;
					exd_addr <= exd_addr + 4096;
				   exd_rd_flag <= 1'b0;
				   exd_cal_sign <= 1'b0;
					
				end
			end
			if(exd_cal_sign == 1'b1)
		   begin
			    if(exd_code_cnt <= 3)
			    begin
			    exd_line_one <= {exd_line_one[95:0],exd_code_dathree[exdt_code_cnt],exd_code_datwo[exdt_code_cnt],exd_code_data[exdt_code_cnt],u_R,u_G,u_B};//5,6,5
				 exd_line_three <= {exd_line_three[95:0],i_R,i_G,i_B,j_R,j_G,j_B};//5,6,5
				 end
				 else begin
				 exd_line_two <= {exd_line_two[95:0],exd_code_dathree[exdt_code_cnt],exd_code_datwo[exdt_code_cnt],exd_code_data[exdt_code_cnt],u_R,u_G,u_B};//5,6,5
				 exd_line_four <= {exd_line_four[95:0],i_R,i_G,i_B,j_R,j_G,j_B};//5,6,5
				 end
				 exd_cal_sign <= 1'b0;
				 exd_wr_data_on <= 1'b1;
				 exd_code_cnt <= exd_code_cnt + 1;
             exdt_code_cnt <= exdt_code_cnt + 1;				 
		   end
			if(exd_wr_data_on == 1'b1)
			begin
				exd_wr_data_on <= 1'b0;
				exd_wr_data_off <= 1'b1;
			end
			if(exd_wr_data_off == 1'b1)
			begin
				exd_wr_data_off <= 1'b0;
				exd_rd_flag <= 1'b1;
			end
			exd_wr_flag1 <= exd_wr_off;
			//exd_wr_flag2 <= exd_wr_flag1;
			if(!exd_wr_flag1 && exd_wr_off)//上升沿检测
			begin
			   exd_wr_flag <= 1'b1;
				exd_wr_state <= 1'b0;
				exd_i <= exd_i + 1;
				
				//lbp_finish <= 1'b1;
			end
			
		end
	end
end



/*****************************************************************************/
//MIG的DDR控制器程序例化
/*****************************************************************************/
      mig_39_2 #
      (
         .C3_P0_MASK_SIZE                (16),
         .C3_P0_DATA_PORT_SIZE           (128),
         .DEBUG_EN                       (0),   //   = 0, Disable debug signals/controls.
         .C3_MEMCLK_PERIOD               (3200),
         .C3_CALIB_SOFT_IP               ("TRUE"),            // # = TRUE, Enables the soft calibration logic,
         .C3_SIMULATION                  ("FALSE"),           // # = FALSE, Implementing the design.
         .C3_RST_ACT_LOW                 (1),                 // # = 1 for active low reset         change for AX516 board
         .C3_INPUT_CLK_TYPE              ("SINGLE_ENDED"),
         .C3_MEM_ADDR_ORDER              ("ROW_BANK_COLUMN"),
         .C3_NUM_DQ_PINS                 (16),
         .C3_MEM_ADDR_WIDTH              (13),  
         .C3_MEM_BANKADDR_WIDTH          (3)
         )
      mig_37_inst
      (
         .mcb3_dram_dq			                 (mcb3_dram_dq),
         .mcb3_dram_a			                 (mcb3_dram_a), 
         .mcb3_dram_ba			                 (mcb3_dram_ba),
         .mcb3_dram_ras_n			              (mcb3_dram_ras_n),
         .mcb3_dram_cas_n			              (mcb3_dram_cas_n),
         .mcb3_dram_we_n  	                    (mcb3_dram_we_n),
         .mcb3_dram_odt			                 (mcb3_dram_odt),
         .mcb3_dram_reset_n			           (mcb3_dram_reset_n),	
         .mcb3_dram_cke                        (mcb3_dram_cke),
         .mcb3_dram_dm                         (mcb3_dram_dm),
         .mcb3_dram_udqs                       (mcb3_dram_udqs),
         .mcb3_dram_udqs_n	                    (mcb3_dram_udqs_n),
         .mcb3_rzq	                          (mcb3_rzq),
         .mcb3_zio	                          (mcb3_zio),
         .mcb3_dram_udm	                       (mcb3_dram_udm),
         .c3_sys_clk	                          (clk_50M),
         .c3_sys_rst_i	                       (reset_n),                 			
			.c3_calib_done	                       (c3_calib_done),
         .c3_clk0	                             (c3_clk0),                 //User clock
			.spi_clk	                             (spi_clk),                //AX516 board: for spi clock 
		   .vga_clk	                             (vga_clk),                 //AX516 board: for vga clock 
         .c3_rst0	                             (c3_rst0),			
			.mcb3_dram_dqs                        (mcb3_dram_dqs),
			.mcb3_dram_dqs_n	                    (mcb3_dram_dqs_n),
			.mcb3_dram_ck	                       (mcb3_dram_ck),			
			.mcb3_dram_ck_n	                    (mcb3_dram_ck_n),				
			
         // User Port-0 command interface
         .c3_p0_cmd_clk                  (c3_clk0),          //c3_p0_cmd_clk->c3_clk0			
         .c3_p0_cmd_en                   (c3_p0_cmd_en),
         .c3_p0_cmd_instr                (c3_p0_cmd_instr),
         .c3_p0_cmd_bl                   (c3_p0_cmd_bl),
         .c3_p0_cmd_byte_addr            (c3_p0_cmd_byte_addr),
         .c3_p0_cmd_empty                (c3_p0_cmd_empty),
         .c3_p0_cmd_full                 (c3_p0_cmd_full),	
			
         // User Port-0 data write interface 			
         .c3_p0_wr_clk                   (c3_clk0),          //c3_p0_wr_clk->c3_clk0
			.c3_p0_wr_en                    (c3_p0_wr_en),
         .c3_p0_wr_mask                  (c3_p0_wr_mask),
         .c3_p0_wr_data                  (c3_p0_wr_data),
         .c3_p0_wr_full                  (c3_p0_wr_full),
         .c3_p0_wr_empty                 (c3_p0_wr_empty),
         .c3_p0_wr_count                 (c3_p0_wr_count),
         .c3_p0_wr_underrun              (c3_p0_wr_underrun),
         .c3_p0_wr_error                 (c3_p0_wr_error),	
			
         // User Port-0 data read interface 
			.c3_p0_rd_clk                   (c3_clk0),          //c3_p0_rd_clk->c3_clk0
         .c3_p0_rd_en                    (c3_p0_rd_en),
         .c3_p0_rd_data                  (c3_p0_rd_data),
         .c3_p0_rd_full                  (c3_p0_rd_full),			
         .c3_p0_rd_empty                 (c3_p0_rd_empty),
         .c3_p0_rd_count                 (c3_p0_rd_count),
         .c3_p0_rd_overflow              (c3_p0_rd_overflow),
         .c3_p0_rd_error                 (c3_p0_rd_error)

       );

endmodule
