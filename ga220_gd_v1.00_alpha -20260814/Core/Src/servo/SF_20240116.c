#include "main.h"
#include "stm32F4xx_hal.h"
#include "stm32F4xx_hal_gpio.h"
#include "math.h"
#include "SF.h"
#include "app.h"
#include "GEO_Track_C.h"


FWControlTypeDef FWControl;
GDControlTypeDef GDControl;
FWSensorTypeDef FWSensor;
GDSensorTypeDef GDSensor;


extern REC_IMAGE_CMD_U rec_cmd_buffer;
extern REC_FW_CMD_U rec_fw_fy_buff;//接收数据
extern uint32_t WHGBMQ_Data;
extern __IO uint16_t ADC1_CovertedValue[ADC1_CHANEL_NUM * ADC1_COLLECT_NUM];
float gd_i_temp = 0;
//-------------cmd_para--------------//
float fw_v_lead = 0;
float gd_v_lead = 0;	
float fw_a_lead = 0;
float gd_a_lead = 0;	
float fw_p_now = 0;
float gd_p_now = 0;


SevroXWStateTypDef SevroXWState;
FWServoTypedef FWServo;
//FYServoTypedef FYServo;

//--------------变量定义-------------//
uint8_t PTstate = 0x01;//伺服工作模式
char Test_flag = 0;//测试状态

uint32_t  FW_BMQ_DATA;

//使能标志位
uint8_t FW_EN; 
uint8_t FY_EN; 
		 
float Cos_FW = 0.0;//方位角余弦
float Sec_FW = 0.0;//方位角正割
float Sin_FW = 0.0;//方位角正弦
float Csc_FW = 0.0;//方位角余割

char GD_XW_flag = 0,XW_Down_state = 0,XW_UP_state=0;
char Motion_state = 0;//


uint8_t FW_ManualBrake = 0;//手动刹车
uint8_t GD_ManualBrake = 0;
 
//-------------------数据处理------------------//


//--------收藏-------//
uint8_t GD_Withdrawal_flag = 0;
uint8_t FW_Withdrawal_flag = 0;


//--------测试--------//
uint8_t test_en = 0;
uint8_t FW_Loop = 0,FY_Loop = 0;
float U_give=0,I_give=0,Ea_give = 0,Ev_give=0,P_give=0;
float Test_Fre=0;

//---------函数实现--------//
void alldeal(void)
{
	
 //------传感器数据处理------//		
		FW_ManualBrake = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
       
		CurrentInit();
		fwbmqdispose();
		gdbmqdispose();
		fbdispose(); 
	
       if(FW_ManualBrake)
	   {		   
		 allprocess();
	   }
       else	 
	   {
		clr_all();
		all_brake();	   	   	   
	   }		   
	
// //------功能实现----------//
  
//			if(test_en)
//				 {		

//	  //----------------------俯仰调试------------------//		
//						if(FY_Loop == 1 )			 
//		            Test(1,1,0,U_give,Test_Fre,1000);	//电流环开环
//						else if(FY_Loop == 2 )	
//								 Test(1,1,1,I_give,Test_Fre,1000);	//电流环闭环
//						else if(FY_Loop == 3 )	
//								 Test(1,5,1,Ea_give,Test_Fre,1000);	//轴角加速度闭环	
//						else if(FY_Loop == 4 )	
//								 Test(1,6,1,Ev_give,Test_Fre,1000);//俯仰轴角速度闭环	
//						else if(FY_Loop == 5 )	
//								 Test(1,7,1,P_give,Test_Fre,10000);//俯仰位置闭环	

//		//------------------------方位调试-------------------//
//						if(FW_Loop == 1 )
//								 Test(2,1,0,U_give,Test_Fre,1000);//电流环开环
//						else if(FW_Loop == 2 )
//								 Test(2,1,1,I_give,Test_Fre,1000);	//电流环闭环
//						else if(FW_Loop == 3 )
//								   Test(2,6,1,Ev_give,Test_Fre,1000);//轴角速度闭环	
//						else if(FW_Loop == 4 )
//								 	 Test(2,7,1,P_give,Test_Fre,1000);//位置闭环	

//					}
//					else
//					{
//						clr_all();
//						all_brake();
//					}
//	
	}
void Test(char Axis,char Loop,char O_or_C,float Amp,float Freq,float Time)
{
	static uint32_t Test_Count = 0;
	Test_Count++;
	if(Test_Count<=((uint32_t)(Time*1000)))
	{
	 if(Axis==1)//俯仰
	{
		switch(Loop)
		{
		  case(1)://电流环
			{
			  if(O_or_C==0)//开环
				{			  
					if(Freq==0)
						GDControl.U_give = Amp;
					else
						GDControl.U_give = Amp*sin(0.00628*Freq*Test_Count);										
					gd_en();
					Test_flag = 3;
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						GDControl.I_give = Amp;
					else
						GDControl.I_give = Amp*sin(0.00628*Freq*Test_Count);
     					
      GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
			gd_en();	
					Test_flag = 1;
				  }
			  break;
			 }
		  case(2)://加速度环
			{	
				Test_flag = 1;
			 	if(O_or_C==0)//开环
				{
				  if(Freq==0)
						GDControl.I_give = Amp;
					else
						GDControl.I_give = Amp*sin(0.00628*Freq*Test_Count);	
					
					  gd_en();
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						GDControl.A_give = Amp;
					else
						GDControl.A_give = Amp*sin(0.00628*Freq*Test_Count);	
					GDControl.I_give = gdaloop(GDControl.Kp_A,GDControl.Ki_A,GDControl.SamT,GDControl.Kg_A,GDControl.BW_A,GDControl.Imax,GDControl.A_give,GDControl.A_fb);
					gd_en();
				  }
			  break;
			 }
		  case(3)://速度环
			{
			 	Test_flag = 1;
			 	if(O_or_C==0)//开环
				{
				  if(Freq==0)
						GDControl.I_give = Amp;
					else
						GDControl.I_give = Amp*sin(0.00628*Freq*Test_Count);	
					
					  gd_en();
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						GDControl.V_give = Amp;
					else
						GDControl.V_give = Amp*sin(0.00628*Freq*Test_Count);	
							
					GDControl.I_give = gdvloop(GDControl.Kp_V,GDControl.Ki_V,GDControl.SamT,GDControl.Imax,Up_Limit,Down_Limit,GDControl.V_give,GDControl.V_fb,GDSensor.angle);
          GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
					gd_en();						
				  }
			  break;
			 }
		  case(4)://跟踪环
			{
			 	Test_flag = 1;
			 	if(O_or_C==0)//开环
				{
			//----跟踪环不存在开环---------//
				  }
				else if(O_or_C==1)//闭环
				{
//				  GDControl.V_give = gdtloop_TV1(GDControl.Kp_1_T1,GDControl.Ki_1_T1,GDControl.bound_T1,GDControl.Kp_2_T1,GDControl.Ki_2_T1,GDControl.SamT,GDControl.Vmax,Tv_angle_gd);
					GDControl.I_give = gdvloop(GDControl.Kp_V,GDControl.Ki_V,GDControl.SamT,GDControl.Amax,Up_Limit,Down_Limit,GDControl.V_give,GDControl.V_fb,GDSensor.angle);
					GDControl.I_give = gdaloop(GDControl.Kp_A,GDControl.Ki_A,GDControl.SamT,GDControl.Kg_A,GDControl.BW_A,GDControl.Imax,GDControl.A_give,GDControl.A_fb);

					gd_en();
				  }
			  break;
			 }		
		  case(5)://轴角加速度环
			{
			 	Test_flag = 1;
			 	if(O_or_C==0)//开环
				{			  
				  }
				else if(O_or_C==1)//闭环
				{		  
				  if(Freq==0)
						GDControl.Eacc_give = Amp;
					else
						GDControl.Eacc_give = Amp*sin(0.00628*Freq*Test_Count);	

					GDControl.I_give = gdealoop(GDControl.Kp_Eacc,GDControl.Ki_Eacc,GDControl.SamT,GDControl.Imax,GDControl.Eacc_give,GDControl.Eacc_fb,0);		
					gd_en();	
				  }
			  break;
			 }
		  case(6)://轴角速度环
			{
			 	Test_flag = 1;
			 	if(O_or_C==0)//开环
				{
					if(Freq==0)
						GDControl.I_give = Amp;
					else
						GDControl.I_give = Amp*sin(0.00628*Freq*Test_Count);	
					
					gd_en();
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						GDControl.Ev_give = Amp;
					else
						GDControl.Ev_give = Amp*sin(0.00628*Freq*Test_Count);	
					
					GDControl.I_give = gdevloop(GDControl.Kp_Ev,GDControl.Ki_Ev,GDControl.SamT,GDControl.Imax,Up_Limit,Down_Limit,GDControl.Ev_give,GDControl.Ev_fb,GDSensor.angle,0);
          GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
		     	gd_en();		  
					
				  }
			  break;
			 }
		  case(7)://位置环
			{
			 	Test_flag = 1;
			 	if(O_or_C==0)//开环
				{
				//----位置环不存在开环---------//			  
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						GDControl.P_give = Amp;
					else
						GDControl.P_give = Amp*sin(0.00628*Freq*Test_Count);	
          
					GDControl.Ev_give =  gdploop(GDControl.Kp_P,GDControl.Ki_P,GDControl.SamT,Up_Limit,Down_Limit,GDControl.bound_P,GDControl.Ev_Set,GDControl.Evmax,GDControl.P_give,GDControl.P_fb,0);					
			    GDControl.I_give = gdevloop(GDControl.Kp_Ev,GDControl.Ki_Ev,GDControl.SamT,GDControl.Imax,Up_Limit,Down_Limit,GDControl.Ev_give,GDControl.Ev_fb,GDSensor.angle,0);
					GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
					gd_en();		
	
				  }
			  break;
			 }			
		 }
	 }
	else if(Axis==2)//方位
	{
	 switch(Loop)
		{
		  case(1)://电流环
			{
			  if(O_or_C==0)//开环
				{			  
					if(Freq==0)
						FWControl.U_give = Amp;
					else
						FWControl.U_give = Amp*sin(0.00628*Freq*Test_Count);					
					
					fw_en();
					Test_flag = 3;
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						FWControl.I_give = Amp;
					else
						FWControl.I_give = Amp*sin(0.00628*Freq*Test_Count);	
					  FWControl.U_give = fwiloop(FWControl.Kp_I,FWControl.Ki_I,FWControl.SamT_I,FWControl.Umax,FWControl.I_give,FWControl.I_fb);
					fw_en();
					Test_flag = 2;
				  }
			  break;
			 }
		  case(2)://加速度环
			{	
				Test_flag = 2;
			 	if(O_or_C==0)//开环
				{
				  if(Freq==0)
						FWControl.I_give= Amp;
					else
						FWControl.I_give = Amp*sin(0.00628*Freq*Test_Count);	
					
					fw_en();
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						FWControl.A_give = Amp;
					else
						FWControl.A_give = Amp*sin(0.00628*Freq*Test_Count);	
					
					FWControl.I_give = fwaloop(FWControl.Kp_A,FWControl.Ki_A,FWControl.SamT,FWControl.Kg_A,FWControl.BW_A,FWControl.Imax,FWControl.A_give,FWControl.A_fb,fabs(GDSensor.angle),FWControl.bound_A);
					fw_en();
				  }
			  break;
			 }
		  case(3)://速度环
			{
			 	Test_flag = 2;
			 	if(O_or_C==0)//开环
				{
				  if(Freq==0)
						FWControl.I_give = Amp;
					else
						FWControl.I_give = Amp*sin(0.00628*Freq*Test_Count);	
					
					fw_en();
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						FWControl.V_give = Amp;
					else
						FWControl.V_give = Amp*sin(0.00628*Freq*Test_Count);	
							
					FWControl.I_give = fwvloop(FWControl.Kp_V,FWControl.Ki_V,FWControl.SamT,FWControl.Imax,Left_Limit,Right_Limit,FWControl.V_give,FWControl.V_fb,FWSensor.angle,0,FWControl.bound_V);
					FWControl.U_give = fwiloop(FWControl.Kp_I,FWControl.Ki_I,FWControl.SamT_I,FWControl.Umax,FWControl.I_give,FWControl.I_fb);
					fw_en();
				  }
			  break;
			 }
		  case(4)://跟踪环
			{
			 	Test_flag = 2;
			 	if(O_or_C==0)//开环
				{
			//----跟踪环不存在开环---------//
				  }
				else if(O_or_C==1)//闭环
				{
					fw_en();
				  }
			  break;
			 }		
		  case(5)://GEO环
			{
			 	Test_flag = 2;
			 	if(O_or_C==0)//开环
				{			  
			//----GEO环不存在开环---------//
				  }
				else if(O_or_C==1)//闭环
				{		  
				  if(Freq==0)
						FWControl.GEO_give = Amp;
					else
						FWControl.GEO_give = Amp*sin(0.00628*Freq*Test_Count);	

					FWControl.V_give = fwgeo(FWControl.Kp_G,FWControl.Ki_G,FWControl.SamT,FWControl.bound_G,FWControl.V_Set,FWControl.Vmax,FWControl.GEO_give,FWControl.GEO_fb,Sec_FW);
					FWControl.A_give = fwvloop(FWControl.Kp_V,FWControl.Ki_V,FWControl.SamT,FWControl.Amax,Left_Limit,Right_Limit,FWControl.V_give,FWControl.V_fb,FWSensor.angle,GDSensor.angle,FWControl.bound_A);
				  FWControl.I_give = fwaloop(FWControl.Kp_A,FWControl.Ki_A,FWControl.SamT,FWControl.Kg_A,FWControl.BW_A,FWControl.Imax,FWControl.A_give,FWControl.A_fb,GDSensor.angle,FWControl.bound_A);	
				  }
			  break;
			 }
		  case(6)://轴角速度环
			{
			 	Test_flag = 2;
			 	if(O_or_C==0)//开环
				{
					if(Freq==0)
						FWControl.I_give = Amp;
					else
						FWControl.I_give = Amp*sin(0.00628*Freq*Test_Count);	
					
					fw_en();
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						FWControl.Ev_give = Amp;
					else
						FWControl.Ev_give = Amp*sin(0.00628*Freq*Test_Count);	
					
					FWControl.I_give = fwevloop(FWControl.Kp_Ev,FWControl.Ki_Ev,FWControl.SamT,FWControl.Imax,Left_Limit,Right_Limit,FWControl.Ev_give,FWControl.Ev_fb,FWSensor.angle,0);
					FWControl.U_give = fwiloop(FWControl.Kp_I,FWControl.Ki_I,FWControl.SamT_I,FWControl.Umax,FWControl.I_give,FWControl.I_fb);
					fw_en();		  
				  }
			  break;
			 }
		  case(7)://位置环
			{
			 	Test_flag = 2;
			 	if(O_or_C==0)//开环
				{
				//----位置环不存在开环---------//			  
				  }
				else if(O_or_C==1)//闭环
				{
				  if(Freq==0)
						FWControl.P_give = Amp;
					else
						FWControl.P_give = Amp*sin(0.00628*Freq*Test_Count);	
          
					FWControl.Ev_give = fwploop(FWControl.Kp_P,FWControl.Ki_P,FWControl.SamT,Left_Limit,Right_Limit,FWControl.bound_P,FWControl.Ev_Set,FWControl.Evmax,FWControl.P_give,FWControl.P_fb,0);
					FWControl.I_give = fwevloop(FWControl.Kp_Ev,FWControl.Ki_Ev,FWControl.SamT,FWControl.Imax,Left_Limit,Right_Limit,FWControl.Ev_give,FWControl.Ev_fb,FWSensor.angle,0);		
					FWControl.U_give = fwiloop(FWControl.Kp_I,FWControl.Ki_I,FWControl.SamT_I,FWControl.Umax,FWControl.I_give,FWControl.I_fb);
					fw_en();		
					
				  }
			  break;
			 }			
		 }
	  }
	 }
	else
	{
	 clr_all();
	 all_brake();
	} 
}

void commandprocess(void)
{
	switch(rec_cmd_buffer.rec_cmd_struct.cmd)
	{
		case(Brake): 
		{		
			
		 PTstate = P_Brake;				 
		 break;
		}		
		case(VLead):
		{
			fw_v_lead = (int16_t)((rec_cmd_buffer.rec_cmd_struct.data[1]<<8)|rec_cmd_buffer.rec_cmd_struct.data[0])*0.01;
			gd_v_lead = (int16_t)((rec_cmd_buffer.rec_cmd_struct.data[2]<<8)|rec_cmd_buffer.rec_cmd_struct.data[2])*0.01;      
      PTstate = P_VLead;	
		  break;
		}
		case(ALead):
		{
			
      PTstate = P_ALead;	
		  break;
		}
	  case(GEOLead):
		{

      PTstate = P_GEOLead;	 			
		  break;
		}	
		case(Track):
		{

      PTstate = P_Track;	 			
		  break;
		}
		case(Withdrawal):
		{
			
      PTstate = P_Withdrawal;	
		  break;
		}
	  case(LOCK):
		{
			
      PTstate = P_LOCK;	
		  break;
		}		
		case(DELOCK):
		{

			PTstate = P_DELOCK;	
		  break;
		}
		case(CLB):
		{
			PTstate = P_CLB;	
		  break;
		}
		case(STEP):
		{
      fw_p_now = FWControl.P_fb;
      gd_p_now = GDControl.P_fb;	
			
      if(rec_cmd_buffer.rec_cmd_struct.data[0]==1)
				FWControl.P_give = fw_p_now-((int16_t)((rec_cmd_buffer.rec_cmd_struct.data[2]<<8)|rec_cmd_buffer.rec_cmd_struct.data[1])*0.01);
			else if(rec_cmd_buffer.rec_cmd_struct.data[0]==2)
				FWControl.P_give = fw_p_now+((int16_t)((rec_cmd_buffer.rec_cmd_struct.data[2]<<8)|rec_cmd_buffer.rec_cmd_struct.data[1])*0.01);

			if(rec_cmd_buffer.rec_cmd_struct.data[3]==1)
				GDControl.P_give = gd_p_now+((int16_t)((rec_cmd_buffer.rec_cmd_struct.data[5]<<8)|rec_cmd_buffer.rec_cmd_struct.data[4])*0.01);
			else if(rec_cmd_buffer.rec_cmd_struct.data[3]==2)
				GDControl.P_give = gd_p_now-((int16_t)((rec_cmd_buffer.rec_cmd_struct.data[5]<<8)|rec_cmd_buffer.rec_cmd_struct.data[4])*0.01);

			if(FWControl.P_give>180)				
			 FWControl.P_give = FWControl.P_give-360;
			else if(FWControl.P_give<-180)
				FWControl.P_give = FWControl.P_give+360;
			
			if(GDControl.P_give>85)				
			 GDControl.P_give = 85;
			else if(GDControl.P_give<-3)
				GDControl.P_give = -3;			
			
			PTstate = P_STEP;
		  break;
		}		
	}
}

float fwval , fyval;
uint8_t WorkMode = 0xB4;
float fw_miss = 0;
float gd_miss = 0;
float fw_a_lead_test = 0;
//extern float yaw_lxy_miss;
extern int32_t fw_comp;
extern int32_t gd_comp;
void allprocess(void)
{
	switch(PTstate)
	{
		case(P_Brake):  
		{
			WorkMode = 0xB4;			
			clr_fw_all();
			clr_gd_all();
			all_brake();			
      break;
		}
		case(P_VLead):  
		{
			if(rec_cmd_buffer.rec_cmd_struct.cmd != 0)
			{
				fw_v_lead = (int16_t)((rec_cmd_buffer.rec_cmd_struct.data[1]<<8)|rec_cmd_buffer.rec_cmd_struct.data[0]);
			gd_v_lead = (int16_t)((rec_cmd_buffer.rec_cmd_struct.data[3]<<8)|rec_cmd_buffer.rec_cmd_struct.data[2]);
			}
			
			
      if(fw_v_lead>50)
				fw_v_lead = 50;
			else if(fw_v_lead<-50)
				fw_v_lead = -50;

      if(gd_v_lead>50)
				gd_v_lead = 50;
			else if(gd_v_lead<-50)
				gd_v_lead = -50;	
			
      GD_Withdrawal_flag = 0;
			FW_Withdrawal_flag = 0;
			
			clr_gd_g();
			clr_gd_t_1();	
			clr_gd_t_2();	
			clr_gd_t_3();	
			clr_gd_v();
			clr_gd_a();
     
		
			GDControl.Ev_give = gd_v_lead;
			GDControl.I_give = gdevloop(GDControl.Kp_Ev,GDControl.Ki_Ev,GDControl.SamT,GDControl.Imax,Up_Limit,Down_Limit,GDControl.Ev_give,GDControl.Ev_fb,GDSensor.angle,0);
			GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
			gd_en();				
						
			clr_fw_g();
			clr_fw_t_1();	
			clr_fw_t_2();	
			clr_fw_t_3();	
			clr_fw_v();
			clr_fw_a();
          
			FWControl.Ev_give = fw_v_lead;
			FWControl.I_give = fwevloop(FWControl.Kp_Ev,FWControl.Ki_Ev,FWControl.SamT,FWControl.Imax,Left_Limit,Right_Limit,FWControl.Ev_give,FWControl.Ev_fb,FWControl.P_fb,0);
			fw_en();

			WorkMode = 0xB8;                //上报速度运动中
			break;
		}	
	  case(P_ALead):
		{
			if(rec_cmd_buffer.rec_cmd_struct.cmd != 0)
			{
				fw_a_lead = (int16_t)((rec_cmd_buffer.rec_cmd_struct.data[1]<<8)|rec_cmd_buffer.rec_cmd_struct.data[0]);
			gd_a_lead = (int16_t)((rec_cmd_buffer.rec_cmd_struct.data[3]<<8)|rec_cmd_buffer.rec_cmd_struct.data[2]);
			
			}
			
			fyval = gd_a_lead;
			fwval = fw_a_lead;
			if(fw_a_lead>180)
				fw_a_lead = fw_a_lead - 360;
			else if(fw_a_lead < -180)
			  fw_a_lead = fw_a_lead + 360;
			
			if(gd_a_lead>82)
				gd_a_lead = 82;
			else if(gd_a_lead<-3)
				gd_a_lead = -3;
			
			GD_Withdrawal_flag = 0;
			FW_Withdrawal_flag = 0;

			clr_gd_g();
			clr_gd_t_1();	
			clr_gd_t_2();	
			clr_gd_t_3();	
			clr_gd_v();
			clr_gd_a();
			clr_gddg();
			
			GDControl.P_give = gd_a_lead;
			GDControl.Ev_give = gdploop(GDControl.Kp_P,GDControl.Ki_P,GDControl.SamT,Up_Limit+FY_Mech_Angle,Down_Limit+FY_Mech_Angle,GDControl.bound_P,GDControl.Ev_Set,GDControl.Evmax,GDControl.P_give,GDControl.P_fb,0);			
			GDControl.I_give = gdevloop(GDControl.Kp_Ev,GDControl.Ki_Ev,GDControl.SamT,GDControl.Imax,Up_Limit+FY_Mech_Angle,Down_Limit+FY_Mech_Angle,GDControl.Ev_give,GDControl.Ev_fb,GDSensor.angle,0);
			GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
			gd_en();	
		
			clr_fw_g();
			clr_fw_t_1();	
			clr_fw_t_2();	
			clr_fw_t_3();	
			clr_fw_v();
			clr_fw_a();
			clr_fwdg();
						
			FWControl.P_give =  fw_a_lead;
			FWControl.Ev_give = fwploop(FWControl.Kp_P,FWControl.Ki_P,FWControl.SamT,Left_Limit,Right_Limit,FWControl.bound_P,FWControl.Ev_Set,FWControl.Evmax,FWControl.P_give,FWControl.P_fb,0);
			FWControl.I_give = fwevloop(FWControl.Kp_Ev,FWControl.Ki_Ev,FWControl.SamT,FWControl.Imax,Left_Limit,Right_Limit,FWControl.Ev_give,FWControl.Ev_fb,FWControl.P_fb,0);
			fw_en();			
			break;
		}	
	  case(P_GEOLead): 
		{
			if(rec_cmd_buffer.rec_cmd_struct.cmd != 0)
			{
				gd_a_lead = OrientLoad.Orient_Load_FY;//    lxy_out.pitch_deg + (INS.Pitch + (float)gd_comp * 0.01);
				fw_a_lead = OrientLoad.Orient_Load_FW;//lxy_out.yaw_deg - (INS.Yaw + 90 + (float)fw_comp * 0.01);
			}
			
			if(fw_a_lead>180)
				fw_a_lead = fw_a_lead - 360;
			else if(fw_a_lead < -180)
			  fw_a_lead = fw_a_lead + 360;
//			if(fw_a_lead>180)
//				fw_a_lead = 180;
//			else if(fw_a_lead<=-180)
//			  fw_a_lead = -180;
			
			if(gd_a_lead>85)
				gd_a_lead = 85;
			else if(gd_a_lead<-3)
				gd_a_lead = -3;
			
			GD_Withdrawal_flag = 0;
			FW_Withdrawal_flag = 0;

			clr_gd_g();
			clr_gd_t_1();	
			clr_gd_t_2();	
			clr_gd_t_3();	
			clr_gd_v();
			clr_gd_a();
			clr_gddg();
			
			GDControl.P_give = gd_a_lead;
			GDControl.Ev_give = gdploop(GDControl.Kp_P,GDControl.Ki_P,GDControl.SamT,Up_Limit+FY_Mech_Angle,Down_Limit+FY_Mech_Angle,GDControl.bound_P,GDControl.Ev_Set,GDControl.Evmax,GDControl.P_give,GDControl.P_fb,0);			
			GDControl.I_give = gdevloop(GDControl.Kp_Ev,GDControl.Ki_Ev,GDControl.SamT,GDControl.Imax,Up_Limit+FY_Mech_Angle,Down_Limit+FY_Mech_Angle,GDControl.Ev_give,GDControl.Ev_fb,GDSensor.angle,0);
			GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
			gd_en();	
		
			clr_fw_g();
			clr_fw_t_1();	
			clr_fw_t_2();	
			clr_fw_t_3();	
			clr_fw_v();
			clr_fw_a();
			clr_fwdg();
						
		  FWControl.P_give =  fw_a_lead;
			FWControl.Ev_give = fwploop(FWControl.Kp_P,FWControl.Ki_P,FWControl.SamT,Left_Limit,Right_Limit,FWControl.bound_P,FWControl.Ev_Set,FWControl.Evmax,FWControl.P_give,FWControl.P_fb,0);
			FWControl.I_give = fwevloop(FWControl.Kp_Ev,FWControl.Ki_Ev,FWControl.SamT,FWControl.Imax,Left_Limit,Right_Limit,FWControl.Ev_give,FWControl.Ev_fb,FWControl.P_fb,0);
			fw_en();			
		 break;
		}			
		case(P_Track):
		{
			WorkMode = 0xB9;
			
			if(rec_cmd_buffer.rec_cmd_struct.cmd != 0)
			{
				fw_miss = rec_cmd_buffer.rec_cmd_struct.fw_miss_distance*0.01;
				gd_miss = rec_cmd_buffer.rec_cmd_struct.fy_miss_distance*0.01;
			}
			
			if(fw_miss>5)
				fw_miss = 5;
			else if(fw_miss<-5)
				fw_miss = -5;
			if(gd_miss>5)
				gd_miss = 5;
			else if(gd_miss<-5)
				gd_miss = -5;

				clr_gd_g();
				clr_gd_t_2();	
				clr_gd_t_3();	
				clr_gd_v();
				clr_gd_a();
				clr_gddg();
			
				GDControl.Ev_give = gdtloop_TV1(10,5,0.1,10,5,GDControl.SamT,20,gd_miss);//10 5
			  GDControl.I_give = gdevloop(GDControl.Kp_Ev,GDControl.Ki_Ev,GDControl.SamT,GDControl.Imax,Up_Limit,Down_Limit,GDControl.Ev_give,GDControl.Ev_fb,GDSensor.angle,0);
			  GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
			  gd_en();	

			  clr_fw_g();
				clr_fw_t_2();	
				clr_fw_t_3();	
				clr_fw_v();
				clr_fw_a();
				clr_fwdg();
			
        FWControl.Ev_give = fwtloop_TV1(7,10,0.1,7,10,FWControl.SamT,30,fw_miss,GDSensor.Sec_GD);				
				FWControl.I_give = fwevloop(FWControl.Kp_Ev,FWControl.Ki_Ev,FWControl.SamT,FWControl.Imax,Left_Limit,Right_Limit,FWControl.Ev_give,FWControl.Ev_fb,FWControl.P_fb,0);		
				fw_en();						
			break;
		}
	  case(P_Withdrawal): 
		{   			
			clr_gd_g();
			clr_gd_t_1();	
			clr_gd_t_2();	
			clr_gd_t_3();	
			clr_gd_v();
			clr_gd_a();
			clr_gddg();
			
			GDControl.P_give = 0;		
			GDControl.Ev_give = gdploop(GDControl.Kp_P,GDControl.Ki_P,GDControl.SamT,Up_Limit+FY_Mech_Angle,Down_Limit+FY_Mech_Angle,GDControl.bound_P,GDControl.Ev_Set,GDControl.Evmax,GDControl.P_give,GDControl.P_fb,0);
			GDControl.I_give = gdevloop(GDControl.Kp_Ev,GDControl.Ki_Ev,GDControl.SamT,GDControl.Imax,Up_Limit+FY_Mech_Angle,Down_Limit+FY_Mech_Angle,GDControl.Ev_give,GDControl.Ev_fb,GDSensor.angle,0);
			GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
			gd_en();	
		
			clr_fw_g();
			clr_fw_t_1();	
			clr_fw_t_2();	
			clr_fw_t_3();	
			clr_fw_v();
			clr_fw_a();
			clr_fwdg();
			
			FWControl.P_give = 0;
			FWControl.Ev_give = fwploop(FWControl.Kp_P,FWControl.Ki_P,FWControl.SamT,Left_Limit,Right_Limit,5,60,60,FWControl.P_give,FWControl.P_fb,0);
			FWControl.I_give = fwevloop(FWControl.Kp_Ev,FWControl.Ki_Ev,FWControl.SamT,FWControl.Imax,Left_Limit,Right_Limit,FWControl.Ev_give,FWControl.Ev_fb,FWControl.P_fb,0);		
			fw_en();			
		 break;
		}	
		case(P_CLB):  
		{

			break;
		}
		
		case(P_STEP):
		{  

			
			clr_gd_g();
			clr_gd_t_1();	
			clr_gd_t_2();	
			clr_gd_t_3();	
			clr_gd_v();
			clr_gd_a();
			clr_gddg();
				
			GDControl.Ev_give = gdploop(GDControl.Kp_P,GDControl.Ki_P,GDControl.SamT,Up_Limit+FY_Mech_Angle,Down_Limit+FY_Mech_Angle,GDControl.bound_P,GDControl.Ev_Set,GDControl.Evmax,GDControl.P_give,GDControl.P_fb,0);
			GDControl.I_give = gdevloop(GDControl.Kp_Ev,GDControl.Ki_Ev,GDControl.SamT,GDControl.Imax,Up_Limit+FY_Mech_Angle,Down_Limit+FY_Mech_Angle,GDControl.Ev_give,GDControl.Ev_fb,GDSensor.angle,0);
			GDControl.U_give = gdiloop(GDControl.Kp_I,GDControl.Ki_I,GDControl.SamT_I,GDControl.Umax,GDControl.I_give,GDControl.I_fb);
			gd_en();	
		
			clr_fw_g();
			clr_fw_t_1();	
			clr_fw_t_2();	
			clr_fw_t_3();	
			clr_fw_v();
			clr_fw_a();
			clr_fwdg();
			
			FWControl.Ev_give = fwploop(FWControl.Kp_P,FWControl.Ki_P,FWControl.SamT,Left_Limit,Right_Limit,5,60,60,FWControl.P_give,FWControl.P_fb,0);
			FWControl.I_give = fwevloop(FWControl.Kp_Ev,FWControl.Ki_Ev,FWControl.SamT,FWControl.Imax,Left_Limit,Right_Limit,FWControl.Ev_give,FWControl.Ev_fb,FWControl.P_fb,0);		
			fw_en();	
			
		 break;
		}	
	
	
	}
}


void initcontrolpar(void)
{
//-----------高低-----------//
//  GDControl.Kp_I = 0.0032*200000; 
//	GDControl.Ki_I = 200000;  
  GDControl.Kp_I = 0.0025*350000*0.9;   
	GDControl.Ki_I = 350000*0.9;   
	GDControl.SamT_I = 0.001;
	GDControl.Umax = 4000;
	GDControl.I_give = 0.0;
	GDControl.I_fb = 0.0;//电流环控制参数
	
	GDControl.Kp_A = 0.0; 
	GDControl.Ki_A = 0.0;    
	GDControl.SamT = 0.001;
	GDControl.Kg_A = 0.0;
	GDControl.BW_A = 0.0;
	GDControl.Imax = 4;
	GDControl.A_give = 0.0;
	GDControl.A_fb = 0.0;//加速度
	
	GDControl.Kp_V = 0.0;
	GDControl.Ki_V = 0.0;
	GDControl.Amax = 0.0;
  GDControl.V_give = 0;
	GDControl.V_fb = 0;//速度 
	
//	GDControl.Kp_Eacc = 0.1*0.0015*0.8;   //轴角加速度参数
//	GDControl.Ki_Eacc = 0.1*1.0*0.8;  	
//	GDControl.Eamax = 2000;
	
//	GDControl.Kp_Ev = 18;   //轴角速度环参数 带加速度环
//	GDControl.Ki_Ev = 60;   

	GDControl.Kp_Ev = 0.3*1.3*2.5;  //0.38
	GDControl.Ki_Ev = 1.5*1.3*2.5; //6.5



  GDControl.Ev_give = 0;
	GDControl.Ev_fb = 0;//轴角速度
	
	GDControl.Kp_P = 10;
	GDControl.Ki_P = 0;
	GDControl.bound_P = 6;
	GDControl.Ev_Set = 61;//80
	GDControl.Evmax = 61;//80
	GDControl.P_give = 0;
	GDControl.P_fb = 0;//角位置
	
	GDControl.Kp_1_T1 = 2;
	GDControl.Ki_1_T1 = 2;
	GDControl.bound_T1 = 0.1;
	GDControl.Kp_2_T1 = 2;
	GDControl.Ki_2_T1 = 2;//42
	GDControl.Vmax = 0;
	GDControl.T_fb = 0;
	
	GDControl.Kp_1_T2 = 2;
	GDControl.Ki_1_T2 = 2;
	GDControl.bound_T2 = 0.1;
	GDControl.Kp_2_T2 = 2;//4.5
	GDControl.Ki_2_T2 = 2;
	
	GDControl.Kp_1_T3 = 0.0;
	GDControl.Ki_1_T3 = 0.0;
	GDControl.bound_T3 = 0.0;
	GDControl.Kp_2_T3 = 0.0;
	GDControl.Ki_2_T3 = 0.0;
	
	GDControl.Kp_G = 0;
	GDControl.Ki_G = 0.0;
	GDControl.bound_G = 0;
	GDControl.V_Set = 0;
	GDControl.GEO_give = 0;
	GDControl.GEO_fb = 0;
	
	GDControl.timer = 0;
	GDControl.Danger_count = 0;
	GDControl.T_disturb = 0.0;
//-----------方位----------//
//  FWControl.Kp_I = 800*0.8;     //1300
//	FWControl.Ki_I = 400000*0.8;   //500000
//  FWControl.Kp_I = 600000*0.0031*0.5;     
//	FWControl.Ki_I = 600000*1*0.5;   
  FWControl.Kp_I = 700; //500   
	FWControl.Ki_I = 200000; 
	FWControl.SamT_I = 0.001;
	FWControl.Umax = 4000;
	FWControl.I_give = 0;
	FWControl.I_fb = 0;//电流环控制参数
	
	FWControl.Kp_A[0] = 0;
	FWControl.Kp_A[1] = 0;
	FWControl.Kp_A[2] = 0;
	FWControl.Kp_A[3] = 0;
	FWControl.Kp_A[4] = 0;
	FWControl.Kp_A[5] = 0;//反序排列
	FWControl.Ki_A[0] = 0;
	FWControl.Ki_A[1] = 0;
	FWControl.Ki_A[2] = 0;
	FWControl.Ki_A[3] = 0;
	FWControl.Ki_A[4] = 0;
	FWControl.Ki_A[5] = 0;
	FWControl.SamT = 0.001;
	FWControl.Kg_A = 0.0;
	FWControl.BW_A[0] = 0.0;
	FWControl.BW_A[1] = 0.0;
	FWControl.BW_A[2] = 0.0;
	FWControl.BW_A[3] = 0.0;
	FWControl.BW_A[4] = 0.0;
	FWControl.BW_A[5] = 0.0;
	FWControl.Imax = 5;
	FWControl.A_give = 0;
	FWControl.A_fb = 0.0;
	FWControl.bound_A[0] = 0;
	FWControl.bound_A[1] = 0;
	FWControl.bound_A[2] = 0;
	FWControl.bound_A[3] = 0;
	FWControl.bound_A[4] = 0;
	
	FWControl.Kp_V[0] = 0;
	FWControl.Kp_V[1] = 0;
	FWControl.Kp_V[2] = 0;
	FWControl.Kp_V[3] = 0;
	FWControl.Kp_V[4] = 0;
	FWControl.Kp_V[5] = 0;
	FWControl.Ki_V[0] = 0;
	FWControl.Ki_V[1] = 0;
	FWControl.Ki_V[2] = 0;
	FWControl.Ki_V[4] = 0;
	FWControl.Ki_V[5] = 0;
	FWControl.Amax = 2000;
  FWControl.V_give = 0.0;
	FWControl.V_fb = 0.0;
	FWControl.bound_V[0] = 0;
	FWControl.bound_V[1] = 0;
	FWControl.bound_V[2] = 0;
	FWControl.bound_V[3] = 0;
	FWControl.bound_V[4] = 0;
	
	//k = 343
//	FWControl.Kp_Ev = 3.8*0.053*1.0;      
//	FWControl.Ki_Ev = 3.8*1*1.0;   

	FWControl.Kp_Ev = 3.8*0.053*2.5*2.3;//0.5     
	FWControl.Ki_Ev = 3.8*1*2.5*2.3; //9.5
	
  FWControl.Ev_give = 0; 
	FWControl.Ev_fb = 0;//轴角速度
	
	FWControl.Kp_P = 15;
	FWControl.Ki_P = 0;
	FWControl.bound_P = 6;
	FWControl.Ev_Set = 80;
	FWControl.Evmax = 80;
	FWControl.P_give = 0.0;
	FWControl.P_fb = 0.0;
	
	FWControl.Kp_1_T1 = 0;
	FWControl.Ki_1_T1 = 0;
	FWControl.bound_T1 = 0;
	FWControl.Kp_2_T1 = 0;
	FWControl.Ki_2_T1 = 0;
	FWControl.Vmax = 0;
	FWControl.T_fb = 0.0;
	
	FWControl.Kp_1_T2 = 0;
	FWControl.Ki_1_T2 = 0;
	FWControl.bound_T2 = 0;
	FWControl.Kp_2_T2 = 0;
	FWControl.Ki_2_T2 = 0;
	
	FWControl.Kp_1_T3 = 0.0;
	FWControl.Ki_1_T3 = 0.0;
	FWControl.bound_T3 = 0.0;
	FWControl.Kp_2_T3 = 0.0;
	FWControl.Ki_2_T3 = 0.0;
	
	FWControl.Kp_G = 0.0;
	FWControl.Ki_G = 0.0;
	FWControl.bound_G = 0.0;
	FWControl.V_Set = 0.0;
	FWControl.GEO_give = 0.0;
	FWControl.GEO_fb = 0.0;
	
	FWControl.timer = 0;
	FWControl.Danger_count = 0;
	FWControl.T_disturb = 0.0;
}

void initdatapar(void)
{
//------------高低--------//
	GDSensor.Current = 0;
	
	GDSensor.BMQDATA_origin = 0;
	GDSensor.BMQDATA_zero = 1042844+2912;//11018
  GDSensor.BMQDATA = 0;
	
	GDSensor.GEO_angle = 0;
	
	GDSensor.acc = 0;
	GDSensor.angle = 0;
	GDSensor.angle_init = 0;
	GDSensor.ev = 0;
	GDSensor.gyro = 0;
	GDSensor.gyrobias = 0;
	
	GDSensor.Tbl_angle = 0;
	
	GDSensor.scandir = 1;
	GDSensor.sacn_v_limit = 60.0;
	GDSensor.XW_state = 0;
//------------方位--------//
  FWSensor.Current =0;
  
	FWSensor.BMQDATA_origin_old = 0;
  FWSensor.BMQDATA_origin = 0;

	FWSensor.BMQDATA_zero = 715360;//3584752

  FWSensor.BMQDATA = 0;
	FWSensor.lap = 0;
	FWSensor.angle_Vel = 0;
	
	FWSensor.GEO_angle = 0;
	
	FWSensor.acc = 0;
	FWSensor.angle = 0;
	FWSensor.ev = 0;
	FWSensor.gyro = 0;
	FWSensor.gyrobias = 0;
	
	FWSensor.acc_Hg = 0;
	FWSensor.gyro_Hg = 0;
	FWSensor.gyrobias_Hg	= 0;
	
	FWSensor.Tbl_angle = 0;
	
	FWSensor.scandir = -1;
	FWSensor.sacn_v_limit = 10.0;
	FWSensor.XW_state = 0;
}

void fbdispose(void)
{
	 GDControl.I_fb = GdCurrentButter(2,200,0.001,gd_i_temp);//250gd_i_temp;
	 FWControl.I_fb = rec_fw_fy_buff.rec_cmd_struct.fw_current*0.001;
	
	 GDControl.Ev_fb = GDSensor.ev;
	 FWControl.Ev_fb = FWSensor.ev;

	 GDControl.Eacc_fb = GDSensor.eacc;
	 FWControl.Eacc_fb = FWSensor.eacc;

	 GDControl.P_fb = GDSensor.angle; 
	 FWControl.P_fb = FWSensor.angle;
	
}			 

uint8_t gdencodererrcnt = 0;
uint8_t Last_cmd = 0;
uint8_t Now_cmd = 0;
uint8_t bmq_test_cnt = 0;
float FW_Mech_Angle,FY_Mech_Angle;

void gdbmqdispose(void)
{			
	GDSensor.BMQDATA_origin_old = GDSensor.BMQDATA_origin;
	GDSensor.BMQDATA_origin = WHGBMQ_Data;
	
  GDSensor.BMQDATA =(int32_t)(WHGBMQ_Data-GDSensor.BMQDATA_zero);	
  GDSensor.angle = Bmq_cof*GDSensor.BMQDATA;
 
	 if(GDSensor.angle>180.0)
		 GDSensor.angle = GDSensor.angle-360;
	 else if(GDSensor.angle<-180.0)
		 GDSensor.angle = GDSensor.angle+360;
	 
	 
	 GDSensor.ev = GdAxisEvDis(2,30,0.001,GDSensor.angle);  
	 
	 
	 	GDSensor.Cos_GD = cos(GDSensor.angle*Angle_cof_rad);
		GDSensor.Sec_GD = 1.0/GDSensor.Cos_GD;
	 
	 GDSensor.eacc = GdeAccDis(50,0.001,GDSensor.ev);
	 if(GDSensor.eacc > 2000)
		 GDSensor.eacc = 2000;
	 else if(GDSensor.eacc < -2000)
		 GDSensor.eacc = -2000;
}

 void fwbmqdispose(void)
{
    FW_BMQ_DATA = rec_fw_fy_buff.rec_cmd_struct.bpm_cal;
		FWSensor.BMQDATA_origin_old = FWSensor.BMQDATA_origin;
		FWSensor.BMQDATA_origin = FW_BMQ_DATA;
	
	if(FWSensor.BMQDATA_origin>FWSensor.BMQDATA_zero) //设置零位
	{
	  FWSensor.BMQDATA =  FWSensor.BMQDATA_zero - FWSensor.BMQDATA_origin;
	  FWSensor.angle = Bmq_cof*FWSensor.BMQDATA;		
	}
	else
	{
	  FWSensor.BMQDATA = (int32_t)(FWSensor.BMQDATA_zero - FWSensor.BMQDATA_origin);
		FWSensor.angle = Bmq_cof*FWSensor.BMQDATA;
	} 
	if(FWSensor.angle>180)
		FWSensor.angle = FWSensor.angle-360;
	else if(FWSensor.angle<-180)
	  FWSensor.angle = FWSensor.angle+360;


   if((FWSensor.BMQDATA_origin_old>=2091327)&&(FWSensor.BMQDATA_origin<=5825))
	       FWSensor.lap--;
	 else if((FWSensor.BMQDATA_origin_old<=5825)&&(FWSensor.BMQDATA_origin>2091327))
	       FWSensor.lap++;

	 FWSensor.angle_Vel = (FWSensor.lap*2097152+FWSensor.BMQDATA)*Bmq_cof;	//FWSensor.angle_Vel数据类型double
	 FWSensor.ev = FwAxisEvDis(2,50,0.001,FWSensor.angle_Vel);   	 
	 FWSensor.eacc = FweAccDis(70,0.001,FWSensor.ev);
 }


void evmeancal(void)
{
 static uint16_t ev_sum_time = 0;
 static float GDevSum = 0,FWevSum = 0;
	
 ev_sum_time++;
 if(ev_sum_time<=1000)
 {
	 GDevSum = GDevSum+GDSensor.ev;
	 FWevSum = FWevSum+FWSensor.ev;
	 
	 if(ev_sum_time==1000)
	 {
	   if(fabs(GDevSum*0.001)<0.1)
		 {
		  if(fabs(GDevSum*0.001)<0.1)
	      Motion_state = 1;
		   else 
			  Motion_state = 0;
		 }
		else
			 Motion_state = 0;
		
		 ev_sum_time = 0;
		 GDevSum = 0;
	   FWevSum = 0;
	  }
 }
}


//-----限位判断----------//		
uint8_t Pre_PTState = 0;
uint8_t XW_cnt = 0;
uint8_t Stop_Flag = 0;
void ReadXW(void)
{

	if((GDSensor.angle-FY_Mech_Angle)>=30.0f)
 {
	 SevroXWState.UP_XW_flag = 1;
	 SevroXWState.Down_XW_flag = 0;
   SevroXWState.FY_XW_flag = 1;
	 
	 
 }

 else if((GDSensor.angle-FY_Mech_Angle)<=-90.0f)
 {
	 SevroXWState.UP_XW_flag = 0; 
	 SevroXWState.Down_XW_flag = 1;
   SevroXWState.FY_XW_flag = 1;

 }
 else
 {
	 SevroXWState.UP_XW_flag = 0; 
   SevroXWState.Down_XW_flag = 0;
	 SevroXWState.FY_XW_flag = 0;
 }

 
}

//--------电机控制------------//
uint8_t GDOverCurrent_Flag = 0, FWOverCurrent_Flag = 0;
uint16_t GD_overspeed_Count = 0;
uint8_t GD_dis = 0;
uint8_t FW_dis = 0;
uint8_t fwdj_cnt = 0,fydj_cnt = 0;
void gd_en(void)
{
	//过流保护
	if(GDControl.timer<=1000)
	{
	   GDControl.timer++;
		  if(fabs(1.0*GDControl.I_fb)>=4)//4.6A
				 GDControl.Danger_count++;
	    if(GDControl.timer==1000)
			{
				if(GDControl.Danger_count>=800)
				{
				 GDOverCurrent_Flag = 1;        //上报
					
	       FY_EN = 0;
		     PTstate = Brake;
         GD_dis = 1;
				}
				else
					GD_dis = 0;
				
				GDControl.timer = 0;
				GDControl.Danger_count = 0;
	   }	
	 }
	
   	//超速保护
	 if(fabs(GDSensor.ev)>=100)
	 {
	   GD_overspeed_Count++; 
	  }
	 else
		 GD_overspeed_Count = 0;
	 
	 if(GD_overspeed_Count>=1000)
	 {
			FY_EN = 0;
			PTstate = Brake;
      GD_dis = 1;
		 
		  GD_overspeed_Count = 0;
	  }
		else
			GD_dis = 0;
	if(!GD_dis)
	 {	 
		 fydj_cnt ++;
     FY_EN = 1;
		 FW_PWM(-1.0*GDControl.U_give);
	 }


}
uint16_t FW_overspeed_Count = 0;
void fw_en(void)
{
	//过流保护
 	if(FWControl.timer<=1000)
	{
	   FWControl.timer++;
		  if(fabs(1.0*FWControl.I_fb)>=5)//5.8A
				 FWControl.Danger_count++;
			
	    if(FWControl.timer==1000)
			{
				if(FWControl.Danger_count>=800)
				{
         FWOverCurrent_Flag = 1;            //上报
	       FW_EN = 0;
		     PTstate = Brake;
				 FW_dis = 1;
				}
				else
					FW_dis = 0;
				
	      FWControl.timer = 0;
				FWControl.Danger_count = 0;
	   }	
	 }
	
   	//超速保护
	 if(fabs(FWSensor.ev)>=210)
	 {
	   FW_overspeed_Count++; 
	  }
	 else
		 FW_overspeed_Count = 0;
	 
	 if(FW_overspeed_Count>=1000)
	 {
			FW_EN = 0;
			PTstate = Brake;
      FW_dis = 1;
		  FW_overspeed_Count = 0;
	  }
   else 	
			FW_dis = 0;	
	 if(!FW_dis)
	 {	 
		  fwdj_cnt ++;		 
			FW_EN = 1;
		 	
	 }
}
void gd_brake(void)
{
	GD_dis = 1;
	FY_EN       =  0;
	GDControl.I_give       =  0.0;
	GDControl.Ev_give       =  0.0;
	GDControl.timer        =  0;
	GDControl.Danger_count =  0;
}
void fw_brake(void)
{
	FW_dis = 1;
	TIM3->CCR1             =  0;
	FW_EN                  =  0;
	FWControl.I_give       =  0.0;
	FWControl.U_give       =  0.0;
	FWControl.timer        =  0;
	FWControl.Danger_count =  0;
}
void all_brake(void)
{
  gd_brake();
  fw_brake();
}

void FW_PWM(float fwugive)
{
	if(FY_EN)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	
  if(fwugive>0)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	else
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	
	if(fwugive>4200)
		fwugive = 4200;
  else if(fwugive<-4200)
    fwugive = -4200;		
	
	 TIM3->CCR1 = fabs(fwugive);
}

uint32_t gd_u_d_sum = 0;
float gd_u_d_avg = 0;
float gd_u_d = 0;

void CurrentInit(void)
{
	gd_u_d_sum = ADC1_CovertedValue[0]+ADC1_CovertedValue[1]+ADC1_CovertedValue[2]+ADC1_CovertedValue[3]+ADC1_CovertedValue[4]+ADC1_CovertedValue[5]+ADC1_CovertedValue[6]+ADC1_CovertedValue[7]
               +ADC1_CovertedValue[8]+ADC1_CovertedValue[9];
	gd_u_d_avg = gd_u_d_sum*0.1;
	gd_u_d     = gd_u_d_avg*0.80586080586;
	gd_i_temp  = (gd_u_d-1650)*0.01515151515;		
}

void Dawei_PD(void)//到位状态判断
{

		
}