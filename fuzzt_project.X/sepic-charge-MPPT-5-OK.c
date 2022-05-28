// *********************************************************************************//
// Purpose : 	
//				

//  V1:び锭狾狠块筿溃, I1:び锭狾狠块筿溃瑈, V2: 筿筿溃, I2:筿筿筿瑈

// *********************************************************************************//

#include 	"xc.h"
#include	<pwm.h>					// 盢pwmㄧΑ郎
//#include	"adc10.h"					// 盢adc10ㄧΑ郎
#include 	"C30EVM_LCD.h"  			// 盢LCDㄧΑ郎
  	
#include <timer.h>
#include <adc10.h>


#define		FCY			7372800 * 4
#define     pwmf   		100000       								// pwmf=100kHz     PETER=Fcy/100k       (Tpwm= 1/100K = 0.01ms )
#define 	PWM_DUTY	PDC3										//﹚竡 PWM_DUTY  疭﹚PWMぇDUTY既竟
																																														




unsigned int      		ValuePDC ;
unsigned char     		ASCII_Buf[10] ;
float		         	T = 0.0001 ; 
float                	c = 45 ;															//c=45 AH 筿甧秖
float		        	ADC_V1 = 0, ADC_I1 = 0, ADC_V2 = 0, ADC_I2 = 0 ;
																					
//******************************** 妓闽把计 ************************************//
float		         	R_V2 = 14.3,  R_I2 = 0 ;	   		// R_V2 筿筿筿溃砞﹚  ,R3_I2  筿疊筿瑈砞﹚
float         			V1_limite = 53, I1_limite = 9.90, V2_limite = 28.6, I2_limite = 9.90 ;

//============================ADAPTIVE MPPT 闽把计======================================================//
float	       	    	power1 = 0, power2=0, power1_old=0;
float		        	V1 = 0, I1 = 0, V2 = 0, I2 = 0 ;
float                   V1_old=0, I1_old=0, V2_old=0, I2_old=0;
float                   dp=0,dv=0,u_mppt=0,dv_ref=0, v_pv_max=40;  // び锭狾秨隔筿溃 //
float                   v_ref, sum_mppt=0,b=20;         //  b=0.009;   //     b=0.002 程瞯發萝秸竊瞯
float                   kp=100,ki=20000,kd=0.00001; //  MPPT PID 把计
//**********************************************************************************//
																									
//******************************** PID_I( ) 闽把计 ************************************//
float			        kp_I =1, ki_I = 100, kd_I = 0.00001 ; //  PID_I筿瑈癹隔 PID把计
float               	u = 0, u1 = 0, u2=0   ; 			
float		    		e = 0, e1 = 0, e2 = 0 ;				//MPPT PID_I粇畉	
				
						

//**********************************************************************************//
//******************************** PID_V( ) 闽把计 ************************************//

float                   kp_V=0.1,ki_V=100,kd_V= 0.00001;
float                   kp_I2=0.12 ,ki_I2=100 ,kd_I2= 0.00001 ;
float                   u3=0, u4=0, u5, u6, u_V2 = 0, u1_V2 = 0, setpoint_I2, MPPT_V, setpoint_V1 ;
float               	e_V2 = 0, e1_V2 = 0, e2_V2 = 0, e3_V2 ;								//PID_V筿溃癹隔粇畉
float               	e_I1 = 0, e1_I1 = 0, e2_I1 = 0, e3_I1 ;	//PID_I筿瑈ず癹隔粇畉								
float			    	e_I2 = 0, e1_I2 = 0, e2_I2 = 0, e3_I2 ;
//=============================================================================================================//



//******************************** MPPT 闽把计 ***********************************//
						
float           		R2_I2 = 0,Vp , V_1 ;
float		         	V1_1 = 0, V1_2 = 0, I1_1 = 0, I1_2 = 0 ;	//	V1_1侣穝筿溃, V1_2穝筿溃, I1_1侣筿瑈, I1_2穝筿瑈
int 					Count,  MinSec,Sec,Sec_Time;
		

														
//*************************************************************************************//                                               
union
{
	unsigned Word ; 
	struct		
		{	
			unsigned ADC_DONE : 1 ;
			unsigned : 15 ;
		};
} Flags ;


// union 盢ㄏ8じ跑计ByteAccess籔SystemFlag挡篶跑计ㄏノ癘拘砰
// ぃΑじ笲衡惠―
union 	{
			unsigned char ByteAccess ;
			struct 	{
						unsigned Bit0: 1 ;
						unsigned Bit1: 1 ;
						unsigned Bit2: 1 ;
						unsigned unused : 5 ;
					} ;
		} SystemFlag ;


//======================================================================================================================
 
//const 	char	My_String1[]="V1:    I1:    R " ;					// ﹃ Program Memory ( const )
//    	char	My_String2[]="V2:    I2:      " ;					// ﹃ Data Memory



void   		Setup_LCD( void ) ;
void 		Timer1_Initial( void ) ;
void		Timer3_Initial( void ) ;
void show_R_I2(void);
void  show_data(void);
void	show_u(unsigned int The_Number );
void		ADC10_Initial( void ) ;
//void		uitoa( unsigned char *, unsigned int ) ;
void     	MotPWM_Initial( void ) ;
void		MPPT( void ) ;
//void    	Show_ADC( void ) ;
void		PID_I( void ) ;
void		PID_V( void ) ;
void        fuzzy_mppt(void);
//unsigned char	BCD_TO_BIN(unsigned char BCD);
//unsigned char  BIN_TO_BCD(unsigned char BIN);
void  Delay(unsigned int );
void  show_v_ref(void );
void  show_mppt(void );
void  PID_V( void ); 

//unsigned char	Write_second(unsigned char addr , unsigned char data) ;
//int		EEPROM_ByteWrite(unsigned char CMD , unsigned char Addr , unsigned char Data ) ;
//int		EEPROM_ByteRead(unsigned char CMD , unsigned char Addr  );
//void	sitoa(unsigned char, unsigned char *TXdata ) ;
//void	Show_Time(void) ;
//void	read_second(void) ;
//unsigned char  I2C_ACKPolling(unsigned char CMD);





//======================================= 捌祘Α================================//

  _FOSC(CSW_FSCM_OFF & XT_PLL16);   								//XT with 16xPLL oscillator, Failsafe clock off
  _FWDT(WDT_OFF);                								//Watchdog timer disabled
  _FBORPOR(PBOR_OFF & MCLR_EN);     							//Brown-out reset disabled, MCLR reset enabled
  _FGS(CODE_PROT_OFF);       									//Code protect disabled


//******************************* ADCい耞狝叭祘Α **********************************
void	_ISR _ADCInterrupt(void)
{
    	Flags.ADC_DONE = 1 ;
    	ADC_V1 = ADCBUF1 ;
    	ADC_I1 = ADCBUF2 ;
		ADC_V2 = ADCBUF3 ;
    	ADC_I2 = ADCBUF0 ;

       // V1_old=V1;
       // I1_old=I1;
       // V2_old=V2;
       // I2_old=I2;

     // V1=(ADC_V1/1024.0)*45;
	//  I1=(ADC_I1/1024.0)*50;
	//  V2=(ADC_V2/1024.0)*28.6;
	//	I2=(ADC_I2/1024.0)*50; 

      V1=(ADC_V1/1024.0)*V1_limite;
	  I1=(ADC_I1/1024.0)*I1_limite;
	  V2=(ADC_V2/1024.0)*V2_limite ;
	  I2=(ADC_I2/1024.0)*I2_limite; 
      power1=V1*I1 ;
      power2=V2*I2 ;
      //  MPPT( );

    	IFS0bits.ADIF = 0 ;
//LATEbits.LATE1=!LATEbits.LATE1; 


}
//***********************************************************************************//

//***************************** TIMER1 い耞狝叭祘Α *********************************//
void	_ISR _T1Interrupt(void)				
{

       MinSec += 1 ;				//患糤丁夹

	if (MinSec == 1000)		          //–60Ω盢 Min(だ)篨夹砞﹚1
		{
			
            Sec+=1;
                  
            if (Sec==1) 
                show_data();
            if (Sec==2)
              {
                show_R_I2();
                show_mppt( );
               
              } 
            if (Sec==3) 
              {
                show_v_ref( );  
                Sec = 0 ;  
              } 
            MinSec =0; 
		 } 
       
   
      // show_data();
      //  MPPT();
		IFS0bits.T1IF = 0 ;
    //  LATEbits.LATE1=!LATEbits.LATE1;
}


void	_ISR _T3Interrupt(void)				
{
       // MPPT();
		IFS0bits.T3IF = 0 ;
//LATEbits.LATE1=!LATEbits.LATE1;
}


//================================ 弄  DS1307丁  Timer 4 い耞捌祘Α==========================



//=================================== Main 祘Α=============================================//
int		main( void )
{
        unsigned char i ;
        OpenLCD( ) ;     
//	    Setup_LCD( ) ;    	
 		Timer1_Initial( ) ;   		
    	Timer3_Initial( ) ;
		ADC10_Initial( ) ;
    	MotPWM_Initial( ) ; 
    	Flags.Word = 0 ;
        v_ref=v_pv_max*0.2;

		for( i = 1;i <= 50;i ++ )
           {
           		PWM_DUTY =+ i ; 
                Delay(5);
           }

       kp=100,ki=20000,kd=0.00001; //  MPPT PID 把计
       sum_mppt=PWM_DUTY;

     while (1)
           {
             
              while(I2>=0.1*c && V2<14.0)
                    {
                          R_I2=0.1*c;
                          PID_I( ) ;
                    }
                   

           //   while(I2<=0.01*c && V2<14.0)
            //        {
           //               R_I2=0.01*c;
            //              PID_I( ) ;
            //        }
                             
              while(V2>=14.0 && I2>=0.01*c)
                      {
                          R_I2=0.01*c;
                          PID_V();
                      }
              
           //   while( 0.0*c<=I2 && I2<=0.1*c) MPPT();
                while(0<=I2 && I2<=0.1*c)  MPPT();

          } 
} 


void Delay(unsigned int loop)
{
    unsigned int i;

     for(i=0;i<loop;i++);
} 


//===============================   MPPT   続莱┦˙禯耑笆猭 ============================================================//   

void 	MPPT(void)
{
        
                                            // power1 程穝ΩP_pv 妓瞯,   power_old 玡Ω P_pv 妓瞯//
     // power1=V1*I1 ;
        dp=power1-power1_old;
       // dv=V1-V1_old;
         dv=V1_old-V1; 
        if(dp!=0&&dv!=0)
          {
             dv_ref=b*(dp/dv);
	         v_ref=v_ref+dv_ref;
             e=v_ref-V1;
             u_mppt = sum_mppt+kp*(e-e1)+ki*T*e+kd/T*(e-2*e1+e2) ;
              if ( u_mppt < 0.0 )	                           PWM_DUTY = (int)  0.001*2*PTPER ;     //  ValuePDC = 0.001*2*PTPER ;    
              if (u_mppt > 0.8*2*PTPER )	                   PWM_DUTY = (int)  0.8*2*PTPER ;       //   ValuePDC = 0.8*2*PTPER ;
              if (0.0 <= u_mppt && u_mppt <=  0.8*PTPER)        PWM_DUTY = (int) 2*u_mppt ;               //  ValuePDC = 2*u ;
          }  
            // PWM_DUTY=ValuePDC;

            //  if(dp==0||dv==0) PWM_DUTY = 2*sum_mppt ;// PWM_DUTY = 2*u_mppt ;
               
              power1_old=power1;
              V1_old=V1;
              I1_old=I1;
              V2_old=V2;
              I2_old=I2;  
              e2=e1;
              e1=e;
              sum_mppt=u_mppt;
              //show_mppt();
             
 }      


void	PID_I(void)
{
    	e_I1 = R_I2 - I2 ;                 
   	//	u1=u2+kp_I*(e_I1-e1_I1)+ki_I*T*e_I1+kd_I/T*(e_I1-2*e1_I1+e2_I1); //  PID control recurcive Rule
       	u1=u2+kp_I*(e_I1-e1_I1)+ki_I*T*e_I1; //  PI control recurcive Rule

		if        (u1 >= 0.8*PTPER)                	PWM_DUTY = 0.6*2*PTPER ;   // ValuePDC = 0.8*2*PTPER ;
		else  if  (u1 <= 0.0)             		  	PWM_DUTY=  0.001*2*PTPER ;  // ValuePDC = 0.001*2*PTPER ; 
		else  if  (0 <= u1&&u1 <= 0.6*PTPER)       	PWM_DUTY = 2*u1;           // ValuePDC = 2*u1 ; 
                                                                             
       // PWM_DUTY=ValuePDC;

		e3_I1 = e2_I1 ;           
		e2_I1 = e1_I1 ;
		e1_I1 = e_I1 ;
		u2=u1 ;
}


void	PID_V( void )
{
    	e_V2 = R_V2 - V2 ;

   // 	setpoint_I2 = u3+kp_V*(e_V2-e1_V2)+ki_V*T*e_V2+kd_V/T*(e_V2-2*e1_V2+e2_V2) ;
        setpoint_I2 = u3+kp_V*(e_V2-e1_V2)+ki_V*T*e_V2 ;   //   PI 
    //  setpoint_I2 = kp_V*e_V2 ;

        if( setpoint_I2 >= R_I2 )						setpoint_I2 = R_I2 ;
		else if( setpoint_I2 <= 0.01*c )				setpoint_I2 = 0.01*c ;

		e3_V2 = e2_V2 ;
		e2_V2 = e1_V2 ;
		e1_V2 = e_V2 ;
		u3=setpoint_I2 ;

// 筿瑈ず癹隔 PID control recurcive Rule
       
        e_I2 = setpoint_I2 - I2 ;
	//	u5 = u2+kp_I2*(e_I2-e1_I2)+ki_I2*T*e_I2+kd_I2/T*(e_I2-2*e1_I2+e2_I2) ;
        u5 = u2+kp_I2*(e_I2-e1_I2)+ki_I2*T*e_I2 ;   // PI 

		if 		( u5 >= 0.8*PTPER )				PWM_DUTY = 0.8*2*PTPER ;
		else if ( u5 <= 0.0 )					PWM_DUTY = 0.001*2*PTPER ;
		else if ( 0 <= u5 && u5<= 0.8*PTPER )	PWM_DUTY = 2*u5 ;
	
		e3_I2 = e2_I2 ;
		e2_I2 = e1_I2 ;
		e1_I2 = e_I2 ;
		u2 = u5 ;

}

void	Timer1_Initial( void )
 {
		ConfigIntTimer1( T1_INT_PRIOR_6 & T1_INT_ON ) ; // Disable Timer 3 Interrupt

    	OpenTimer1( T1_ON & T1_IDLE_STOP & T1_GATE_OFF & T1_PS_1_1& T1_SOURCE_INT ,
                      (((long)FCY/1000 )) );      // Set for 1ms
 }               


void	Timer3_Initial( void )
{
		ConfigIntTimer3( T3_INT_PRIOR_5 & T3_INT_OFF ) ; // Disable Timer 3 Interrupt

		OpenTimer3( T3_ON & T3_IDLE_STOP & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT ,
				    (((long)FCY/10000 )) ) ;   // Set for 0.1 mS
}


void 	ADC10_Initial(void)
{
		ADPCFG = 0xFF00;
//		ADPCFG = 0xFFFB;				// AN2/RB2 is Analog , others are Digital;
 		ADCON1 = 0x0046;				// 0b0000 0000 0100 0110 
									    // Auto convert using TMR3 as trigger source
								    	// A/D Sample Auto-Start
//      ADCON1 = 0x00EC;
//		ADCON2 = 0x0000;				// ADCON2 = 0000 0000 0000 0000
								    	// Don't scan inputs , SMPi = 00 ( Interrupt for each sample/convert )
		ADCON2 = 0x030C;
	 	ADCSSL = 0x0000;				// no scan input selected .......
 

//	    ADCON3 = 0x160F;
		ADCON3 = 0x0F3F;				// TAD = 8 Tcy , SAMC = 15 TAD 

	    ADCHS =  0x0003 ;
//		ADCHS =  0x02 ;					// ADCHS = 0b 00000000 00000110

		IEC0bits.ADIE = 1 ;				// Enable AD interrupt
		IPC2bits.ADIP = 7 ;				// Set Priority to 7 	>> highest !!

//		IPC2bits.ADIP = 6 ;				// Set Priority to 7 	>> highest !!
		ADCON1bits.ADON = 1 ;			// turn ADC ON

}


//*****************************************************************************************
//                              闽既竟砞﹚
//*****************************************************************************************

void 	MotPWM_Initial(void)
{
		IEC2bits.PWMIE = 0 ;	// Disable PWM Interrupt !!
		IEC2bits.FLTAIE = 0 ;	

	    OVDCON = 0xff00 ;		// active PWM1L  OUTPUT !!

//		OVDCON = 0x0300 ;			// active PWM1L PWM1H OUTPUT !!

	    TRISE = 0xffc0 ;		 // TRISE= 1 1 1 1   1 1 1 1   1 1 0 0  0 0 0 0  PWM3H  PWM3L  PWM2H  PWM2L  PWM1H  PWM1L  
	
//		PTCON = 0xa000 ;		// Configure as 0b  1010 0000 0000 0000 
								// PWM Time Base OFF , PWM Time Base OP in free running Mode  1 Tcy (1:1 prescale)
	    PTCON = 0x8000 ;		//	TODO : タ絋砞﹚ PTCON
								// Configure as 0b 1000 0000 0000 0000
								// PWM Time Base Prescale = 1:1
								// PWM Time Base OP in free running Mode 

//		PWMCON1 = 0x0077 ;		// Configure as 0b  0000000000   0 1 0 0 0 1 ( PEN3H  PEN2H  PEN1H  PEN3L  PEN2L  PEN1L)  
								// PWM I/O in complementary Mode and only PWM1L/H as PWM output 
	

        PWMCON1 = 0x0fff ;		//	TODO : タ絋砞﹚ PWMCON1
								// Configure as 0b0000 0011 0001 0001
								// PWM I/O in independent Mode and only PWM1L  as PWM output 


    	PWMCON2 = 0x0002;		// Configure as 0b0000000000000010 

//		DTCON1 = 0x0101 ;		// Configure as 0b 0000 0010(dead Time Unit B)   00 00(Dead Time Unit A  is Tcy) 00 10 ( 2*Tcy Dead Time Long )         

	                            //  Dead Time Clock provided From dead Time Unit A 
      

//		FLTACON = 0x0001 ;    // PWM1H  PWM1L  Pin pairs is Controlled by Fault Input A , 
                              //bit 8-15 is Faov1L-Faov4H, When Faov1L-Faov4H is 0,The corresponding PWM Pin outputis Driven Inactive
    	FLTACON = 0x0000;      // When Faov1L-Faov4H is 1 ,The corresponding PWM Pin  output  is Driven active

//		IPC9bits.PWMIP = 6 ;

		// ---------------------------------------------------------------------------------
		// The Switching Frequency !!
		// PWM resolution >= 10 bits , 
		// PDCx[1:15] compare with PTMR [0:14]
		// PDCx(0) compare with MSB of prescaler counter
		// So, PTPER is 9 bit if resolution of PDCx is 10 bit
		// Setting PWM Frequency =100K
		//  PTPER = ( (7372800*4)/ 100000 ) -1 = 293.912.2 = 294
		// PWM Frequency will be Fcy/294 = 100.0K
		// Formular !! PTPER = (Fcy/(FPWM*PTMR Prescaler)) - 1 
		// ---------------------------------------------------------------------------------
	  
        PTPER = 294 ;			// PWM Time Base Period Register 100kHz
	   // PTPER = 588 ;	        // PWM Time Base Period Register 50kHz
      //  ValuePDC = 0.01*2*PTPER ;

		//PDC1 = ValuePDC ;
	   //PDC2 = ValuePDC ;
	   // PDC3 = ValuePDC ;
  

}


void	uitoa( unsigned char *ASCII_Buf , unsigned int IntegerValue )
{
	
unsigned int        TempValue ;
unsigned char       ZeroDisable ;
unsigned int        BaseQty ;
unsigned int        Loop ;

		ZeroDisable = 1 ;
		BaseQty = 10000 ;

		for ( Loop = 0 ; Loop < 4 ; Loop ++)
		{
			TempValue = IntegerValue / BaseQty ;

			if ( TempValue > 0) 	
			{
				*ASCII_Buf++ = (unsigned char)TempValue + '0' ;
				ZeroDisable = 0 ;
			}
			else if	( ZeroDisable == 0 )	*ASCII_Buf++ = '0' ;
				
			IntegerValue = IntegerValue - ( TempValue * BaseQty ) ;
			BaseQty = BaseQty / 10 ;

		}
		*ASCII_Buf++ = (unsigned char)IntegerValue + '0' ;
		*ASCII_Buf = (unsigned char) 0x00 ;
		
}


unsigned char  BCD_TO_BIN(unsigned char BCD)
{
	int BIN,BIN_1,BIN_10;
	BIN_1=BCD&0x0F;
	BIN_10=BCD>>4;
	BIN=BIN_1+(BIN_10*10);
		return BIN ;

}

unsigned char  BIN_TO_BCD(unsigned char BIN)
{
	int BCD,BCD_1,BCD_10;
	BCD_10 = BIN/10;
	BCD_1  = BIN%10;
	BCD=(BCD_10<<4)+BCD_1;
		return BCD ;

}



void	show_v(unsigned int The_Number  )
{

    	unsigned char	Temp_Char_100, Temp_Char_10, Temp_Char ;
        
		Temp_Char_100 = The_Number /100 ;		                     //κ计计
		putcLCD( Temp_Char_100 + '0' ) ;			                //锣传ASCII絪絏'0'絪絏计
		Temp_Char_10 = (The_Number - Temp_Char_100*100)/10 ;	   //计计
		putcLCD( Temp_Char_10 + '0' ) ;
          
        putcLCD( 0x2e ) ;    //   计翴 
		Temp_Char = The_Number - ( Temp_Char_100*100 + Temp_Char_10 * 10 ) ;	//计计
		putcLCD( Temp_Char + '0' ) ;

}

/*


void	show_i(unsigned int The_Number  )
{

    	unsigned char Temp_Char_1000,Temp_Char_100, Temp_Char_10, Temp_Char ;
        
		Temp_Char_1000 = The_Number /1000 ;		                       //计计
		putcLCD( Temp_Char_1000 + '0' ) ;			                   //锣传ASCII絪絏'0'絪絏计
		Temp_Char_100 = (The_Number - Temp_Char_1000*1000)/100 ;	   //κ计计
		putcLCD( Temp_Char_100 + '0' ) ;
          
        putcLCD( 0x2e ) ;    //   计翴 
		Temp_Char_10 = (The_Number - ( Temp_Char_1000*1000 + Temp_Char_100 * 100 ))/10 ;	//计计
		putcLCD( Temp_Char_10 + '0' ) ;                                              
        Temp_Char = The_Number - ( Temp_Char_1000*1000 + Temp_Char_100 * 100+Temp_Char_10 * 10 ) ; //计计
        putcLCD( Temp_Char + '0' ) ;                                                 

}

*/

/*
void	show_i (unsigned int The_Number  )
{

    	unsigned char	Temp_Char_100, Temp_Char_10, Temp_Char ;
        
		Temp_Char_100 = The_Number /100 ;		            //κ计计
		putcLCD( Temp_Char_100 + '0' ) ;			    //锣传ASCII絪絏'0'絪絏计


        putcLCD( 0x2e ) ;  //   计翴 
		Temp_Char_10 = (The_Number - Temp_Char_100*100)  /10 ;	   //计计
		putcLCD( Temp_Char_10 + '0' ) ;
    	Temp_Char = The_Number - ( Temp_Char_100*100 + Temp_Char_10 * 10 ) ;	//计计
		putcLCD( Temp_Char + '0' ) ;

}
*/


void	show_i(unsigned int The_Number  )
{

    	unsigned char Temp_Char_1000,Temp_Char_100, Temp_Char_10, Temp_Char ;
        
		Temp_Char_1000 = The_Number /1000 ;		                       //计计
		putcLCD( Temp_Char_1000 + '0' ) ;

        putcLCD( 0x2e ) ;	//   计翴 
		                   //锣传ASCII絪絏'0'絪絏计
		Temp_Char_100 = (The_Number - Temp_Char_1000*1000)/100 ;	   //κ计计
		putcLCD( Temp_Char_100 + '0' ) ;
        Temp_Char_10 = (The_Number - ( Temp_Char_1000*1000 + Temp_Char_100 * 100 ))/10 ;	//计计
		putcLCD( Temp_Char_10 + '0' ) ;                                              
        Temp_Char = The_Number - ( Temp_Char_1000*1000 + Temp_Char_100 * 100+Temp_Char_10 * 10 ) ; //计计
        putcLCD( Temp_Char + '0' ) ;                                                 

}

void  show_data(void) 
 {
   	
    //char CLR_DISP=0x001;		// Clear the Display

    WriteCmdLCD(0x001) ;					// Configure LCD - Clear Display
	LCD_L_Delay() ;

	setcurLCD(0,0) ;			// Set LCD cursor
	putrsLCD("V1:") ;
    show_v(V1*10 );

	setcurLCD(7,0) ;
    putrsLCD(" ") ;
    setcurLCD(8,0) ;
    putrsLCD("I1:") ;
    show_i(I1*1000 );
   //  setcurLCD(14,0) ;
    // putrsLCD("RI2") ;

    setcurLCD(0,1) ;			// Set LCD cursor
	putrsLCD("V2:") ;
    show_v(V2*10 );

	setcurLCD(7,1) ;
    putrsLCD(" ") ;
    setcurLCD(8,1) ;
    putrsLCD("I2:") ;
    show_i(I2*1000 );

    //show_i(I2*100 );
   // setcurLCD(14,1) ;
   // show_i(R_I2*100 );

}

void show_R_I2(void)
  {
    //uint8_t CLR_DISP=0x001;		// Clear the Display
   	WriteCmdLCD(0x001) ;					// Configure LCD - Clear Display
	LCD_L_Delay() ;

    setcurLCD(0,0) ;
    putrsLCD("R_I2:") ;

    setcurLCD(5,0) ;
    show_i(R_I2*1000 );
  }

 void show_mppt(void )
{
   
   long double dutycycle;
  // char CLR_DISP=0x001;		// Clear the Display
  // WriteCmdLCD(CLR_DISP) ;	// Configure LCD - Clear Display
  // LCD_L_Delay() ;
 
    setcurLCD(0,1) ;
    putrsLCD("MPPT_U:") ;
    setcurLCD(7,1) ;
    putrsLCD(" ") ;
    setcurLCD(8,1) ;
    dutycycle=(u_mppt/273.0)*100.0;  //PTPER=273
    show_u(dutycycle*1000 );
    setcurLCD(13,1) ;
    putrsLCD("%") ;
}
void  show_pid(void )
{
  //  char CLR_DISP=0x001;		// Clear the Display
  // 	WriteCmdLCD(CLR_DISP) ;					// Configure LCD - Clear Display
  //	LCD_L_Delay() ;
    setcurLCD(0,1) ;
    putrsLCD("                ") ;
    putrsLCD("PID WORKING") ;

}    


void	show_u(unsigned int The_Number  )
{

    unsigned char Temp_Char_1000,Temp_Char_100, Temp_Char_10, Temp_Char ;
        
		Temp_Char_1000 = The_Number /1000 ;		                       //计计
		putcLCD( Temp_Char_1000 + '0' ) ;

       // putcLCD( 0x2e ) ;	//   计翴 
		                   //锣传ASCII絪絏'0'絪絏计
		Temp_Char_100 = (The_Number - Temp_Char_1000*1000)/100 ;	   //κ计计
		putcLCD( Temp_Char_100 + '0' ) ;
        Temp_Char_10 = (The_Number - ( Temp_Char_1000*1000 + Temp_Char_100 * 100 ))/10 ;	//计计
		putcLCD( Temp_Char_10 + '0' ) ;  
        putcLCD( 0x2e ) ;	//   计翴                                    
        Temp_Char = The_Number - ( Temp_Char_1000*1000 + Temp_Char_100 * 100+Temp_Char_10 * 10 ) ; //计计
        putcLCD( Temp_Char + '0' ) ;                                             

}

void show_v_ref(void )
 {
   // char CLR_DISP=0x001;		// Clear the Display

    WriteCmdLCD(0x001) ;					// Configure LCD - Clear Display
	LCD_L_Delay() ;

	setcurLCD(0,0) ;			// Set LCD cursor
	putrsLCD("v_ref:") ;
    setcurLCD(6,0) ;
    putrsLCD(" ") ;
    setcurLCD(8,0) ;
    show_v(V1*10 ); 
 }