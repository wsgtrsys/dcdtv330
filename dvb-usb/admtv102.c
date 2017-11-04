#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include "dvb_frontend.h"

#include "admtv102.h"
#include "admtv102_priv.h"

enum fe_bandwidth {
        BANDWIDTH_8_MHZ,
        BANDWIDTH_7_MHZ,
        BANDWIDTH_6_MHZ,
        BANDWIDTH_AUTO,
        BANDWIDTH_5_MHZ,
        BANDWIDTH_10_MHZ,
        BANDWIDTH_1_712_MHZ,
};


/* define the register address and data of Tuner that need to be initialized when power on */ 
u8 AddrDataMemADMTV102_UHF[100]=
{
//Addr Data
 0x10,0x6A,      // LNA current, 0x4C will degrade 1 dB performance but get better power consumption.
 0x11,0xD8,      // Mixer current
 0x12,0xC0,      // Mixer gain
 0x15,0x3D,      // TOP and ADJ to optimize SNR and ACI 
 0x17,0x97,      // TOP and ADJ to optimize SNR and ACI //*9A->97,Changed TOP and ADJ to optimize SNR and ACI
 0x18,0x03,      // Changed power detector saturation voltage point and warning voltage point //* new insert
 0x1F,0x17,      // VCOSEL,PLLF RFPGA amp current 
 0x20,0xFF,      // RFPGA amp current
 0x21,0xA4,      // RFPGA amp current
 0x22,0xA4,      // RFPGA amp current
 0x23,0xDF,      // Mixer Bias control
 0x26,0xFA,      // PLL BUFFER CURRENT
 0x27,0x00,      // CONVCOL/H for VCO current control
 0x28,0xFF,      // CONVCOBUFL/H for VCO buffer amplifier current control
 0x29,0xFF,      // CONDIV1/2 for first and second divider current control
 0x2A,0xFF,      // CONDIV3/4 for third and last divider current control
 0x2B,0xE7,      // CONDIV5 for third and last divider current control
 0X2C,0xFF,      // CONBUF0/1 for L-Band Buffer amp and first Buffer amp current control
 0x2E,0xFB,      // CONBUF4 for forth Buffer amp current control
 0x30,0x80,      // LFSW(Internal Loop Filter) to improve phase noise //* F8->80
 0x32,0xc2,      // LOOP filter boundary 
 0x33,0x80,      // DC offset control
 0x34,0xEC,      // DC offset control
 0x39,0x96,      // AGCHM AGC compensation value when LNA changes
 0x3A,0xA0,      // AGCHM AGC compensation value when LNA changes
 0x3B,0x05,      // AGCHM AGC compensation value when LNA changes
 0x3C,0xD0,      // AGCHM AGC compensation value when LNA changes
 0x44,0xDF,     // BBPGA needs to stay on for current silicon revision
 0x48,0x23,      // current for output buffer amp //*23->21
 0x49,0x08,      // gain mode for output buffer amp
 0x4A,0xA0,      // trip point for BBVGA
 0x4B,0x9D,      // trip point for RFPGA
 0x4C,0x9D,      // ADJRSSI warning point
 0x4D,0xC3,      // PLL current for stability PLL lock
 0xff,0xff
}; 

u8 AddrDataMemADMTV102_VHF[100]=
{
//Addr Data
 0x10,0x08,      // LNA current
 0x11,0xc2,      // Mixer current
 0x12,0xC0,      // Mixer gain
 0x17,0x98,      // TOP and ADJ to optimize SNR and ACI //* 9A->98
 0x1F,0x17,      // VCOSEL,PLLF RFPGA amp current 
 0x20,0x9b,      // RFPGA amp current
 0x21,0xA4,      // RFPGA amp current
 0x22,0xA4,      // RFPGA amp current
 0x23,0x9F,      // Mixer Bias control
 0x26,0xF9,      // PLL BUFFER CURRENT
 0x27,0x11,      // CONVCOL/H for VCO current control
 0x28,0x92,      // CONVCOBUFL/H for VCO buffer amplifier current control
 0x29,0xBC,      // CONDIV1/2 for first and second divider current control
 0x2B,0xE7,      // CONDIV5 for third and last divider current control
 0x2D,0x9C,     
 0x2E,0xCE,      // CONBUF4 for forth Buffer amp current control
 0x2F,0x1F,
 0x30,0x80,      // LFSW(Internal Loop Filter) to improve phase noise
 0x32,0xc2,      // LOOP filter boundary 
 0x33,0x80,      // DC offset control
 0x34,0xEC,      // DC offset control
 0x48,0x29,      // current for output buffer amp
 0x49,0x08,      // gain mode for output buffer amp
 0x4A,0xA0,      // trip point for BBVGA
 0x4B,0x9D,      // trip point for RFPGA
 0x4C,0x9D,      // ADJRSSI warning point
 0x4D,0xC3,      // PLL current for stability PLL lock
 0xff,0xff
}; 


u8 PLLRegSetTable [10][3]=
{ // 0x24,0x31,0x38
	{0x0F,0x04,0x50}, //13MHz //0x24: 0xnB-> 0xnF
	{0x1F,0x04,0x50}, //16.384MHz 
	{0x2F,0x04,0x50}, //19.2MHz
	{0x3F,0x04,0x50}, //20.48MHz
	{0x4F,0x15,0x51}, //24.576MHz
	{0x5F,0x15,0x51}, //26MHz
	{0x5A,0x04,0x51}, //30.4MHz  //* special set for 30.4MHz case 
	{0x6F,0x15,0x51}, //36MHz
	{0x7F,0x15,0x51}, //38.4MHz
	{0x3F,0x04,0x50}, //20MHz
};


static int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off debugging (default:off).");

#define dprintk(args...) do { if (debug) {printk(KERN_DEBUG "admtv102: " args); printk("\n"); }} while (0)

// Reads a single register
static int admtv102_readreg(struct admtv102_priv *priv, u8 reg, u8 *val)
{
	struct i2c_msg msg[2] = {
		{ .addr = priv->cfg->i2c_address, .flags = 0,        .buf = &reg, .len = 1 },
		{ .addr = priv->cfg->i2c_address, .flags = I2C_M_RD, .buf = val,  .len = 1 },
	};

	if (i2c_transfer(priv->i2c, msg, 2) != 2) {
		printk(KERN_WARNING "admtv102 I2C read failed\n");
		return -EREMOTEIO;
	}
	if (debug >= 2)
		printk("%s(reg=0x%02X) : 0x%02X\n", __func__, reg, *val);
	return 0;
}

// Writes a single register
static int admtv102_writereg(struct admtv102_priv *priv, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	struct i2c_msg msg = {
		.addr = priv->cfg->i2c_address, .flags = 0, .buf = buf, .len = 2
	};

	if (i2c_transfer(priv->i2c, &msg, 1) != 1) {
		printk(KERN_WARNING "admtv102 I2C write failed\n");
		return -EREMOTEIO;
	}
	if (debug >= 2)
		printk("%s(reg=0x%02X, val=0x%02X)\n", __func__, reg, val);
	return 0;
}

// Writes a set of consecutive registers
static int admtv102_writeregs(struct admtv102_priv *priv,u8 *buf, u8 len)
{
	struct i2c_msg msg = {
		.addr = priv->cfg->i2c_address, .flags = 0, .buf = buf, .len = len
	};
	if (i2c_transfer(priv->i2c, &msg, 1) != 1) {
		printk(KERN_WARNING "mt2060 I2C write failed (len=%i)\n",(int)len);
		return -EREMOTEIO;
	}
	return 0;
}

/* ========================================================================= */
/** check all fo these!!!! */

//u8 g_icp,
//g_convco,g_curTempState,g_CTUNE_CLKOFS;
//int g_VHFSet=UHFSupport;
//int g_TunerPLLType=REFCLK16384;
//ChannelInfo  FindChannel[50];
//MPEG_PROGINFO myMpegInfo[MaxProgNum];
//int FindProgNum;   // indicate how many program is found in one certain channel

/*
The card should not do this as it prevents multiple cards from working!!!!!
struct admtv102_priv *state=NULL;
void myAdiInit(struct admtv102_priv *state)
{
	state = state;
	/// func calls with   struct admtv102_priv *state,
}
*/

//*****************************************************************************
// Function:  Configure Tuner chip registers                                            
// Input: target -- Device I2C address 
//        AddrData -- Register address and Config data array 
// Output: None
//*****************************************************************************
void ConfigTuner(struct admtv102_priv *state, u8 *AddrData)
{
   int i=0;
   u8 addr, data;

   while (AddrData[i]!=0xFF)
   { 
     addr = AddrData[i++];
     data = AddrData[i++];
     admtv102_writereg(state, addr, data);
   }
}

//*****************************************************************************
// Function:  Set Tuner LPF configuration                                            
// Input: target -- Tuner I2C device Address 
//        refClkType -- Tuner PLL Type 
//        lpfBW -- Band width , in MHz unit
// Return: None
//*****************************************************************************
void SetLPF(struct admtv102_priv *state, u32 refClkType, u8 lpfBW)
{
	u8 tuneval;
	u8 t;
	int ret;

	admtv102_writereg(state, 0x15, (u8)( 0x38| ((lpfBW-3)& 0x07) ));
	admtv102_writereg(state, 0x25 , (_EXTUNEOFF << 2) | (_TUNEEN<<1) );  
	msleep(10);  //change from 1 to 10 

	tuneval=0x10; //default value 
	ret = admtv102_readreg(state, 0x0F, &t);
	if(ret == 0)
		tuneval = t;	
	
    admtv102_writereg(state, 0x25 , (_EXTUNEON << 2) | (_TUNEEN << 1) );   //change Tuning mode : auto-tune => manual tune(hold mode).

	if (refClkType==ADMTV102_REFCLK30400)
		admtv102_writereg(state, 0x25, (u8)(((tuneval+state->CTUNE_CLKOFS)<<3) | (_EXTUNEON << 2) | (_TUNEEN << 1)) );   //Write CTUNE val. in order to store tuned value.
	else
		admtv102_writereg(state, 0x25, (u8)((tuneval<<3) | (_EXTUNEON << 2) | (_TUNEEN << 1)) );   //Write CTUNE val. in order to store tuned value.

    return;
}

//******************************************************************************
// Function: Tuner PLL Register Setting 
// Input: target -- Tuner I2C device Address 
//        RegDat -- Reg set value table   
// Return: success flag
//******************************************************************************
int TunerPLLRegSet(struct admtv102_priv *state, u8* RegDat, int TunerPLLType)
{
	int i=0;
	u8 addr, data;
	u8 splitid;
	u8 t;

   if(ADMTV102_REFCLK30400==TunerPLLType)
	  {
	    addr=0x24;
	    admtv102_readreg(state, 0x00, &splitid);

		if (0x0E==splitid)
			data=REFCLK30400_CLKSEL_REG_SPLITID0E;  // 0x5A 
	    else if(0x0F==splitid)
			data=REFCLK30400_CLKSEL_REG_SPLITID0F;  // 0x6A, for mass product 
		else
			data=RegDat[i];

	    admtv102_writereg(state, addr, data);
	    i++;
	  }
	else {
		addr=0x24;
		data=RegDat[i++];
		admtv102_writereg(state, addr, data);
	}

	addr=0x31;
	data=RegDat[i++];
	admtv102_writereg(state, addr, data);
 
	addr=0x38;
	data=RegDat[i++];
	admtv102_writereg(state, addr, data); 
	
	return 1;
}
//*****************************************************************************
// Function:  Distinguish Tuner Chip type by read SplidID                                              
// Input: target -- Device I2C address 
//        
// Return: Tuner Type, MTV102 or ADMTV102 
//*****************************************************************************
int GetTunerType(struct admtv102_priv *state)
{
	u8 splitid,RetTunerType; 

	admtv102_readreg (state, 0x00, &splitid);

	state->CTUNE_CLKOFS=CTUNE_CLKOFS_SPLIT0E;
	switch(splitid)
	{
	case 0x0E:
		state->CTUNE_CLKOFS=CTUNE_CLKOFS_SPLIT0E;
		RetTunerType=Tuner_ADMTV102;
		break;
	case 0x0F:  // for mass product version
		state->CTUNE_CLKOFS=CTUNE_CLKOFS_SPLIT0F;
		RetTunerType=Tuner_ADMTV102;
		break;
    case 0x08:
    case 0x0A:
		RetTunerType=Tuner_MTV102;
		break;
    default:
		RetTunerType=Tuner_NewMTV102;
    }

	return(RetTunerType);
}

//*****************************************************************************
// Function:  Main Function for Tuner Initialization
//            only support ADMTV102 type, do not support MTV102 any more                                              
// Input: None
// Return: None
//*****************************************************************************
void TunerInit(struct admtv102_priv *state)
{
	int TunerPLLType = state->cfg->ref_clk_type;

	GetTunerType(state);
	state->icp=0;
	state->convco=0;
	state->curTempState=HIGH_TEMP; 
	if(TunerPLLType<0 || TunerPLLType>9 ) 	TunerPLLType=1;
	if(state->VHFSet== VHFSupport)
		ConfigTuner (state, &AddrDataMemADMTV102_VHF[0]);
	else
		ConfigTuner (state, &AddrDataMemADMTV102_UHF[0]);

	TunerPLLRegSet(state, &PLLRegSetTable[TunerPLLType][0],TunerPLLType);
	SetLPF(state, TunerPLLType, 8);
}

//*****************************************************************************
// Function:  frequency setting for ADMTV102                                              
//  Input:
//               target    : I2C address of tuner
//               frequency : Tuner center frequency in MHz
//               lpfBW     : Channel Bandwidth in MHz (default: 8- 8MHZ)
//               refClkType: Tuner PLL reference clock type
//  frequency setting formula
//               LOfrequency=(Clockfrequency/PLLR*(PLLN+PLLF/2^20))/PLLS;
//  Return: None 
//*****************************************************************************
void SetTunerFreq(struct admtv102_priv *state, u32 frequency, u32 lpfBW)
{
	u32    MTV10x_REFCLK; 
	u32    PLLFREQ,Freq;
	u32    DIVSEL=0,VCOSEL=0;	
	u8    PC4=0,PC8_16=0,DATA47,_temper;
	u32    Seg_num;
	u8    PLLR; 
	u32    lofreq;
	u32   PLLN, PLLF,tmp;
	u32    MultiFactor,sub_exp,div_1,div_2;

	printk("%s(freq=%d, bw=%d)\n", __func__, frequency, lpfBW);
	
	state->frequency = frequency;
	// judge if the VFHset has conflict with frequency value or not 
	if( frequency >400 &&  VHFSupport==state->VHFSet) // VHF is 174MHz ~ 245MHz 
	{
		state->VHFSet=UHFSupport;
		TunerInit(state);
	}
	else if(frequency <400 &&  UHFSupport==state->VHFSet)
	{
		state->VHFSet=VHFSupport;
		TunerInit(state);
	}

	PLLR=1;
	lofreq  = frequency*1000;
	DIVSEL  = LO2PLL_Freq(lofreq);
	Seg_num = (0x01 << DIVSEL);
	
	if(Seg_num >=1 && Seg_num <=16)
	{ PLLFREQ = lofreq* (16/Seg_num);
		Freq=frequency* (16/Seg_num);
	}
	else 	
	{ PLLFREQ = lofreq*2;
		Freq=frequency*2;
	}

	switch(state->cfg->ref_clk_type)
	{
	case ADMTV102_REFCLK13000:
		MTV10x_REFCLK = 130;  // 13*MultiFactor
		MultiFactor=10;	
		sub_exp=1;
		div_1=5;
		div_2=13; // 130=13*5*(2^1)
		break;
   case ADMTV102_REFCLK16384: { 
	   MTV10x_REFCLK = 16384; 
	   MultiFactor=1000;
	   sub_exp=14;
	   div_1=1;
	   div_2=1; // 16384== 2^14 
	   break; }
   case ADMTV102_REFCLK19200: {
	   MTV10x_REFCLK = 192; 
	   MultiFactor=10;
	   sub_exp=6;
	   div_1=1;
	   div_2=3; //  192 = 2^6 *3 
	   break;}
   case ADMTV102_REFCLK20480: {
	   MTV10x_REFCLK = 2048; 
	   MultiFactor=100;
	   sub_exp=11;
	   div_1=1;
	   div_2=1; //  2048 = 2^11   
       break;}
   case ADMTV102_REFCLK24576: {
	   MTV10x_REFCLK = 24576; 
	   MultiFactor=1000;
	   sub_exp=13;
	   div_1=1;
	   div_2=3; // 24576 = 2^13*3 
       break; }
   case ADMTV102_REFCLK26000: {
	   MTV10x_REFCLK = 260; 
	   MultiFactor=10;	
	   sub_exp=2;
	   div_1=5;
	   div_2=13; // 260=13*5*(2^2)   
       break;}
   case ADMTV102_REFCLK30400: {
		MTV10x_REFCLK = 304;
	   MultiFactor=10;
	   PLLR = 2 ;
	   SetLPF(state, ADMTV102_REFCLK30400,(u8)lpfBW);
		admtv102_writereg(state, 0x19, PLLR); //Ref. Clock Divider PLL0 register , bit[7:4] is reserved, bit[3:0] is PLLR
	   sub_exp=4;
	   div_1=1;
	   div_2=19; // 304 = 16*19   
	   break;
	   }
   case ADMTV102_REFCLK36000: { 
	   MTV10x_REFCLK = 360;
	   MultiFactor=10;	
	   sub_exp=3;
	   div_1=5;
	   div_2=9; // 360=2^3*9*5 	   
	   break;}
   case ADMTV102_REFCLK38400:{ 
	   MTV10x_REFCLK = 384; 
	   MultiFactor=10;
	   sub_exp=7;
	   div_1=1;
	   div_2=3; // 384 = 2^7 *3    
	   break;}
   case ADMTV102_REFCLK20000:{ 
	   MTV10x_REFCLK = 200; 
	   MultiFactor=10;	
	   sub_exp=3;
	   div_1=5;
	   div_2=5; // 200=8*5*5
       break;}
   default: { 
	   MTV10x_REFCLK = 16384; 
	   MultiFactor=1000;
	   sub_exp=14;
	   div_1=1;
	   div_2=1; // 16384== 2^14 
	   break;}
   }
   
   PLLN = (Freq*MultiFactor* PLLR/MTV10x_REFCLK ); //new formula, liu dong, 2007/12/11
   tmp = ((PLLR*Freq*MultiFactor/div_1) << (20-sub_exp) )/div_2;   
   PLLF = tmp - (PLLN<<20);

   DATA47 = 0x10; //default Value
   admtv102_readreg(state, 0x2f, &DATA47);
   
   if (PLLN<=66)   
   {   
       //4-prescaler
       PC4    = 1;
       PC8_16 = 0;
       DATA47 = (u8)(DATA47 | (PC4<<6) | (PC8_16<<5)); 
   }
   else
   {
       //8-prescaler
       PC4    = 0;
       PC8_16 = 0;
       DATA47 = (u8) (DATA47 | (PC4<<6) | (PC8_16<<5)); 
   }

   // do a reset operation 
   admtv102_writereg(state, 0x27, 0x00 ); //added : v1.6.3 at PM 20:39 2007-07-26
   //PLL Reset Enable
   admtv102_writereg(state, 0x2f, DATA47 );      //Reset Seq. 0 => 1  : //updated : v1.6.3 at PM 20:39 2007-07-26
   admtv102_writereg(state, 0x2f, (u8) (DATA47 | 0x80));

   if ((PLLFREQ*2)<2592000)  // 648*1000*2*2 according to data sheet
      VCOSEL = 0;
   else   VCOSEL = 1;

   // read 0x09 register bit [5:0]
   _temper=0x0C;
   admtv102_readreg(state, 0x09, &_temper) ;  //To get Temperature sensor value
   _temper &= 0x3f; // get low 6 bits 
   
   dpPhaseTuning(state, lofreq,_temper); //these value may changes: g_icp=0, state->convco=0, state->curTempState=HIGH_TEMP;
   
   if (lofreq>400000)  //if iRF=UHF
   {   
      admtv102_writereg(state, 0x1a, (u8)((state->icp<<2)| ((PLLN&0x300)>>8)));    // change 1C to 7C. 2007/07/09
   }
   else 
   {
       admtv102_writereg(state, 0x1a, (u8)( 0xFC | ( ( PLLN&0x300 ) >> 8 )) );
   }

   //PLL Setting
   admtv102_writereg(state, 0x1b, (u8)(PLLN & 0xFF));
   admtv102_writereg(state, 0x1c, (u8)( (DIVSEL<<4) | (VCOSEL<<7) | ((PLLF&0xF0000)>>16) ) );
   admtv102_writereg(state, 0x1d, (u8)( (PLLF&0x0FF00)>>8 )   );
   admtv102_writereg(state, 0x1e, (u8)( PLLF&0xFF ));
   admtv102_writereg(state, 0x15, (u8)( 0x38| ((lpfBW-3)& 0x07) ));

   //PLL Reset
   admtv102_writereg(state, 0x2f, DATA47);      //Reset Seq. 0 => 1 => 0 : //updated : v1.5 at PM 20:39 2007-06-8
   admtv102_writereg(state, 0x2f, (u8)( DATA47 | 0x80 ));
   admtv102_writereg(state, 0x2f, DATA47);
   
   admtv102_writereg(state, 0x27, state->convco );
   if (lofreq>400000)    admtv102_writereg(state, 0x29, 0xBF );	 
   return; 	
}
//*****************************************************************************
// Function:  calculate DIVSEL according to lofreq                                             
//  Input:   lofreq  -- Frequency point value in kHz unit 
//  Return:  m_DIVSEL -- DIVSEL in register 0x1C  
//*****************************************************************************
int  LO2PLL_Freq(u32 lofreq)
{
   u32 fdefLoBoundFreq=940000;

   int	Seg_num, m_DIVSEL=0;
    
   Seg_num=(int)(lofreq/(fdefLoBoundFreq/16));

   while(Seg_num>0x01)
   {
     Seg_num=(Seg_num>>1);
     m_DIVSEL++;
   }
   return(m_DIVSEL) ;
}

//******************************************************************************
// Function: Change CONVCO value according to Temperature Sensor(0x27[0:4])
// Input: lofreq -- frequency in kHz unit 
//        temper -- Tuner 0x09 register content  
// Return: None

// ?? is this messing around with globals!!?!?!?

//******************************************************************************
void dpPhaseTuning(struct admtv102_priv *state, u32 lofreq, u8 temper)
{  
  if (fDegVcoApply)
  {
    if(state->VHFSet== VHFSupport) {
   	if (temper<=vlowDegBoundary)  //low boundary
	{
	  state->convco=rglowDegCONVCO_VHF; 
	  state->curTempState=LOW_TEMP;
	}
	else
	  if (temper>=vhighDegBoundary) //high boundary
	  {
		state->convco=rgHighDegCONVCO;
		state->curTempState=HIGH_TEMP;
	  }	
    }
   else {   	
    if (temper<=vlowDegBoundary)  //low boundary
    {
      state->convco=rglowDegCONDIV;
      state->icp=0x3F;
     state->curTempState=LOW_TEMP;
    }
    else
      if (temper>=vhighDegBoundary) //high boundary
      {
	state->convco=rgHighDegCONDIV;
	if (( lofreq > 610000  ) && ( lofreq < 648000  )) //610MHz ~ 648MHz
	    state->icp=0x1F;
	else  state->icp=0x3F;
	state->curTempState=HIGH_TEMP;
      }
      else
	  {  if (state->curTempState)
	     {
		 state->convco=rglowDegCONDIV;
		 state->icp=0x3F;
	      }
	     else
	      {	state->convco=rgHighDegCONDIV;
		if (( lofreq > 610000  ) && ( lofreq < 648000  )) //610MHz ~ 648MHz
		  state->icp=0x1F;
		else  state->icp=0x3F;
	      }	
	  }
     }
  }
  return;
}
//******************************************************************************
// Function: Do tuner temperature compensate, it can be called by processor 
//           for every 5~10 seconds. This may improve the tuner performance.  
//  
// Input: lofreq -- frequency in kHz unit 
// Return: None
//******************************************************************************
void    TunerTemperatureComp(struct admtv102_priv *state, long lofreq)
{  
    u8  Ori_Reg_0x1A,Reg_0x1A,_temper;

    Ori_Reg_0x1A=0xFC; 
    _temper=0x0C;
    admtv102_readreg(state, 0x09, &_temper) ;  //To get Temperature sensor value
    _temper &= 0x3F; // get low 6 bits 

    dpPhaseTuning(state, lofreq, _temper);

    admtv102_readreg(state, 0x1A, &Ori_Reg_0x1A) ;  //To get Temperature sensor value

    // reserve bit 7, bit1 and bit 0
    Reg_0x1A= (u8)((state->icp <<2) | (Ori_Reg_0x1A & 0x83)); 
    // write state->icp into 0x1A register bit[6:2] 
    admtv102_writereg(state, 0x1A, Reg_0x1A);
    admtv102_writereg(state, 0x27, state->convco);


}

/* ****************************************************************************
 *  Function: calculate signal power using Tuner related registers 
 *  Input: None
 *   Return: signal power in dBm unit  
 *  Note: The precise is about 10dB
 * ****************************************************************************/
#define COMP497 497
#define COMP530 -530

#define FREQ474 474
#define FREQ858 858


#if 0
int   TunerRSSICalc(struct admtv102_priv *state, int freq)
{
 	int PowerDBValue=0; // in dBm unit
	unsigned char RF_AGC_LowByte =0,RF_AGC_HighByte=0,LNA_Gain=0,GVBB=0;
	int RF_AGC;
	int CaseNumber;
	int Coef[5];
	//int CompTab[2]={ 497, -530};
	//int FreqTab[2]={ 474, 858 };
	int Freq_Comp,BBAGC_Part;
	unsigned char Val_0x3D,Val_0x3e,Val_0x3f;

	
	
	Freq_Comp= -( 
		(COMP530 - COMP497) *
			(freq - FREQ474) /
				(FREQ858 - FREQ474) 
		+ COMP497 ); //0.01dB

	GVBB=0;
	admtv102_readreg (state, 0x05, &RF_AGC_LowByte); 
	admtv102_readreg (state, 0x06, &RF_AGC_HighByte); 
	
	RF_AGC= (RF_AGC_HighByte & 0x01);
	RF_AGC= (RF_AGC<<8) + RF_AGC_LowByte;
	
	admtv102_readreg (state, 0x0d, &LNA_Gain); 
	LNA_Gain = ( (LNA_Gain & 0x60 )>> 5 );

	admtv102_readreg (state, 0x04, &GVBB); 
	GVBB=(GVBB&0xf0)>>4;

	Val_0x3D=0;
	Val_0x3e=0;
	Val_0x3f=0;
	
#if 0
	admtv102_readreg (state, lowDemodI2CAdr, 0x3d, &Val_0x3D);
	admtv102_readreg (state, lowDemodI2CAdr, 0x3e, &Val_0x3e);
	admtv102_readreg (state, lowDemodI2CAdr, 0x3f, &Val_0x3f);
	if (Val_0x3f & 0x10) 
		Val_0x3f= (Val_0x3f & 0x0f);// two's complement to offset binary
	else 
		Val_0x3f= (Val_0x3f|0x1f);      //
#endif

	BBAGC_Part= Val_0x3f;
	BBAGC_Part = ( BBAGC_Part <<8) + Val_0x3e; 

	CaseNumber=0; // default algorithm
	if (LNA_Gain==0 && GVBB==4)  CaseNumber=1; //[0,-31dBm], case 1 
	if (LNA_Gain==3 && GVBB==4 && RF_AGC<383)  CaseNumber=2; //[-34dBm,-59dBm], case 2
	if (LNA_Gain==3 && GVBB>4 && RF_AGC>=383)  CaseNumber=3; //[-61Bm,-93dBm], case 3
	if ( (LNA_Gain==0 && GVBB>4) || (LNA_Gain==3 && GVBB<4) ) CaseNumber=4; //[-2,-52dBm], case 4 
	if (LNA_Gain==3 && GVBB>4 && RF_AGC<383)  CaseNumber=5; //[-53,-78dBm], case 5 
	if (GetTunerType(state,lowTunerI2CAdr)==Tuner_MTV102) CaseNumber=0; //use default formula
	
	//printk("RSSI Calc use formula %d\n", CaseNumber);
	// basically all the tests I've found have ended up with cast 5!!!1
	
	
	// if BBAGC > 0.97, case number is forced to 0, liu dong, 2008/04/22
	if(Val_0x3f >= 0x10) CaseNumber=0;

	switch(CaseNumber)
	{  case 1:  { 
		      Coef[0]=-25;  Coef[1]=-1000; Coef[2]=-287; Coef[3]=-11400 ; Coef[4]=-310;  
		      BBAGC_Part=((Coef[3]*BBAGC_Part)/8192) + 5016;   // 5016 = Coef[3]*0.44
	              PowerDBValue=Coef[0]*RF_AGC+Coef[1]*LNA_Gain+Coef[2]*(GVBB-5)+BBAGC_Part+Coef[4]+Freq_Comp;
	              break;
		    }
       case 2:  { 
		  Coef[0]=-13;  Coef[1]=-752; Coef[2]=214; Coef[3]=1000 ; Coef[4]=886; 
		  BBAGC_Part=((Coef[3]*BBAGC_Part)/8192)- 440;   // 440 = Coef[3]*0.44
                  PowerDBValue=Coef[0]*RF_AGC+Coef[1]*LNA_Gain+Coef[2]*(GVBB-5)+BBAGC_Part+Coef[4]+Freq_Comp;
	          break;
		}
       case 3:  { 
                  Coef[0]=-12;  Coef[1]=-952; Coef[2]=-347; Coef[3]=-11400 ; Coef[4]=120; 
		  BBAGC_Part=((Coef[3]*BBAGC_Part)/8192) + 5016;   // 502 = Coef[3]*0.44
	          PowerDBValue=Coef[0]*RF_AGC+Coef[1]*LNA_Gain+Coef[2]*(GVBB-5)+BBAGC_Part+Coef[4]+Freq_Comp;
	          break;
		}
       case 4:  { 	
  		  Coef[0]=-12;  Coef[1]=-1000; Coef[2]=-330; Coef[3]=-11400 ; Coef[4]=70; 
		  BBAGC_Part=((Coef[3]*BBAGC_Part)/8192) + 5016;   // 5016 = Coef[3]*0.44
	          PowerDBValue=Coef[0]*RF_AGC+Coef[1]*LNA_Gain+Coef[2]*(GVBB-5)+BBAGC_Part+Coef[4]+Freq_Comp;             
	          break;
		}
       case 5:  { 	
		  Coef[0]=-12;  
		  Coef[1]=-952; 
		  Coef[2]=-280; 
		  Coef[3]=-11400 ; 
		  Coef[4]=120; 
		  
		  BBAGC_Part=((Coef[3]*BBAGC_Part)/8192) + 5016;   // 502 = Coef[3]*0.44
		  // LNA Gail == alway s3 ... (due to case)
		  // @ 562 => Good:AGC < 290 ,  (GVBB-5) < 5 
		  // @ 602 => Good:AGC < 290 ,  (GVBB-5) < 4
		  // @ 746 ?? => Good:AGC < 290 ,  (GVBB-5) < 4  * at this freq,  
		  
		  
		  //printk("(freq=%d) RF_AGC=%d, LNA_Gain=%d, GVBB=%d, BBAGC_Part=%d, Freq_Comp=%d\n",
			//	freq, RF_AGC, LNA_Gain, GVBB-5, BBAGC_Part, Freq_Comp);
		  
		  PowerDBValue=	Coef[0] * RF_AGC +
						Coef[1] * LNA_Gain +
						Coef[2] * (GVBB-5) +
						BBAGC_Part +
						Coef[4] +
						Freq_Comp;
	          break;
		}
       default:  {  	
	          Coef[0]=-12;  Coef[1]=-830; Coef[2]=800;
	          PowerDBValue=RF_AGC*Coef[0]+LNA_Gain*Coef[1] +Coef[2];
               	  BBAGC_Part=  -1000*BBAGC_Part/8192;	
		  PowerDBValue=PowerDBValue-300*(GVBB-5)+BBAGC_Part + Freq_Comp;  // for 8934 case 
                  break;
		}
	}
	
	return(PowerDBValue);
}

 
 
int TunerRSSICalcAvg(struct admtv102_priv *state, int freq) 
{
	
	int i,RSSI_val_total=0;	
	for(i=0;i<RSSIAveTimes;i++) // measure RSSI for RSSIAveTimes times.
	{ 
		RSSI_val_total+= TunerRSSICalc(state, freq);   	    
	     	     
	} 
	return RSSI_val_total/RSSIAveTimes;
}
#endif

int  TunerPllLockCheck(struct admtv102_priv *state)
{  
   int TunerI2C_status, DemodI2C_status1, DemodI2C_status2; //record the I2C access status 
   int Tuner_pll_lock, Demod_CALOCK, Demod_AutoDone; //record the bit status 
   int check_mode; 
   int Tuner0x06Lock,TunerADout;
	u8 t;
    
   TunerI2C_status=DemodI2C_status1=DemodI2C_status2=0;
   Tuner_pll_lock=Demod_CALOCK=Demod_AutoDone=0;
    
   check_mode=1;

   // check if the Tuner PLL is locked or not 
	admtv102_readreg(state, 0x06, &t);
	if ((t & 0x02) == 0x02)
		Tuner0x06Lock=1; 
	else
		Tuner0x06Lock=0; 

	admtv102_readreg(state, 0x04, &t);
	TunerADout=0xff;
	TunerADout = t & 0x0f;

   if ( ( ( TunerADout > TunerADOutMin ) && ( TunerADout < TunerADOutMax ) ) && Tuner0x06Lock )  //pll lock cross check
	  Tuner_pll_lock=1;
   else Tuner_pll_lock=0;
   return Tuner_pll_lock;
}

/* ========================================================================= */


static int admtv102_set_params(struct dvb_frontend *fe)
{
	struct admtv102_priv *priv;
	u32 freq;
	int i;
	int ret = 0;
	//u8 bw = 8;
	struct dtv_frontend_properties  *c = &fe->dtv_property_cache;
	u32 delsys = c->delivery_system;
	u32 bw = c->bandwidth_hz;
	
	priv = fe->tuner_priv;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1); /* open i2c_gate */

	freq = c->frequency / 1000 / 1000; // Hz -> MHz

	priv->bandwidth = c->bandwidth_hz;
	if (delsys == SYS_DVBC_ANNEX_B) ;
	


//	if (fe->ops.info.type == FE_OFDM) {
//		priv->bandwidth =  c->u.ofdm.bandwidth;
//		printk("params->u.ofdm.bandwidth = %d\n", c->u.ofdm.bandwidth);
	
//		switch (c->u.ofdm.bandwidth) {
//			case BANDWIDTH_6_MHZ:
//				bw = 6;
//				break;
//			case BANDWIDTH_7_MHZ:
//				bw = 7;
//				break;
//			case BANDWIDTH_8_MHZ:
//				bw = 8;
//				break;
//		}
//	}

	SetTunerFreq(priv, freq, bw);

	//Waits for pll lock or timeout
	i = 0;
	do {
		if (TunerPllLockCheck(priv)) {
			printk("Tuner PLL Locked\n");
			break;
		}
		msleep(4);
		i++;
	} while (i<10);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0); /* close i2c_gate */

	return ret;
}

static int admtv102_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct admtv102_priv *priv = fe->tuner_priv;
	*frequency = priv->frequency;
	return 0;
}

static int admtv102_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct admtv102_priv *priv = fe->tuner_priv;
	*bandwidth = priv->bandwidth;
	return 0;
}

static int admtv102_init(struct dvb_frontend *fe)
{
	struct admtv102_priv *state = fe->tuner_priv;

	TunerInit(state);  // init Tuner   		
	SetTunerFreq(state, 746, 8);
	return 0;
}

static int admtv102_sleep(struct dvb_frontend *fe)
{
	return 0;
}

static int admtv102_release(struct dvb_frontend *fe)
{
	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
	return 0;
}

static const struct dvb_tuner_ops admtv102_tuner_ops = {
	.info = {
		.name           = "Analog Device ADMTV102",
		.frequency_min  =  48000000,
		.frequency_max  = 860000000,
		.frequency_step =     50000,
	},

	.release       = admtv102_release,

	.init          = admtv102_init,
	.sleep         = admtv102_sleep,

	.set_params    = admtv102_set_params,
	.get_frequency = admtv102_get_frequency,
	.get_bandwidth = admtv102_get_bandwidth
};

/* This functions tries to identify a MT2060 tuner by reading the PART/REV register. This is hasty. */
struct dvb_frontend * admtv102_attach(struct dvb_frontend *fe, struct i2c_adapter *i2c, struct admtv102_config *cfg)
{
	struct admtv102_priv *priv = NULL;
	u8 id = 0;

	priv = kzalloc(sizeof(struct admtv102_priv), GFP_KERNEL);
	if (priv == NULL)
		return NULL;

	priv->cfg      = cfg;
	priv->i2c      = i2c;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1); /* open i2c_gate */

	if (admtv102_readreg(priv,0,&id) != 0) {
		kfree(priv);
		return NULL;
	}

#if 0
	if (id != PART_REV) {
		kfree(priv);
		return NULL;
	}
#endif
	//printk(KERN_INFO "ADMTV102: successfully identified (ID = %d)\n", id);
	memcpy(&fe->ops.tuner_ops, &admtv102_tuner_ops, sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0); /* close i2c_gate */

	return fe;
}
EXPORT_SYMBOL(admtv102_attach);

MODULE_AUTHOR("David T.L. Wong");
MODULE_DESCRIPTION("Analog Device ADMTV102 silicon tuner driver");
MODULE_LICENSE("GPL");
