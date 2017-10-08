struct admtv102_priv {
	struct admtv102_config *cfg;
	struct i2c_adapter   *i2c;

	u32 frequency;
	u32 bandwidth;

	int VHFSet;
	u8 icp;
	u8 convco;
	u8 curTempState;
	u8 CTUNE_CLKOFS;
};

#define Tuner_MTV102		0x00 
#define Tuner_ADMTV102		0x01 
#define  Tuner_NewMTV102	0x02 

#define  VHFSupport               1  // 
#define  UHFSupport               0  // 

#define vlowDegBoundary    7
#define rglowDegCONDIV     0xEC
#define vhighDegBoundary   9
#define rgHighDegCONDIV    0x00
#define fDegVcoApply       1
#define HIGH_TEMP	   0
#define LOW_TEMP	   1
#define rglowDegCONVCO_VHF     0x9D
#define rgHighDegCONVCO        0x00

#define _EXTUNEOFF   0  
#define _EXTUNEON    1  
#define _TUNEEN      1 
#define _TUNEDIS     0 
#define CTUNEOFS     0x01  //default = 0 ;  

#define CTUNE_CLKOFS_SPLIT0E   0x09 /*added at v1.6.8*/
#define CTUNE_CLKOFS_SPLIT0F   0x00 // for mass product 
#define REFCLK30400_CLKSEL_REG_SPLITID0E   0x5A   //for split ID is 0x0e
#define REFCLK30400_CLKSEL_REG_SPLITID0F   0x6A   //for split ID is 0x0f

#define  TunerADOutMin  2
#define  TunerADOutMax  12

void ConfigTuner(struct admtv102_priv *state, u8 *AddrData);
void TunerInit(struct admtv102_priv *state);
int  GetTunerType(struct admtv102_priv *state);
void SetTunerFreq(struct admtv102_priv *state, u32 frequency, u32 lpfBW);
void SetLPF(struct admtv102_priv *state,u32 refClkType,u8 lpfBW);
int  LO2PLL_Freq(u32 lofreq);
int  TunerPLLRegSet(struct admtv102_priv *state, u8 *RegDat, int TunerPLLType);
int   TunerRSSICalc(struct admtv102_priv *state,int freq);
int   TunerRSSICalcAvg(struct admtv102_priv *state,int freq);
void dpPhaseTuning(struct admtv102_priv *state,u32 lofreq, u8 temper);
void TunerTemperatureComp(struct admtv102_priv *state,long lofreq);
int  CheckTunerI2C(struct admtv102_priv *state);
