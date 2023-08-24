/*  * Modbus C for: Morningstar® ProStar MPPT______________________________________________
 *  * This program is OpenSource: released under Artistic-2.0 license
 *  ****************** by sunja AT centerflowing.com Copyright © 2023 **********************
 * correct way: [-fsyntax-only : Only run the preprocessor, parser and type checking stages.]
 * cc -Wall $(pkg-config libmodbus --cflags) mcStarMPPT.c $(pkg-config libmodbus --libs) -lm -o mcPStarMPPT
 
 *  - 	Logs	-	logs 0x8000 ... 0x8FFF data[256] each is 32bytes(16 modbus registers). 
 *						------ sortby hourmeter (ignoring 0x000000 and 0xFFFFFF) ------
 *	-	Coils	-	reset logged totals in memory, clear faults, alarms... 
 
 *  -  Profiles: --decimal thousands [0.00n] can be rounded to nearest (Float16) hundreth!
 *				#------------------------------ie. 15.5660 => 15.5625 ,  15.5670 => 15.5703

 ****************************************************************************************************
 *	**possible conflicts are if MAX_CHAR length is less than 256 on compiling system...?
 *  **possible conflicts with Y2038 time_t date bug!
 *  * Display-text output will break if data structs are rearranged, due to getIndex() not being used for all..
 *    		Combo-fields will also break if not read in right order..
 	//--MaxLimit string lengths of global config settings:? (<128)? - fileOut , profileDir , (profile)
	//----total char strln > 256: ... printOut(), jsonOut():jsontxt[655] is biggest.
 *
 *   	+cvs output!, !externalize configuration!, +wide char support, 
 *  ++ Profiles: custom descriptions vs. translation, +only in profiles [debug for now] filter in chkProfile(), 
 * +------cmd arg: -d toggle display ,  late night normalization as option?? not for -s[ilent]
 *	ToDo:	default defaults, static to functions, printOut()?, chkProfile() parsing combine for settings,
			 HVD checking and set value to 0 in parseProfileValue(), escapeStr("file",), 
 
 	RAM 0x0001: voltage multiplier - voltages stored as 12v & multiplied! PStar==1,2, TStar==1,2,4 (12,24,48)
Notes:
	* voltages in profiles & structs are set for 12v. 24v & higher systems are auto-multiplied by charger!
	* "use lower EQ V for new batteries" 
	* Temp Compensation shifts voltages upwards [when cold].
 */

#include <stdio.h>  
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <time.h>
#include <errno.h>	
#include <stdarg.h>
#include <math.h>
#include <modbus.h>	
#include <ctype.h>		//usedfor: isspace() adds 48bytes
#include <stdint.h>		//
#include <unistd.h>		//usedfor:
#define CUPDATEABLE 14 	/*  */
#define UPDATEABLE 37 	/*  */
#define MSMPPT    0x01	/* Fallback modbus address of the MPPT */ //---:[0xE034] */
#define VERSION   1.01
int MAX_CACHE_SIZE = 200000;		//-:max file size (bytes) before rebuilding

/* //---------------------CONFIGURATION---------------------------------------//: */
//static char externalize_config=0; //--:read configuration from file and use below as fallbacks? 
//--------------------------------------:   will create file on startup if none exists.
unsigned char modbusId=0;  //:modbus address of the MPPT [0xE034]: 0==Default (MSMPPT)
char display=1; 
static char date_format[64] = "%a %b %d %Y"; //--format for dates (w/o time)
char json=0; char cvs=0; //---create json [or cvs] files
static const char* fileOut = "PStarMPPT_"; 	//--output files. (json,logs,cvs,etc.) 
static const char* outDir = ""; 			//--default output directory. [last slash required...] 
/* Charger Updates & Backups *///-filenames:[profileDir+profile+profileTag+".txt"]
static const char* profileDir = "./profiles/"; 	//--default profile directory. [last slash required...] 
static char profile[64] = ""; 					//--name of default profile (for update cmd)
static char profileTag[24] = "_profile";  		//--:appended to all profiles (required)
static char profileBackups[32] = "settings";  	//-:file name prefix for backups
/* External Date Cache *///-------filename:[profileDir+log_cache+"_"+fileOut+".txt"]
static const char real_date_logs=1;  //-:enable small external cache for [rl] dating of logs.
static char log_cache[32] = "logCache";  //-:file prefix
//--Date format in log cache:-------------------------can be: m-d-yyyy , d-m-yyyy , yyyy-d-m
static const char log_cache_month_position = 1;  //--:Month is: 1=first, 2=second, 3=third
/* Locked EEPROM values for security or safety reasons */
static int locked_registers[UPDATEABLE] = {0xE034,0xE035,0xE01A}; //modbus,meterbus,temp comp.
static const char update_all_defaults=0; //-:enable builtin 'update all' cmd.. (security & safety risk!)
/* Display note for your battery's voltages: Specific to battery make and type. (default is completely generic!)  */
static char batteryVoltagesNote[255] = "Generic Battery Voltages:\n 100%:[12.7] volts, 90%:[12.6], 80%:[12.5], 70%:[12.3], 60%:[12.2], 50%:[12.1], 40%:[12.0], 30%:[11.9], 20%:[11.8], 10%:[11.7]";
/*  Max / Min Voltages - All input voltages 12v in eeprom!  */
static float maxV = 0; 	//:profile input safety check.. [default fallback=18] (EQ||HVD==highest)
static float minV = 0;  //:profile input safety check.. [(>10.3 volts) , default fallback=11.5]
/* Debug */ 
//--0=print Display, 1=print RAM/EEPROM & display, 2=verbose debug, >=3=no writes, >3=+logCache & profiles debug...
char debug = 0; 

/* //-------------- BUILTIN UPDATES TO SEND -------------------------------------// */
/*/------------------------------------------------:(see arguments)
 * --COILS-----------------------------------------: 
 * ------------------------------------------------: */
static int coil_updates[CUPDATEABLE] = { 0x0016 }; //-:cmd arg "resetC" triggers these coils
//--NonArg choices---------:   
// load disconnect:0x0001, charge disconnect:0x002
// eeprom update: 0x0016, lighting test:0x0020, factory reset: 0x00FE
//---EQ note: 0x0013 (:hack making EQ coil 0x0000 findable)
 
/*/------------------------------------------------:
 * --EEPROM UPDATES--------------------------------: 
 * ------------------------------------------------:  
 */
//--:updates[] built-in here only used with cmd 'update':
//---:leave empty to disable (a possible safety risk if battery types change)
static int updates[UPDATEABLE] = 
		{ }; 
//--Charge Updates [times,voltage]:
//	{0xE002,0xE003}; 	//absorb times	
//	{0xE000,0xE001,0xE004,0xE005, 0xE01A, 0xE010}; //voltages
//--EQ-------------------:
//	{0xE007,0xE008,0xE009,0xE00A}; 
//--Status led-----------:
//	{0xE030,0xE031,0xE032,0xE033};
/* -----------------------end config-------------------------------------------- */


/* //---------------------GLOBAL VARS---------------------------//Do Not Change: */
//typedef unsigned long ulint;  //--:..unused. lint
typedef unsigned int uint; 
typedef unsigned short ushort; //'uint16_t' (aka 'unsigned short') 
static const char cvsH[] = "Register,new value,description,\n";
static const char cvsL[] = "Hourmeter,unix date OR date string,\n";
static char update=0; 		//  1==write, 2==(deprecated currently,use profile)
static char trigger_coils=0; 	//static char vmultiplier=0;
char polling=0; char just_EEPROM = 0; 
static char warn=1; static char sil = 0; 
static char searching=0; char raw=0; static const char maxLogs=6;
static char nH = 6; //-:normalize time for log search & cache [24-nH]: 6(hrs)==21600secs==6pm
static char lbuf_input=0;	//-:buffer logs from input
static time_t c_time;   static long now=0; 
static char file_out[128] = ""; static char file_poll[128] = ""; 
static char file_logs[128] = ""; static char file_logCache[128] = ""; 
//--------units types: new type for hourmeters=='l'
	static char days[] = "days"; static char secs[] = "secs"; static char hours[] = "hours";
	static char mins[] = "mins"; static char blank[] = "-";  static char bits[] = "bit"; 
	//static char Ah[] = "Ah"; 	 static char kWh[] = "kWh";  //char voltage[] = ""; char celsius[] = "C";

/* //---------------------GLOBAL RAM STRUCTURES----------------------------//: */
//----union to hold registers parsed value:
union Value { float fv; signed int dv; char sv[255]; long lv;   }; 
typedef union Value uValue;
//---Data structures:---------------------------------------------://
struct PaRam { uint logical; uint hexa; char unit[9]; char string[255]; //--
			float f_def;   char calc[128];  char typ;   //-.f_def is float to hold defaults! then parsed in profile..
			char updated;  char group[24]; uint16_t basev;  uint16_t f_new;	// uint16_t: max==65535 
			uValue value;  char sv[16];  }; 			//--union for parsed values, (char[16] for profileIn)
typedef struct PaRam RamObj; //typedef for functions
//--Bulk reading: +static?
struct bulkData { uint start; ushort i; }; 
//--Logs:
struct logStruct { time_t date; RamObj log[16]; char date_s[128]; }; //--
typedef struct logStruct LogObj;
//--log cache input:
struct logCacheStruct { long hrmtr; time_t udate; char date_s[128]; char line[256]; char nHrs; int xt; }; //--
//typedef struct logCacheStruct LogCacheObj;
static struct logCacheStruct logCache [256]; //+1, logCache[0] is metadata
//--Coils:
struct CoilsObj { uint logical; uint hexa; char *string; int trigger; int off; };
//--Profile inputs:
static struct PaRam profileIn [UPDATEABLE];
//--external settings / translations:
//struct configStruct { char key[25]; char value[128];  }; //--
//static struct PaRam configIn [CONFIGURABLE];

//------------------------FUNCTIONS AT TOP--------------function prototypes--//
float F16ConvertToF32(uint16_t f16);
uint16_t fl_to_half(const float x);
uint as_uint(const float x);
//uint16_t F32ConvertToF16(float f32); //broken! unused!!

//--Update function:------------: 
static void readModbus(modbus_t *ctxx, uint start, ushort length, uint16_t *datas);
static uint16_t writeUpdate(modbus_t *ctxx, RamObj *inStruct);  //:write data, check, return current value
static short readLogs(modbus_t *ctxx, uint start, short tnum, LogObj *logs); //-:returns number of logs filled
static short parseLogs(uint start, short logx, LogObj *logs, uint16_t datal[], short bnum);
static short searchLogs(modbus_t *ctxx, uint start, long search, short tnum, LogObj *logs);
static const long readLogCache(char doo[]);  
static const long writeLogCache(char dool);
static void searchLogCache(long hrmeter, time_t *, int *); 
//static void printLogs(LogObj *logs, size_t logsN);
static const char* chkProfile(char doo, char* profileName);
static char parseProfileValue(char doo[], int pi, int ep, RamObj *eprom);
//--Get Registers:--------------:
void upRegisters(modbus_t *ctxx, RamObj *inStructA, int num, int *registers, int rn); 

//--Parse Memory Value:const 
const RamObj parseValue(RamObj *inStruct, uint16_t dvalue);	//*--Main Loop function  void??
//--Combine double address items:--(previous data must already be parsed):
static long combine(int hex, uint16_t lo, short num, RamObj *ramIn); 
//--Settings State translation:-:
const char* getStateString(int hex, long dvalue);
uint16_t dipsChargeMode(uint16_t dvalue); 
//--get register/coil indexes:---------:
int getIndex(int hex, size_t total, RamObj *inStruct);
int getCoil(int hex, size_t total, struct CoilsObj *inStruct);
//--Utils:----------------------:
static const char* appendStr(int num, ...); //-:numbers need to be stringified before passing!
const float secsToHours(int num); //float
static const int searchStr(const char* search, const char* string);
static char* escapeStr(const char* doo, char* string);
static char* strReplace(const char* search, const char *replace, const char* string);
void trimP (char *str); 	//-inplace trim
//char* trim (char *str); 	//-non inplace trim
//--Print to cli or File:-------:
static short writeFile (char action[], time_t date, char content[]); 
static short createProfile(char action[], int nume, RamObj *eprom0, char ctime_s[]); 
static void printOUT(int ind, char type, RamObj *inStruct); //...to cli
static void jsonOut(char action[], int cr, RamObj *ram0, int ce, RamObj *eprom0); 
void pollingOut(modbus_t *ctxx, int count, RamObj *ram0, char* string); 


/* /______________________________Main()__________________________________________/ */

int main(int argc, char *argv[]) {
	size_t num, nume; char* action = ""; char skip_RAM = 0; char mrk=0; char logOpt=0; //char action[] = "";
	short logn=0; short tnum=0; long lint=0; long lbuf=0; //: logs...long OR unsigned long?
	//char config_file[128]=""; //-:external config file:
	//strncpy(config_file,appendStr(3, outDir, fileOut, "_config.txt"),(size_t)128);
	//--BUFFERs----------------------:
	FILE *fp;	char sbuf[128]=""; char bufs[256]=""; //--output buffering! 
	//----------------------------------TIME / DATE-------------------------------------------:	
 	c_time = time(NULL);  char ctime_s[128]; //
	//char* ctime_s = NULL; ctime_s = malloc((size_t)128);  //ctime_s = ctime(&c_time);
		
	/**  EEPROM - defaults entered here are used for: creating new profiles when disconnected from charger, 
	 ** 		the 'revert' and 'revert all' cmds [when configuration update_all_defaults is enabled..]
	 **			defaults can be Floats or Float16, they are run thru chkProfile() similar to profile values. 
	 ------------- ALL VOLTAGES MUST BE 12v BASED! They are multiplied for systems running @ >12v !---------
	---------------------------------------------------------------------------------------------------------- */
	//int eereserved[] = {0xE00B,0xE00C, 0xE00E,0xE00F, 0xE011,0xE012, 0xE014-19, 
	//															(0xE020-21), 0xE028-2F, 0xE039-3F}; //--out of total
	struct PaRam eprom [53] = {											//--Custom Defaults:
		//--Charger Settings:
		{0, 0xE000,"v",		"absorb v [full] @ 25C", 19290, .calc="f16"}, 	//(14.70v)****
		{0,	0xE001,"v", 	"float v @ 25C", 19136, .calc="f16"},			//(13.50)	****13.50v** *0 to disable**
		{0, 0xE002,"secs",	"absorb time", 10800},						//(3hrs)* 
		{0, 0xE003,"secs",	"low bat. extra absorb time",14400}, 		//(4hr)*longer absorb
		{0, 0xE004,"v", 	"trigger extra absorb",18996, .calc="f16"}, //(12.40)* (75%standard)
		{0,	0xE005,"v", 	"skip float",18983, .calc="f16"},    		//(12.30)* lowvoltage=absorbAllDay
		{0,	0xE006,"secs", 	"exit float if time below v",1800}, 		//(30mins)
		//---EQ: corrective=3-4hrs 60~180 days, maintenance=1-2hrs every 30,60,90 days 
		{0,	0xE007,"v", 	"EQ voltage @ 25C",19367, .calc="f16"}, 	//(15.30) *****(lower for newer)
		{0,	0xE008,"days", 	"days b/t EQ",28},							//Big or small (bi monthly?) EQ
		{0, 0xE009,"secs",	"EQ time-out limit @ > bulk, < EQv",14400},	//(4hr)
		{0,	0xE00A,"secs", 	"EQ time limit: @ EQv",7200}, 				//(2hr[7200],3hr[10800])  			
		{0, 0xE00D,"x", 	"Alarm on changes",0}, /*/// */
		//---Temp Compensation & Max Voltages:------------:
		{0,	0xE010,"v", 	"Ref.reg Max Charge Vout limit",0, .calc="f16"},	//MS disabled.(~15v) temp comp.	
		{0, 0xE013,"A", "Charger max Amps limit",0, .calc="f16"}, /*/// */
		{0,	0xE01A,"v", 	"temp comp. (12V lead-acid ~ -0.03 V/C)", 42926, .calc="f16"}, //(-0.02999)!standard!!	
		{0, 0xE01B,"v",		"HVD",19520, .calc="f16"}, {0, 0xE01C,"v","HVR",19174, .calc="f16"},  //hvd 17.0v
		{0, 0xE01D,"v","Abs. uncomp. Max Charge Vout limit", 0, .calc="f16"}, 	//MS disabled. compared to 0xE010
		//--Charging Temperatures:-------------------:ram 0x002C, 0x002D: min -20C standard!  
		//-cold charging: 10C(50F)? to 0C?! no charging below 0C!   //-max battery temp:  80==21760, 51deg C, 
		{0, 0xE01E,"C","max comp. temp clamp",21120,.calc="f16"}, {0, 0xE01F,"C","min comp. temp clamp",65506,.calc="f16"},  
		{0, 0xE020,"C","ETb_lo_limit_100",0,.calc="f16"}, 	  {0, 0xE021,"C","low temp. limit_0 cutoff",0,.calc="f16"},
		//!!!!Etb_lo_100: 10C? 0 to disable? 	 
		/*/---Load Settings:--------------------------------: // */
		 // +{0xE022..0xE027,  "Load Settings"}  //LVD(11.5v,11.398v), LVDR(12.7v,12.60)
		 {0, 0xE022,"v","LVD",18880, .calc="f16"},		{0, 0xE023,"v","LVR",19034, .calc="f16"},
 		 {0, 0xE024,"v","Load HVD", .calc="f16"}, 	{0, 0xE025,"v","Load HVR", .calc="f16"}, //0, 14.5v
		 {0, 0xE026,"x","LVD Load current comp. (ohms)",9134, .calc="f16"}, //.014999 ohms !standard!!
		 {0, 0xE027,"mins","LVD warning timeout",4},	  
		
		//---LED Status:-----------------------------------:  (morningstar defaults,visually useful)
		{0, 0xE030, "v",	"LED ChargingBulk G->G/Y",19110, .calc="f16"},		//(13.2968,13.45) charging/charged	
		{0, 0xE031, "v",	"LED ChargingFull G/Y->Y",19073, .calc="f16"},		//--(13.00*,12.65) 	standard:100%
		{0, 0xE032, "v",	"LED Charging Y->Y/R ",19027, .calc="f16"},			//--(12.6484,12.30) standard:65%  
		{0, 0xE033, "v",	"LED Charging Y/R->R ",0, .calc="f16"},				//--(0,12.14) 		standard:58%
		/*/---MultiCharger settings:------------------------: // */
		 {0,0xE034, "x", "Modbus net ids",1},  {0,0xE035, "x", "Meterbus net ids",1},  
		 {0,0xE036, "v", "Fixed Vmp",0, .calc="f16"}, {0,0xE037,"x","Vmp fraction of Voc",0, .calc="f16"}, //fraction=(0 to 0.99)
		 {0,0xE038, "A", "Charge current limit", .calc="f16"}, 
		
		//---Read only totals:------------------------------:  , .group="LO/HI",
		//"resettable totals (hours,Loads,Ah,kWh)"		 //-same as in RAM.. [002A and 002B]
		 {0,0xE040,"lhours","Hourmeter bitfield LO"}, {0,0xE041,"lhours","Hourmeter bitfield HI", .calc="comb"},
		 {0,0xE042,"A",	"Load resettable LO"},	 	{0,0xE043,"A",	"Load resettable HI", .calc="comb=n*0.1"},
		 {0,0xE044,"A",	"Load total LO"},	 		{0,0xE045,"A",	"Load total HI", .calc="comb=n*0.1"},
		 {0,0xE046,"A",	"Charge resettable LO"},	{0,0xE047,"A",	"Charge resettable HI", .calc="comb=n*0.1"},
		 {0,0xE048,"A",	"Charge total LO"},	 		{0,0xE049,"A",	"Charge total HI", .calc="comb=n*0.1"},
		 {0,0xE04A,"kWh","kWh resettable", .calc="f16"}, {0,0xE04B,"kWh","kWh total", .calc="f16"}, //n*0.1 ??
		{0,	0xE04C,"v", 	"battery voltage 24hr min", .calc="f16"}, 
		{0,0xE04D,"v", 		"battery voltage 24hr max", .calc="f16"},   
		{0, 0xE04E,"v", 	"solar voltage 24hr max", .calc="f16"},  
		{0, 0xE04F,"days", 	"days since EQ"},  
	};
	
	/* ---------------------- BUILD RAM STRUCTURE  --------------------------------- */
	//int reserved[9] = {3,4, 10,11,12,13,14,15,16 }; //--out of 82 total (doc. logical is +1 to real [hexadecimal]!!)	
	//--Faults/Alarms: Array (0x0022,0x0045), Controller (0x0038/9,0x0047/8), Load (0x002F,0x0046)
	//RAM: 0x0001 (volt multiplier) scaling is F16;  0x002A, 0x002B Kwh scaling is F16 (without n*0.1!) 
	struct PaRam ram [70] = { 
		//---Raw voltages:
		{1, 0x0000, "x", "software"}, 		{2, 0x0001, "x", "voltage multiplier",.calc="f16"},
		{5, 0x0004, "v", "3.3V supply",.calc="f16"}, {6, 0x0005, "v", "12V supply",.calc="f16"}, 
		{7, 0x0006, "v", "5V supply",.calc="f16"}, 
		{8, 0x0007, "v", "Gate Drive",.calc="f16"},	 {9, 0x0008, "v", "meterbus v\n",.calc="f16"},
		//---Current voltages, amps:---------------------:
		{17, 0x0010, "A", "Charge amps",.calc="f16"}, 		{18, 0x0011, "A", "array amps",.calc="f16"},
		{19,0x0012,"v","Battery voltage!",.calc="f16"},		{20,0x0013,"v","array voltage",.calc="f16"},  
		{21,0x0014,"v","Load v",.calc="f16"}, 				{23,0x0016,"A","Load amps",.calc="f16"}, //-load
			//!!out-of-order for bulk!!
		{22,0x0015,"A","Battery current (net)",.calc="f16"}, {24,0x0017,"v","battery sense wire",.calc="f16"},
		{25,0x0018,"v","battery (slow 60s)",.calc="f16"}, 	{26,0x0019,"A","battery amps (slow 60s)",.calc="f16"},
		//---Temperatures:--------------------------------:
		{27,0x001A,"C","heatsink temp",.calc="f16"}, 		{28,0x001B,"C","ambient/RTS temp",.calc="f16"},
		{29,0x001C,"C","ambient temp",.calc="f16"}, 		{30,0x001D,"C","RTS temp",.calc="f16"},
		 // {31..33,"Phase inductors temps"}
		//---Charger:------------------------------------:  
		{34,0x0021,"-","Charger state"}, 				{35,0x0022,"bit","Array fault"}, 	
		{36,0x0023,"v","Battery - slow filter (25s)", .calc="f16"}, 
		{37,0x0024,"v","Battery regulator target v", .calc="f16"}, //--!*MPPT target V! 
		{38,0x0025,"v","Regulator slave v (multiple chargers)", .calc="f16"}, //--"resettable (Ah,kWh) totals"--: 
		{39,0x0026,"A","Ah resettable HI", .group="HI"},{40,0x0027,"A","Ah Resettable LO", .group="LO", .calc="comb=n*0.1"},//!
		{41,0x0028,"A","Ah Total HI", .group="HI"},		{42,0x0029,"A","Ah Total LO", .group="LO", .calc="comb=n*0.1"}, //!
		{43,0x002A,"kWh","Charge Resettable", .calc="f16"}, {44,0x002B,"kWh","Charge Total", .calc="f16"}, //"n*0.1"! 
		//--low temp foldback limits! 0xE020-21 !!
		{45,0x002C,"C","low temp OK: goto 100%",.calc="f16"}, {46,0x002D,"C","low temp cutoff",.calc="f16"},
		//---Load Status:--------------------------------:{47..54,"Load Status"}, 
		{47,0x002E,"-","Load State"}, 				{48,0x002F,"bit","Load fault"}, 
		{49,0x0030,"v","Load LVD",.calc="f16"}, 	{50,0x0031,"v","Load HVD",.calc="f16"}, 
		{51,0x0032,"A","Load Ah HI", .group="HI"}, 	{52,0x0033,"A","Load Ah LO", .group="LO", .calc="comb=n*0.1"}, 
		{53,0x0034,"A","Load Ah total HI", .group="HI"}, {54,0x0035,"A","Load Ah total LO", .group="LO", .calc="comb=n*0.1"},
		//---Misc Settings:-------------------------------: {55..56,"hourmeter: in operation"}   //since mfdate?
		{55,0x0036,"lhours","Hourmeter bitfield HI"},	 {56,0x0037,"lhours","Hourmeter bitfield LO", .calc="comb"},
		{57,0x0038,"bit","Alarm bitfield HI"},	{58,0x0039,"bit","Alarm bitfield LO", .calc="comb"}, //--alarm bitfields
		{59,0x003A,"bit","Dip Switch"}, 		{60,0x003B,"-","SOC LED"},  			
		//---MPPT:----------------------------------------:
		{61, 0x003C, "W", "Charger output",.calc="f16"}, 		{62,0x003D,"v", "Vmp array (sweep)",.calc="f16"}, 
		{63,0x003E,"W","Array Power (sweep)",.calc="f16"}, 	{64,0x003F,"v", "Voc array (sweep)",.calc="f16"}, 
		{65,0x0040,"v","Array Target",.calc="f16"}, 		
		//---Daily LOGS: --reset at night-----------------:   , .group="day"
		{66,0x0041,"v","minBattery - (daily)",.calc="f16"}, {67,0x0042,"v","maxBattery - (daily)",.calc="f16"}, 
		{68,0x0043,"A","charge - (daily)",.calc="f16"}, 	{70,0x0045,"bit","Array Faults (day)"},	
		{69,0x0044,"A","Load Ah - (daily)",.calc="f16"}, {71,0x0046,"bit","Load faults (day)"},	//!!out-of-order for bulk!!
		{72,0x0047,"bit","Alarm bitfield HI (day)",.group="HI"}, 
		{73,0x0048,"bit","Alarm bitfield LO (day)",.group="LO",.calc="comb"}, //(0x0038)
		{74,0x0049,"secs","Absorb time (day)"}, 		{75,0x004A,"secs","EQ time (day)"}, 
		{76,0x004B,"secs","Float time (day)"}, 			{77,0x004C,"v","maxArray (day)",.calc="f16"},
		//---Misc-----------------------------------------:
		{78,0x004D,"-","Charge Status LED"},    {79,0x004E,"hours","Lighting Mode on (non0)/off (0)",65535}, // !!
		{80,0x004F,"v","Fixed Vmp (overrides 0x0050)",0,.calc="f16"},{81,0x0050,"x","Vmp fraction of Voc",0,.calc="f16"},
		{82,0x0051,"x","Alarm or Fault on changes"}
	};
	
	//--Coil resets:-------------------------------:
	struct CoilsObj coils [14] = {
		{1, 0x0000, "EQ toggle"}, {2, 0x0001,"Load disconnect: 0xFF00",0xFF00,0}, {3, 0x0002,"Charge disconnect",1,0},
		{17, 0x0010, "Ah Reset"}, {18, 0x0011, "Clear Ah Total"}, {19, 0x0012, "kWh Reset"},
		{21, 0x0014, "Clear Faults"}, {22, 0x0015, "Clear Alarms"}, {23, 0x0016, "Force EEPROM Update"}, 
		{25, 0x0018, "Clear kWh Total"}, {26, 0x0019,"Clear All Battery v min/max"}, {27, 0x0020,"Lighting Test"},
		{255, 0x00FE, "Factory Reset"}, {256, 0x00FF, "Reboot Charger"} 
	};
	
	/** ----------------------------Parse CMDS:------------------------------------------
	//----------------------------------------------------------------------------------- */
	{	 short i = 1; //--argv[0] is program name.
  	 while (i < argc) { //printf("\t%d>arg[ %s ]\n", i, argv[i]);
	    if (strcmp(argv[i], "-h")==0) { printf("\n__________ProStarMPPT* OpenSource Charge Control Software_____________\n" 
			"\n %s (no options = default display output)\n\n"
			" %s [output|control options] [update|logs options]\n\n"
			"Output Options:\n\t poll, json, json+ (display info), debug, debug+ (verbose debug), eeprom,\n"
				"\t -d (toggle info display), -s (silent)\n"
				"\t debugn (to stdout), raw (output raw float16 values)\n\n"  //, 
			"Control Options:\n\t reboot,   EQ [ON | OFF],\n"
				"\t reset_charge (kwh & Ah resetables), reset_totals,\n\t reset_all (all resetables), resetC (custom)\n\n"
			"Update Options:\n\t update  : (uses default profile if set), \n"
				"\t profile [nameof | validate nameof | create | backup]\n"
				"\t\t profile create - will use built-in defaults if offline, otherwise live values\n"
				"\t revert  : update using values built-in during compilation (disabled by default)\n\n"
			"Daily Logs:\n\t logs n [options] [start x || days x || hrs x]\n"
				"\t\t n being: number-to-return, x = [start addr || days ago || hrs hourmeter]\n"
				"\t Options: \n"
				"\t\t-b n | --buffer n  :buffer log dates with n hours\n"
				"\t\t--debug  :additional log debug output\n" //-:debug=2 or gt
				"\t 'logs all'  : will read All logs from charger, upto max of 256.\n"
				"\t Output options: can be specified before logs cmd..\n\n"
			"Utilities:\n\t convert [float value]   (...to F16)\n"
				"\t convert f16 [F16 value] (...to float)\n\n"
				"\t-v   print program version number and info.\n"
				"\n( see readme file for more info. --v%.2f-- )\n"
				"*(ProStarMPPT® is registered trademark to Morningstar Corp.)\n"
				"*( www.morningstarcorp.com )\n"
			"\n____*_*_* OpenSource Artistic-2.0 license © 2023 by sunja - centerflowing.com *_*_*____\n\n", 
			//"(may also be copied under terms of GNU General Public License)\n\n",
				argv[0],argv[0],VERSION); 
			exit(0); } 
		if (strcmp(argv[i], "json")==0) { json=1; display=0; }   
		else if (strcmp(argv[i], "json+")==0) { json=1; display=1; } 
		//else if (strcmp(argv[i], "cvs")==0) {  cvs=1; display=0; } //:disabled cvs
		else if (strcmp(argv[i], "debug")==0) { debug = 1; }
		else if (strcmp(argv[i], "debug+")==0) { debug = 2; } 
		//--coils:
		else if (strcmp(argv[i], "reboot")==0) { 
			trigger_coils = 1; memset(&coil_updates[0], '\0', CUPDATEABLE); 
			coil_updates[0] = 0x00FF; //int 
		}
		else if (strcmp(argv[i], "reset_charge")==0) { 
			trigger_coils = 1; memset(&coil_updates[0], '\0', CUPDATEABLE); 
			coil_updates[0] = 0x0010;   coil_updates[1] = 0x0012; //Ah, Kwh  
		} else if (strcmp(argv[i], "reset_totals")==0) { 
			trigger_coils = 1;  memset(&coil_updates[0], '\0', CUPDATEABLE); 
			coil_updates[0] = 0x0010;   coil_updates[1] = 0x0012; //Ah, Kwh  
			coil_updates[2] = 0x0011;   coil_updates[3] = 0x0018; //Ah, Kwh Totals
		} else if (strcmp(argv[i], "reset_all")==0) { 
			trigger_coils = 1;  memset(&coil_updates[0], '\0', CUPDATEABLE); 
			coil_updates[0] = 0x0010;   coil_updates[1] = 0x0012; //Ah, Kwh  
			coil_updates[2] = 0x0011;   coil_updates[3] = 0x0018; //Ah, Kwh Totals
			coil_updates[4] = 0x0014;   coil_updates[5] = 0x0015; //faults, alarms
			coil_updates[6] = 0x0019; //battery min/max (clears on reboot)
		} else if (strcmp(argv[i], "resetC")==0 && coil_updates[0]) { trigger_coils = 1; } //--:custom builtin
		else if (strcmp(argv[i], "EQ")==0) { i++; memset(&coil_updates[0], '\0', CUPDATEABLE); 
			coil_updates[0] = 0x0013; //EQ addr hack.
			if (argv[i] && strcmp(argv[i], "ON")==0) { trigger_coils = 1; }
			else if (argv[i] && strcmp(argv[i], "OFF")==0) { trigger_coils = 2; }
			else return -1;
		}
		//--Updates, Profiles: 
		else if (strcmp(argv[i], "update")==0) { 
			//--Update shortcut: use default "*_profile" if exists, else parse updates[]...
			if (profile[0] && profile[1]) { //--:"default" profile exists: 
				if (strcmp(profile, "new")==0 || strcmp(profile, "print")==0 || strcmp(profile, "-")==0) { 
					printf(">error: unsupported or reserved profile name!\n"); return 0;  }
				strncpy(profile, chkProfile('p',profile), sizeof(profile)); //(char*)profile 
			} else { fprintf(stderr,">No Default profile!\n"); return -1; } //display=1; //just_EEPROM = 1;   
			action="current_settings";  skip_RAM=1; break; 
		} 
		else if (strcmp(argv[i], "revert")==0) { 
			profile[0]='-'; profile[1]='\0'; i++; debug=2;
			if (argv[i] && strcmp(argv[i], "all")==0 && update_all_defaults) { 
				//-:builtin eprom values (deprecate?) -off by default-
				mrk=8; printf(">WARNING< Updating All Values to compiled Defaults!\n"); }
			else if (updates[0]) { mrk=1; //-:only use builtin updates[] (mark for parsing as profile)
				printf(">Updating to compiled Defaults!\n"); } 
			else { printf(">Built-in values unavailable.\n\n"); return -1; }    
			action="current_settings";  skip_RAM=1; break;
		}
		//--Profile cmds/options:
		else if (strcmp(argv[i], "profile")==0) { i++; 
			if (!argv[i]) return -1;  //---both "create" & "backup" exit after createProfile():
			else if (strcmp(argv[i], "create")==0) { strcpy(profile,"new"); action="current_settings"; mrk=5;
				skip_RAM=1; display=0; break; } //-uses: if (!modbus) builtin:current !!!!
			else if (strcmp(argv[i], "backup")==0) { strcpy(profile,"print"); action="current_settings"; 
				skip_RAM=1; display=0; break; } //-current eeprom  
			else if (strcmp(argv[i], "validate")==0) { action="validate"; i++; mrk=5;
				bufs[0]='\0'; strncpy(bufs, argv[i], sizeof(bufs));  //--Filter filename!
				//profile = chkProfile('p',bufs); } //-first parse-----------:Read Profile 
				strncpy(profile, chkProfile('p',bufs), sizeof(profile)); } 			//escapeStr("file",bufs)
			//--reserved names:
			else if (strcmp(argv[i], "new")==0 || strcmp(argv[i], "print")==0 || strcmp(argv[i], "-")==0) { 
				printf(">error: unsupported or reserved profile name!\n"); return 0;  }
			//--user profile update:
			else { action = "current_settings"; bufs[0]='\0'; //-:backup first.
				strncpy(bufs, argv[i], sizeof(bufs)); //--Filter filename here?  escapeStr("file",bufs) !?! 
				strncpy(profile, chkProfile('p',bufs), sizeof(profile)); //-first parse-----------:Read Profile 
			} 
			
			//--Profile Header:
			if (!profile[0]){ fprintf(stderr,"Error opening profile: [  ]\n"); return -1; }   
			  printf("\n%d.__________Using Profile: %s ____________\n",debug, profile); 
			memset(&bufs[0], '\0', sizeof(bufs));	 
			skip_RAM=1; display=1; break; //display=0;
		}
		//--polling:
		else if (strcmp(argv[i], "poll")==0) { polling = 1;  }	//polling options: ... i++; break; 
		//--dry run, no write.   
		else if (strcmp(argv[i], "debugn")==0) { debug = 3; } //--Output: 0,1,2==save files, 3=>stdout 
		//--debug 2: mostly extra debug info for logs.. , 'update' cmd: use builtin defaults!
		//--display/show raw values. w/:debug: <..2==write files , 3==stdout , 
		else if (strcmp(argv[i], "raw")==0) { raw = 1; } 
		else if (strcmp(argv[i], "eeprom")==0) { just_EEPROM = 1; display=0; } 
		//--numeric exit errors:
		else if (strcmp(argv[i], "-s")==0) {  sil = 1;  }
		//--toggle display: 			 Header vs. info display
		else if (strcmp(argv[i], "-d")==0) { display=display?0:1; } //--needs to be unique char!!!
		//--Logs:
		else if (strcmp(argv[i], "logs")==0) {  action = "logs"; i++; if (!argv[i]) return -1;
			//if (strcmp(argv[i],"cache")==0) { action = "debugc"; i++; mrk=5; break; } //mrk debug!!!!
			//--Number of logs to get: 			//short tnum, long lint
			else if (strcmp(argv[i],"all")==0) { i++; logOpt=4; break; } //--:read All logs
			else if (sscanf(argv[i], "%hd",&tnum)==1) { if (tnum<1 || tnum >256) return -1; i++; }
		 	while (i < argc) { //printf("\t\t%d>arg[ %s ]\n", i, argv[i]); //--loop remaining log cmds--
				if (strcmp(argv[i], "--debug")==0) { i++; debug=debug>2?debug:2; } 
				//--:turn on/toggle date buffering:
				else if (strcmp(argv[i], "--buffer")==0 || strcmp(argv[i], "-b")==0) { 
					i++; lbuf_input=1;  //--custom buffer hrs:           --upto 6?? months--  ???  
					if (sscanf(argv[i], "%ld",&lbuf)==1) { if (lbuf<1 || lbuf >94320) return -1; } 
				} //--Start from log register:
				else if (argv[i] && strcmp(argv[i], "start")==0) { i++;  if (!argv[i]) return -1;
					lint = strtol(argv[i],NULL,16); logOpt=1;
					if (lint<0x8000 || lint>0x8FFF) { printf(">unsupprted log start: %ld\n\n",lint); return -1; }
				} //-- -- -- **Search Options** -- -- -- --:
				//--days to go backwards from now:
				else if (argv[i] && strcmp(argv[i], "days")==0) { i++;  if (!argv[i]) return -1;
					lint = strtol(argv[i],NULL,10); logOpt=2;  
					//--search can be bigger if long gaps of unuse..
					if (lint<1 || lint>700) { printf(">unsupprted log time.\n\n"); return -1; } 
					else { logn = (short) lint; } lint=0;
				//--hrmeter search:
				} else if (argv[i] && strcmp(argv[i], "hrs")==0) { i++;  if (!argv[i]) return -1;
					strncpy(bufs,argv[i],19); lint = strtol(bufs,NULL,10);  logOpt=3; //max hr len.
					memset(&bufs[0], '\0', sizeof(bufs));	 
					if (lint<1) { printf(">unsupprted log hrs: %ld\n",lint); return -1; }
				} //printf("Logs: %s [%d] %d, %ld\n",action,logOpt,tnum,lint);return 1; 
				else if (argv[i]) { printf(">Error with options input!\n\n"); return 1; }
				i++;
		 	}
			break; //--
		}
		//--utilities:
		else if (strcmp(argv[i], "convert")==0) { i++;  float vf=0.0; uint16_t v16=0;
			if (!argv[i]) { printf("\t convert [float value]   (...to F16)\n"
				"\t convert f16 [F16 value] (...to float)\n\n"); }
			//--F16 to Floats:
			if (argv[i] && strcmp(argv[i], "f16")==0) { i++; if (!argv[i]) return -1; strncpy(bufs,argv[i],10); 
				vf=atof(bufs); if (!vf || vf>65535 || vf < 0) { printf("null\n"); return -1; } //< -65535
				v16 = (uint16_t) vf; vf = F16ConvertToF32(v16); 	printf(">Float of [%hu]: %f\n\n",v16,vf); } //
			//--Float to F16: 
			else { if (!argv[i]) return -1; strncpy(bufs,argv[i],10); 
				vf=atof(bufs); if (!vf || vf>65535 || vf < -65535) { printf("null\n"); return -1;} //< 
				v16=fl_to_half(vf); 	//--"... fl_to_half[ ],  F32ConvertToF16[ %hu ]", F32ConvertToF16(vf)
				printf(">F16 of [%f]: %hu \n\n",vf,v16); }
			return 0; } 
		//
		else if (strcmp(argv[i], "-v")==0) { printf("%s \n\n--- ProStarMPPT* Opensource Charge Control Software ---\n"
			"-----------------------------------------------------------\n"
			" release version %.2f\n-----------------------------------------------------------\n"
			"*ProStarMPPT® is registered trademark to Morningstar Corp.\n"
			"*( www.morningstarcorp.com )\n\n"
			"____*_*_* OpenSource Artistic-2.0 license © 2023 by sunja - centerflowing.com *_*_*____\n\n",
			//"(may also be copied under terms of GNU General Public License GNUv.2+ or later)\n\n"
			argv[0], VERSION); return 0; }
		//--Error:
		else { printf(">Error with args\n\n"); exit(1); }
		//--next arg:
		i++; if (i > 4) break;
	 }
	 //--Turn OFF data exports & Updates if toggling coils: Only Cli output!
	 if (trigger_coils) { json=0; cvs=0; update=0; polling=0;  }  
	 if (sil) { display=0; }
	 	//---maybe in future create special data export just for coils..
	}  
	
	/* ----- Parse External Settings files ----- */
	//--open, validate, configure global settings:
	//if (externalize_config && chkProfile('s',config_file)) { configureG(); } //:update globals...
	///////////////////////////////////////////////
	//--Default Max/Min: (...divide by vmultiplier if accepting>12v inputs.)
	maxV=maxV>18?18.0:(maxV<12?18:maxV); 		//--max acceptable v. safety check
	minV=minV>13?11.5:(minV<10.3?11.5:minV); 		//--min acceptable v. safety check (lithium can be low?)
	
	/*/--Date / Time:---------------------------------------------------------------------------------:*/
	//----Add time to date:
	snprintf(sbuf,(size_t)80,"%s%s",date_format, " %H:%M:00"); 
	//if (debug) strcat(date_format, ", %H:%M:00");  //log_debug
	bufs[0]='\1'; 	//printf(">>Debug %d Format: %s\n",debug,date_format); //	
	now = strftime(bufs,(size_t) 128, sbuf, localtime(&c_time)); 	//:time into string
	if (now == 0 && bufs[0] != '\0') { strncpy(ctime_s,"-error with date-",127); } 	//:error check -  
	else { strncpy(ctime_s,bufs,127); }  				 	//ctime_s[ strlen(ctime_s) -1 ] = '\0';
	now=0; memset(&bufs[0], '\0', sizeof(bufs)); memset(&sbuf[0], '\0', sizeof(sbuf));  //:cleanup... */
	
	/*/--File Names:----------------------------------------------------------------------------------:*/
	strncpy(file_out,appendStr(2, outDir, fileOut),(size_t)128); 
	strncpy(file_poll,appendStr(3, outDir, "polling_", fileOut),(size_t)128);
	strncpy(file_logs,appendStr(3, outDir, "logs_", fileOut),(size_t)128);
	if (!log_cache[0]) {strncpy(log_cache,"logCache",(size_t)32);}
	strncpy(file_logCache,appendStr(5, profileDir, log_cache, "_", fileOut, ".txt"),(size_t)128);
	//profiles...in createProfile(), chkProfile()

	
	/* --------------------------------- DEBUG Structs ----------------------------------- */
		//--COMBINES--------------------------------------------------------:
		 // 0x0027=Ah Re		// 0x0029=Ah total		
		 //--Loads:  //0x0033=Load Ah	//0x0035=Load Ah total 
		 //--Hours, Alarms: //0x0037=hourmeter	//0x0039=alarm	//0x0048=Alarm daily
		//EEPROM:  0xE041, 0xE043, 0xE045, 0xE047, 0xE049//-------------------------:
	
	/* \\-----------------------------------------------------------------------------// * /
	printf("\n\n#----MATH-----#\n");
	printf(">12600 secs == %.2f\n", secsToHours(12600));
	
	//return 1; // * /
	//--------------------------Debug Registers to file & stdout--------------------: //
	//--stdout loop marker label:
	loop_one:
	if (debug == 2) { //--:create debug file
		//snprintf(bufs, sizeof(bufs), "%ld", c_time); 
		snprintf(bufs, sizeof(bufs), "%s", appendStr(2, fileOut, "_debug.txt")); //, bufs
		  //printf(">[ %s ]\n", bufs); return 1;
		fp = fopen (bufs, "wb");   bufs[0] = '\0';   //ab
	}	else { fp = stdout; } //--:stdout  
	setvbuf(fp, bufs, _IOFBF, sizeof bufs); // BUFSIZ
	
	fprintf(fp,"\n\n#--------------------RAM------------------------------#\n");
	num = sizeof(ram) / sizeof(ram[0]); //use %lu to printf
	for (int ii=0; ii < num; ii++) { //------------------------------
		fprintf(fp,"%d:%d \t[ 0x%04X ] - (%s) %s\n", ii,ram[ii].logical,(int) ram[ii].hexa,ram[ii].unit,ram[ii].string);	
	}
	fflush(fp);
	if (debug != 2) { fprintf(stderr, "\nThis is after RAM!!!\n"); fflush(stderr); }
		
	fprintf(fp,"\n\n#------------------EEPROM-----------------------------#\n");
	num = sizeof(eprom) / sizeof(eprom[0]); //use %lu to printf
	for (int ii=0; ii < num; ii++) { //-------------------------------
		fprintf(fp,"%d: \t[ 0x%04X ] - (%s) %s\n", ii, (int) eprom[ii].hexa, eprom[ii].unit, eprom[ii].string ); 
	}
	fflush(fp); 
	if (debug != 2 ) {  fprintf(stderr, "\nThis is after EEPROM!!!\n"); fflush(stderr); }
	
	fprintf(fp,"\n\n#------------------COILS-----------------------------#\n");
	num = sizeof(coils) / sizeof(coils[0]); //use %lu to printf
	for (int ii=0; ii < num; ii++) { //-------------------------------
		fprintf(fp,"%d:%d \t[ 0x%04X ] - %s\n", ii, coils[ii].logical, (int) coils[ii].hexa, coils[ii].string ); }
	fflush(fp); 
	if (debug != 2 ) { fprintf(stderr, "\nThis is after COILS!!!\n"); fflush(stderr); }
	
		//return 1;
	if (debug == 2) { fclose (fp); debug = 7;  //--:debug==2
		 printf("------------File created Looping Again---------------\n"); goto loop_one; }
		 
	fprintf(fp,"\n>final flush!\n");  fflush(fp); 	  //fclose (fp); //--fclose stdout ENDS output!!!!
	setvbuf(fp, NULL, _IOFBF, 0);  memset(&bufs[0], '\0', sizeof(bufs));	 
	//--output loops end. back to program
	debug = 3; 	goto program_end;
	//Debug-------------------------------------------------------------------! */
	
	/* ------------------------------ MODBUS CONNECTION  ---------------------------------- */
	if (modbusId <= 0 || modbusId > 255) modbusId = MSMPPT;
	//--#setup link----:#
	modbus_t *ctx;	int rc;
	 
	/* Set up a new MODBUS context */
	ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 2);
	if (ctx == NULL) {
		if (!sil) { fprintf(stderr, "Unable to create the libmodbus context\n"); }
		else { fprintf(stdout, "-1\n");  }
		if (mrk != 5) return -1;
	}
	/* Set the slave id to the ProStar MPPT MODBUS id */
	modbus_set_slave(ctx, modbusId);
	
	/* Open the MODBUS connection to the MPPT */
    if (modbus_connect(ctx) == -1) {
        if (!sil) { if (mrk==5){fprintf(stderr,"------------[....Modbus OFFLINE....]\n");} // && debug
			else fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno)); }
		else { fprintf(stdout, "-1\n");  }
        modbus_free(ctx);
        if (mrk!=5) return -1;  //-:no connection.
		//--offline actions: mrk=5==offline profile creation.     else 
		else if (strcmp(action,"validate")!=0) action="backupprofile"; //-:new profile using builtin defs. 
		//-:[offline dev debug path!]: 
		//else if (strcmp(action,"debugc")!=0) action="backupprofile";
		else mrk=0; //-:validate profile
    }  
	//-:connection is live so skip profileIn & use live values for new profile! 
	else if (mrk==5) { mrk=0; }
	//-----------------------------------:

//---Logs:           //, &&cacheDParseDebug
static void *cdLabels[] = { &&cacheDParse0, &&cacheDParse1, &&cacheDParse2, &&cacheDParse3, &&cacheDParse4 }; 
//--debug logs block:
if (strcmp(action,"debugc")==0) { // && debug > 2
	/* --- Log Cache Debug ---------------------------------------- */
	printf(">>OFFLINE DEBUGGING--------------------------\n");
	return 1;
}	

	num = sizeof(ram) / sizeof(ram[0]);
	nume = sizeof(eprom) / sizeof(eprom[0]);


	/*/###############################----PROFILES----##########################################-:   */
	//--Updating Using builtin Defaults:--//--Build before backup Loop.. parse .f_defs into profileIn struct:  
	//------------mrk == 1 (updates[]), == 8 update_all_defaults, == 5 (create profile)
	if (mrk) {   int y=0; // --offline validate--
		for (int ii=0; ii < (int)nume; ii++) { if (eprom[ii].hexa > 0xE038) break;
			//--:Create new profile: using builtin defaults... 
			if (strcmp(profile,"new")==0) {  profileIn[y] = eprom[ii]; 
				profileIn[y].value.fv = eprom[ii].f_def; 
				if (parseProfileValue("defaults", y, ii, eprom) < 0) continue; //just send eprom[ii]
				eprom[ii].basev = eprom[ii].f_new; //--:copy new update value for createProfile() 
				y++; continue; 
			}
			//--:Only if in updates[] otherwise skip:--- (arg: 'update' w/o default profile mrk=1)
			if (mrk==1) { int di=0; 
				for (int *d = &updates[0]; d < &updates[UPDATEABLE]; d++){ if (*d==eprom[ii].hexa){di=1; break;} } 
				if (!di) { continue; } 
			} 
			//--:'update all' arg? (mrk=8,profile=='-'){...all eprom values used...}
			//-//--Build Updates:------------------------------------------------:
			profileIn[y] = eprom[ii]; 	
			//--assign builtin defaults: f_def 		
			profileIn[y].value.fv = profileIn[y].f_def; 
			y++; 
	}	} 
	
	print_out_profile: ;	//--label for creating profile with current Charger data: 
	
	/*/--Profile template:--------------------------------------------------------:/*/
	//---Create new profile, backup, or backup before updating w/profile:
	if (strcmp(action,"backupprofile")==0) { //|| mrk==5  //mrk: 8==revertAll, 1==revert to updates[], 5==new prof, 
		//-:mrk==5? profile="new", offline(action="bkupp") OR online(action="current_settings"=>"bkupp")  
		//-:backup cmd: (profile="print",action="current_settings"=>"bkupp")
		//--Defaults or Current EEPROM:---------------------------: 
		if (strcmp(profile,"new")==0) { display=1; createProfile("new", nume,eprom,ctime_s); return 0; } 
		//--:Backup Settings:-------------------------------------://could be done in createProfile()..
		else {  char raw2=raw;
			//---'backup' or updating: if (strcmp(profile,"print")!=0){ update }
			if (debug<3) { raw=1; snprintf(bufs,sizeof(bufs),"%s",profileBackups); //--:create filename 
			} else {  if (bufs[0]) memset(&bufs[0],'\0',sizeof(bufs)); //--:stdout
				printf(">Backing up EEPROM settings to stdout:\n\n"); } 
			//--print out profile: 
			createProfile(bufs,nume,eprom,ctime_s); if (bufs[0]) memset(&bufs[0],'\0',sizeof(bufs));
			if (raw2!=raw) { raw=raw2; }
		}
		/*/--Continue to update EEPROM: ------------------------------------*/
		if (strcmp(profile,"print")==0) {  return 0; }
		//--continue on to Profile:
		else { action="profile_update"; if (debug) printf("\n"); }  //debug=1;  
		//if (debug) printf("\n>Backup created...Going on to validate profile\n");//return 0; //debug!!
	} 
	/* ################################--PROFILES Parse Update--######################################### */
//if (action[0] && strcmp(action,"current_settings")==0){action="backupprofile"; goto print_out_profile;}//offline debug!!
	if (strcmp(action,"profile_update")==0 || strcmp(action,"validate")==0) { //backup FIRST!!
		//------check dipsChargeMode() first:
		if (strcmp(action,"validate")!=0) {
			uint16_t mode = 0; readModbus(ctx, 0x003A, 1, &mode);
			mode = dipsChargeMode(mode); //printf(">dips mode: %hu\n",mode); return 0;
			//--if not set to 'custom' print overwrite & backup warnings: (do in createProfile() too)
			if (mode!=8) { warn=1; 
				printf("!WARNING! Charge Mode %hu will be overwritten! Charger not in custom settings mode.\n\n",mode); }
		}
		//--Build Update using profile:		
		//----chkProfile() opens and preparses data into profileIn[]. 
		if (display) printf(">Profile [ %s ] updates found for EEPROM:\n\n",profile); //--redundant if?
		memset(&updates[0], '\0', sizeof(updates)); short upi=0; short upt=0; setbuf(stdout, NULL);
		for (int i=0;i < UPDATEABLE;i++) { signed char is=0; 
			//----Take in profileIn[i] , Output updates[] & eprom[e].f_new ! --------------: 
			if (!profileIn[i].hexa) { if (!sil){ printf("%s\n",(debug?">End of inputs..":"")); } break; } 
			upt++; //:total updates marker 
			//--Skip//Check for security locked registers:--//
			if (locked_registers[0]) {  
				for (int *d = &locked_registers[0]; d < &locked_registers[UPDATEABLE]; d++) { 
					if (*d==profileIn[i].hexa) { is=1; break; } } 
				if (is) { fprintf(stderr," 0x%04X > !WARNING! This register is locked.\n", profileIn[i].hexa);
					continue; }  
			} //end locked
			
			//--Match profile update to supported eprom[] registers:  
			signed short ey = getIndex(profileIn[i].hexa,nume,eprom); 
			if (ey == -1){ fprintf(stderr,"\n***Unsupported: 0x%04X [%d] *skipping!*\n",profileIn[i].hexa,profileIn[i].hexa+1);
				continue; }
			//--into Float value:--------------: (builtin==mrk)
			if (!mrk) profileIn[i].value.fv = atof(profileIn[i].sv); //--save float!  strtol(token,NULL,10/16)
			//else .fv == eprom[].f_def (update) //--using builtin defaults
			
			/*/--Parse Profile FUNCTION:------------------------------------------------------ */
			is = parseProfileValue("up", i,ey,eprom); 
			if (is < 0) continue;  //-:skip if bad value
			//--Add to global updates[]:--------------------------------------------------------: 
			else { updates[upi] = profileIn[i].hexa;   upi++; 
				/*/--Custom user description? (do separately from update?):-----------------: */
				if (debug && profileIn[i].string[0]) {  //replace?appendStr(3,eprom[e].string," ",profileIn[i].string)
					strncpy(eprom[ey].string, profileIn[i].string, sizeof(eprom[ey].string) );
				} //debug only for now... * /
				if (display) { //
					snprintf(bufs,sizeof(bufs),"%f",profileIn[i].value.fv);  	//
					printf(" 0x%04X =  %s %s [%d],\t\"%s\"\n", profileIn[i].hexa, strReplace(".000000",".00",bufs), 
						eprom[ey].unit, eprom[ey].f_new, (eprom[ey].string[0]?eprom[ey].string:"")); 
					fflush(stdout); memset(&bufs[0], '\0', sizeof(bufs));  }//:description 
			}
			fflush(stderr);  //--goto next register...
		} //--end loop
		//--Cleanup: [profileIn[] not used after this...]:------------------:
			//memset(&profileIn[0],'\0',sizeof(profileIn));
		/*/--Update Notifications: --------------------------------------- */
		if (! updates[0]) { fprintf(stderr,"\nProfile had problems, or no updates were found. Exiting...\n"); return 1; }
		else if (strcmp(action,"validate")==0) { 
				printf("\n>Validated Profile [ %s ]: found %d valid updates out of %d total inputs.\n\n", profile,upi,upt); 
				return 0; }
		else if (warn) { printf(">Valid Profile updates: [ %d ] out of %d\n\n"
				"!WARNING incorrect input values are at your own risk!\n"
				"\t!Validation does not mean correct!\n"
				"!Check the documentation of the batteries for correct values!\n", upi,upt);  //Out of %d total\n!
			//--interactive update question..-----------------------:
			printf("\n>Update?...[yY/nN]$> ");
			char input[3], ok;
			fgets(input, 3, stdin); sscanf(input, "%1s", &ok);
			if (ok=='y'||ok=='Y') { printf(">Updating...\n"); } else return 1;
		} else { printf(">Valid Profile updates: [ %d ] out of %d\n\n>Updating...\n", upi,upt); }
		
		/*/--Update EEPROM with profile: --------------------------------------- */
		//printf("\n---Profile debug: debug=%d, update=%d, action=%s, profile=%s\n\n", debug,update,action,profile);
		update=debug<3?1:0;  //sil=0; action = "updating"; 
		skip_RAM=0; just_EEPROM=1;  display=1; mrk=0;//:(builtin==8)
		if (debug>1) printf("---Updates debug: debug=%d, update=%d, action=%s, profile=%s\n\n", debug,update,action,profile);
		//if (debug>1) return 0;//debug!!!!!!!!!!!!!!!!!!!
	} //end profile update parse.
	
	
	
	
	//################################################################################################
	//--Get Current Hourmeter:-----------------------------------------: 
	int tmp[2] = {0x0036,0x0037}; upRegisters(ctx, ram, num, tmp, 2); 
	 short xx=0; xx = getIndex(0x0037,num,ram); //--:Global! int?
	now = ram[xx].value.lv; xx=0; //:now !!!
	/* ####################################--Header--#########################################-: */
	uint16_t data[1]; //--:Global for individual reads. 'unsigned short' %hu  --deprecated except for here!--
	//--:Date / time variables set:---------------------//
	char* setting_s; 
	setting_s = (! update)?(strcmp(action,"logs")==0?"Logs":"Settings"):(update==2?"*Reverting Defaults*":"*Updating*");
	if (display && strcmp(action,"current_settings")!=0) { //--(skip for backups)
		printf("################################################################\n"
			"## Modbus MCProStarMPPT - %s - [%s] \n"
			"################################################################\n", setting_s, ctime_s);
		/* #-------------------------CHARGER STATUS ID DATA----------------------------# */
		//--read ------------------CHARGER STATUS:----------------:
		rc = modbus_read_registers(ctx, 0x0021, 1, data);  
		  if (rc == -1) {	fprintf(stderr, "!%s\n", modbus_strerror(errno));  return -1; }
		//--print charger state & debug:   //char CState[32] = ""; strcat(CState, );
		printf("Charger State: %s\t\t\tHrmeter:%ld, Debug: %d\n",getStateString(0x0021, (uint16_t) data[0]),now,debug);
		if (profile[0]) { printf("Profile: %s \n", profile); }//else if (polling){ " [%ld] ", now; }
		data[0]=0; 
		/*/--read ------------------CHARGER  INFO:----------------: not Working!
		uint16_t sdata[3]; //Device Identification addr: (0x2B, subcode 0x0E)
		rc = modbus_read_registers(ctx, 0x00, 3, sdata);  
		 if (rc == -1) {	fprintf(stderr, ">%s\n", modbus_strerror(errno));  return -1; }  // */
		//--print -----------------Charger Info (below HEADER):-------:
		//printf("Charger ID: %hu [ %hu ] %hu\t|\tDebug: %d\n%s\n\n", sdata[0],sdata[1],sdata[2], debug, CState);
		//if using data[]: memset(&data[0], '\0', sizeof(data)); 
	}
	
		
	/* _____________________________ EARLY OPTIONS _______________________________________ */
	//--POLLING:-------------------------------------------------------------------:
	if (polling) {	pollingOut(ctx, num, ram, ctime_s); 
		goto program_end;
	} 
	
	
	/* ############################--Update Log Cache--###############################-: */
	//--Update/Write log cache:--------------------------------:
	long latest=0;	struct tm *tmStruct;  tmStruct = localtime (&c_time);   
	if (real_date_logs) {  //...avoid late night? prefer afternoon?
		if (tmStruct->tm_hour > 4 && tmStruct->tm_hour < 24){ 
				latest = writeLogCache(1); }
		else {  latest = writeLogCache(0); } //create if (!logcachefile)
		if (latest && latest <= (now-48)) { latest = writeLogCache(1); } //--:update anyway
	} //* */
	
		
	/* #################################--LOGS--######################################-: */
	//--LOGS:------------------------------------------------------------------------//---:
	if (strcmp(action,"logs")==0) { goto logBlock; } 
	
	
	/* ############################ TRIGGER COILS ###################################### */
	//-----add message (json) export opt, instead of stdout...
	if (trigger_coils) { xx=0;  printf("\n\nTriggering Coil Resets: \n");
		size_t numc = sizeof(coils) / sizeof(coils[0]); 
		size_t upC = CUPDATEABLE; //sizeof(coil_updates) / sizeof(coil_updates[0]);
		for (short ii=0; ii< (int)upC; ii++) { short ci;
			if (!coil_updates[ii]) break; //--done. end of coil_updates.  	
			//--Link coil_updates[] to coils struct...:-----------------------:
			//--EQ coil:  --EQ address !hack!--
			if (coil_updates[ii]==0x0013) { xx=1;  coil_updates[ii] = 0x0000; ci = 0; } 
			else ci = getCoil(coil_updates[ii], numc, coils);
			//--error exit:
			if (!coils[ci].logical || ci == -1) { //--
				printf(">Error Triggering Coil! [ 0x%04X ] \n", coil_updates[ii]); return -1; } 
			else printf(">Triggering Coil: [ 0x%04X ] : %s\n", coil_updates[ii], coils[ci].string);
			fflush(stdout); 
			//-------------------------------:
			int coilup = trigger_coils==2?0:1;
			//--First check if an ON/OFF coil:------------------: (Load & Charge disconnects)
			if (coil_updates[ii] == 0x0001 || coil_updates[ii] == 0x0002) {  
				uint8_t coilData [1]; 
				//--assign correct ON value:--------:
				coilup = coils[ci].trigger;
				//--Read if ON (disconnected) or OFF (normal mode):-------------:
				rc = modbus_read_bits(ctx, (int) coil_updates[ii], 1, coilData); 
			 	if (rc == -1) { fprintf(stderr, "%s\n", modbus_strerror(errno)); return -1; }
				if (coilData[0] && debug < 3) { //--DISCONNECTED ...Clear and goto next one
					rc = modbus_write_bit(ctx, (int) coil_updates[ii], 0); 
			 		if (rc == -1) { fprintf(stderr, "%s\n", modbus_strerror(errno)); return -1; }
					printf(">...Coil toggled CLEAR\n"); continue; 
			}	} //else turn on coil.. 
			//--Trigger Coil:-----------------------------------:
			if (debug < 3) { rc = modbus_write_bit(ctx, (int) coil_updates[ii], coilup); 
				 if (rc == -1) { fprintf(stderr, "%s\n", modbus_strerror(errno)); return -1; }
		}	}
		//--finish: if EQ toggled, display current target voltage. 
		if (xx) {	memset(&coil_updates[0], '\0', sizeof(coil_updates));  coil_updates[0] = 0x0024; 
			upRegisters(ctx, ram, num, coil_updates, 1); tnum = getIndex(0x0024,num,ram);  //ram[tnum].value.fv;
			printf("\nBattery Target V:	%.2f v\n",ram[tnum].value.fv);
		} 
		printf("\nfinishing.... done.\n\n");   return 0;  //:exit
		/*  if (debug) { uint8_t coilData [1];
				rc = modbus_read_bits(ctx, (int) coil_updates[ii], 1, coilData); 	//---read first:
				if (rc == -1) { fprintf(stderr, "%s\n", modbus_strerror(errno)); return -1; }
				printf(">EQ reads.. [%u]\n", coilData[0]); } // */
	} 
	
	/* #------------------------ DEBUG -------------------------------------------------# */
	


	/* ###########################################################################################
	############################## BULK READ RAM Registers ######################################### 
	________________________________________________________________________________________________ */
	//-Build bulk data:
	struct bulkData bulk [16]; //, *bulkp; 
	short y=0; xx = 1; //:xx declared under Header
  if (! skip_RAM) {  	//-RAM---
  	//--Build Bulk Block:
	bulk[0].start = ram[0].hexa; bulk[0].i = 1;
	for (int ii=1;ii<num;ii++) { 
		if (ram[ii].hexa == (bulk[y].start+xx)){ bulk[y].i++; xx++; } 
		else { y++; bulk[y].start = ram[ii].hexa; bulk[y].i = 1; xx=1; }  
	} // * /
	
	//--Loop bulk registers array:   (sizeof(bulk) / sizeof(bulk[0]));
	for(short i=0;i<16;i++) { if (!bulk[i].i) break; //--endof (bulk[].start will break on 0x0000 !!)
		const int dsize = bulk[i].i;	uint16_t datas [dsize];   
		//--bulk read:
		readModbus(ctx, bulk[i].start, bulk[i].i, datas);
		//printf("\nSTARTING RAM: 0x%04X: \t[ %d ]\n", bulk[i].start, (int) datas[0]);
	
		int e = getIndex(bulk[i].start,num,ram);  short z=0;
		//--Assign data to ram struct in order... (replace datas[z] with *dp !)
		for (uint16_t *dp = &datas[0]; dp < &datas[dsize]; dp++) { 
			// * /--Combines Hi-LO:
			if (strncmp(ram[e].calc,"comb",4)==0) { ram[e].value.lv = combine(ram[e].hexa, datas[z], num,ram); //*dp
				if (ram[e].value.lv == -1){ ram[e].value.lv = 0; }  
				}  //65506???? -*dp
			//--Parse data values into register struct:
			ram[e] = parseValue(&ram[e], datas[z]); 
			//--Print out:
			if (!just_EEPROM && debug) { printOUT(z, ram[e].typ, &ram[e]); } //
			e++; z++;
		}  
	} //
  	//--Reset data:----------------:
	memset(&bulk[0],'\0',sizeof(bulk)); 
  }  //---------------------------------- done RAM ---------------------------------------------  */
	
	/* ################################## Read the EEPROM ####################################  */
	//----Build Bulk Read:
	bulk[0].start = eprom[0].hexa; bulk[0].i = 1; y=0; xx = 1;
	for (int ii=1;ii<nume;ii++) { 
		if (eprom[ii].hexa == (bulk[y].start+xx)){ bulk[y].i++; xx++; } 
		else { y++; bulk[y].start = eprom[ii].hexa; bulk[y].i = 1; xx=1; }  
	} 
	
	if (!sil && debug && strcmp(action,"current_settings")!=0) { 
		printf("\n--------------------------------------------------------------------" 
			"\nBulk EEPROM Read: \t[ %zu ]\n", nume); }
	//--Loop bulk registers array:
	for(short i=0;i<16;i++) { if (!bulk[i].i) break; //--endof (bulk[].start will break on 0x0000 !!)
		const int dsize = bulk[i].i;		uint16_t datas [dsize];   
		//--bulk read:
		readModbus(ctx, bulk[i].start, bulk[i].i, datas);
		if (debug>1 && strcmp(action,"current_settings")!=0) { 
			printf("\n%d: STARTING bulk eprom: 0x%04X: \t[ %d ]\n", i, bulk[i].start, (int) datas[0]); }
	
		int e = getIndex(bulk[i].start,nume,eprom);  short z=0;
		//--Assign data to eprom struct in order... (replace datas[z] with *dp !)
		for (uint16_t *dp = &datas[0]; dp < &datas[dsize]; dp++) { 
			char this_one = 0;
			// *------ Write New EEPROM Value ---------------------------------- * /
			if (update && strcmp(action,"current_settings")!=0) { 	
				uint16_t new_value[1];  int upN = UPDATEABLE; //-redundant
				//--check if data is set for updating: --------
				for (int y=0; y < upN; y++) { if (!updates[y]) break;	//no more.
					if (eprom[e].hexa==updates[y]) { this_one = 1; } }
				if (this_one) {	//--Update!----------------- && *dp!=eprom[e].f_new)
					printf(">Updating...");
					//--Pre Update value:-------: 
					eprom[e] = parseValue(&eprom[e], datas[z]);	 
					printOUT (0, eprom[e].typ, &eprom[e]); //
					//--write new value -----------------------:
					new_value[0] = writeUpdate (ctx, &eprom[e]); //--:write! 
					//--Test if update success ----------------:
					if ((new_value[0] == datas[z]) & display) {    //----updated to same value
						printf(">0x%04X ...(rewrote same value.)\n",eprom[e].hexa); //eprom[ii].updated = 2;
					} else if (new_value[0] == eprom[e].f_new) { 
						eprom[e].updated = 1;   if (debug) printf("\n"); //hack fix?
					//--Error writing:-------------------------:
					} else { printf(">Error writing [ 0x%04X ] new value [%hu]!"
									" current value: %hu \n", eprom[e].hexa, eprom[e].f_new, new_value[0]); 
						if (debug) { printf(">Problem parsing update!\n"); return -1; } //if not: ...
					} 
					//--Pass new value: instead of previous---:
					datas[z] = new_value[0]; 
				} //else { printf(">Skipping writing same value...\n"); }	
			}
			// * /--Combines LO-Hi:------------: EEPROM order is LO HI !!! RAM is opposite!
			if (strncmp(eprom[e].calc,"comb",4)==0) { eprom[e].value.lv = combine(eprom[e].hexa, datas[z], nume,eprom); 
				if (eprom[e].value.lv == -1){ eprom[e].value.lv = 0; }  
				} //-*dp  !save in eprom[e] 
			//--Parse data values into register struct:
			eprom[e] = parseValue(&eprom[e], datas[z]); 
			//--Print out:
			if ((!update && (just_EEPROM || (debug && strcmp(action,"current_settings")!=0 && !sil))) || 
						this_one || (update && debug)) { printOUT (z, eprom[e].typ, &eprom[e]); }
			e++; z++;
		}  
	} //return 1; // */
	
	/* ############################## LOOP Options OR Output Data ########################### */
	//--Create Profile of current settings: -----------------------------------------:
	if (action[0] && (strcmp(action,"current_settings")==0)) { action="backupprofile"; goto print_out_profile; }
	
	//--close modbus:
	modbus_free(ctx);

	//--json:-------------------------------: 
	if (json) { jsonOut(setting_s, (int)num,ram, (int)nume,eprom); }  
	
	
	//--Display----------------------------------------------------------------:
	display_loop: ;
	//--skip display if debug: 
	if (debug > 2 || ! display) { goto program_end; }
	/* /---open output buffer to file/stdout:-----------------------------:
	else if (debug < 2) { //--deprecated
		snprintf(bufs, sizeof(bufs), "%ld", c_time); 
		snprintf(bufs, sizeof(bufs), "%s", appendStr(3, fileOut, bufs, "_debug.txt")); //
		  //printf(">[ %s ]\n", bufs); return 1;
		fp = fopen (bufs, "ab");   bufs[0] = '\0';   //wb
	}	else { fp = stdout;  }  //debug==2
	setvbuf(fp, bufs, _IOLBF, sizeof bufs); //_IOFBF    ,,BUFSIZ
	// */fp = stdout; //-!!!
	//--FULL DISPLAY---------------------------------------:    //create int buffer for multi-getIndex() calls!!!!!!
	fprintf(fp,"\n						Charger State:	%s \n", ram[getIndex(0x0021,num,ram)].value.sv);
	fprintf(fp,"__Current Volts & Amps__________________________________________________________\n\n"); 
	fprintf(fp,"Battery:		%f v		Solar Array: 		%f v\n", (ram[getIndex(0x0017,num,ram)].value.fv>2 ? ram[getIndex(0x0017,num,ram)].value.fv:ram[getIndex(0x0012,num,ram)].value.fv), ram[getIndex(0x0013,num,ram)].value.fv);
	fprintf(fp,"Charging Amps:		%f amps		Solar amps:		%f amps\n\n", ram[getIndex(0x0010,num,ram)].value.fv, ram[getIndex(0x0011,num,ram)].value.fv);
	
	//Solar Max power (sweep):	%f wamps, ram[getIndex(0x003E,num,ram)].fv !!!!!
	fprintf(fp,"Charger output:		%f W		Array target:		%f v\n", 
				ram[getIndex(0x003C,num,ram)].value.fv, ram[getIndex(0x0040,num,ram)].value.fv); //!0x003E
	fprintf(fp,"Target V for Batteries:	%f v		Array Vmp (sweep):	%f\n", ram[getIndex(0x0024,num,ram)].value.fv, ram[getIndex(0x003D,num,ram)].value.fv); 
	fprintf(fp,"Temperature (battery): 	%f C		Ambient temperature:	%f C\n\n", 
				ram[getIndex(0x001D,num,ram)].value.fv, ram[getIndex(0x001C,num,ram)].value.fv);

	fprintf(fp,"Charge total: 		%.4f KWh		Ah total:		%.2f Ah\n", 
				(ram[getIndex(0x002B,num,ram)].value.fv), ram[getIndex(0x0029,num,ram)].value.fv); //-
	fprintf(fp,"Charge (resetable):	%.4f Kwh		Ah (resetable):		%.2f Ah\n", 
				(ram[getIndex(0x002A,num,ram)].value.fv), ram[getIndex(0x0027,num,ram)].value.fv);
	fprintf(fp,"Median Charge Volts:	%.3f v  		SOC led:		%s\n",  
			(ram[getIndex(0x0027,num,ram)].value.fv ? (ram[getIndex(0x002A,num,ram)].value.fv*1000) / ram[getIndex(0x0027,num,ram)].value.fv:0.0), ram[getIndex(0x003B,num,ram)].value.sv); 
		
	fprintf(fp,"\n__Daily_________________________________________________________________________\n\n");
	fprintf(fp,"min battery volts:	%f v		Solar Power (sweep):	%.2f W\n", 
				ram[getIndex(0x0041,num,ram)].value.fv, ram[getIndex(0x003E,num,ram)].value.fv);
				
	fprintf(fp,"max battery volts:	%f v		Max Array:		%f v\n", 
				ram[getIndex(0x0042,num,ram)].value.fv, ram[getIndex(0x004C,num,ram)].value.fv);
				
	fprintf(fp,"Absorb time:		%.2f hrs		Charge:			%f Ah\n", 
				secsToHours(ram[getIndex(0x0049,num,ram)].value.dv), ram[getIndex(0x0043,num,ram)].value.fv);
				
	fprintf(fp,"Float time:		%.2f mins		EQ time:		%.2f mins\n", 
				(ram[getIndex(0x004B,num,ram)].value.dv/60.0), (ram[getIndex(0x004A,num,ram)].value.dv/60.0));
				
	fprintf(fp,"\n__Charger Targets________________________________________________________________\n\n");
	fprintf(fp,"Absorb voltage: 	%f v		Max Charging volts:	%f v\n", 
				eprom[0].value.fv, eprom[getIndex(0xE010,nume,eprom)].value.fv);
	fprintf(fp,"Absorb time: 		%.1f hrs			Float voltage:		%f v\n", 
				secsToHours(eprom[2].value.dv), eprom[1].value.fv);	//--#!!!
	fprintf(fp,"Absorb time+: 		%.2f hrs		Float under exit:	%.1f mins\n", 
				secsToHours(eprom[3].value.dv), (eprom[getIndex(0xE006,nume,eprom)].value.dv/60.0)); 	//--#!!!
	fprintf(fp,"Absorb time+ trigger: 	%.3f v		Float skip:		%f v\n", 
				eprom[4].value.fv, eprom[5].value.fv);
	
	fprintf(fp,"\n__Equalize______________________________________________________________________\n\n");
	fprintf(fp,"EQ voltage: 		%f v		days:			%d days\n", eprom[7].value.fv, eprom[8].value.dv);
	fprintf(fp,"EQ timeout: 		%.2f hrs		days since last:	%d days\n", 
				(eprom[getIndex(0xE00A,nume,eprom)].value.dv/3600.0), eprom[getIndex(0xE04F,nume,eprom)].value.dv);
	fprintf(fp,"EQ under timeout:	%.2f hrs		F:			$ hu v\n", (eprom[getIndex(0xE009,nume,eprom)].value.dv/3600.0));
	
	fprintf(fp,"\n__Settings & Info_________________________________________________________________\n[%s]\n", ctime_s);
	fprintf(fp,"%s\n\n", batteryVoltagesNote); //filtering??
	fprintf(fp,"DipS:	%s\n__Alarms / Faults:__\n(0x0022): %s\n", 
				ram[getIndex(0x003A,num,ram)].value.sv, ram[getIndex(0x0022,num,ram)].value.sv);
	fprintf(fp,"%s\n(day): %s\n", ram[getIndex(0x0039,num,ram)].value.sv, ram[getIndex(0x0048,num,ram)].value.sv);
	
	/*/------Print OUT to stdout or file or both:-----------------------------------------:
	 fflush(fp); 	//--file - append to debug file
	if (debug == 2) {  fclose (fp); debug = 1; //7
		 printf("------------File created Looping Again---------------\n"); goto display_loop; }
	//else if (debug == 7) debug = 2; //--output loops end. back to program
	//fclose (fp); //--fclose stdout ENDS output!!!!
	// */
	setvbuf(fp, NULL, _IOFBF, 0);
	
	
	/* --------------- ************ End Main Program Runtime ************ ------------------ */
	program_end: ;
	if (display) { if (!polling) printf("\n...ending.....Done.\n");
		printf("\n________________________________________________________________________________\n\n\n\n");
	}
	else if (!sil) { printf("\n----------------\n\n"); }
	return(0); //0?
	

	/* ####################################--LOGS--#########################################-: */
	//--LOGS:------------------------------------------------------------------------------//---:	
	logBlock: ;
	{ //-:logBlock: if(strcmp(action,"logs")==0) 
		struct logStruct logs [256];     //ex. logs[x].log[0].value.lv => hrs , logs[x].log[y].typ => value
		long search=0; uint start=0; //int logical; //Only start is needed!......
		char strbuf [256]=""; char sbt=0; //:sbt==is logDate inbetween..
		//--Log Cache scope variables:    [available from main:short xx,char skip_RAM,char mrk,] 
		long earLog=0; //:earliest log hrmtr 
		short nextCD=0; long timeSkipA=0; long timeSkipB=0;   
		time_t cacheDate=0; int cindex=0; short cdMarker=0;   
		time_t targetD=0; 
		//----Add debug time to date:  
		if (debug) strcat(date_format, ", %H:%M:00");  //log_debug
		
		//--LogCache:-----------------------------:
		//---logCache[0].xt == real number of cached dates! starting at 1.
		if ( !lbuf_input && real_date_logs ) { long cht = readLogCache(""); 
			earLog = logCache[logCache[0].xt].hrmtr-logCache[logCache[0].xt].nHrs-21; //--earliest log time w/buffer
			if (debug) { printf(">Log cache, latest [ %ld ] readLogCache: [ %d ] \n\n", 
										logCache[1].hrmtr-logCache[1].nHrs, (int)cht); }
		} else { logCache[0].xt=0; }
		
		//--Normalize Time to help find correct log hourmeters:----------------------:
		//---nNow used for creating search! , normT for creating targetDate! ...
		long nNow=now; long normT=0; 								//--now==hours  check nNow for type probs! 
		//--normalize search: --to 4-6PM, before logs are written in winter--  [def:nH==6/21600]
			//ex. 	now==2pm, hrofday=13, 23-13=10, nNow=(now+10)-nH
		int hrofday=0; tmStruct = localtime (&c_time); //:needed reinit. struct tm *tmStruct;   //double tm declare!!
		if (tmStruct != NULL) { hrofday=tmStruct->tm_hour+1;  c_time -= tmStruct->tm_min*60; //0-59 ???c_time???????
			//--late night? normalize search to previous day: gets confusing. (optional?)
			//if (late_night && hrofday < 3) { nNow = (now - hrofday) - nH; 
			//		normT = (c_time-(hrofday*3600))-(nH*3600); 	} //-:c_time targetD buffer
			//else{ //--increase*/decrease to normalize
			nNow = (24-hrofday); normT = (c_time+(nNow*3600))-(nH*3600);  nNow = (now + nNow) - nH;//def:6pm 
		} //} 
		if (debug>2) printf(">localtime hr: %d [%d] -> normalizing hours to %ld\n\n",tmStruct->tm_hour,hrofday,nNow);
		
		/*/--------logOpts: 1==start, 2==days (date), 3==hrs ---------------------------------------:// */
		tnum=tnum?tnum :3; //--minimum to get
		//*/--Start addr: user or default start addr:---------------------------------:
		if (logOpt==1) { start = (uint) lint; 	snprintf(strbuf, (size_t) 16, "0x%04X", start);
			strncpy(strbuf, appendStr(3,"Reading logs from ",strbuf," ..."), sizeof(strbuf)/sizeof(strbuf[0])); }
		else { start = 0x8000; } //logical = start+1; 
		//*/--Days:-------------------------------------------------------------------:	
		//-----Search options: search for hourmeter and start there...  
		if (logOpt==2){ 
			//-----logn= short;   search, now, lint = long;  nNow = (signed) long!!		
			search = (nNow - (logn*24)); //--:set search to desired/normalized day (hrs) 
			if (!search || search<0) { printf("Error with logs.\n\n"); exit(1); }
			if (debug) { printf(">Searching days ago [ %ld ]...\n",search);
				if (lbuf_input) printf(">With buffer input of %ld hours.\n",lbuf);  }
			
			/*/--First LogCache Search: match search to targetDate hourmeter:----------------: */
			// * //--Find a Date from logCache: modify search & lbuf...  
			if (logCache[0].xt && !lbuf_input) {  lbuf=0; 	
				targetD = (normT-((logn*24)*3600));   //--use normT normalized c_time...!
				cacheDate=targetD;	searchLogCache(search, &cacheDate, &cindex); //:ts	
				//-----cindex: 0=gt last, 1(+)=within 48hrs, 2(minus)=lt & gt previous, 3(total+)=lt oldest
								
				//--Parse returned cache search:-------------------------------:
				cdMarker=0; goto cacheDParse; 	//-:parse block  
				cacheDParse0: ;  				//-:return label
				//--No cache Date, but lbuf avail:-------------------------: 
				//if (cindex>logCache[0].xt && debug>2) { printf(">Target Date is before any log cache dates.\n"); } 
				//--Found Match! date cache:-------------------------------:else 
				if (cacheDate && cindex>0 && cindex<=logCache[0].xt) { cdMarker=-1; } //date set below!
				//--Search found possibly close matches:-------------------:nextCD==cindex-1;
				else if (cacheDate && cindex<0) {   cindex=cindex*-1; sbt=1; } //-:still required  
				//--Nothing found in cache:  --match is after-- (cindex==0)
				
				if (debug>1) {  //if (debug>3) 
					//--:  buffered date    :---------------------------:
					time_t bhr = c_time-(((now-search)+lbuf)*3600); 
					//time_t bhr = normT-(((nNow-search)+lbuf)*3600); //all buffers??? !!!!!!!!!!!!!!!!!!!
					printf("[%d]: %ld >> recomputed date: %sSearching [ %ld ] - buffer of %ld hours\n", cindex,logCache[cindex].hrmtr-logCache[cindex].nHrs,ctime(&bhr),search,lbuf); 
					//printf(">debug exit.\n"); exit(0);
				}
			} //end search logCache block */
			//--Display/debug info:--------------------------------------------:
			snprintf(strbuf, (size_t) 16, "%d", logn);
			strncpy(strbuf, appendStr(3,"Searching logs ",strbuf," days ago..."), sizeof(strbuf)/sizeof(strbuf[0])); 
		} //-:end Daysago init
		//*/--Hourmeter Log Search:---------------------------------------------------:
		else if (logOpt==3 && now < lint) { printf(">...error with request time\n"); return -1; }
		else if (logOpt==3){ search = lint; snprintf(strbuf, (size_t) 128, "%ld", lint);  //------:Hrmeter date 
			strncpy(strbuf, appendStr(3,"Searching log hourmeter ",strbuf,"..."), sizeof(strbuf)/sizeof(strbuf[0])); 
		} 
		//-----Old Log search, exit if searching before oldest log, OR read All logs:--------:
		//-------------do this always? search start could be narrowed...
		if (logn>200 || logOpt==3 || logOpt==4) {  //:find oldest log, and check hourmeter
			long oldestLog = 0; uint startX = 0;  
			//--get latest log:---------: 		//memory address=logs[a].log[0].hexa
			searchLogs(ctx, start, (nNow-24), (short) 2, logs);
			if (!logs[1].log[0].hexa) { startX = logs[0].log[0].hexa; } 
			else { startX = logs[1].log[0].hexa; }
			//--get oldest log:---------:		//latest wraps around to oldest.
			readLogs(ctx,startX, (short)2,logs);
			if (logs[1].log[0].value.lv) { oldestLog=logs[1].log[0].value.lv;  } //-:oldest hrmtr
			 if (debug) printf(">Oldest log hourmeter: %ld\n", oldestLog);
			if (oldestLog) { 
				if (logOpt==4) { start=logs[1].log[0].hexa; tnum=256; //-:oldest addr hexa (return ALL logs)256?
					strncpy(strbuf, "Reading All logs ...", sizeof(strbuf)/sizeof(strbuf[0])); } 
				else if (search<oldestLog) { if (!sil) printf("\nSearch is before oldest log! exiting.\n"
					"-----------------------------------------------------------------\n\n"); 
					exit(0); } 
		}	}
		//------------------------ END LOG OPTIONS INITIALIZATION -------------------------------//
		if (start < 0x8000 || start > 0x8FFF) { start=0x8000; }  //redundant check?
		
		if (!sil) { 
			snprintf(sbuf, sizeof sbuf, "(%ld hours)", lbuf); //-:use sbuf to print lbuf_input
			snprintf(bufs, sizeof bufs, "Now: %ld , unix: %ld , ", now,c_time); //lbuf_input !!!
			printf("\n>Daily Logs:----------------------------------------------------:\n"
				"--Reminder: Log dates are based on continual use from today--\n%s:date buffering: [%s] %s\n%s retrieving %d logs\n", 
				(debug?bufs:""), (lbuf_input?"on":(logCache[0].xt?"cache":"off")),
				(lbuf_input?sbuf:""), strbuf, tnum); 
			if (debug>2) printf("[%d]>Reading Log: 0x%04X, searching for %ld\n\n",debug,start,search);
			memset(&strbuf[0], '\0', sizeof(strbuf));	  //--not needed if using snprintf next...
			//if (json || cvs) sil=1;  //-:silence all further generic prints..
		}	
		/*/--Read Logs & Parse:--------------------------------------------------------------------// */
		short totalLogs=0;
		//--Search:--------------------------------------------------------------------:
		if (search) { 
			totalLogs=searchLogs(ctx, start, search, tnum, logs); 
		//--No Search:-----------------------------------------------------------------:
		} else { totalLogs=readLogs(ctx,start,tnum,logs); }  //
		//--fill debug logs here.!
		
		if (!logs[0].log[0].value.lv){ if (!sil) printf("\nNO LOGS!...found!\n" 
				"\n================================================================\n"
				"================================================================\n\n"); return -1; } 
		 size_t logsx = sizeof(logs)/sizeof(logs[0]);//(size_t)256
		//ffeature:sort logs?...--: voidsort(logs,now,search);
		
		/*/--Create json: log header data:-------------------------------:// */
		if (json) { 
			if (logOpt==1){snprintf(sbuf,sizeof(sbuf),"\"logread\":\"start %d\",\"hrmeter\":%ld",start,now);}
			else if (logOpt==2){snprintf(sbuf,sizeof(sbuf),"\"logread\":\"search day %hu\",\"hrmeter\":%ld",logn,now);}
			else if (logOpt==3){snprintf(sbuf,sizeof(sbuf),"\"logread\":\"search hr %ld\",\"hrmeter\":%ld",search,now);}
			else if (logOpt==4){snprintf(sbuf,sizeof(sbuf),"\"logread\":\"all\",\"hrmeter\":%ld",now);}
			//-------js: json.logs[0].data[0].value; ~=> logs[0].meta.udate;
			snprintf(bufs, sizeof(bufs), "{\"action\":\"logs\",%s,\"max\":%d,\"date\":\"%s\",\"udate\":%d,\"logs\":[", 
															sbuf,totalLogs,escapeStr("json",ctime_s), (int)c_time); 
			writeFile("logsc", c_time, bufs);   memset(&sbuf[0], '\0', sizeof(sbuf));
			//memset(&bufs[0], '\0', sizeof(bufs));  //--not needed if using snprintf next...
		}
		
		
		float daysago = 0; time_t logDate; 
		//__Print Loop: -----------------------------------------------------------------------------//--: 
		if (!sil && !json) printf("\n>Found (%d) Logs:______________________________________ \n",totalLogs); 
		for (short a=0; a<(int)logsx; a++) { //read all logs <256 !!!!-----------------------------//--:
			if (!logs[a].log[0].hexa){ if (!sil && !json){printf("...Last log\n");} break; } //last
			if (!logs[a].date){ logDate=0; //--no date:----------------------------:
				if (!sil && !json) printf("\n-----------------------------\n>Date: [ null ] :\n"); 
			} else { //--Parse hours into a date:----------------------------------: 
				//time_t curr_hrs = c_time/60/60; 				//--:unix hrs
				logDate = logs[a].date; 	logs[a].date=0;		//-:hrs here
				//--hrs ago: !!subtract logDate from now and add buffer to get days ago, then subtract from time.
				//ee = (nNow - logDate); //nNow is normalized but breaks things	** !!!! **
				//if (debug>1) printf("<norm daysago=%f >\n",(ee/24.0));
				long ee = (now - logDate); //:hrsago unbuffered :logDate exact exactness 
				//if (debug>1) printf("<raw daysago=%f >\n",(ee/24.0));
				
				/* //----------  Modify Date buffer --------------------------------------------------//  */
				//!--Use first logCache search: --------------------:   ...logDate is 1hr past sunset.. 
				if (a==0 && cdMarker==-1) { logs[a].date = cacheDate;  } //-:search was exact match in cache
				//!--User input buffer: (increase hrs for correct date calc)------:
				else if (lbuf_input){ ee+=lbuf; if (debug>1) printf(">>>buffering %ld hours\n",lbuf); }   
				
				//!--LogCache: skip first search-opt log, +skip if newer than cache. 1st cache search for..:
				//else if (logCache[0].xt && logDate <= logCache[1].hrmtr+36 && (a || logOpt!=2)) {
				else if (logCache[0].xt && (a||logOpt!=2)) { //-:(skip 1st daysago search)
					//(logDate>=earLog &&  || (logDate<earLog && !a && logOpt==2))
					/*/--LogCache: modify dates here, modify search above... */
					//-----logCache[0].xt == real number of cached dates! starting at 1
					char cdp=0; //cdp==true if searchLogCache(logDate,) , sbt==true if cindex<0 (for skipping reparse)
					//--SearchCache: new || prev. was latest || before all & !lbuf || log-wrap-around:...
					//if (!cindex || cindex==1 ||(cindex>logCache[0].xt && !lbuf)||(a && logDate < logs[a-1].log[0].value.lv)){
					if (!cindex || cindex==1 ||(logDate < earLog && !a)||(a && logDate < logs[a-1].log[0].value.lv)){
						//-----(!cindex && !a)
						nextCD=0; cacheDate=0; searchLogCache(logDate, &cacheDate, &cindex);  cdp=1; sbt=0;
						//--jump to parsing block:
						cdMarker=1; goto cacheDParse; //-:parse block  
						cacheDParse1: ;   
						//-set log date if match:  
						if (cindex>0 && cindex<=logCache[0].xt) {  logs[a].date = cacheDate; } 
						else if (cindex<0){ cindex=cindex*-1; sbt=1; } //
					} else if (! nextCD && cindex && cindex<=logCache[0].xt) {  nextCD = cindex-1; } //-:init nextCD
					
					if (logs[a].date) { if (debug>1) printf(".");  } //-:exact match skip:dummy (exact: nextCD can eq 0!) 
					else if (cdp) {  if (debug>1) printf(">>>done.\n");  }
					/*/--Above searchLogCache() skipped: check logDate buffers or re-search below... */
					//---:before: cindex>.xt & nextCD==0, after: cindex==0 & nextCD==0, 
					//else if (cindex>logCache[0].xt || !nextCD || nextCD<0) {  		?after? nextCD should eq 0..
					else if (logDate<earLog || !nextCD || nextCD<0 || (logDate>logCache[1].hrmtr+36)) {  
						if (debug) printf(">>>Nothing more doing, before or after logCache. [buffer:%ld]\n",lbuf); 
					} //--below uses nextCD to increment.. 
					
					//--if match use next cache date:  !!!!----------time calculation --------------!!!!!!!
					else if ((logDate+8) > logCache[nextCD].hrmtr && (logDate-22) < logCache[nextCD].hrmtr){
						cindex=nextCD;  sbt=0; //timeSkipB=0;//zero     
						if (debug){ printf(">>>Next cache date is a match>>[%d]\n",cindex); }
						//--recalculate lbuf if needed: 
						cdMarker=3; goto cacheDParse; //-:parse block  
						cacheDParse3: ;
						logs[a].date = logCache[cindex].udate; 
					}
					//--before next cache date - no change to lbuf: ..otherwise recalculate below: (and set sbt) 
					else if ((sbt || !lbuf) && logDate < logCache[nextCD].hrmtr) { 
						if (debug){ snprintf(sbuf,sizeof(sbuf),"(buffer of %ld hrs)",lbuf);
							printf(">>>before next cache date>> %s\n", (!lbuf? "(no buffer)":sbuf)); } 
					} 
					
					/* //----------- After Next Cache Date or update buffer ------------------- */
					else { //--recalculate lbuf, nextCD , timeSkipB 
						if (logDate < logCache[nextCD].hrmtr) { 
							if (debug) { snprintf(sbuf,sizeof(sbuf),"buffer of %ld)",lbuf);
								printf(">>>before next cache date. (recalculating... %s\n", (!lbuf? "no buffer)":sbuf)); }
							cindex=-cindex;
							cdMarker=4; goto cacheDParse; //-:parse block  
							cacheDParse4: ;	 cindex=cindex*-1; sbt=1; 
						}
						else { //--re-search logcache: (logDate > logCache[nextCD].hrmtr)
							if (debug) { printf(">>>Log date after next cache date. Searching again..\n"); }
							nextCD=0; cacheDate=0; searchLogCache(logDate, &cacheDate, &cindex); cdp=1;
							cdMarker=2; goto cacheDParse; //-:parse block  
							cacheDParse2: ;	 sbt=0; //..reset sbt mark
							//--set log date if match: 
							if (cindex>0 && cindex<=logCache[0].xt) { logs[a].date = cacheDate; }
							else if (cindex<0){ cindex=cindex*-1; sbt=1; }
							//else if (cindex>logCache[0].xt) {  } //before any logs...
						}
					}  		
				}  //--:end logCache block //-:else if (logCache[0].xt && a) { cindex = 0; }//reset? after latest cache.
				
				cdMarker=0; 	
				if (logs[a].date) { cdMarker=1;  } 	//--:logcache date match! 
				if (lbuf && !lbuf_input) { ee+=lbuf; } //--:add buffer! (user lbuf added above..)
				
				
				/*/--date and to string:--------------------------: */
				if (!logs[a].date){ logs[a].date = c_time-(ee*3600); }	//--:time_t
				//--check buffered date hour:----buffered log dates should be in evening if correct..
				//----buffering inexact times in cache can result in hrmtr drift...
				if (!lbuf_input && cdMarker!=1 && logCache[0].xt && lbuf>4) { 
					struct tm *tmStruct; tmStruct = localtime (&logs[a].date); 
					if (tmStruct != NULL && tmStruct->tm_hour < 10) { //-<10am? probably previous day in buffer... 
						logs[a].date = (logs[a].date-(tmStruct->tm_hour*3600))-(nH*3600); 	//-:norm time 
						if (debug>1) printf(">logDate hrs: %d am -> buffering to previous day (~%dpm)\n",
								tmStruct->tm_hour,(int)nH);  //daysago++; 
				}	}  
				//--Daysago calc:----------------------------------:
				daysago = ((normT-logs[a].date)/3600)/24.0;  //c_time or normT???
				//if (debug) printf("< daysago=%f > (normT w/buf)\n",daysago);
				
				daysago = daysago<0 ? daysago*-1:daysago;  //<0.2==normalized/evening time allowance
				daysago = (daysago-floor(daysago))<0.23? floor(daysago):ceil(daysago);
				//--------------------------------------------------
				
				/* Final Displayed Date */
				if (date_format[4] == 'b') { date_format[4] = 'B'; } //-:full-month-name hack
				strbuf[0]='\1';	  //--
				int len = strftime(strbuf,(size_t) 127, date_format, localtime(&logs[a].date));  
				//--error check:
				if (len == 0 && strbuf[0] != '\0') { strncpy(logs[a].date_s,"-error with date-",127); }
				else { strncpy(logs[a].date_s,strbuf,127); } memset(&strbuf[0], '\0', sizeof(strbuf));
								//-add null to str?:  logs[a].date_s[ strlen(logs[a].date_s) -1 ] = '\0';
			} //--end log date parsing */
			
			
			/*/--JSON: log meta data & data----------------------------------------------//: */
			if (json) { //--log metadata:---------------------------: //strbuf 
				snprintf(bufs,sizeof(bufs),
				   "%s{\"meta\":{\"logId\":\"0x%04X\",\"daysago\":%.2f,\"udate\":%ld,\"approxiDate\":\"%s\"},\"data\":[",
					(a>0?",":""),logs[a].log[0].hexa,daysago,logs[a].date,escapeStr("json",logs[a].date_s));
				writeFile("logs", c_time, bufs); //: \"meter\":%ld, logDate
				//--Log data:--------------------------------: skip combine HI's?
				jsonOut("logs", (int)16, logs[a].log, 0, NULL); 
				writeFile("logs", c_time, "]}"); //:close log[x] data
				continue; 
			}
			
			/*/--Print Out:---------------------------------------------------------------//:sorting? */
			if (debug>2 || raw) {  printf(">Log [%d]-----------------------:\n",a);  //:debug 
				printf("%s %s\n",(cdMarker==1?"Cached Date:":"Approximate Date:"),logs[a].date_s); 
				if (cindex && timeSkipB){ snprintf(sbuf,sizeof(sbuf),"(date is within [%ld] hrs)",timeSkipB); }//-:buffer
				printf(" (%.0f days ago)\t %s\n", daysago, (cindex && timeSkipB)?sbuf:""); 
				//--iterate: 
				for (short x=0; x<16; x++) { printOUT (x, 'z', &logs[a].log[x]); } 
				printf("----------------------------------\n\n"); 
			} else {
				// * //--Formated:--------------------------------------------: * /
				printf("################################## 0x%04X #####################################\n",logs[a].log[0].hexa);
				printf("%s	%s\n",(cdMarker==1?"Cached Date:\t":"Approximate Date:"),logs[a].date_s); 
				if (cindex && timeSkipB>4) printf("\t\t(date is within [%ld] hrs)\n",timeSkipB);//-:buffer vagueness >6?
				printf("Time of Log: %ld hrs		(%.0f days ago)\n",logs[a].log[0].value.lv, daysago);
				printf("Min Battery volts:	%f v	Max Array volts:	%f v\n",logs[a].log[8].value.fv, logs[a].log[12].value.fv);
				printf("Max Battery volts:	%f v	Load Ah:		%f Ah\n",logs[a].log[9].value.fv, logs[a].log[11].value.fv);
				printf("Absorb time:		%.2f hrs	Charge:			%f Ah\n",secsToHours(logs[a].log[13].value.dv), logs[a].log[10].value.fv);
				printf("Float time: 		%.2f mins	EQ time: 		%.2f mins\n",logs[a].log[15].value.dv/60.0, logs[a].log[14].value.dv/60.0);
				printf("------------------------------------------------------------------\n");
				printf("Alarms: 	\n%s",logs[a].log[3].value.sv);
				printf("%s",logs[a].log[5].value.sv);  //Load Faults: 	
				printf("%s",logs[a].log[7].value.sv);  //Array Faults:	
				printf("\n###############################################################################\n");
			} 
		} //-:Done Looping Logs. - close json logs array and file:
		if (json) { writeFile ("logs", c_time, "]}\n"); if (!sil){printf(">Logs written to json..\n\n");} return 0; }
		//-:close
		printf("\n----------------------------------------------------------------\n\n\n\n");
		return 0;
		
		/* -----------------------------------------------------------------------------------------
		//--Cache Date parsing block:---------------------------------------------------------------//
		* ------------------------------------------------------------------------------------------// */
		cacheDParse: ;
		/*/--Multi-LogCache Search: search precision, get accurate dates.. matched to external cache log
		 	//---cindex: 0=gt last, 1(+)=within 48hrs, 2(minus)=gt cindex & lt next, 3(total++)=lt oldest
			//----timeSkipA/B are Hours diff from continuous use [hrmtr] date == buffer.----: */
			//---- computed log dates == c_time-( ((now-logDate)+lbuf)*3600 )  
		 	//--logCache[cindex].nHrs = hrs to add to cacheD to normalize time..affects what date a log is given.
		 	if (debug>2) printf("parsing [%d] in goto block:\n",cdMarker); //" %ld",targetD
		 	/*/--Before cache Dates:-------------------------------------------------: // */
			if (cindex>logCache[0].xt) { if (debug) printf(">Target Date is before any log cache dates.\n"); 
				//--Use oldest cache date for buffer:----------------------: {been here already?} 
				cindex--; //--:oldest log, change back before returning! 
				timeSkipB = (c_time-logCache[cindex].udate)/3600 - (now-logCache[cindex].hrmtr);
				if (cdMarker==0||cdMarker==-1) { //init-ing search  && cindex>logCache[0].xt)
					//--Diff b/t target date and cache dates: ¡¡targetD date is OLDEST possible!!
					float last = (logCache[cindex].udate-targetD)/3600; //-:hrs/24.0   //long expectedD = search;
					 if (debug>1) printf(">>timeSkipB: %ld  ,  oldest: %f\n",timeSkipB,last);																	
					//--refactor search:-------------------:
					search = logCache[cindex].hrmtr-last; //-:using last
				  	 if (debug>1) printf(">>oldest [%d] %ld timestamp to target being used. :> %.2f \n",cindex, 
														logCache[cindex].hrmtr-logCache[cindex].nHrs, last/24); 
				} else { 
					 if (debug>1) printf(">>oldest [%d] %ld timestamp to target being used. \n",cindex, logCache[cindex].hrmtr-logCache[cindex].nHrs); 
				}  
				if (debug>1) printf(">>(New hrmtr correctness: [%ld]) using oldest value\n",timeSkipB);
				if (timeSkipB<0){ timeSkipB = 0; } //timeSkipB*-1
				nextCD=cindex; lbuf=timeSkipB; //--:lbuf - last known buffer... 
				cindex++;      timeSkipB=0; //zero. no need to display buffer. 
				if (debug && cdMarker && lbuf) printf("Buffer of %ld hours\n", lbuf);
			}
			
			/*/--Found Match! date cache:--------------------------------------------: // */
			else if (cacheDate && cindex>0) { 
				if (cdMarker==0||cdMarker==-1) search=logCache[cindex].hrmtr;  
				nextCD=cindex-1;     //-can correct time here..cachetime -> logtime.
				//--lbuf needed for dating other logs: ??used???
				lbuf = ((c_time - logCache[cindex].udate)/3600)-(now-logCache[cindex].hrmtr);  
				if (lbuf<0){ lbuf=0; } timeSkipB=0; //zero
				if (debug) { printf(">>Found exact match (buffer: %ld)\n",lbuf);  } 
			}
			
			/*/--Search found possibly close matches:--------------------------------: // */
			else if (cacheDate && cindex) { 	cindex=cindex*-1;  nextCD=cindex-1;   
				//--:target gt previous, lt next------------------------------: 
				float previous = 0;	 float nextCache = 0;  long expectedD=0; //:checking date buffer
				//--Date Buffer:-----------------gaps in time since now-------:
				//-hrs before/after:	buffer: timeSkipA is min, timeSkipB is max. 
				timeSkipA = (c_time-logCache[nextCD].udate)/3600 - (now-logCache[nextCD].hrmtr);
				timeSkipB = (c_time-logCache[cindex].udate)/3600 - (now-logCache[cindex].hrmtr);
				//notes:---------------------------------------:   (previous+nextCache)=total hrs;
				//-Find which cache ts date/buffer is better base for date...  
				//--if (time b/t != hrs b/t):--------:   ..lbuf==timeSkipB most of time...
				//---(([nextCD].udate-[cindex].udate)/3600 != ([nextCD].hrmtr-[cindex].hrmtr)) { timeSkipB }
				//--logCache dates nonexact!  neg values due to normalization, fixed below.  !round down lbuf?
				
				if (cdMarker==0||cdMarker==-1) { //init-ing search:-----:   
					//--Diff b/t target date and cache dates: ¡¡targetD date is OLDEST possible!!
					previous = (targetD-logCache[cindex].udate)/3600; //-:hrs/24.0   !(-lbuf)
					nextCache = (logCache[nextCD].udate-targetD)/3600;//-:hrs/24.0   !(+lbuf)
					//--refactor search, check buffer:
					search = logCache[cindex].hrmtr+previous; //-:using previous 
					expectedD = search; //search is hrmeter base! targetD is time based! both are normalized!
				} else { //-Logs:---------------------------------------: subtract logCache[cindex].nHrs ?!?
					targetD = normT-(((nNow-logDate)+timeSkipA)*3600); //  !normalize check?!     
					//--Check buffer Range:  use c_time? normT? , ?(nNow just inits search)
					/* example: targetD=6pm+secs , targetDa=6pm minus secs  , targetDb=4pm (iso-timeSkipB+=2hr) * /
					 int targetDb = logCache[cindex].udate+((logDate-logCache[cindex].hrmtr)*3600);//needs iso-timeSkipB
					 int targetDa = logCache[nextCD].udate-((logCache[nextCD].hrmtr-logDate)*3600);//timeSkip builtin
					 //---targetDa is same as targetD (other than secs).
					printf(">in-between Debug: TargetD = %ld , a=%d , b=%d \n", targetD, targetDa, targetDb); // */
					previous = (targetD-logCache[cindex].udate)/3600;   //-:hrs/24.0 		
					nextCache = (logCache[nextCD].udate-targetD)/3600;  //-:hrs/24.0
					expectedD = logCache[cindex].hrmtr+previous+timeSkipB; //-:check date overrun!
				}
				if (debug>1) { //--debug:---------------------------------:
					printf(">>nextCache: %.02f  ,  PREV: %.02f \n",nextCache,previous);	
					printf(">>(New hrmtr correctness: [%ld : %ld])\n", timeSkipA,timeSkipB); //using second value
				}
				//if (timeSkipA<0){ timeSkipA=timeSkipA*-1; } if (timeSkipB<0){ timeSkipB=timeSkipB*-1; } 
				if (timeSkipA<0){ timeSkipA=0; } if (timeSkipB<0){ timeSkipB=0; } //-:zero both. needed.
				/* ----- timeSkipB is displayed as buffer ------ */ 
				//--Isolate timeSkipB:  timegaps b/t timeSkipB and timeSkipA may affect search or may not...
				if (timeSkipB != timeSkipA) { timeSkipB= timeSkipB-timeSkipA; }	
				else { timeSkipB = 0; }
				
				//-recalculate lbuf to bring hrsago to correct date.:    timeSkipB becomes buffer! 
				//--DEBUGing ---------------------- timeSkipB---------:-----------: !!!
				if (debug>1 && (logCache[nextCD].udate-logCache[cindex].udate)/3600 != (logCache[nextCD].hrmtr-logCache[cindex].hrmtr)) { long expDn = (logCache[nextCD].hrmtr-nextCache);
					printf(">TimeSkip in hrmtrs: %ld ts != %ld hrm -  later base?:[%ld] %s\n",((logCache[nextCD].udate-logCache[cindex].udate)/3600), (logCache[nextCD].hrmtr-logCache[cindex].hrmtr), expDn, 
							(expDn<logCache[cindex].hrmtr?"(wrong!)\n":"(ok)")); 
				} //--:DEBUG! ---------------------------- */
				
				//--Basic Buffer Check: use later if gt:----------: previous: 3 weeks(504) ?longer? 30 days??
				if (expectedD>logCache[nextCD].hrmtr || previous>504) { //&& previous>nextCache
					lbuf = timeSkipA;  //-!
					if (cdMarker==0||cdMarker==-1) search = logCache[nextCD].hrmtr-nextCache;  
					if (debug>1) printf(">>later timestamp [%d] %ld to target %.2f days. (previous %ld: %.2f)\n",nextCD, 
							logCache[nextCD].hrmtr-logCache[nextCD].nHrs,nextCache/24,logCache[cindex].hrmtr-logCache[cindex].nHrs,previous/24); //
				//} else if ((logCache[nextCD].hrmtr-nextCache) < logCache[cindex].hrmtr) { for precision..??
				//--Full buffer:--------------------------------:
				} else {  lbuf = timeSkipA+timeSkipB; //--info/debug message: roundup(previous/24)
					//if (cdMarker==0||cdMarker==-1) search += timeSkipB;  
					if (debug>1){ printf(">>previous timestamp [%d] %ld to target %.2f days. (later %ld: %.2f)\n",
							cindex,logCache[cindex].hrmtr-logCache[cindex].nHrs,previous/24,
							logCache[nextCD].hrmtr-logCache[nextCD].nHrs,nextCache/24); }
				}  
				/*/--cache buffering debug: 
				if (debug && logCache[cindex].nHrs) { //not true if using nextCD...
					printf("cached date-time normalized %d hours\n",(int)logCache[cindex].nHrs); 
				} // */
				//--cleanup--------------------:
				if (timeSkipB<0){ timeSkipB = timeSkipB*-1; }
				cindex=cindex*-1;  //--:reset
				if (debug && lbuf) printf("Buffer of %ld hours\n", lbuf);
			}
			/*/--After cache dates, Nothing found: (cindex==0)-----------------------------------: // */ 
			else { timeSkipB=0;  nextCD=0; 
				if (debug) { printf(">Nothing found in cache: logCache[ %d ] = %ld\n",cindex,cacheDate);
					if (lbuf) printf("Buffer of %ld hours\n", lbuf); } 
			}
			//--Final Debug:---------------------:
			//----Before: cindex>.xt & nextCD==0, 	--After: cindex==0 & nextCD==0, ----
			//----Exact: nextCD can eq 0!, 			--Between: cindex<0 & nextCD ----
		//**--return to jumplabel position:--------------------------------:
		if (cdMarker==-1) { cdMarker=(sizeof(cdLabels)/sizeof(cdLabels[0]))-1;  }
		goto *cdLabels[cdMarker]; //-jump back
	}//----------------------------------END LOGS BLOCK------------------------------------ */	
	//--:end Main:
}




/* ######################------------- FUNCTIONS ----------------################################
* ################################################################################################# */
// * //-------------ALL string function calls need checking! to avoid char overruns!------?!!!  *  //
static void readModbus(modbus_t *ctxx, uint start, ushort length, uint16_t *datas) {
	int rc = -1; ushort x=0;
	while (rc == -1 && x < 16) { //-:with timeout
		if (!ctxx) {  fprintf(stderr,">read: reseting modbus!\n");
			// * Set up a new MODBUS context * /
			ctxx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 2);
			if (ctxx == NULL) {
				if (!sil) { fprintf(stderr, ">read: Unable to create the libmodbus context\n"); }
				exit(EXIT_FAILURE); // return -1;	
				}
			// * Set the slave id to the ProStar MPPT MODBUS id * /
			modbus_set_slave(ctxx, modbusId);
			// * Open the MODBUS connection to the MPPT * /
		    if (modbus_connect(ctxx) == -1) {
		        if (!sil) { fprintf(stderr, ">read: Connection failed: %s\n", modbus_strerror(errno)); }
		        modbus_free(ctxx);
		        exit(EXIT_FAILURE); // return -1;
		}    }  
		
		//--bulk read:	//uint16_t datas [length];   
		rc = modbus_read_registers(ctxx, start, length, datas);
		if (rc == -1) {	if (debug) fprintf(stderr, ">read: %s\n", modbus_strerror(errno)); 
			x++; } //printf(">read: start: 0x%04X x %d\n",start,length);
	} //--sixteen tries should be enough..
	if (rc == -1) { if (start>0x8000) { return; } //-:dont exit on failure for logs
		else exit(EXIT_FAILURE); }
} 

//const RamObj //--used w/ pollingOut..
void upRegisters(modbus_t *ctxx, RamObj *inStructA, int num, int *registers, int rn) { //rn==# of registers
	//--in-place editing of RamObj----------------------: (ex: upRegisters(ctx, ram, num, 3hexaArray, 3);)
	uint16_t rdata[rn]; 	
	//--Fill register struct-- 
	for (int i=0; i<rn; i++) {
		//--read:
		readModbus(ctxx, registers[i], 1, rdata);
		//--parse values into RamObj ram[] or eprom[]: 
		int ri = getIndex (registers[i],num,inStructA);   //-
		//--Combines-------------------------------:
		if (strncmp(inStructA[ri].calc,"comb",4)==0) {   
			inStructA[ri].value.lv  = combine(inStructA[ri].hexa, rdata[0], num,inStructA); 
			if (inStructA[ri].value.lv == -1){ inStructA[ri].value.lv = 0; }
		}
		*(inStructA + ri) = parseValue (&inStructA[ri], rdata[0]);
	}
}

static uint16_t writeUpdate(modbus_t *ctxx, RamObj *inStruct) {
	//--write data, check, return current value:
	uint16_t wdata[1]; 
	int rc;	//--#link already live!
	//--Write Update:-------------------------------------: 
	if (update && debug < 3) { //--write NEW registers:
	 	if (debug && !sil){ printf(">Updating to 0x%04X [ %d ]\n", inStruct->hexa,inStruct->f_new); }
		rc = modbus_write_register(ctxx, (int) inStruct->hexa, inStruct->f_new);  
		 if (rc == -1) { fprintf(stderr, "%s\n", modbus_strerror(errno)); }
	} else { printf(">not writing....[ 0x%04X ] ", inStruct->hexa); }
	 //--Check if Value written correctly:-------------------: */
	rc = modbus_read_registers(ctxx, (int) inStruct->hexa, 1, wdata); 
	 if (rc == -1) { fprintf(stderr, "%s\n", modbus_strerror(errno)); modbus_free(ctxx); return -1; }
	/* /--data check: -...compare after call...-
	if (wdata[0]!=inStruct.f_new) { fprintf(stderr,"\n>Error writing new value [%hu]!"
								" current value: %hu \n\n", inStruct.f_new, wdata[0]); } //- */
	//--Return Current value:
	return wdata[0];
} 

static short readLogs(modbus_t *ctxx, uint start,short tnum,LogObj *logs) { 
	//--return total found: logs [256 max?]... (logs[] init top of logBlock)
	//----------maybe pass oldestLog.hexa: oldest-to-end, then wrap around till null?
	memset(&logs[0], 0, (size_t)256); //-:clear
	short total = 0;  short bi=0; char leftovers=0; char lp=0;
	
	/*/--Create Bulk:---------: ------------------------------------------ --- // */
	//----(C libmodbus max reads ~112 registers [6-7 logs])-----------------:  char maxLogs=6; 
	//printf(">!!!!Bulk size max == %f , ceil %d / 6 \n",ceil(tnum/6.0),tnum); 
	//total = ceil(tnum/6.0);
	struct bulkData bulk [(size_t)ceil(tnum/(maxLogs*1.0))];  //-:ceil(x/6.0) increases +1 for remainders! 
	short remaining = (0x9000 - start)/16;  //-:split:(reused below for total bulk[]):  remaining=(0x9000-oldest.hexa)/16;
	//--Break into chunks of 6 logs:-------------------------: (bulk[].start is updated during reading)
	if (debug>1) printf("\n>ReadLogs( %d )\n",tnum); //1st start most important! rest can be modified..
	if (tnum>maxLogs) { leftovers = tnum%maxLogs;  //--:last bulk[].i      
		 if (debug>2 && leftovers) printf(">...leftovers -- %d\n",leftovers); 
		bulk[0].start = start;
		for (int uu=0; uu<tnum/maxLogs; uu++) { 
			if (uu) { bulk[bi].start = bulk[bi-1].start+((1*maxLogs)*16); } //uu*
			//--some logs read twice if readLogs start is reread after null-loop-around:
			if (bulk[bi].start>0x8FFF) {bulk[bi].start=0x8000; lp=1;} 
			if (lp && bulk[bi].start>=bulk[0].start) { if (debug>2){printf(">...last bulk[ %d ]...\n",bi);} 
				bulk[bi].start=0; leftovers=0; break; }
			bulk[bi].i=maxLogs;  
			
			 if (debug>2) printf(">...bulk %d - %d - starting: 0x%04X\n",bi,bulk[bi].i,bulk[bi].start);  //-----debug
			bi++; //-:increment 
		} 
		if (leftovers) { bulk[bi].i = leftovers;  bulk[bi].start=bulk[bi-1].start+(maxLogs*16);   
			if (bulk[bi].start>0x8FFF) {bulk[bi].start=0x8000;} //split????
			 if (debug>2) printf(">...bulk %d (leftovers) - %d - starting:0x%04X\n",bi,leftovers,bulk[bi].start);  //-----debug
			bi++; 
		} //printf(">!.next bulk %d - bulk[%d] 0x%04X , bulk[%d]: 0x%04X\n",u,u-2,bulk[u-2].start,u-1,bulk[u-1].start);
		remaining = (short) (sizeof(bulk)/sizeof(bulk[0])); //-:
		//--end multi bulk[] split.
	//--Two Bulk - Split overflow:-----------------------------------------: 
	} else if (start!=0x8000 && remaining < tnum) { bulk[0].start=start; bulk[0].i=remaining;//*16;  //-:
		bulk[1].start=0x8000; bulk[1].i=(tnum-remaining); remaining=2; } //-:split log read
	//--One Bulk:-----------------------------------------------: 
	else { 	bulk[0].start=start; bulk[0].i=tnum; remaining=1; } 
	//--Done creating Bulk[]:
	if (debug && tnum>2){ printf(">Total Log Reads  - %d\n",remaining); }  //exit(1);
	
	
	/* -----------------  Loop Bulk Reads -------------------------------------------------- 
	________________________________________________________________________________________ */
	short logx=0; 	//+'total' is redundant to logx
	short bulkT=remaining; remaining=0; leftovers=1;//-:leftovers used to avoid looping cleared area twice
	lp=0; //--:some logs read twice if readLogs start is reread after null-loop-around
	uint bstart=bulk[0].start;
	//--loop bulk[], filling in logs w/ datal:-----------------------------------------------: 
	for (short b=0; b<bulkT; b++) { bi=0; if(debug>1 && tnum>1)printf(">----Read [%d]\n",b); //-:sizeof(bulk)
		if (!bulk[b].i) { continue; } //--empties  		||==65535
		if (bulk[b].start > 0x8FFF) { bulk[b].start=0x8000; lp=1; } //end. goto beginning.
		if (debug>1 && tnum>1) printf(">.read..0x%04X:......return %d.........%s\n",bulk[b].start,bulk[b].i,(lp?"lp":""));
		if (lp && bulk[b].start>=bstart) { bulk[b].start=0; break; }
		uint16_t datal [bulk[b].i*16]; 	short p=0;
		
		//--checking overflow:------------------------------------------------:
		if (bulk[b].i>1) { remaining = (0x9000 - bulk[b].start)/16; 
			//--split if end of logs.    //--
			if (remaining < bulk[b].i) { readModbus(ctxx, bulk[b].start, remaining*16, datal);
				if (debug>1 && tnum>1) printf(">...bulk [%d] split - %d - 0x%04X\n",b,remaining,bulk[b].start);
				p = parseLogs(bulk[b].start,logx,logs, datal,remaining); 
				bi+=p; total+=p; logx+=p; memset(&datal[0],'\0', sizeof(datal));
				if (p<remaining && searching) { if (debug>1){printf("readLogs: ..last log...\n");} break; } //-:lastlog
				bulk[b].start=0x8000; bulk[b].i = bulk[b].i-remaining; lp=1;
			}  remaining=0;
		}	 //and split read of bulk[b]
		
		readModbus(ctxx, bulk[b].start, bulk[b].i*16, datal);
			//fflush(stdout); printf(">..read..0x%04X:..%d x %d\n",bulk[b].start, datal[0], bulk[b].i);
		//--parse multi logs:  if (p==bulk[b].i) good;  
		p = parseLogs(bulk[b].start,logx,logs, datal,bulk[b].i); //start @ logx w/start, parse bulk[b].i logs..
		
		//--Search Return:----------------------------------------------------:
		if (p<bulk[b].i && (searching || !leftovers)) { total+=p; //-:!leftovers marks skipahead section
				if (debug>1){printf("readLogs: ...last log...\n");} break; } //-:lastlog
		//--SkipAhead... & fill rest of bulk[b] logs--------------------------:
		else if (p<bulk[b].i && leftovers){ bi+=p; total+=p; logx+=p; short px=logx; //-:start with logs[px]  
			/*/--blank means logs hit cleared area:---------------- *///(size is b/t ~58 logs and ~2) 
			leftovers=0; //-:-----Do NOT loop around to nulls again (if tnum is >logs)--------
			if (debug){printf("readLogs %d: ...skipping forward...found %d\n",b,total);} 
			/* -------- skip forward mini search loop ------------ */
			uint16_t datalm[maxLogs*16]; char lx=0; 
			char marker=0; start = bulk[b].start+(21*16); 
			while(!marker){ 
				if (start>0x8FF0) { start=0x8000; lp=1; } //-:wrap. (last ram of last log)
				else if (start<0x8000) { start=0x8000; } 
				memset(&datalm[0],'\0', sizeof(datalm)); //-:reset: logs, lx, 
				//--checking overflow:-------------------------:
				remaining = (0x9000 - start)/16; lx=0; 
				if (remaining < maxLogs) { readModbus(ctxx,start,remaining*16,datalm); lx=(remaining-1); }
					//printf(">skipin!eologs! start: 0x%04X , lx:%d , ==%d\n",start,lx+1,datalm[0]); }
				else { readModbus(ctxx,start,maxLogs*16,datalm); remaining=maxLogs; lx=5;  } 
				//--skip ahead and find next non-null log:--both 1st & last null---:
				if ((datalm[0]==65535 || !datalm[0]) && (datalm[lx*16]==65535 || !datalm[lx*16])){ //-:!logDates
						start+=10*16; } 
				else if (datalm[0] && datalm[0]!=65535 && start==0x8000) { marker=1; }//!!right?yes?
				else if (datalm[0] && datalm[0]!=65535){ start-=3*16; }
				else if ((datalm[0]==65535 || !datalm[0]) && datalm[16] && datalm[16]!=65535) { //:found start
						start+=16; marker=1; }//printf(">skipinOut! start: 0x%04X , lx:%d\n",start,lx); }
				else if (datalm[0]==65535 || !datalm[0]) {start+=16;} 
			} //-:exit while loop. 	
			//--Start is at oldest log------------------------------:
			if (remaining>(bulk[b].i-bi)) { remaining=(bulk[b].i-bi); lx=5; }
			if (lp && start>=bstart) {break;}
			
			readModbus(ctxx,start,remaining*16,datalm);
			p=parseLogs(start,px,logs, datalm, remaining); 
			memset(&datalm[0],'\0', sizeof(datalm)); //-:zero 
			start+=p*16; px+=p; bi+=p; total+=p; 
			if (p<remaining) { if (debug>1){printf("readLogs: skipforward...last log...\t");} break; } //-:lastlog
			//if (debug>2) printf(">!!skipping found:%d  (next start: 0x%04X , lx:%d)\n",total,start,lx);
			if (lp && start>=bulk[0].start) {break;}
			
			//--continue bulk[]s if done, else fill rest of bulk[b] logs:
			if (bi>=bulk[b].i) { logx+=(px-logx); 
				if (b+1<bulkT && bulk[b+1].i){ bulk[b+1].start=start; //--:update next .start
					if (debug>2){printf(">!!skip! continue to next Bulk: start: 0x%04X\n",start);} } 
				continue; }
			else if (lx<5) { lx=bulk[b].i-bi; if(debug>2)printf(">!second skip read\n"); 
				readModbus(ctxx,start,(lx*16),datalm); 
				p = parseLogs(start,px,logs, datalm, lx); //-:parse into next empty
				memset(&datalm[0],'\0', sizeof(datalm)); //-:zero 
				start+=p*16; px+=p; bi+=p; total+=p; 
				if (p<lx) { if (debug>1){printf("readLogs: skipforward...last log...\t");} break; } //-:lastlog
			}
			//--end skip forward parse---: all bulk[b] logs should be filled...goto next bulk[]	
			if (b+1<bulkT && bulk[b+1].i) { bulk[b+1].start = start; }  //-:update next bulk[].start
			logx+=(px-logx);    //--- px=>next logx , bi=># of bulk[b].i , total=<tnum , -----------------
				if (debug>2) printf(">!end of skipping! start: 0x%04X , total:%d\n",start,total);
			//goto next bulk[]..
		} else { //--All bulk[b] good. increment..
			logx+=p; total+=p; //bi+=p;
			if (debug && tnum>1){printf("readLogs %d: ...found %d\n",b,total);}
			//--update next bulk[].start: (can be wrong if a previous bulk[] skipped forward..) 
			if (b+1<bulkT && bulk[b+1].i) { bulk[b+1].start = bulk[b].start+(p*16);  }
		}
		//--continue to next bulk[]			
	} //-:end bulk */
	if (debug && tnum>1) printf(">found [%d] total logs in logRead\n",total);    
	return total; //done. end read all logs.
}

static short parseLogs(uint start, short logx, LogObj *logs, uint16_t datal[], short bnum) {
	short ret=0;  //---------------logx=='start filling log here', bnum=='# of logs to fill'
	short di=0;   //---di=='start at datal[di]'
	if (!datal[di] || datal[di]==65535) return ret; //logx; //-:index of log to be filled
	if (bnum<1 || bnum>maxLogs) bnum=maxLogs; //-:max bulk[] logs  //char maxLogs=6; 
	//----------------------------------------------------------------------------------
	// * { 	uint32 hourx2; uint32 alarmx2; uint32 loadFaultx2; uint32 arrayFaultx2;
	//		uint16_t minV;  uint16_t maxV; uint16_t ah; uint16_t ahl; uint16_t maxAV;
	//		uint16_t absorbT; uint16_t eqT; uint16_t floatT;  } // * /
	//first 8 are combines:  {logical++,start++, 
	RamObj logT[16] = { {.unit="lhours", .string="hourmeter HI"}, //65535 default
			{.unit="lhours", .string="hourmeter LO", .calc="comb"},      //hourmeter is uint32 !!
			{.unit="bit", 	.string="Alarm HI"},
			{.unit="bit", 	.string="Alarm LO", .calc="comb", .group="state=0x0048"},  //ffff default
			{.unit="bit", 	.string="Load Faults HI"},
			{.unit="bit", 	.string="Load Faults LO", .calc="comb", .group="state=0x0046"},
			{.unit="bit", 	.string="Array Faults HI"},
			{.unit="bit", 	.string="Array Faults LO", .calc="comb", .group="state=0x0045"},
			//next 8 are day totals:
			{.unit="v", 	.string="min battery v",.calc="f16"},
			{.unit="v", 	.string="max battery v",.calc="f16"},
			{.unit="A", 	.string="Charge (daily)",.calc="f16"},   //nan defaullt
			{.unit="A", 	.string="Load Ah",.calc="f16"},
			{.unit="v", 	.string="Max Array v",.calc="f16"},
			{.unit="secs", 	.string="Absorb time"},					//65535 default
			{.unit="secs", 	.string="EQ time"},
			{.unit="secs",	.string="Float time"} 	//!!!!secs not mins
	};	
	//printf("testing: %d: starting logs [%d] , total [%d]\n",tnum,logx,bnum);
	 
	for (short yi=0; yi<bnum; yi++){ //fill logs[] starting @ logx and datal[di] 
			if (debug>1 && bnum>1) printf(">Log: %d [%d]----------------------------------------------:\n",yi,logx);
		//---Fill each logs[logx] with log[datal[*16]]:-----------------:
		/* //--Loop and assign/parse values into structs, then into logs[]:---------------------:  */
		//--Log struct: 
		RamObj log[16]; memcpy(log,logT,sizeof(logT)); 
		
		//--early null return: --return # of good logs...: [multi:array index of log to be filled]
		if (datal[di]==65535 || (!datal[di] && !datal[di+1])) { log[0].value.lv=0;
			memcpy(logs[logx].log, log, sizeof(log)); return ret; }
		
		//x=0; for (uint16_t *dp = &datal[di]; dp < &datal[16]; dp++) {  //pointer way.. maybe &log[0]
		for (short x=0; x<16; x++) {  //-:Single log  //probably simpler 
			//--ram:
			log[x].hexa=start; log[x].logical=start+1; //:fill struct 
			//Combines--------------------------------------:
			if (strncmp(log[x].calc,"comb",4)==0) { //previous must be filled..
				log[x].value.lv = combine(log[x].hexa, datal[di+x], 16,log); //*dp
				if (log[x].value.lv == -1){ log[x].value.lv = 0; }  
				}  //--Skip parsing HI combines? parsing not needed....
			/* -- Parse Log data -- */
			if (strcmp(log[x].unit,bits) == 0 && strncmp(log[x].group,"state=",6)!=0 ) { //bit HI's and hrmeter
				log[x].basev = datal[di+x]; 	log[x].typ = 's';  } //*dp
			//-- Parse data values into register struct:
			else { log[x] = parseValue(&log[x], datal[di+x]); } //filled: .basev, .dv||.fv||.sv||.lv , .typ !
			
			start++;
			//--Debug Print out: 
			//if (debug>3) printOUT (x, 'z', &log[x]); //>4?
		} 
		/* -- ADD log to logs[] array: ---------------------- */
		memcpy(logs[logx].log, log, sizeof(log)); 
		
		//--Fill logs Metadata: -----------------------------------  logs[a].date,logs[a].date_s;
		/* Hourmeter as date. ...real date after function: Dates Approximate */ 
		logs[logx].date = (time_t) (log[1].value.lv)?log[1].value.lv:0; //--:hours ,time_t seconds*3600
		if (debug>1 && bnum>1) printf(">parseLOGs! 0x%04X hourmeter value: [ %ld ]:\n",log[0].hexa, logs[logx].log[1].value.lv);
		//--increment:
		logx++; di+=16; ret++; 
	} //-end multi parse loop
	
	//ret = 1; //-:true/false: --------- OR (if multi: logx++,index of log to be filled) ?
	//--return number of logs parsed before hitting a null.. 
	return ret;
}

static short searchLogs(modbus_t *ctxx, uint start, long search, short tnum, LogObj *logs) { 
	//if (!search) return -1;
	char strbuf [256]=""; char bufbuf [128]=""; //strlen(long)==19, 
	searching=1;
	//--Search logs with limit on iterations:--------------------------------:
	short iteration=0; 
	while (iteration<51) { iteration++; 
		if (debug>2) { printf("%s",strbuf); //--print prev. iteration
			snprintf(strbuf,(size_t) 8,">%hu ",iteration); } //--:next  " search: "
		else { memset(&strbuf[0], '\0', sizeof(strbuf)); } //fprintf(fp,"\n"); //-:empty fprintf(fp,) needed.
		
		if (start < 0x8000) { start=0x9000-(tnum*16);  //beginning. goto end.
			if (start<0x8000) start=0x8000; }  // printf("...Starting from end of Logs..\n");
		else if (start > 0x8FFF) { start=0x8000; }   //end. goto beginning. search mini-loop
		
		readLogs(ctxx,start,1,logs); //--start as search query.  ++ now, c_time
		long logDate = logs[0].log[0].value.lv;  //--:first log item time. logs[0].date; 
		
		if (logDate) {   // * -- Search precision:  hourmeter -- * /
			//--Found it:----------------------------------------------:  		
			if (search >= (logDate-12) && search < logDate+6) { break; }        				//!found! First.
			//if (search >= (logDate-12) && search < logDate+(((tnum-1)*24)+6)) { break; }  	//!found! Bracket.
				//--sameday done below: ns>0 && ns<16 //sameday (logDate - 21)
						
			long ns = logDate - search;
			//---if search is < logDate, ns is positive, move backwards in logs..
				snprintf(bufbuf, (size_t)64, "logdate %ld - [ 0x%04X ]start\t",logDate, start); 
				strncat(strbuf,bufbuf,strlen(bufbuf));
			//--Search More:                 						cmd: date -v-14d
			if (ns < 0) { 		//--search is gt logDate:-----------------------------------:
				if (ns < (80*-24)) { start= start+(80*16); strcat(strbuf,">a<\n");  } 		//+gt 80 days later
				else if (ns < (25*-24)) { start= start+(25*16); strcat(strbuf,">b<\n");  }	//+gt 30 days later
				else if (ns < (12*-24)) { start= start+(12*16); strcat(strbuf,">c<\n");  }  //+gt 9 days later
				else if (ns < (6*-24)) { start= start+(6*16); strcat(strbuf,">c6<\n");  }  	//+gt 5 days later
				else if (ns < (tnum*-24)) { start= start+(tnum*16); strcat(strbuf,">d<\n");  } //+gt n days later
				//--:should be in bracket now: 
				else if (tnum>2 && ns < ((tnum-2)*-24)) { start= start+16; strcat(strbuf,">d1<\n"); } //+shift forward
				else if (ns < (3*-24)) { start= start+(3*16); strcat(strbuf,">d3<\n");  }  	//+gt 3 days later
				//--in bracket or breaking a racing condition b/t e & j:
				//else if (tnum>1 || polling==9) { strcat(strbuf,">gt bracket<\n"); break; } //bracket && ns < -8
				else if (ns < -9 && polling<6) { start= start+16; strcat(strbuf,">e<\n"); polling++; } //+move 1 day later //6?8?
				else { strcat(strbuf,">gt<\n"); break;  }							//--:sameday evening or next day
			} else if (ns > 0) { //--search is lt logDate:-----------------------------------:
				if (ns > (80*24)) { start= start-(80*16); strcat(strbuf,">f<\n");  } 		//-gt 80 days before
				else if (ns > (25*24)) { start= start-(25*16); strcat(strbuf,">g<\n");  }	//-gt 30 days before
				else if (ns > (12*24)) { start= start-(12*16); strcat(strbuf,">h<\n");  } 	//-gt 9 days before
				else if (ns > (6*24)) { start= start-(6*16); strcat(strbuf,">h6<\n");  }  	//-gt 5 days before
				else if (ns > (tnum*24)) { start= start-(tnum*16); strcat(strbuf,">i<\n");  } 	//-gt n days prev
				//--:should be in bracket now: 
				else if (ns > (3*24)) { start= start-(3*16); strcat(strbuf,">i3<\n");  }  	//-gt 3 days before
				//---make sure previous day data is returned with? 
				//			 (&& tnum < 3)
				//---Search is that day, or previous:-----:raised from 14,
				else if (ns > 18) { start= start-16; strcat(strbuf,">j<\n");  polling+=5;  	//-move 1 day before?
					if (polling>15) break; } //loop break
				else { strcat(strbuf,">lt<\n"); break; }							//--:sameday
			} else { strcat(strbuf,"><\n"); break; } //exact. iteration=0; 
			//...continuing search...
		//--logDate==null. try a bit earlier:--------------------------: (polling stops backward-forward loops)
		//--------maybe check oldest log hourmeter... to reduce iterations if older
		} else { strcat(strbuf,"> no logdate. going earlier\n"); start = start - ((tnum*16)+16); polling=8;  }		 
		if (iteration >= 50) { fprintf(stderr,">ERROR! no search. too many iterations!\n"); search=0; break; }	
	} //--:end of while loop */ 
	short tL=1;
	//if (debug>2){ printf("-%s",strbuf); } //--needed for debug3?
	//--Search found start, read all logs:
	if (tnum>1){ tL=readLogs(ctxx,start,tnum,logs); } //--:full search!
	searching=0;
	memset(&strbuf[0], '\0', sizeof(strbuf));
	return tL;	
}

static const long writeLogCache(char dool) {
	dool = dool?(dool==3? 3:1):0; //3==rebuilding, 0==create if !exists, 1==update, 
	//--append last line of cache: hourmeter, c_time,
	long latestHr=0; char e = 0; char fpbuf [512]=""; //char str[128]="";
	//printf(">Log Cache to %s.\n",file_logCache);
	//--Check file, create or read/write:--------------------------------------:
	struct stat exists;   	short st = lstat(file_logCache, &exists);			
	if ((st == -1 && errno == ENOENT) || dool==3) {   char tmp[48] = "";
		if (dool!=3) { snprintf(tmp, (size_t)48, "%ld, %ld,", now,c_time); }  //-:rebuilding cache
		//--write header w/ cvs def line if new file:
		snprintf(fpbuf,(size_t)512,"%s%s\n%s\n",cvsL,
			"#--first line MUST be exactly as above! (last comma opt.)\n"
			"//--line comments can be of either // or #\n"
			"//---endofline comments must use #\n"
			"#!beginLogs - everything above this bookmark is ignored.",
			tmp);
		e = writeFile("logCachec",c_time,fpbuf); 
	 	//printf(">error? %s\n",fpbuf); 
		latestHr = now;
	} else if (st == -1) {  fprintf(stderr,">ERROR with log cache file! [%s]\n\n",file_logCache);  }
	else if (!dool) { //-:Init file above or return latest:
		latestHr = readLogCache("lastLog"); e=2;  //fclose (fpp); -:Init file or return..
		if (display && latestHr > (now-48)) { printf("Log Cache: current..\n\n");  }
		//-logcache exists and is current. //-:with time preference
	} 
	else { //fclose(fpp); --read latest cache date, only write if gt ~24 hours ago...
		latestHr = readLogCache("lastLog"); //
		if (latestHr && latestHr < (now-21)) { //--Write data to file: && latestHr != now
			snprintf(fpbuf,(size_t) 19,"%ld",now);
			e = writeFile("logCache",c_time,fpbuf);   
			if (e) { latestHr = now;	if (display) printf("Log Cache: updated.\n\n"); } 
		} else { e=2; if (display) printf("Log Cache: current.\n\n"); } //: %ld",latestHr return -1; 
	}
	
	if (!e) {  fprintf(stderr,">ERROR with creating log cache file! [%s]\n\n",file_logCache); } // exit(1); 
	//--e==1 if success, latestHr== :
	return latestHr;
}

static const long readLogCache(char doo[]) {
	//--read logs date cache file, return last line data OR total found.
	//------filter input using strspn().
	long ret=0; int st=1; short rbytes=96; int ploc=0; //special debug? 3=>5;
	//--max check: [logCache[st].udate > ts]?):-----------:
	time_t ts=c_time;
	//---check fails when: 1.cache is rebuilt with normT, then read.. 2.ts is lt normalized cacheDate!
	//--End of day: 12 midnight.
	int hrofday=0; struct tm *tmStruct; tmStruct = localtime (&c_time);
	if (tmStruct != NULL) {  hrofday=tmStruct->tm_hour+1;
		hrofday=(24-hrofday); ts += hrofday*3600;
	}
	//--open file, parse:---------------------------------------------:
	FILE *fpp;	char rebuildCache=0;
	fpp = fopen (file_logCache, "r"); //b 
	//--Read from file:-----------------------------------------------:
	if (fpp)  {  char str[128]=""; int tmp=0; 
		if (debug>1) printf(">%s Log Cache:--------:[%s]\n", 
			(strcmp(doo,"lastLog")==0?"Checking":"Reading"), file_logCache);
		//----Require and skip first cvs def line..  error if not present.
		if( strncmp(fgets(str,127,fpp),cvsL,strlen(cvsL)-2)!=0 ) { fprintf(stderr,">ERROR with cvs file!\n"); exit(1); }
		memset(&str[0],'\0',sizeof(str));		
		//--get size of file:  //printf(">size of file %d\n",size);
		fseek(fpp,0, SEEK_END);	int size = ftell(fpp); int file_size = size; ploc=size;    
		//--Rebuild?: backup and recreate at end of func: !!!
		if (file_size > MAX_CACHE_SIZE) { rebuildCache=1; }
		fseek(fpp,-rbytes, SEEK_END); //-read from end
		fread(str, rbytes, 1, fpp); int readIts=1; //-:read multiplier
		long preNl=0;
		
		//--Loop profile lines:----------------------------------------------------------:
		do { if (readIts!=-1) readIts++; 
			str[ strlen(str) ] = '\0';  //+1
			int nls = searchStr("\n",str); preNl=0; preNl = strcspn(str,"\n");
			  if (debug>4) { printf(">before nl: %lu , total: %d\nbuffer=[%s]\n\n", preNl,nls,str); }//--
			  if (!nls) { preNl=rbytes; continue; }
			 
			char * last_nl = strrchr(str, '\n'); //-if no nl??
			char * last_line = last_nl+1; //
			int next=0; int nlx=0;
			
			while (nls>0) { last_line--;  int resulTmp=0; 
				if (*last_line=='\n' && next!=1) {  last_line++; tmp++; //tmp used for file line numbering..
					if (*last_line=='\n') {next=1; nls--; last_line--; continue;} //printf(">>...EMPTY LINE!! %d\n",tmp);  
					char lineBuff[128]=""; strncpy(lineBuff,last_line,127); 
					//--Trim end-of-line:   +fufeature: last_line too?
					size_t i = strcspn(lineBuff,"\n"); if (i && i<strlen(lineBuff)){ lineBuff[i]='\0'; }
					//else if (!i) {  printf(">>NO newline!! %zu \n",i); preNl=strlen(lineBuff); next=1;  continue; }
					 //printf("\n>>lineBuff %d: [%s] %zu\n",tmp,lineBuff,i); //:last_line [%s]:\n>>,last_line
					 
					trimP(lineBuff); if (!lineBuff[0]) { next=1; nls--; continue; } //printf(">>...empty line\n"); 
					//--Skip/trim comments, or end at bookmark:
					if (strncmp(lineBuff,"#!beginLogs",11)==0) { readIts=0; break; } //bookmark. 
					else if (strncmp(lineBuff,"#",1)==0 || strncmp(lineBuff,"//",2)==0) { last_line = last_line-2; 
						next=1; nls--; continue; } 
					//--Trim end-of-line comments:   
					i = strcspn(lineBuff,"#"); if (i && i<strlen(lineBuff)){ lineBuff[i]='\0'; last_line[0]='\0';} 
					
					//--Data:-------------------------------------------------------------:
					//--first value: logmeter hours---------------------------------------:
					char* token = strtok(lineBuff,","); 
					//--Trim whitespace, Error if not valid: ||token==NULL  ||strncmp(token,"\n",1)==0
					if (token) {trimP(token);} if (!token) {next=1; nls--; continue;} 
					
					if (strspn(token,"1234567890")!=strlen(token)) { //-- || strtol(token,NULL,10)==0 !!!!!works???
						printf(">ERROR: log cache hourmeter [%s] unspported.\n\n",token); 
						next=1; nls--; continue; } 
					//--Convert hourmeter string into number:-----------------------: resulTmp=1;
					else { logCache[st].hrmtr = strtol(token,NULL,10); }
					//--check hourmeter
					if (logCache[st].hrmtr < 0 || logCache[st].hrmtr > now) { //return ret; } stderr,
						printf(">ERROR: log cache hourmeter [%s] unspported.\n\n",token); 
						next=1; nls--; continue;} 
					
					//--data Value:----------------------------------------------:
					token = strtok(NULL, ",\n"); 
					//--trim whitespace,etc... && ! isspace((unsigned int)token[0])
					if (token) { trimP(token); resulTmp=strlen(token); }
					else { resulTmp = 0; }  						
					 //printf(">resulTmp: %d [%s]\n",resulTmp,token);
					if (!resulTmp) { 
						fprintf(stderr,">ERROR no date for hourmeter [%d]. line %d\n", (int)logCache[st].hrmtr, tmp); 
						next=1; nls--; continue; } //--skip if nothing, warn or error.
					
					//---Parse values:------------------------:
					logCache[st].nHrs=0;
					//--filter/check type of input: unix OR date string?------:
					if (strspn(token,"1234567890")==resulTmp) { //--timestamp:-------------------------------:
						logCache[st].udate = (time_t) strtol(token,NULL,10); 
						//--check/filter unix date number:
						if (logCache[st].udate <= 0 || logCache[st].udate > ts) { 
							fprintf(stderr,">ERROR: log date cache, line %d unspported. %ld\n",tmp,logCache[st].udate); logCache[st].udate=0; next=1; nls--;  continue; }
						//--date string:---------------------:	
						  if (debug>3) printf(">line %d found cache date [ %ld ],\n",tmp, logCache[st].udate); //
						//--normalize cached date to ...def[6pm==21600]:-----------------: 
						//---logCache[cindex].nHrs = time added to cached Date to normalize
						//...iow:  cached date time was before noon if (logCache[cindex].nHrs)...
						struct tm *tmStruct; tmStruct = localtime (&logCache[st].udate);  
						if (tmStruct != NULL) { //-:?when to use when not? here is easiest but messiest: 
							if ((tmStruct->tm_hour+1)<13) { char hrx = 24-(tmStruct->tm_hour+1); 
								logCache[st].nHrs = hrx-nH;  //saved. nH==normalization hours.
								logCache[st].udate = logCache[st].udate+(logCache[st].nHrs*3600); 
								//--hrmtr needs to reflect change: (internally, printfs print true..)
								logCache[st].hrmtr = logCache[st].hrmtr + logCache[st].nHrs;
						}	}
					} 
					else if (strspn(token,"1234567890-")==resulTmp) { //--timestamp from string:----------:
						strncpy(logCache[st].date_s, token, 127); logCache[st].date_s[127]='\0'; 
						  if (debug>3) printf(">line %d found cache date string [ %s ]\n",tmp, token); 
						struct tm logD = {0};  logD.tm_isdst = -1; int ret=0;
						//--configured format:
						if (log_cache_month_position==1) {
							ret = sscanf(logCache[st].date_s, "%d-%d-%d", &logD.tm_mon, &logD.tm_mday, &logD.tm_year); }
						else if (log_cache_month_position==2) { 
							ret = sscanf(logCache[st].date_s, "%d-%d-%d", &logD.tm_mday, &logD.tm_mon, &logD.tm_year); }
						else { ret = sscanf(logCache[st].date_s, "%d-%d-%d", &logD.tm_year, &logD.tm_mday, &logD.tm_mon); }
						if (!ret) { fprintf(stderr,">ERROR: log cache date, length [%d] unspported.\n",resulTmp); 
								next=1; nls--; continue; } //[%s],logCache[st].date_s 
						//--into timestamp: assume hrmtr is from evening... ------------:
						logD.tm_mon -= 1; logD.tm_year -= 1900; logD.tm_hour = 24-nH; //4/6/8?pm (normalized hr)
						logCache[st].udate = mktime(&logD);
						//--nomalizing hrmtr not possible.
							 
						//--check/filter unix date number:
						if (logCache[st].udate < 0 || logCache[st].udate > ts) { 
							fprintf(stderr,">ERROR: log cache date, line %d unspported.\n",tmp); 
							logCache[st].udate=0; next=1; nls--; continue; }
						//--debug:ctime(&logCache[st].udate)
						if (debug>3) printf(">%d date numbers. month=%d day=%d year=%d\n",
							ret,logD.tm_mon,logD.tm_mday,logD.tm_year);	
					 } 
					 else { fprintf(stderr,">ERROR: no valid log cache date.\n"); } //,resulTmp
					
					//--its good, save date string, and save line for rebuild: 
					if (logCache[st].udate) { nlx++;	
						//--
						if (!logCache[st].nHrs) logCache[st].nHrs=0;
						//--is .date_s even used?????
						strftime(logCache[st].date_s,(size_t) 127, date_format, localtime(&logCache[st].udate));
						if (debug>3) { printf(">>[%ld]\ttimeStamp : %ld , Date: %s\n", logCache[st].hrmtr-logCache[st].nHrs,  logCache[st].udate, logCache[st].date_s); } 
						if (rebuildCache) { strncpy(logCache[st].line,lineBuff,(size_t)256);  } 
					}
					
					//--Break when logCache is full: printf("%d>>>%ld\n",st,logCache[st].udate); 
					if (st==1 && strcmp(doo,"lastLog")==0) { readIts=0; break; } //deprecate!?!
					if (st>257) { readIts=0; break; }
					
					nls--; st++; 
					next=1; continue; 
				}  //:end if(last_line=='\n'...)
				//else if (next!=1) {  //-fufeature: search/skip-to next last nl...
					//last_line = strstr(last_line, "\n")+1; //printf(">found \n");
				//}
				next=0; 
			} //:--end  while(nls) [newlines found in str (last_line)]---------------
			//--Exit or read next block of file:-------------------------------------:
			//--Break if beggining or wrapped-around-end of file:
			if (readIts<1){ if (debug>3){printf(">Found beginning of file or max reads.\n\n");} readIts=0; }
			else { 
				int readTo = (rbytes*readIts);	//-
				//Step backwards: including excess from last iteration..
				if (readTo>file_size) { fseek(fpp, 0, SEEK_SET); rbytes = (readTo-file_size); readIts=-1; } //!+preNl
				else { fseek(fpp, (-readTo), SEEK_END);  } //rbytes += preNl;
				size = ftell(fpp); //printf(">location of read %d\n",size);
				if (debug>3){printf("\n>...seeking to %d*%d+%ld...%d<=%d\n",rbytes,readIts,preNl,size,ploc);}
				ploc = size; 
				//-------error:skips reading begining of file if rbytes is big!! solution:?
				//if (size>ploc){ if (debug>3) printf(">Found beginning or end of file or max reads.\n\n"); 
				//		readIts=0; break; }
			} 
			memset(&str[0],'\0',sizeof(str)); //--:clear
		} while (fread(str, rbytes+preNl, 1, fpp) && readIts!=0);  // fgets(str,127,fpp)//( != EOF);
		//--Done:_________________________________________________________________________________
		//--logCache[0] is for cache metadata:
		//logCache[0].xt=st;  //:--real number of cache dates! starting at 1--:
		
		//--return sucess: "lastLog" ["latest"] ONLY used in writeLogCache()//deprecate!?! , 
		if (st && strcmp(doo,"lastLog")==0) {  if (!logCache[0].xt){logCache[0].xt=st;} ret=logCache[1].hrmtr; }
		else { logCache[0].xt=st-1;   ret = logCache[0].xt; };  //ret can be deprecated!? return success bool?latest?
		
		/*/--Rebuild logCache file if too big:--------------------------------: */
		if (rebuildCache && strcmp(doo,"nobuild")!=0) { //--backup old file_logCache:
			readLogCache("nobuild");
			char ftmp[sizeof(file_logCache)+24]; snprintf(ftmp,sizeof(ftmp),"%s.%ld.bak.txt",file_logCache,c_time);
			if (rename(file_logCache, ftmp)!=0) { 
				fprintf(stderr,">ERROR: log cache is too big. unable to remake file [%s].\n",file_logCache);  }
			//--Create new logCache & fill w/ current data:------: 
			writeLogCache(3);
			for (int x=logCache[0].xt; x>0; x--) { snprintf(str,(size_t)128,"%ld",logCache[x].hrmtr); 
				writeFile("logCache", logCache[x].udate, str);
				//writeFile("logCache",0,logCache[x].line);
			} if (!sil) printf(">Rebuilt logCache: %s\n",file_logCache); //? stderr?
		}	
				
	} else { fprintf(stderr,">Error opening log cache! [%s]\n",file_logCache); exit(1); } 
	fclose (fpp);
	
	return ret;
}

static void
searchLogCache(long hrmeter, time_t *searchDate, int *cachedex) {
	//--return closest logCache item, and array index marker:
	//----cindex: 0=gt last, 1(plus)=within 48hrs, 2(minus)=gt cindex & lt next, 3(total)=lt oldest
	int x; char success=0; //int i=0; 
	int total = logCache[0].xt; //-:metadata index 
	if (debug) {
		if (*searchDate!=0) { printf(">>Searching searchDate: %ld\n",*searchDate); }
		else { printf(">>Searching hourmeter: %ld\n",hrmeter);  } }
	
	for (x=1;x<=total;x++) { // ) 1 day: 86400, 8hrs=28800, 12hrs=43200, 16hrs=64800 , 20hrs=72000
		//---Match within ?? hrs:----searchDate ALWAYS needs to be normalized >12 for accuracy!
		//-------cache is normalized to 6pm! *searchDate/hrmtr< (logCache[x]...+43000) can be reduced? 
		
		if (*searchDate!=0) { //--:--searchDate used for setting search: (mostly)
			if (*searchDate>(logCache[x].udate-43200) && *searchDate<(logCache[x].udate+43000)) { success=1; break; }
			else if ((x+1)<=total && *searchDate<logCache[x].udate && *searchDate>(logCache[x+1].udate+43200)) {
				success=2; x++; break; } //--:lt logCache, gt previous logCache.
			else if (*searchDate<logCache[x].udate && x==total) { success=3;  break; } //:before cache
		} else { //--:Searching Hourmeter used for iterating logs: 
			if ((hrmeter+12)>=logCache[x].hrmtr && (hrmeter-12)<logCache[x].hrmtr) { success=1; break; }
			else if ((x+1)<=total && hrmeter<logCache[x].hrmtr && hrmeter>(logCache[x+1].hrmtr+12)){ 
				success=2; x++; break; } //--:bracket
			else if (hrmeter<logCache[x].hrmtr && x==total) { success=3;  break; } //:before cache
		}
	}   if (debug) printf(">>Success %d cache[%d]: %ld : %s - %ld %s\n\n",success,(success!=2?x:-x),logCache[x].hrmtr-logCache[x].nHrs,logCache[x].date_s,logCache[x].udate, (logCache[x].nHrs?"(normalized)":"") );
	//--pointers: -logCache[x].nHrs
	if (success==3) { *searchDate = logCache[x].udate; *cachedex = total+1;  }  	//--3:before oldest cache date ()
	else if (success==2) { *searchDate = logCache[x].udate; *cachedex = -x;}		//--2:gt cache date, lt next
	else if (success) {	*searchDate = logCache[x].udate; *cachedex = x; } 			//--1:eq cache date
	else { *searchDate = 0; *cachedex = 0; }										//--0: gt cache date
}

static short writeFile (char action[], time_t date, char content[]) { 
	//ts is global now. logCache rebuild uses non-c_time ts!
	//--returns 1 on success.
	const char *outtxt; 
	char bs[75]; snprintf (bs, sizeof(bs), "%ld", date); 
	//--File names: ------- 
	if (strcmp(action, "json")==0 || strcmp(action, "jsonc")==0) {
		outtxt = appendStr(3,file_out,bs,(cvs?".cvs":".json")); } //
	else if (strcmp(action, "polling")==0 || strcmp(action, "pollingc")==0) {
		outtxt = appendStr(3,file_poll,bs,(cvs?".cvs":".json")); } //
	else if (strcmp(action, "logs")==0 || strcmp(action, "logsc")==0) {   
		outtxt = appendStr(3,file_logs,bs,(cvs?".cvs":".json")); }
	else if (strcmp(action,"logCache")==0 || strcmp(action, "logCachec")==0) {  
		outtxt = file_logCache;
		//--add hourmeter and timestamp to cache:
		if (strcmp(action,"logCache")==0) { snprintf(bs,(size_t) 75, "%s, %ld,\n",content,date); strcpy(content,bs); }
	}//
	//--return if debug >= 3 :---------------------------: 
	if (debug>2) { printf("\n...Debug prevented writing to file: [ %s ]\n\n", outtxt);  return 1;  }	 //-to stdout???
	
	//--Create file or open for appending:---------------:
	FILE *fp;		
	if (strcmp(action, "jsonc")==0 || strcmp(action, "logsc")==0 || strcmp(action, "logCachec")==0 || strcmp(action, "pollingc")==0) { 
		fp = fopen (outtxt, "wb"); //} else { fp=NULL; }
		if (!sil) printf("\n...Creating new file: [ %s ]\n\n", outtxt);   //-created file
	}
	else { fp = fopen (outtxt, "ab"); } //-append instead of wb 
	if (fp) { //--:print file content:
		fprintf(fp, "%s", content); 
		fclose (fp);
		return 1;
	}
	fprintf(stderr,">!Error writing to file! [%s]\n\n",outtxt); exit(EXIT_FAILURE);  //return 0; //
}

//--Quick polling option:
void pollingOut(modbus_t *ctxx, int num, RamObj *ram0, char* string) {
	FILE *ffp; //char buf[50]; 
	//--Update Specific datas:
	int polls[11] = {0x0021,0x0017,0x0012,0x0013,0x0024,0x0010,0x004B,0x0011,0x0049,0x0043,0x004A};
	upRegisters(ctxx, ram0, num, polls, 11);	//--fill specific Data:
	//--Output options:
	if (json) {  char bufs [256] = "";  //--create file w/base info:
		snprintf(bufs, sizeof(bufs), "{\"action\":\"polling\",\"date\":\"%s\",\"udate\":%ld,\"hrmeter\":%ld", 
																escapeStr("json",string), c_time, now); 
		writeFile("pollingc", c_time, bufs); 
		//-create a minimal PaRam struct for jsonOut:
		struct PaRam pollSt [11]; for (int e=0; e<11; e++) { pollSt[e] = ram0[getIndex(polls[e],num,ram0)]; }
		jsonOut("polling", 11,pollSt, 0,NULL);  
		writeFile("polling", c_time, "}"); //:close file
			exit(EXIT_SUCCESS); //if (){ } else exit(EXIT_FAILURE);  
	} //else if (cvs) */
	ffp = stdout; 
	
	//---Polling Output:-------------------------------------:
	fprintf(ffp,"\n						Charger State:	%s \n", ram0[getIndex(0x0021,num,ram0)].value.sv);
	fprintf(ffp,"__Current Volts & Amps__________________________________________________________\n\n"); 
	fprintf(ffp,"Battery:		%f v		Solar Array: 	%f v\n", ram0[getIndex(0x0017,num,ram0)].value.fv, ram0[getIndex(0x0013,num,ram0)].value.fv);
	fprintf(ffp,"Battery Target V:	%f v		Solar Amps:	%f amps\n", ram0[getIndex(0x0024,num,ram0)].value.fv, ram0[getIndex(0x0011,num,ram0)].value.fv);
	fprintf(ffp,"@ charger v:		%.3f v		Charging Amps:	%f amps\n", ram0[getIndex(0x0012,num,ram0)].value.fv, ram0[getIndex(0x0010,num,ram0)].value.fv);
	fprintf(ffp,"Absorb time:		%.2f mins		Charge (day):	%.2f Ah\n", ram0[getIndex(0x0049,num,ram0)].value.dv/60.0, ram0[getIndex(0x0043,num,ram0)].value.fv);
	fprintf(ffp,"Float time:		%.2f mins		EQ time:	%.2f mins\n", ram0[getIndex(0x004B,num,ram0)].value.dv/60.0, ram0[getIndex(0x004A,num,ram0)].value.dv/60.0);
	//"Slow: [60s %.3fv], [25s: %.3fv]",ram0[getIndex(0x0023,num,ram0)].fv,
	//--
	fflush(ffp);  //--goto program_end; label after calling this function.
}
//--
static void jsonOut(char action[], int cr, RamObj *ram0, int ce, RamObj *eprom0) {
	//--Format and escape data for json,cvs:-----------------------------------------: 
	char jsontxt[655]; //!!!!!!!!!big enough to hold .string, .sv, and all..!!!
	//char *jsontxt=""; 	
	char f_act[8]; char buf[255];  //-string values need large buffer
	//--Date & program action (reading, reverting defaults, new updates, ...):
	char *ctime_s = ctime(&c_time);	ctime_s[ strlen(ctime_s) -1 ] = '\0';
	
	//--Appending to File:------: 
	if (strcmp(action,"logs")==0 || strcmp(action,"polling")==0) { strcpy(f_act,action);  } 
	//--Create Settings File:---:  FILE_OUT_unixdate.json  (different date formats?)
	else {	writeFile ("jsonc", c_time, ""); strcpy(f_act,"json");	
		//--add profile: \"profile\":\"%s\"  ,profile)
		snprintf(jsontxt, sizeof(jsontxt), "{\"date\":\"%s\",\"udate\":%ld,\"hrmeter\":%ld,\"action\":\"%s\"%s", ctime_s, 
				c_time, now, action, ((profile[0] && update)?appendStr(3,",\"profile\":\"",profile,"\""):"")); 
		writeFile (f_act, c_time, jsontxt);
	} //exit 0; 
	
	//--Iterate all data:----------------------------------------------------------:
	for (int ii=0; ii<cr; ii++) { //-just buffering value: full line below!!!!
		jsontxt[0] = '\0'; //memset(&jsontxt[0],'\0',sizeof(jsontxt)); //--not needed if using snprintf..
		/* ---ram, logs---  */
		if (ram0[ii].typ > 0) {//this check doesnt actually tell if ram0[ii] exists!!
			if (ram0[ii].typ == 'd') {
				snprintf(buf, sizeof(buf), "\"value\":%d", ram0[ii].value.dv);
			} else if (ram0[ii].typ == 'l') { //
				snprintf(buf, sizeof(buf), "\"value\":\"%ld\"", ram0[ii].value.lv);
			} else if (ram0[ii].typ == 's') { //
				snprintf(buf, sizeof(buf), "\"value\":\"%s\"", escapeStr("json",ram0[ii].value.sv));
			} else if (ram0[ii].typ == 'f') {  
				if (raw) snprintf(buf, sizeof(buf), "\"value\":%d", ram0[ii].basev); //raw 
				else snprintf(buf, sizeof(buf), "\"value\":%f", ram0[ii].value.fv);
			} else { snprintf(buf, sizeof(buf), "\"value\":%d", ram0[ii].value.dv); }
			//--Output ---------------------------------------------------------------:
			if (strcmp(action,"logs")==0) { //--array of data:   
				snprintf(jsontxt, sizeof(jsontxt), 
					"%s{\"key\":%d,\"hexkey\":\"0x%04X\",\"unit\":\"%s\",\"string\":\"%s\",%s,\"type\":\"%c\"}",
					(ii>0?",":""),ram0[ii].hexa,ram0[ii].hexa, ram0[ii].unit, escapeStr("json",ram0[ii].string), buf, ram0[ii].typ );
			} else { //--keyed data: --polling, settings--  skip combine HIs?
				snprintf(jsontxt, sizeof(jsontxt), ",\"0x%04X\":{\"unit\":\"%s\",\"string\":\"%s\",%s,\"type\":\"%c\"}",
					ram0[ii].hexa, ram0[ii].unit, escapeStr("json",ram0[ii].string), buf, ram0[ii].typ );
			}
			//--Write data:
			writeFile (f_act, c_time, jsontxt);
	} 	}
	//--Return early for logs, polling:---------------------------------------:
	if (strcmp(action,"logs")==0 || strcmp(action,"polling")==0) { return; } //writeFile (f_act, c_time, "}"); 
	
	//--Add second data array (EEPROM):---------------------------------------:
	for (int ii=0; ii<ce; ii++) {
		/* ---eeprom---  */
		if (eprom0[ii].typ > 0) { 
			if (eprom0[ii].typ=='d') { 
				snprintf(buf, sizeof(buf), "\"value\":%d", eprom0[ii].value.dv);
			} else if (eprom0[ii].typ == 'l') { //
				snprintf(buf, sizeof(buf), "\"value\":\"%ld\"", eprom0[ii].value.lv);
			} else if (eprom0[ii].typ=='s') { //STRINGS NEED ESCAPING!
				snprintf(buf, sizeof(buf), "\"value\":\"%s\"", escapeStr("json",eprom0[ii].value.sv));
			} else if (eprom0[ii].typ=='f') { 
				if (raw) snprintf(buf, sizeof(buf), "\"value\":%d", eprom0[ii].basev); //raw
				else snprintf(buf, sizeof(buf), "\"value\":%f", eprom0[ii].value.fv);
			} else { snprintf(buf, sizeof(buf), "\"value\":%d", eprom0[ii].value.dv); }
			//--Write json: --settings--
			snprintf(jsontxt, sizeof(jsontxt), 
				",\"0x%04X\":{\"unit\":\"%s\",\"string\":\"%s\",%s,\"updated\":%d,\"type\":\"%c\"}",
				eprom0[ii].hexa, eprom0[ii].unit, escapeStr("json",eprom0[ii].string), buf,
				eprom0[ii].updated, eprom0[ii].typ);
			//-
			writeFile (f_act, c_time, jsontxt); 
	}	}
	//--close json data: --settings--
	writeFile (f_act, c_time, "}"); 	
}

static short createProfile(char doo[], int nume, RamObj *eprom0, char ctime_s[]) {
	//--Output: 0,1,2==save values, >3=>stdout, raw? float16:float;
	//---doo==null if debug && !new : prints to stdout.
	FILE *fp; char file[128]=""; char bu[64]; char buf[255]; short foo=0;
	//--Date: passing ctime_s instead of recreating here...
	
	//--Open Output:-------------------------------------------:
	if (doo[0] && debug<3) { 
		//--File name: doo=="new" OR  profileBackups_ts ----: "current_settings" is global 
		if (strcmp(doo,"new")==0) {  foo=1; strncpy(bu,doo,(size_t)64); } //profile=="new" 
		else { 	snprintf(bu, (size_t)64, "%s_%ld", doo, c_time); } 		//profile==profileBackups_ts_
		snprintf(file, sizeof(file), "%s", appendStr(4, profileDir, bu, profileTag, ".txt")); //fileOut, ??
		//--Open file for writing: ...use: writeFile("doo", date, text); !!!!!!!!!!!!!?
		fp = fopen (file, "wb");    //ab
		if (!fp) { fprintf(stderr,">Error creating requested profile! [%s]\n",file); exit(1); }
		setvbuf(fp, buf, _IOFBF, sizeof buf); // BUFSIZ
	} else { fp = stdout; foo=3; } //--:stdout  

	fprintf(fp,"Register,new value,description,\n"
		"#--first line MUST be exactly as above! (last comma opt.)\n"
		"//Ms ProStar MPPT Charger Settings:------------:\n");
	if (foo==1) fprintf(fp,"#-Logical Addr must be as docs specify.\n"
		"#---trailing zeros on voltages are optional.\n"
		"#----descriptions are optional and [willbe appended?] ...\n"
		"//line comments can start with either // or #\n"
		"//--endofline comments must use #\n"
		"#uncomment any lines to be updated...\n");
	else fprintf(fp,"//%s\n",ctime_s);
	fflush(fp);
	for (int ii=0; ii < nume; ii++) { //print each line:--------------------------%hu
		if (eprom0[ii].hexa > 0xE038) continue; //--skip last read only registers
		strncpy(eprom0[ii].string, strReplace(","," ",eprom0[ii].string), 255); //-escape string
		char str[16]; //--Values to strings, clean for output...
		
		//--? Which eeprom value is being used? [current==.basev or builtin==(.f_def)] //(strcmp(profile,"new")==0) 
		uint16_t tmp = eprom0[ii].basev; //just use .basev for both!
		//--convert to output string:
		if (strcmp(eprom0[ii].calc,"f16") == 0 && !raw) {  snprintf(str,(size_t)16,"%f,",F16ConvertToF32(tmp)); 
			strcpy(str,strReplace(".000000,",".00,\t",str)); //++trim trailing 0's if all 0's!!
			strcpy(str,strReplace("nan,","nan, \t",str)); 
		} else {  snprintf(str,(size_t)16,"%hu,\t",tmp); } 
		//--Print Profile Template: str==value   
		fprintf(fp,"#0x%04X, %s \t[%s] %s\n", (int) eprom0[ii].hexa, str, eprom0[ii].unit, eprom0[ii].string); 
	}
	fflush(fp); setvbuf(fp, NULL, _IOFBF, 0);
	if (foo<3){ fclose(fp); printf(">Created settings %s profile [ %s ].\n", (foo==1?"":"backup"),file); 
	} else { printf("\n"); }
	return 0;
}

const char* chkProfile(char doo, char* profileName) { 
	//--Parse txt || cvs profile in:  		!escapeStr() before function!!!!
	const char *prof;
	if (doo=='p') { //if (strcmp(doo,"p")==0) { 
		prof = appendStr(4, profileDir,escapeStr("file",profileName),profileTag,".txt"); }  //!!!!!!!!!!!filtering!
	//else if (strcmp(doo,"s")==0) { prof = profileName;  } //-:settings file parsing
	//static char prof[125] = ""; prof[0] = '\0'; ///---appendStr is const char*
	//strncpy(prof, appendStr(3, profileName, profileTag,".txt"), sizeof(prof));  //--:either works:
	
	//--open file, parse:---------------------------------------------:
	FILE *fpp;	
	//--Read from file:
	fpp = fopen (prof, "r"); //b 
	if (fpp)  {
		char str[128]=""; int tmp=0; int st=0;
		//----Require and skip first cvs def line..  error if not present.
		if( strncmp(fgets(str,127,fpp),cvsH,strlen(cvsH)-2)!=0 ) { fprintf(stderr,">ERROR with settings file!\n"); exit(1);  }
		//--Loop profile lines:----------------------------------------------------------:
		do {
			int result=0; int eeprom;
			//--Skip first-cvs-line && comment lines: && blanks!
			trimP(str);
			if (tmp==0 || strncmp(str,"#",1)==0 || strncmp(str,"//",2)==0 || !str[0]) { tmp++; continue; }
			//--Trim end-of-line comments:
			size_t i = strcspn(str,"#"); if (i && i<strlen(str)){ str[i]='\n'; } //'\0'; !!! printf("%lu!",i);
			
			//--first value: EEPROM register PDU---------------------------------------:
			char* token = strtok(str,",");
			//--Trim whitespace, Error if not valid:
			if (token) trimP(token);
			//--//--FIRST value [Profile] check -- EEPROM register PDU:----------------:
				if (strncmp(token,"0xE",3)!=0 && strncmp(token,"57",2)!=0) { 
						   fprintf(stderr,">ERROR with settings file line %d! [ %s ] is not valid!\n", tmp,token); exit(1); }
				//--Convert register address value:........ 	_minus one for (documented logical) rl_!
				else if (strncmp(token,"57",2)==0) { result=1; eeprom = atoi(token); eeprom--; }
				//--Convert hexadecimal string into number:-----------------------:
				else { result=1; eeprom = strtol(token,NULL,16); } //--convert hexadecimal!!!!!!!!!!!!!!
				//--check register key again:
				if (eeprom >= 57344 && eeprom < 57401) {  //<57408
						if (debug>3 && result) printf("\n0x%04X==[%d]  ", eeprom,eeprom); 
				} else { fprintf(stderr,">ERROR with settings file! [ %s ] is not valid register value!\n", token); exit(1); }
			//--//--FIRST token [Settings] check:--  :-------------------------------:
			//...
					
			//--Register data values:-----------------------------------:  
			//---after above key, only 2: first is new value, second (optional) description-notes.
			//--do each individually: //while (token != NULL) {  
			//--//--SECOND -----------------------------------------------:
			token = strtok(NULL, ",\n"); 	//--Profile register data Value:
			//--trim whitespace,etc... && ! isspace((unsigned int)token[0])
			if (token) { trimP(token); result=strlen(token); }
			else {result = 0; }  if (!result) { fprintf(stderr,">ERROR no value for register [0x%04X]\n",eeprom); 
				continue; } //--skip if nothing, warn or error. 
			//--Profile: Assign eeprom values:
				profileIn[st].hexa = eeprom; 
				strncpy(profileIn[st].sv, token, (size_t)16); //-:save value (...as string for now...)!!!!!
				if (debug>3) printf("->[%s]\t", token); //
			//--//--THIRD 		------------------------------------------:
			token = strtok(NULL, ",\n"); 
			//--trim whitespace,etc... && ! isspace((unsigned int)token[0])
			if (token) { trimP(token); result=strlen(token); }
			else {result = 0;}   
			//--Profile: Add new register description/notes from profile:
				//---User description string:-----------:
				if (result) { strncpy(profileIn[st].string, escapeStr("str",token), (size_t)101); //!filtering!!
					if (debug>3) printf("->[%s]", token); //--DEBUG!
				}
			st++; tmp++; //tmp not really used much...
		} while (fgets(str,127,fpp));  //(result != EOF);
	} else { fprintf(stderr,">Error opening requested profile! [%s]\n",prof); exit(1); } 
	fclose (fpp);
	
	//--return sucess:
	return profileName; //prof//profileName
}

static char parseProfileValue(char action[], int pi, int ep, RamObj *eprom) {
	//--Globals: profileIn[pi]; 	Not Global (yet): eprom[ep]; 
	//---if (update) eprom[ep].f_new (uint16_t) == profileIn[pi].value.fv (default) || eprom[ep].f_def (if builtins?)
	//---else printing profile: eprom[ep].basev == EEPROM .basev || eprom[ep].f_def
	char skip = 0; char non = -1;
	
	//--warn/error on overflow:
	if (profileIn[pi].value.fv > 65535) { //skip=1;
		fprintf(stderr," 0x%04X [ %s ] !WARNING! new value is not supported!\n",profileIn[pi].hexa,profileIn[pi].sv);
		return non; //!
	}
	//---Convert Update Values: -------------------------------------------------:
	if (strcmp(eprom[ep].calc,"f16")==0) { //--CONVERT TO F16! (volts,amps,temp-comp)
		//---f16:--: labled & very high number: must be f16.
		if (profileIn[pi].value.fv > 400) {  //-f16 input
			profileIn[pi].basev = (uint16_t) profileIn[pi].value.fv; //!
			profileIn[pi].value.fv = F16ConvertToF32(profileIn[pi].basev);
		} //--zero:
		else if (profileIn[pi].value.fv==0) { eprom[ep].f_new = 0; } 
		else  { //---float:--: needs converting:  
			profileIn[pi].basev = fl_to_half(profileIn[pi].value.fv); }
		
		/* !!!!!!!!!!!!!!!!!!!! Voltage Multiplier !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! unneeded. 
		if (strcmp(eprom[ep].unit,"v")==0 && profileIn[pi].value.fv!=0 && vmultiplier>1) { //-: []
			//--if accepting 24v+ values & vmultiplier: down shift here...	
			profileIn[pi].value.fv = profileIn[pi].value.fv/vmultiplier;
			profileIn[pi].basev = fl_to_half(profileIn[pi].value.fv);
		} //  */
		
		/* /-/--Last F16 Checks, Assign .f_new:-----------------------------:  error on NaN  */
		//-:temp comp - check if in range...???
		if (eprom[ep].hexa == 0xE01A) {
			if (profileIn[pi].value.fv < -1 || profileIn[pi].value.fv > 0) { 
				skip=1; fprintf(stderr," 0x%04X = %s %s !WARNING! new compensation value [ %f ] is not supported!\n", 
					profileIn[pi].hexa,profileIn[pi].sv,eprom[ep].unit,profileIn[pi].value.fv);  }
		/*/--Specials: fraction [+mcPT: load Ohm 0...1, +both: fixed vmp pct 0.0...0.99] [f16 15360==1]  */
		} else if (profileIn[pi].hexa==0xE026 || profileIn[pi].hexa==0xE037) {
			if (profileIn[pi].value.fv!=0 && (profileIn[pi].value.fv<0 || profileIn[pi].value.fv>0.99)) {  
				skip=1;	fprintf(stderr," 0x%04X = [ %s ] [%hu] !WARNING! new value is unsupported!\n", 
								profileIn[pi].hexa,profileIn[pi].sv,profileIn[pi].basev);  }
		//--Voltage safety check?--  [18700==10.09]    //ranges: 10...30 or 0 (multiply by battery multiplier?)
		} else if ( strcmp(eprom[ep].unit,"v")==0 //&& profileIn[pi].value.fv != 0	//--:min & max check   != 0xE01A
				&& ((minV && profileIn[pi].value.fv < minV) || (maxV && profileIn[pi].value.fv > maxV)) ) { 
			//--Maxmax: HVD:  			,... 		???????!!!!!!!!!!!!!!!!!!hack!
			if (eprom[ep].hexa == 0xE01B && (maxV && profileIn[pi].value.fv < (maxV+1))) { skip=0; }
			//--:Allowable 0 voltages: -- Turning OFF==0 voltage.   ...or allow all to be zero?
			else if (profileIn[pi].value.fv==0 && (eprom[ep].hexa==0xE001 || eprom[ep].hexa==0xE007 || eprom[ep].hexa==0xE010 || eprom[ep].hexa==0xE01D || eprom[ep].hexa==0xE033 || eprom[ep].hexa==0xE036)) {skip=0;} 
			else { skip=1; 
				fprintf(stderr," 0x%04X = [ %s ] !WARNING! new voltage is very low or very high\n"
									"\t\t voltages must be between %.2f and %.2f !\n", //  [%f! == %hu]
						profileIn[pi].hexa,profileIn[pi].sv,minV,maxV); }//, profileIn[pi].value.fv, profileIn[pi].basev
		//--Scaling/Ranges: celcius ()------------------------------------------:  //NULL is ok???
		} else if ( strcmp(eprom[ep].unit,"C") == 0 && (profileIn[pi].value.fv < -128 || profileIn[pi].value.fv > 127) ) { 
			skip=1; fprintf(stderr," 0x%04X = %s %s !WARNING! new temperature [ %f ] is not supported!\n", 
					profileIn[pi].hexa,profileIn[pi].sv,eprom[ep].unit,profileIn[pi].value.fv); 
		//--Other floats: [15360==1] higher probably?... 
		} else if (profileIn[pi].value.fv!=0 && profileIn[pi].basev < 15360) {
			skip=1;	fprintf(stderr," 0x%04X = [ %s ] [%hu] !WARNING! new value is very low!\n", 
								profileIn[pi].hexa,profileIn[pi].sv,profileIn[pi].basev);
		//--Set update value!:--------------------------------: f_new == uint16_t!!!
		} else if (profileIn[pi].value.fv==0) { skip=1; }  //!-:set above.
		//--
		if (!skip) { eprom[ep].f_new = profileIn[pi].basev;   }
		//* -- NULL F16s? ..some are allowed..   // *
	} 
	
	//--Float input error: stray floats?? ---------------------------------------:
	else if (strcspn(profileIn[pi].sv, ".") < strlen(profileIn[pi].sv)){ skip=1; 
		fprintf(stderr," 0x%04X [ %s ] !WARNING! Unexpected Float Value!\n",profileIn[pi].hexa,profileIn[pi].sv); }
	//--Normal Numbers: (secs,mins,days, ) ------------------------:
	else { 
		if (profileIn[pi].value.fv<0) {
			fprintf(stderr," 0x%04X [ %s ] !WARNING! Unsupported Value!!\n",profileIn[pi].hexa,profileIn[pi].sv); 
			return non; //!!!!!!!!!!!!
		} else { //--Assign update value:----------------------------------------------------: //
			eprom[ep].f_new = (uint16_t) profileIn[pi].value.fv; }
			//profileIn[pi].basev = (uint16_t) profileIn[pi].value.fv;  } //(uint16_t): max==65535
		
		/* /-/----Last nonF16 Filter/s: .f_new holds update value here on.. ------------------- / */ 
		//--Check for null uint16_t: --.calc F16 can have but not these (probably)--
		if (eprom[ep].f_new == 65506) { skip=1;
			fprintf(stderr," 0x%04X = %s %s !WARNING! new value [ %hu ] null is not supported!\n", 
					profileIn[pi].hexa,profileIn[pi].sv,eprom[ep].unit, eprom[ep].f_new); 
		//--Days / time: 
		} else if ( strcmp(eprom[ep].unit,"days")==0 && (eprom[ep].f_new > 255) ) { //profileIn[pi].value.fv  < 0 || 
			skip=1; fprintf(stderr," 0x%04X = %s %s !WARNINGS! new time [ %d ] is not supported!\n", 
						profileIn[pi].hexa,profileIn[pi].sv,eprom[ep].unit,(int)profileIn[pi].value.fv); //%hu
		/* /--time: both checks on profileIn[pi].value.fv done already! (.f_new > 65535 || .fv < 0) skip:
		} else if ((strcmp(eprom[ep].unit,"secs")==0 || strcmp(eprom[ep].unit,"mins")==0) && ) { skip=1; } */
		//--Misc Settings:
		} else if ( eprom[ep].hexa==0xE034 && (profileIn[pi].value.fv  < 1 || eprom[ep].f_new > 247) ) {  //modbus id
			skip=1; fprintf(stderr," 0x%04X = %s %s !WARNINGS! new id [ %d ] is not supported!\n", 
						profileIn[pi].hexa,profileIn[pi].sv,eprom[ep].unit,(int)profileIn[pi].value.fv); //%hu
		} else if ( eprom[ep].hexa==0xE035 && (profileIn[pi].value.fv  < 1 || eprom[ep].f_new > 15) ) { //meterbus id
			skip=1; fprintf(stderr," 0x%04X = %s %s !WARNINGS! new id [ %d ] is not supported!\n", 
						profileIn[pi].hexa,profileIn[pi].sv,eprom[ep].unit,(int)profileIn[pi].value.fv); //%hu
		}
		//--zero out...
		if (skip) eprom[ep].f_new = 0;
	}
	fflush(stderr);
	//--one more check for null F16: if (eprom[ep].f_new == 65506 && eprom[ep].hexa!=)?...
	if (skip) return non;
	else { return 1; }  //eprom[ep].updating = 1; //-:mark in struct
} 

/* #---Main Parse Data Loop: --------------------------------------------------------# */
const RamObj parseValue (RamObj *inStruct, uint16_t dvalue) {  
	//--------------Convert data values to correct output type------------------:const 
	//OUT: back into original struct w/->> correct (converted) value & type assigned
	RamObj outStruct = *inStruct;	//copy?
	//--do any calculations or combines?:   combined:modbus_get_float_abcd();
	outStruct.basev = dvalue;  	//--:save raw value	//dvalue is new value, inStruct holds old value?
	//if (inStruct.updated) {int updated = inStruct.updated;} 
	
	/* ----------------------------Unit Sorting------------------------------------- */
	//-//-Convert-Float:---------------------------------------------------: //0=65535
	if (strcmp(inStruct->calc,"f16") == 0) { outStruct.typ ='f';  
		//--v,amps,C,etc.. *MOST!-----------------: Fudges non-F16 numbers!!!  //%hu - uint16_t
		outStruct.value.fv = F16ConvertToF32(dvalue); 
		if (dvalue == 65535 || !outStruct.value.fv) outStruct.value.fv = 0.0;  //--zero? nan??
		/*/--Scaling modifications: (n*0.1)  ,  
		if (strncmp(outStruct.calc,"comb=n*",7)==0) { //--//0x002A,B? no. !Not used for f16!?!
				outStruct.value.fv = outStruct.value.fv * 0.1; } // */
	}
	//-//-Cast-Ints:-(Raw)-------------------------------------------------:  
	else if ( strcmp(inStruct->unit,days) == 0 || strcmp(inStruct->unit,secs) == 0 || strcmp(inStruct->unit,hours) == 0 || strcmp(inStruct->unit,mins) == 0 || outStruct.hexa == 0xE01E || outStruct.hexa == 0xE01F || outStruct.hexa == 0xE020 || outStruct.hexa == 0xE021) { //
		//*Days,hours,mins,secs,+single #'s
		outStruct.typ ='d';  outStruct.value.dv = dvalue;
	}
	else if (strcmp(inStruct->unit,"lhours") == 0 ) {//--HOURMETERs are NOT dv!!!!!!
		outStruct.value.lv = (inStruct->value.lv)?inStruct->value.lv :(long) dvalue;
		outStruct.typ ='l';  //printf(">LOG!: heres print: %ld\n",outStruct.value.lv);
	}
	//-//-Convert-String:---------------------------------------------:
	else if (strcmp(inStruct->unit,bits) == 0 || strcmp(inStruct->unit,blank) == 0) { 
		//-:Bitfields, States, Combo calcs:-(modbus_get_float_abcd()), -------: //
		outStruct.typ ='s';    char dvString[256] = "";   
		
		//--Parse switches, bitfields:---------------: //++? Ah , kWh , 
		//----Specials: parsed as "state=hexa"
		if (inStruct->group[1] && strncmp(inStruct->group,"state=",6)==0) { int ok=0; 
			sscanf(inStruct->group, "state=0x%04X", &ok); //ram register type
			//if (debug>1) printf(">parsing State! 0x%04X\n",ok);
			strncpy(dvString, getStateString(ok, (inStruct->value.lv? inStruct->value.lv:(long) dvalue)),(size_t) 256); ok=0; }
		/*/------software,hardware models,etc...:  
		else if (inStruct->hexa == 0x000.) {  //:hex as string:
			snprintf(dvString, 255, "0x%04X", dvalue);  } //outStruct.typ ='s'; */
		else {	strncpy(dvString, getStateString(inStruct->hexa, 
									(inStruct->value.lv?inStruct->value.lv:(long) dvalue)),(size_t) 256); }//
		//--Assignment!
		strncpy(outStruct.value.sv, dvString, sizeof(outStruct.value.sv)); 
	}  
	//-//-Other floats & x numbers:
	else {  //--Ram is LO, EEPROM is HI   //not! 0xE04A & 0xE04B!?
		if (strncmp(outStruct.calc,"comb=n*",7)==0 || strncmp(outStruct.calc,"n*",2)==0) { //TriStar has n*0.1 w/o combine
			outStruct.value.fv = (inStruct->value.lv?inStruct->value.lv * 0.1:(int) dvalue * 0.1); outStruct.typ ='f'; } 
		else {	outStruct.value.dv = (int) dvalue; outStruct.typ ='x'; }
		
	}
	//--done:
	return outStruct; 
}

//--Get Specific register/mem structure array index:-------------------:
int getIndex(int hex, size_t total, RamObj *inStruct) {
	//--iterate struct:
	for (short i=0; i < total; i++) {
		if (hex == inStruct[i].hexa) return i; //-
	}
	return -1;
}
int getCoil(int hex, size_t total, struct CoilsObj *inStruct) {
	//--iterate struct:
	for (short i=0; i < total; i++) {
		if (hex == inStruct[i].hexa) return i; //-
	}
	return -1;
}

//--Combine double mem address items:--(previous data must already be parsed):
// ...ram & eeprom ordered oppositely... eprom LO HI , ram HI LO .
static long combine(int hex, uint16_t lo, short num, RamObj *ramIn) { 
/* Combine()s: are ALL uint32 [.value.lv] when combined: other than hourmeter all become getStateStrings strings
 hourmeters=0x0036,0x0037,0x0040,0x0041,+logs   new type for hourmeters=='l'!!!
__signed long should be big enough!__ (even though neg. is not used [except for error]) 
	 2x registers: 	  6553565535 / 24 / 365 = 748,123.92 years
	 signed	int Max:  2147483647 / 24 / 365 = 245,146.53 years
	 signed	long Max: 9223372036854775807 (9.2233720368547758e18) / 24 / 365 = 1052896351239130 years
*/
	long ret; int hi;
	//--Paired mem register:
	int ii = getIndex((hex-1),num,ramIn); if (ii == -1) return -1; //--------------!!!!!!!!!! 
	//--Correct HI LO : (called with 2nd value)-----------:
	char tmp[8] = ""; snprintf(tmp, 8, "0x%04X", hex); //4 --Check hexadecimal
	//--Order paired data correctly:
	if (strncmp(tmp, "0xE", 3) == 0 || strncmp(tmp, "0x8", 3) == 0) { 
			 hi = lo; lo = ramIn[ii].basev;  //printf("Its EEPROM 0x%04X lo is HI!\n", hex);
	} else { hi = ramIn[ii].basev; }	//RAM
	//--Addition needs to be HI << LO !?
	ret = (hi << 16) + lo;
	//--scaling done after
	
	return ret;  //assign to *ramIn here!
}
//--SAFE string handling?:-------------------------: 
static const char* appendStr(int num, ...) {
	//-numbers need to be stringified before passing!
	static char destStr[256] = ""; 
	memset(&destStr[0], '\0', sizeof(destStr)); //remove static?
	//destStr[0]='\0';
	/* Declare a variable of type ‘va_list’. */
	va_list parameters;
	/* Call the ‘va_start’ function. */
	va_start (parameters, num);
	for (short counter = 0; counter < num; counter++) { //
	  /* Get the values of the optional parameters, append to destStr... */
	  const char *sourceStr = va_arg (parameters, char *);  //----: expects string!
	  size_t sn = strlen(sourceStr);
	  if (sn < ( 255 - strlen(destStr))) { //sizeof(destStr) //--!!!!!!!! dblchk for overflow!!!!!!
	  		strncat(destStr, sourceStr, sn);  }
	} /* End use of the ‘parameters’ variable. */
	va_end (parameters); //gcc ignores this...
	/*  // */
	//destStr[ strlen(destStr)+1 ] = '\0';  //?!? 
	return destStr[0]? destStr:"";
}
//--//--seconds to decimal hours----------------: 
const float secsToHours(int num) { 
	float x = (num/60.0)/60.0;   //hrs
	return x;
}
//--//--format strings for text in other funcs, escape here befor output:
static char* escapeStr(const char* doo, char* string) {
	char *nStr = ""; //= string; //?size needs to be gt orig!
	//char nl[], tb[]; //filter: if (strspn(token,"1234567890-")==strlen(token))
	//--escape common:
	//--unnecessary file name filters: 
	if (strcmp(doo, "file")==0) {
		nStr = strReplace("\\","",string);
		nStr = strReplace(":","",nStr);
		//if (strncmp(nStr,"/",1)==0) strReplace("/","",nStr);
		//nStr = strReplace(".","",nStr);  //replace w/regex
		nStr = strReplace("/","",nStr); //replace w/regex
	}
	//--escape specifics: //--regex escape: ["\\] and other js specifics..!
	else if (strcmp(doo, "json")==0 || strcmp(doo, "cvs")==0) {  
		nStr = strReplace("\n","\\n",string);
		nStr = strReplace("\t","\\t",nStr);
		nStr = strReplace("\"","\'",nStr);  //or &quote
	}
	//--escape user [description] strings: (translations)
	else if (strcmp(doo, "str")==0) {  nStr = strReplace("#","",string); }
	//--Search / Replace:
	
	//--return string:----(returned string is not! same size)
	return nStr;
}

//---//--:-----------()
static const int searchStr(const char* search, const char* string) {
	size_t s_l = strlen(search);
	size_t count = 0;
	const char * optr;
	const char * loc;
	//--count occurs
	for (optr = string; (loc = strstr(optr, search)); optr = loc + s_l) { count++;  }
	 //printf(">found %ld occurs\n", count);
	return (int) count;
}

//--//--Search / Replace--:---------------(returned string is not! same size)
static char* strReplace(const char* search, const char *replace, const char* string) {  
	size_t s_l = strlen(search);
	size_t r_l = strlen(replace);
	size_t orig_l = strlen(string);
	size_t count = 0;
	const char * optr;
	const char * loc;
	 //printf(">Replacing %s with %s in string [ %s ]\n", search,replace,string);

	//--count occurs
	for (optr = string; (loc = strstr(optr, search)); optr = loc + s_l) { count++;  }
	 //printf(">found %ld occurs\n", count);
	 
	//--create return:
	size_t rlen = orig_l + count * (r_l - s_l);
	char *ret = malloc( sizeof(char) * (rlen + 1));
	 //printf(">created return string [ %s ]: orig length %ld , new length %ld\n", string,orig_l, rlen);
	if (ret == NULL) { return "__error replacing!__"; }
	char * retp = ret; //-:using *ret to create sizeof final string
	//--
	for (optr = string; (loc = strstr(optr, search)); optr = loc + s_l) {
		size_t skip = loc - optr;
		strncpy(retp, optr, skip); //copy until key occurs
		retp += skip;
		strncpy(retp, replace, r_l);  //insert replacement
		retp += r_l;
	}
	//-copy rest of string
	strcpy(retp, optr); //- */
	 //printf(">New String: [ %s ]\n", ret);
	//--return
	return ret;
}

void trimP (char *str) { 
	//--trim in place: 
	int i; int begin = 0; 
	int end = strlen(str) - 1;
	//--end & beginning:
	while (isspace((unsigned char) str[begin])) begin++;
	while((end >= begin) && isspace((unsigned char) str[end])) end--;
	//--if returning char*: new str== (end - begin)
	//char *ret = malloc( sizeof(char) * (end - begin)); //ret[i-begin]=below
	
	//--shift string in memory:
	for (i=begin; i <= end; i++) { str[i - begin] = str[i]; }
	str[i - begin] = '\0'; //close with null! //i++; 
}

//--//--Iterate thru and print out correct formats to cli (debug,)------------------: 
void printOUT (int ind, char type, RamObj *inStruct) { 
	///--create buffer for inStruct 
	char buf[600]=""; //!!.sv=255!.string=255!!
	char bufValue[19] = "";  //char bufType[3] = ""; 
	//if ((!display && !just_EEPROM) ) return;	//--skip printout function: hack   sil || !debug
	//--Updates:
	short updated = inStruct->updated; 
	//if(update && !debug && !updated){ return; }
	//printf("%s", (updated?"*":"")); //mark updates!? 
	
	//--Logs: 
	char doo[5] = "";  //---little hack to parse logs:
	if (type=='z') { type = inStruct->typ; strcpy(doo,"logs");  }
	
	//--assign current values: //combines==inStruct->lv (used to be: inStruct->dv)
	int dvalue = (int) inStruct->basev; //-:For raw or debug!
	
	//--Format/Print Out line:------------------------------------------------------------------------:
	//--Floats:-----------------------------------------v,amps,C,etc.. *MOST!---------- //%lf ? - uint16_t
	if (strcmp(inStruct->calc,"f16")==0) {  	//else if (type == 'f')  {   }	
		//--format-------------------------: 
		if (raw) snprintf(bufValue, sizeof(bufValue), "%d", dvalue); //raw
		else snprintf(bufValue, sizeof(bufValue), "%f", inStruct->value.fv); 
		//debug:
		snprintf(buf, sizeof(buf), "cf%d:\t", ind);
		//--Current data----------------:  inStruct->value.fv //strcpy(bufType, "cf");
		printf("%s 0x%04X [ %s %s ] %s", (debug>1?buf:""), (int) inStruct->hexa, bufValue, inStruct->unit, inStruct->string);
	}
	//--Ints: *Days,hours,mins,secs,+single #'s----------------------------------------------- //
	else if (type=='d') {    
		//-format-------------------------:
		if (raw) snprintf(bufValue, sizeof(bufValue), "%d", dvalue); //raw
		else snprintf(bufValue, sizeof(bufValue), "%d", inStruct->value.dv);   
		//debug:
		snprintf(buf, sizeof(buf), "rd%d:\t", ind);  //strcpy(bufType, "rd");
		//---print out raw data----------: 
		printf("%s 0x%04X [ %s %s ] %s", (debug>1?buf:""), (int) inStruct->hexa, bufValue, inStruct->unit, inStruct->string);
		
	//--:Bitfields, States, Combo calcs:(modbus_get_float_abcd()), ----------------------------------://
	} else if (type=='s') {   //----------STRINGS NEED ESCAPING!?
		/* //--format-------------------------- */
		//snprintf(bufValue, sizeof(bufValue), "%x", dvalue);  
		//---Print out Bit fields-----------: sv comes after string! else sv comes before!
		if (strcmp(inStruct->unit,bits) == 0) { //*..Same As rawd %hu
				if (strlen(inStruct->value.sv) > 18) {
								snprintf(buf, sizeof(buf), "\n %s", inStruct->value.sv); } //--nl block!
				else { strncpy(buf, inStruct->value.sv, sizeof(buf)); }
				//debug:
				snprintf(bufValue, sizeof(bufValue), "rh%d:\t", ind);
				printf("%s 0x%04X [ %x %s ]  %s %s", (debug>1?bufValue:""), (int) inStruct->hexa, dvalue, 
						inStruct->unit, inStruct->string, buf); // */
						
				/*	snprintf(bufS, sizeof(bufS), "%s \n %s", inStruct->string, inStruct->sv); } //--nl block!
				else { strncat(bufS, inStruct->sv, (strlen(inStruct->sv)-1)); } 
				//strcpy(bufType, "rh"); //buf[0]='\0'; //!?
				printf("%s 0x%04X [ %x %s ]  %s\n", (rh%d:\t), (int) inStruct->hexa, dvalue, 
						inStruct->unit, bufS); // */
		//---Print out State, ...-----------: //!!!Problem!!!
		} else {  //buf=->value.sv; bufValue+=(5,dvalue," ",->unit," ",->value.sv)
			snprintf(bufValue, sizeof(bufValue), "cd%d:\t", ind);	//--:debug  //strcpy(bufType, "cd"); 
			printf("%s 0x%04X [ %d %s %s ]  %s", (debug>1?bufValue:""), (int) inStruct->hexa, dvalue, 
						inStruct->unit, inStruct->value.sv, inStruct->string);  //*%x better?
	}	}
	else { //---print out raw data----------: 	//types= "l", "f", "x"
		//snprintf(bufValue, sizeof(bufValue), "%d", inStruct->basev);
		if (raw) { snprintf(buf,sizeof(buf), "%d", dvalue); }
		else if (type=='f') { snprintf(buf,sizeof(buf), "%f", inStruct->value.fv); }
		else if (type=='l') { snprintf(buf,sizeof(buf), "%ld", (inStruct->value.lv? inStruct->value.lv:(long)dvalue)); }
		else if (type=='x') { snprintf(buf,sizeof(buf), "%d", inStruct->value.dv); }
		else { snprintf(buf,sizeof(buf), "%d", dvalue);  }
		snprintf(bufValue, sizeof(bufValue), "rr%c%d:\t", type,ind);	//--:debug strcpy(bufType, "rr"); 
		printf("%s 0x%04X [ %s %s ] %s", (debug>1?bufValue:""), (int) inStruct->hexa, buf, inStruct->unit, inStruct->string); 		
		/*/old:
		snprintf(bufValue, sizeof(bufValue), "rr%d:\t", ind);	//--:debug strcpy(bufType, "rr"); 
		printf("%s 0x%04X [ %ld %s ] %s", (debug>1?bufValue:""), (int) inStruct->hexa, (inStruct->value.lv? inStruct->value.lv:(long)dvalue), inStruct->unit, inStruct->string); // */
	}
	//--end line:
	if (update && updated) { printf(" (NEW)\n"); }
	else printf("\n");
	
	//--Buffering: Print Out full-line:-----------:
/*	printf("%s%s%d:\t 0x%04X [ %s %s %s ]  %s\n", ((update && updated)?"*":""),bufType, ind, (int) inStruct->hexa, bufValue, inStruct->unit, buf, bufS);  //((update && updated)?" (NEW) ":"")  */
}

uint16_t dipsChargeMode(uint16_t dvalue) {
	//--char return would work too.
	short mode = 0;
	//if (!dvalue) return -1; //-conflicts sealed mode 1
	//--battery charge type:
	if ((dvalue & (1 << 3)) >> 3) { mode = 100;  }	//on
	else { mode = 0;  }								//off
	if ((dvalue & (1 << 4)) >> 4) { mode += 10;  }	//on
	else { mode = 0;  }								//off
	if ((dvalue & (1 << 5)) >> 5) { mode += 1;  }	//on
	else { mode += 0;  }							//off
	
	//translate mode
	if (mode == 111) { mode = 8; } 		//8 - custom
	else if (mode == 110) { mode = 7; } //7 - L-16
	else if (mode == 101) { mode = 6; }	//6 - Flooded
	else if (mode == 100) { mode = 5; }	//5 - Flooded
	else if (mode == 11) { mode = 4; }	//4 - AGM/Flooded
	else if (mode == 10) { mode = 3; }	//3 - sealed
	else if (mode == 1) { mode = 2; }	//2 - sealed
	else { mode = 1; } 					//1 - sealed off-off-off
	//--
	return mode;
}

const char* getStateString(int hex, long dvalue) {
	//--Settings State translations:-------------------------------------------://
	if (dvalue < 0) return "";
	static char retu[254] = "";  //why static??
	memset(&retu[0],'\0',sizeof(retu)); //--zero out static var - //retu[0] = '\0'; 
	//----*using actual nl & tabs, escaped for specific outputs later*!! 
	//char nl[]="\n", tb[]="\t"; 

	//--Charger State:-----------------------------------------------------:
	if (hex == 0x0021) { 
		switch (dvalue) {
			case 0: strcpy(retu, "START"); 
			break;
			case 1: strcpy(retu , "NIGHT_CHECK"); 
			break;
			case 2: strcpy(retu, "DISCONNECT"); 
			break;
			case 3: strcpy(retu, "NIGHT"); 
			break;
			case 4: strcpy(retu, "FAULT"); 
			break;
			case 5: strcpy(retu, "MPPT"); 
			break;
			case 6: strcpy(retu, "ABSORPTION"); 
			break;
			case 7: strcpy(retu, "FLOAT"); 
			break;
			case 8: strcpy(retu, "EQUALIZE"); 
			break;
			case 9: strcpy(retu, "SLAVE"); 
			break;
			case 10: strcpy(retu, "FIXED"); 
			break;			
		}
	}
	else if (hex == 0x0022 || hex == 0x0045) {
		//--Array Fault bitfield-------------------------------------------------:
		long load_fault = dvalue;
		//snprintf(retu, sizeof(retu), "Array self-diagnostic faults:%s", nl);
		//-------------------------------------rm \n and use commas instead????
		strcpy(retu,"Self-diagnostic faults:\n");
		if (load_fault == 0) { 	strcat(retu,"\tNo Faults\n"); } 
		else {
			if (load_fault & 1) { strcat(retu,"\tOvercurrent Phase 1\n");	}
			if ((load_fault & (1 << 1)) >> 1) { strcat(retu,"\tFET(s) Shorted \n");  }
			if ((load_fault & (1 << 2)) >> 2) { strcat(retu,"\tSoftware Bug\n");  }
			if ((load_fault & (1 << 3)) >> 3) { strcat(retu,"\tBattery HVD (disconnect)\n");  }
			if ((load_fault & (1 << 4)) >> 4) { strcat(retu,"\tArray HVD (disconnect)\n");  }
			if ((load_fault & (1 << 5)) >> 5) { strcat(retu,"\tEEPROM setting Edit (reset needed)\n"); }
			if ((load_fault & (1 << 6)) >> 6) { strcat(retu,"\tRTS Shorted\n");	}
			if ((load_fault & (1 << 7)) >> 7) { strcat(retu,"\tRTS was valid, now disconnected\n"); }
			if ((load_fault & (1 << 8)) >> 8) { strcat(retu,"\tLocal temp. sensor failure\n");  }
			if ((load_fault & (1 << 9)) >> 9) { strcat(retu,"\tBattery LVD (disconnect)\n"); }
			if ((load_fault & (1 << 10)) >> 10) { strcat(retu,"\tSlave Control Timeout\n");	}
			if ((load_fault & (1 << 11)) >> 11) { strcat(retu,"\tDIP Switch Changed (excl. DIP 8)\n"); }
		} 
	} //---// load_state----------------------------------------------------------:
	else if (hex == 0x002E) { 
		switch (dvalue) {
			case 0: strcpy(retu, "START"); 
			break;
			case 1: strcpy(retu, "LOAD_ON"); 
			break;
			case 2: strcpy(retu, "LVD_WARNING"); 
			break;
			case 3: strcpy(retu, "LVD"); 
			break;
			case 4: strcpy(retu, "FAULT"); 
			break;
			case 5: strcpy(retu, "DISCONNECT"); 
			break;
			case 6: strcpy(retu, "LOAD_OFF"); 
			break;
			}
	} //--load_fault--------------------------------------------------------------:
	else if (hex == 0x002F || hex == 0x0046) { 
		long load_fault = dvalue;
		//snprintf(retu, sizeof(retu), "Load output faults self-diagnostic:%s", nl);
		strcpy(retu,"Load output faults self-diagnostic:\n");
		if (load_fault == 0) { 	strcat(retu,"\tNo Faults\n"); } 
		else {
			if (load_fault & 1) { strcat(retu,"\tExternal short circuit\n");	}
			if ((load_fault & (1 << 1)) >> 1) { strcat(retu,"\tOvercurrent\n");  }
			if ((load_fault & (1 << 2)) >> 2) { strcat(retu,"\tFETs shorted \n");  }
			if ((load_fault & (1 << 3)) >> 3) { strcat(retu,"\tSoftware bug\n");  }
			if ((load_fault & (1 << 4)) >> 4) { strcat(retu,"\tHVD \n");  }
			if ((load_fault & (1 << 5)) >> 5) { strcat(retu,"\tHeatsink over-temperature \n"); }
			if ((load_fault & (1 << 6)) >> 6) { 	strcat(retu,"\tDIP Switch Changed (excl. DIP 8)\n");	}
			if ((load_fault & (1 << 7)) >> 7) { 	strcat(retu,"\tEEPROM setting edit (reset required) \n"); }
	}	} 
	//--Alarm bitfield HI LOW----------------------------------------------------:
	else if (hex == 0x0039 || hex == 0x0048) { //--hex == 0x0038||
		long alarm = dvalue;
		strcpy(retu,"Controller self-diagnostic Alarms:\n");
		if (alarm == 0) {
			strcat(retu,"\tNo alarms\n");  
		} else {
			if (alarm & 1) strcat(retu,"\tRTS open\n"); 
			if ((alarm & (1 << 1)) >> 1) strcat(retu,"\tRTS shorted\n"); 
			if ((alarm & (1 << 2)) >> 2) strcat(retu,"\tRTS disconnected\n"); 
			if ((alarm & (1 << 3)) >> 3) strcat(retu,"\tThs open\n");
			if ((alarm & (1 << 4)) >> 4) strcat(retu,"\tThs shorted\n");
			if ((alarm & (1 << 5)) >> 5) strcat(retu,"\tHeatsink hot\n");  //active temp limiting
			if ((alarm & (1 << 6)) >> 6) strcat(retu,"\tInductor sensor open\n"); //inductor sensor open
			if ((alarm & (1 << 7)) >> 7) strcat(retu,"\tInductor sensor short\n");	//inductor sensor short
			if ((alarm & (1 << 8)) >> 8) strcat(retu,"\tInductor hot - limiting\n");	//inductor hot! (limiting)
			if ((alarm & (1 << 9)) >> 9) strcat(retu,"\tCurrent limit\n");		//
			if ((alarm & (1 << 10)) >> 10) strcat(retu,"\tI offset\n");
			if ((alarm & (1 << 11)) >> 11) strcat(retu,"\tBattery sense out of range\n");
			if ((alarm & (1 << 12)) >> 12) strcat(retu,"\tBattery sense disconnected\n");
			if ((alarm & (1 << 13)) >> 13) strcat(retu,"\tUncalibrated\n");
			if ((alarm & (1 << 14)) >> 14) strcat(retu,"\tTB 5v\n");
			if ((alarm & (1 << 15)) >> 15) strcat(retu,"\tFP10 supply\n");
			if ((alarm & (1 << 16)) >> 16) strcat(retu,"\t[unused]\n");
			if ((alarm & (1 << 17)) >> 17) strcat(retu,"\tFET open\n");
			if ((alarm & (1 << 18)) >> 18) strcat(retu,"\tIA offset\n");
			if ((alarm & (1 << 19)) >> 19) strcat(retu,"\tIL offset\n");
			if ((alarm & (1 << 20)) >> 20) strcat(retu,"\t3v supply\n"); 
			if ((alarm & (1 << 21)) >> 21) strcat(retu,"\t12v supply\n"); 
			if ((alarm & (1 << 22)) >> 22) strcat(retu,"\tHigh Va current limit\n"); 
			if ((alarm & (1 << 23)) >> 23) strcat(retu,"\tReset\n"); 
			if ((alarm & (1 << 24)) >> 24) strcat(retu,"\tLVD\n"); 
			if ((alarm & (1 << 25)) >> 25) strcat(retu,"\tLog timeout\n"); 
			if ((alarm & (1 << 26)) >> 26) strcat(retu,"\tEEPROM access failure\n"); 
		}
		
	}
	else if (hex == 0x003A) { //--dip_switch bitfield------------------------------:
		unsigned short load_fault= (short) dvalue; char tmp = 0;
		//snprintf(retu, sizeof(retu), "Dip switch settings at power up:  %s", nl); 
		strcpy(retu,"Dip switch settings at power up:  \n");
		//--Charger:
		if (load_fault & 1) { strcat(retu,"\tLighting mode:\t\t ON. Load mode OFF.\n"); }
		else { strcat(retu,"\tNormal Load:\t\t ON. (switch OFF)\n");  }
		//--SysVoltage:
		strcat(retu,"\tSystem voltage: "); 
		if ((load_fault & (1 << 1)) >> 1) tmp=1;
		if ((load_fault & (1 << 2)) >> 2) { if (!tmp) { strcat(retu,"\t12v \n"); } }
		else { if (tmp) { strcat(retu,"\t24v \n"); }
			   else { strcat(retu,"\tAuto \n");  } }    
		//--battery type:
		if ((load_fault & (1 << 3)) >> 3) { strcat(retu,"\tBattery type:\t\t ON");  }
		else { strcat(retu,"\tBattery type:\t\t OFF");  }
		if ((load_fault & (1 << 4)) >> 4) { strcat(retu," ON");  }
		else { strcat(retu," OFF");  }
		if ((load_fault & (1 << 5)) >> 5) { strcat(retu," ON\n");  }
		else { strcat(retu," OFF\n");  }
		//--EQ
		if ((load_fault & (1 << 6)) >> 6) { strcat(retu,"\tEQ auto-settings:\t ON \n");  }
		else { strcat(retu,"\tEQ auto-settings:\t OFF \n");  }
		if ((load_fault & (1 << 7)) >> 7) { strcat(retu,"\tCOM:\t\t\tModbus On \n");  }
		else { strcat(retu,"\tCOM:\t\t\tMeterbus \n");  }
	}
	//--SOC LED_state-of-charge------------------------------------------------------: 
	else if (hex == 0x003B || hex == 0x004D) { //--
		switch (dvalue) {
			case 0: strcpy(retu, "LED START"); 
			break;
			case 1: strcpy(retu, "LED START2"); 
			break;
			case 2: strcpy(retu , "LED BRANCH"); 
			break;
			case 3: strcpy(retu, "Equalize (fast green)"); 
			break;
			case 4: strcpy(retu, "Float (slow green)"); 
			break;
			case 5: strcpy(retu, "Absorption (1 Hz green)"); 
			break;
			case 6: strcpy(retu, "GREEN"); 
			break;
			case 7: strcpy(retu, "GREEN / YELLOW"); 
			break;
			case 8: strcpy(retu, "YELLOW"); 
			break;
			case 9: strcpy(retu, "YELLOW / RED"); 
			break;
			case 10: strcpy(retu, "RED Blink"); 
			break;
			case 11: strcpy(retu, "RED"); 
			break;	
			case 12 ... 19: strcpy(retu, "R/G/B ERROR"); 
			break;
			case 20: strcpy(retu, "OFF"); 
			break;
			case 21: strcpy(retu, "R/G/B x2 - GREEN x2"); 
			break;
			case 22: strcpy(retu, "R/G/B x2 - RED x2"); 
			break;
		}
	}
	else { strcpy(retu, "?"); }
	//--Avoid memory overflows!!:  ( un-useful since strcat crashes if overflow ! )
	//if (strlen(retu) < 1 || strlen(retu) > 254) return "ERROR";
	//--return:
	return retu;
}

//--change usage to documented function??: 
uint16_t fl_to_half(const float x) {
        const uint b = as_uint(x)+0x00001000;
        const uint e = (b & 0x7F800000)>>23;
        const uint m = b & 0x007FFFFF;
        return (b & 0x80000000)>>16 | (e>112)*((((e-112)<<10)& 0x7C00)|m>>13) | 
                ((e<113)&(e>101))*((((0x007FF000+m)>>(125-e))+1)>>1) | (e>143)*0x7FFF;
}
uint as_uint(const float x) { return *(uint*)&x; }

/*/--documented function: float to f16   !does not work! 
uint16_t F32ConvertToF16(float f32) { 
	uint16_t f16 = 0;
	uint32_t f32_u = *(uint32_t*)&f32;
	unsigned sign 		= (f32_u & 0x80000000) >> 31; //
	unsigned exponent 	= (f32_u & 0x7f800000) >> 23; 	//
	unsigned fraction 	= (f32_u & 0x007fffff) >> 13; //
	//check for inf and NaN 
	if (exponent == 0xFF) {
		if (fraction == 0) { //inf detected
			f16 = (sign == 1) ? 0xfc00 : 0x7c00;
		} else { // NaN detected
			f16 = 0x7c01; // output a NaN
		}
	} else { 
		//verify the number if within range 
		if (((int)exponent - 127) >= 15) {
			//overflow, if exponent values are too high, we should return an inf
			f16 = (sign == 1) ? 0xfc00 : 0x7c00; // negative and positive infinity, respectively 
		} else if (((int)exponent - 127) <= -14) { 
			//underflow
		} else { // normal numbers
			f16 = (sign << 15) + ((unsigned)((int)exponent - 127) << 10) + (fraction);
		}
	} return f16;
} // */
//--documented function: F16 to float
float F16ConvertToF32(uint16_t f16) { 
	float f32 = 0;
	unsigned sign = (f16 & 0x8000) >> 15; 		//extract out the sign
	unsigned exponent = ((f16 & 0x7C00) >> 10); //extract out the exponent == 18 always
	float fraction = (f16 & 0x03ff) / 1024.0; 	//extract out the fraction
  
	//check for inf & NaN, 0x7F800000 = +inf 0xFF800000 = -inf 
	if (exponent == 0x1f) {
		if (fraction == 0) {
			int positveInf = 0x7f800000;
			int negativeInf = 0xff800000;
			return (sign == 0) ? *(float*)&positveInf : *(float*)&negativeInf;
		} else { 
			return 0.0 / 0.0; //use 0.0 to generate NaN }}
	}	}
	//check for 0 or subnormal 
	if (exponent == 0 ) {
		if (fraction == 0 ) { // if it is 0 
			if (sign == 1 )  return -0.0;
			return 0.0; // use 0.0 to return a zero in float
		}
		else {
			f32 = fraction * pow(2.0, -14.0); 
			if (sign == 1)	f32 *= -1.0; 
			return f32;
	}	}
	//the number is not a NaN or 0 or subnormal
	f32 = (fraction + 1.0) * pow(2.0, ((int)exponent - 15)); 
	if (sign == 1)  f32 *= -1.0; 
	return f32;
}
/*  */

