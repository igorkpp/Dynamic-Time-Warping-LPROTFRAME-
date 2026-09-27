#ifndef HEADER_ANSIDTW
#define HEADER_ANSIDTW

#include "../EventRec/eventrecdata.h"
#include "../Protection/lprotdata.h"
#include "../blockdef.h"

// BLOCK BASIC DEFINITIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define NAME_ANSIDTW "ANSIDTW"

#define ID_ANSIDTW 0x44D75431U

//Digital inputs
#define ANSIDTW_DIG_INPUT_BLOCKA    "BLOCK_A"   //Digital input for blocking of phase A element
#define ANSIDTW_DIG_INPUT_BLOCKB    "BLOCK_B"   //Digital input for blocking of phase B element
#define ANSIDTW_DIG_INPUT_BLOCKC    "BLOCK_C"   //Digital input for blocking of phase C element
#define ANSIDTW_DIG_INPUT_BLOCK0    "BLOCK_0"   //Digital input for blocking of sequence 0 element

// Digital outputs (AC only)
#define ANSIDTW_DIG_OUTPUT_OPERATE_A  "OPERATE_A"
#define ANSIDTW_DIG_OUTPUT_OPERATE_B  "OPERATE_B"
#define ANSIDTW_DIG_OUTPUT_OPERATE_C  "OPERATE_C"
#define ANSIDTW_DIG_OUTPUT_OPERATE_0  "OPERATE_0"
#define ANSIDTW_DIG_OUTPUT_OPERATE    "OPERATE"

// Analog outputs (AC only)
// Operating ABC0
#define ANSIDTW_AN_OUTPUT_OP_A        "OP_A"
#define ANSIDTW_AN_OUTPUT_OP_B        "OP_B"
#define ANSIDTW_AN_OUTPUT_OP_C        "OP_C"
#define ANSIDTW_AN_OUTPUT_OP_0        "OP_0"
// Restraining ABC0 
#define ANSIDTW_AN_OUTPUT_RE_A        "RE_A"
#define ANSIDTW_AN_OUTPUT_RE_B        "RE_B"
#define ANSIDTW_AN_OUTPUT_RE_C        "RE_C"
#define ANSIDTW_AN_OUTPUT_RE_0        "RE_0"

// Parameters
#define ANSIDTW_WIN_SIZE              "WIN_SIZE"      // data windows -> 32 used in the ANSIDTW paper
#define ANSIDTW_REST_GAIN_ABC         "REST_GAIN_ABC" // Op_ABC > REST_GAIN_ABC * Re_ABC -> no mention in the paper. If not specified, use 1 
#define ANSIDTW_REST_GAIN_0           "REST_GAIN_0"   // Op_0 > REST_GAIN_0 * Re_0 -> 110% used in the ANSIDTW paper
#define ANSIDTW_TCONF_MS              "TCONF_MS"      // trip confirmation time interval -> 1ms used in the ANSIDTW paper

// Event register
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ANSIDTW_EVENT_OPERATE_A       1
#define ANSIDTW_EVENT_OPERATE_B       2
#define ANSIDTW_EVENT_OPERATE_C       4
#define ANSIDTW_EVENT_OPERATE_0       8

// Other constants
////////////////////////////////////////////////////////////////////////////////
#define ANSIDTW_MAXWINSIZE 64 //max window size
#define ANSIDTW_WINSIZE_16 16 //setpoint for window size with 16 samples
#define ANSIDTW_WINSIZE_32 32 //setpoint for window size with 32 samples
#define ANSIDTW_WINSIZE_64 64 //setpoint for window size with 64 samples

#define NCALCS (DEF_N/DEF_NSCAN)
#define MATH_RES ((LPFFLOAT)(1.0E-10))

// General data for ANSIDTW protection function (AC only)
////////////////////////////////////////////////////////////////////////////////
typedef struct ANSIDTWData
{
    // Operational data
    LPFLONG BlockType;   // ID/Type of the current block
    LPFLONG BlockSize;   // Size of the current block

    // Config data
    char     nam[ID_MAXLENGTH]; // Identifier for the block instance
    LPFINT   win_size;          // Window size
    LPFFLOAT rest_gain_abc;     // Restraining multiplication factor for A/B/C
    LPFFLOAT rest_gain_0;       // Restraining multiplication factor for zero-sequence
    LPFFLOAT tconf_ms;          // Pickup confirmation interval [ms]

    // Runtime data
    LPFLONG  StatFlag;      // Multi-purpose status flags
    LPFLONG  StateCalc;     // State machine that controls sample processing
    LPFLONG  PosToPut;      // Circular buffer pointer

    LPFLONG  PickupCnt_A;   // Pickup confirmation counter for phase A
    LPFLONG  PickupCnt_B;   // Pickup confirmation counter for phase B
    LPFLONG  PickupCnt_C;   // Pickup confirmation counter for phase C
    LPFLONG  PickupCnt_0;   // Pickup confirmation counter for zero-sequence

    // Local and remote current windows (A, B, C, 0) -> buffers
    LPFFLOAT IAL[ANSIDTW_MAXWINSIZE];
    LPFFLOAT IBL[ANSIDTW_MAXWINSIZE];
    LPFFLOAT ICL[ANSIDTW_MAXWINSIZE];
    LPFFLOAT I0L[ANSIDTW_MAXWINSIZE];
    LPFFLOAT IAR[ANSIDTW_MAXWINSIZE];
    LPFFLOAT IBR[ANSIDTW_MAXWINSIZE];
    LPFFLOAT ICR[ANSIDTW_MAXWINSIZE];
    LPFFLOAT I0R[ANSIDTW_MAXWINSIZE];

    //Digital inputs
    LPFDIGVAR *BlockA;       //Block entry for phase A
    LPFDIGVAR *BlockB;       //Block entry for phase B
    LPFDIGVAR *BlockC;       //Block entry for phase C
    LPFDIGVAR *Block0;       //Block entry for sequence 0

    // Digital outputs
    LPFDIGVAR Operate_A;   // Trip output for phase A
    LPFDIGVAR Operate_B;   // Trip output for phase B
    LPFDIGVAR Operate_C;   // Trip output for phase C
    LPFDIGVAR Operate_0;   // Trip output for zero-sequence
    LPFDIGVAR Operate;     // Global trip output

    // Analog outputs
    LPFANVAR Op_A;         // Operating ANSIDTW quantity for phase A
    LPFANVAR Op_B;         // Operating ANSIDTW quantity for phase B
    LPFANVAR Op_C;         // Operating ANSIDTW quantity for phase C
    LPFANVAR Op_0;         // Operating ANSIDTW quantity for zero-sequence
    LPFANVAR Re_A;         // Restraining ANSIDTW quantity for phase A
    LPFANVAR Re_B;         // Restraining ANSIDTW quantity for phase B
    LPFANVAR Re_C;         // Restraining ANSIDTW quantity for phase C
    LPFANVAR Re_0;         // Restraining ANSIDTW quantity for zero-sequence
} ANSIDTWData;

// Prototypes for real-time runtime
////////////////////////////////////////////////////////////////////////////////
extern int  ANSIDTWInit(long baddr, ANSIDTWData *Data);
extern void ANSIDTWRun(ANSIDTWData *Data);

// Prototypes for parser
////////////////////////////////////////////////////////////////////////////////
extern void ANSIDTWDeclaration(BlockTemplateData *Template);

#endif
