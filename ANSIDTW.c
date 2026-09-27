////////////////////////////////////////////////////////////////////////////////
// Block Definition - ANSIDTW based pilot protection (AC only)
////////////////////////////////////////////////////////////////////////////////

#include "ANSIDTW.h"
#include "../cfgmacros.h"
#include "../Protection/lprotdata.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// #undef RTEX
#ifdef RTEX

extern LPFRuntime LPFRt; // global runtime instance

////////////////////////////////////////////////////////////////////////////////
// Additional support functions for the block
////////////////////////////////////////////////////////////////////////////////
static LPFFLOAT ANSIDTWMin3(LPFFLOAT a, LPFFLOAT b, LPFFLOAT c)
{
    LPFFLOAT m = a;
    if(b < m) m = b;
    if(c < m) m = c;
    return m;
}

static LPFFLOAT ANSIDTWCostLinear(const LPFFLOAT x[], const LPFFLOAT y[], LPFLONG size)
{
    LPFFLOAT prev_mem[ANSIDTW_MAXWINSIZE];
    LPFFLOAT curr_mem[ANSIDTW_MAXWINSIZE];
    LPFFLOAT *prev;
    LPFFLOAT *curr;
    LPFFLOAT *tmp;
    LPFLONG i, j;

    prev = prev_mem;
    curr = curr_mem;

    prev[0] = (LPFFLOAT)fabs((double)(x[0] - y[0]));
    for(j = 1; j < size; j++)
        prev[j] = (LPFFLOAT)fabs((double)(x[0] - y[j])) + prev[j - 1];

    for(i = 1; i < size; i++)
    {
        curr[0] = (LPFFLOAT)fabs((double)(x[i] - y[0])) + prev[0];
        for(j = 1; j < size; j++)
        {
            curr[j] = (LPFFLOAT)fabs((double)(x[i] - y[j])) +
                      ANSIDTWMin3(prev[j], curr[j - 1], prev[j - 1]);
        }

        tmp = prev;
        prev = curr;
        curr = tmp;
    }

    return prev[size - 1];
}

static void ANSIDTWBuildOrderedSeries(const LPFFLOAT xbuf[],
                                  const LPFFLOAT ybuf[],
                                  LPFLONG start,
                                  LPFLONG size,
                                  LPFINT invertY,
                                  LPFFLOAT xord[],
                                  LPFFLOAT yord[])
{
    LPFLONG i;
    LPFLONG pos;

    pos = start;
    for(i = 0; i < size; i++)
    {
        xord[i] = xbuf[pos];
        yord[i] = invertY ? -ybuf[pos] : ybuf[pos];
        pos++;
        if(pos >= size) pos = 0;
    }
}

static LPFFLOAT ANSIDTWCostFromCircular(const LPFFLOAT xbuf[],
                                    const LPFFLOAT ybuf[],
                                    LPFLONG start,
                                    LPFLONG size,
                                    LPFINT invertY)
{
    LPFFLOAT xord[ANSIDTW_MAXWINSIZE];
    LPFFLOAT yord[ANSIDTW_MAXWINSIZE];

    ANSIDTWBuildOrderedSeries(xbuf, ybuf, start, size, invertY, xord, yord);
    return ANSIDTWCostLinear(xord, yord, size);
}

static LPFLONG ANSIDTWGetConfSamples(const ANSIDTWData *Data)
{
    LPFFLOAT fs;
    LPFFLOAT samples;
    LPFLONG  conf;

    // fs = (LPFFLOAT)(LPFRt.SysDt->N * LPFRt.SysDt->F0);
    fs = LPFRt.SysDt->NSCAN_F0; // corrected by codex
    samples = Data->tconf_ms * fs * (LPFFLOAT)0.001f;
    conf = (LPFLONG)(samples + (LPFFLOAT)0.5f);
    if(conf < 1) conf = 1;
    return conf;
}

////////////////////////////////////////////////////////////////////////////////
// Block execution support functions
////////////////////////////////////////////////////////////////////////////////
int ANSIDTWInit(long baddr, ANSIDTWData *Data)
{
    LPFLONG i;

    baddr = baddr;

    // Check survival data
    if(Data->BlockType != (LPFLONG)(ID_ANSIDTW))
        return ERROR_WRONG_BLOCK_ID;
    if(Data->BlockSize != sizeof(ANSIDTWData))
        return ERROR_WRONG_BLOCK_SIZE;


    //initializes (updates) all inputs pointer with the baddr offset 
    Data->BlockA= (LPFDIGVAR *)((long)Data->BlockA + baddr);
    Data->BlockB= (LPFDIGVAR *)((long)Data->BlockB + baddr);
    Data->BlockC= (LPFDIGVAR *)((long)Data->BlockC + baddr);
    Data->Block0= (LPFDIGVAR *)((long)Data->Block0 + baddr);


    // Runtime data
    Data->StatFlag = 0;
    Data->StateCalc = 0;
    Data->PosToPut = 0;
    Data->PickupCnt_A = 0;
    Data->PickupCnt_B = 0;
    Data->PickupCnt_C = 0;
    Data->PickupCnt_0 = 0;

    for(i = 0; i < ANSIDTW_MAXWINSIZE; i++)
    {
        Data->IAL[i] = 0.0f; Data->IBL[i] = 0.0f; Data->ICL[i] = 0.0f; Data->I0L[i] = 0.0f; // inicialization
        Data->IAR[i] = 0.0f; Data->IBR[i] = 0.0f; Data->ICR[i] = 0.0f; Data->I0R[i] = 0.0f;
    }

    Data->Op_A = 0.0f; Data->Op_B = 0.0f; Data->Op_C = 0.0f; Data->Op_0 = 0.0f;
    Data->Re_A = 0.0f; Data->Re_B = 0.0f; Data->Re_C = 0.0f; Data->Re_0 = 0.0f;

    Data->Operate_A = 0; Data->Operate_B = 0; Data->Operate_C = 0; Data->Operate_0 = 0;
    Data->Operate = 0;

    // Defaults if parser did not set values
    if(Data->win_size <= 0) Data->win_size = ANSIDTW_WINSIZE_32;
    if(Data->rest_gain_abc <= 0.0f) Data->rest_gain_abc = 1.0f;
    if(Data->rest_gain_0 <= 0.0f) Data->rest_gain_0 = 1.10f;
    // if(Data->tconf_ms <= 0.0f) Data->tconf_ms = 1.0f;
    if(Data->tconf_ms < 0.0f) Data->tconf_ms = 1.0f; // corrected by codex

    return OK;
}

void ANSIDTWRun(ANSIDTWData *Data) // ponteiro para o struct ANSIDTWData
{
    LPFLONG conf_samples;
    LPFFLOAT iaL, ibL, icL, iaR, ibR, icR;

    Data->StateCalc = 0;

    while(Data->StateCalc < NCALCS)
    {
        switch(Data->StateCalc)
        {
        case 0:
            iaL = LPFRt.DSPDt->Samples.Skm3.ia; ibL = LPFRt.DSPDt->Samples.Skm3.ib; icL = LPFRt.DSPDt->Samples.Skm3.ic;
            iaR = LPFRt.DSPDt->Samples.Skm3.ia2; ibR = LPFRt.DSPDt->Samples.Skm3.ib2; icR = LPFRt.DSPDt->Samples.Skm3.ic2;
            break;
        case 1:
            iaL = LPFRt.DSPDt->Samples.Skm2.ia; ibL = LPFRt.DSPDt->Samples.Skm2.ib; icL = LPFRt.DSPDt->Samples.Skm2.ic;
            iaR = LPFRt.DSPDt->Samples.Skm2.ia2; ibR = LPFRt.DSPDt->Samples.Skm2.ib2; icR = LPFRt.DSPDt->Samples.Skm2.ic2;
            break;
        case 2:
            iaL = LPFRt.DSPDt->Samples.Skm1.ia; ibL = LPFRt.DSPDt->Samples.Skm1.ib; icL = LPFRt.DSPDt->Samples.Skm1.ic;
            iaR = LPFRt.DSPDt->Samples.Skm1.ia2; ibR = LPFRt.DSPDt->Samples.Skm1.ib2; icR = LPFRt.DSPDt->Samples.Skm1.ic2;
            break;
        default:
            iaL = LPFRt.DSPDt->Samples.Sk.ia; ibL = LPFRt.DSPDt->Samples.Sk.ib; icL = LPFRt.DSPDt->Samples.Sk.ic;
            iaR = LPFRt.DSPDt->Samples.Sk.ia2; ibR = LPFRt.DSPDt->Samples.Sk.ib2; icR = LPFRt.DSPDt->Samples.Sk.ic2;
            break;
        }

        Data->IAL[Data->PosToPut] = iaL; // sample writing
        Data->IBL[Data->PosToPut] = ibL;
        Data->ICL[Data->PosToPut] = icL;
        Data->I0L[Data->PosToPut] = iaL + ibL + icL; // 3I0
        Data->IAR[Data->PosToPut] = iaR;
        Data->IBR[Data->PosToPut] = ibR;
        Data->ICR[Data->PosToPut] = icR;
        Data->I0R[Data->PosToPut] = iaR + ibR + icR; // 3I0

        Data->PosToPut++;
        if(Data->PosToPut >= Data->win_size) Data->PosToPut = 0;

        Data->StateCalc++;
    }
    // referencias positivas dos dois TCs apontam para dentro da zona protegida
    // Op = ANSIDTW(x,-y) ; Re = ANSIDTW(x,y)
    Data->Op_A = ANSIDTWCostFromCircular(Data->IAL, Data->IAR, Data->PosToPut, Data->win_size, 1);
    Data->Re_A = ANSIDTWCostFromCircular(Data->IAL, Data->IAR, Data->PosToPut, Data->win_size, 0);
    Data->Op_B = ANSIDTWCostFromCircular(Data->IBL, Data->IBR, Data->PosToPut, Data->win_size, 1);
    Data->Re_B = ANSIDTWCostFromCircular(Data->IBL, Data->IBR, Data->PosToPut, Data->win_size, 0);
    Data->Op_C = ANSIDTWCostFromCircular(Data->ICL, Data->ICR, Data->PosToPut, Data->win_size, 1);
    Data->Re_C = ANSIDTWCostFromCircular(Data->ICL, Data->ICR, Data->PosToPut, Data->win_size, 0);
    Data->Op_0 = ANSIDTWCostFromCircular(Data->I0L, Data->I0R, Data->PosToPut, Data->win_size, 1);
    Data->Re_0 = ANSIDTWCostFromCircular(Data->I0L, Data->I0R, Data->PosToPut, Data->win_size, 0);

    conf_samples = ANSIDTWGetConfSamples(Data);

    if(Data->Op_A > (Data->rest_gain_abc * Data->Re_A + MATH_RES) && *Data->BlockA==0) Data->PickupCnt_A++;
    else Data->PickupCnt_A = 0;
    if(Data->Op_B > (Data->rest_gain_abc * Data->Re_B + MATH_RES) && *Data->BlockB==0) Data->PickupCnt_B++;
    else Data->PickupCnt_B = 0;
    if(Data->Op_C > (Data->rest_gain_abc * Data->Re_C + MATH_RES) && *Data->BlockC==0) Data->PickupCnt_C++;
    else Data->PickupCnt_C = 0;
    if(Data->Op_0 > (Data->rest_gain_0 * Data->Re_0 + MATH_RES)&& *Data->Block0==0 ) Data->PickupCnt_0++;
    else Data->PickupCnt_0 = 0;

    Data->Operate_A = (Data->PickupCnt_A >= conf_samples);
    Data->Operate_B = (Data->PickupCnt_B >= conf_samples);
    Data->Operate_C = (Data->PickupCnt_C >= conf_samples);
    Data->Operate_0 = (Data->PickupCnt_0 >= conf_samples);

    Data->Operate = Data->Operate_A || Data->Operate_B || Data->Operate_C || Data->Operate_0;
}

#else

////////////////////////////////////////////////////////////////////////////////
// Block parser routines
////////////////////////////////////////////////////////////////////////////////
long ANSIDTWDataParser(long bnum, FILE *arq,
                   BlockTemplateData *BlockTpl, LPFCase *SCase,
                   LPFRuntime *Rt)
{
    ANSIDTWData *ANSIDTW;
    long  offnlinha = 0;
    char  param[200];
    char  orig[200];
    float temp;
    long offset;

    ANSIDTW = (ANSIDTWData*)&Rt->LPFData[SCase->Block[bnum].BlockOffset/sizeof(LPF_MEMORY_VAR)];

    // Checks internal block alignment
    if(ANSIDTW->BlockType != 0 || ANSIDTW->BlockSize != 0)
        InternalError("Incorrect block addressing for block %s, case block %ld\r\n",
                      BlockTpl->Block[SCase->Block[bnum].BlockTemplateNum].BlockName, bnum);

    Verbose(VERB_PARSER_RUN, "\t\t%s parser running for case block #%ld (%s)\r\n",
            BlockTpl->Block[SCase->Block[bnum].BlockTemplateNum].BlockName, bnum,
            SCase->Block[bnum].InstanceName);

#ifdef WIN32
#pragma warning( push )
#pragma warning( disable : 4127 )
#endif

    // Defaults
    ANSIDTW->rest_gain_abc = 1.0f;
    ANSIDTW->rest_gain_0 = 1.10f;
    ANSIDTW->tconf_ms = 1.0f;

    if(GetParameterFromCurrentSection(arq, ANSIDTW_WIN_SIZE, param, &offnlinha, orig))
    {
        temp = (float)atoi(param);
        if(temp == ANSIDTW_WINSIZE_16 ||
           temp == ANSIDTW_WINSIZE_32 ||
           temp == ANSIDTW_WINSIZE_64)
        {
            ANSIDTW->win_size = (LPFINT)temp;
        }
        else
            ParserError(SCase->CaseFile, offnlinha,
                        "Invalid %s = %lf. Should be 16, 32 or 64.\r\n", ANSIDTW_WIN_SIZE, temp);
    }
    else
    {
        ParserError(SCase->CaseFile, offnlinha, "Count not find %s.", ANSIDTW_WIN_SIZE);
    }

    if(GetParameterFromCurrentSection(arq, ANSIDTW_REST_GAIN_ABC, param, &offnlinha, orig))
    {
        temp = (float)atof(param);
        if(temp >= 1.0f && temp <= 10.0f)
            ANSIDTW->rest_gain_abc = temp;
        else
            ParserError(SCase->CaseFile, offnlinha,
                        "Invalid %s = %lf. Should be between 1.0 and 10.0\r\n",
                        ANSIDTW_REST_GAIN_ABC, temp);
    }

    if(GetParameterFromCurrentSection(arq, ANSIDTW_REST_GAIN_0, param, &offnlinha, orig))
    {
        temp = (float)atof(param);
        if(temp >= 1.0f && temp <= 10.0f)
            ANSIDTW->rest_gain_0 = temp;
        else
            ParserError(SCase->CaseFile, offnlinha,
                        "Invalid %s = %lf. Should be between 1.0 and 10.0\r\n",
                        ANSIDTW_REST_GAIN_0, temp);
    }

    if(GetParameterFromCurrentSection(arq, ANSIDTW_TCONF_MS, param, &offnlinha, orig))
    {
        temp = (float)atof(param);
        if(temp >= 0.0f && temp <= 1000.0f)
            ANSIDTW->tconf_ms = temp;
        else
            ParserError(SCase->CaseFile, offnlinha,
                        "Invalid %s = %lf. Should be between 0.0 and 1000.0 [ms]\r\n",
                        ANSIDTW_TCONF_MS, temp);
    }

    // STAGE C -------------------------------------------------------------------------------------
    //Searches for all block input addresses amount the application list of blocks and outputs
    if(GetParameterFromCurrentSection(arq, ANSIDTW_DIG_INPUT_BLOCKA, param, &offnlinha, orig))
    {
        if(SearchBlockOutput(param, BlockTpl, SCase, bnum, &offset)) //looks for the output that should drive the enable input 
        {
            ANSIDTW->BlockA=(void*)offset; 
        }
        else
        {
            ParserError(SCase->CaseFile, offnlinha,
                        "Could not find output specified [%s]\r\n",orig);
        }
    }
    else
    {
        ParserError(SCase->CaseFile, offnlinha,
                "Block input %s not specificed.\r\n",
                ANSIDTW_DIG_INPUT_BLOCKA);
    }

    // STAGE C -------------------------------------------------------------------------------------
    //Searches for all block input addresses amount the application list of blocks and outputs
    if(GetParameterFromCurrentSection(arq, ANSIDTW_DIG_INPUT_BLOCKB, param, &offnlinha, orig))
    {
        if(SearchBlockOutput(param, BlockTpl, SCase, bnum, &offset)) //looks for the output that should drive the enable input 
        {
            ANSIDTW->BlockB=(void*)offset; 
        }
        else
        {
            ParserError(SCase->CaseFile, offnlinha,
                        "Could not find output specified [%s]\r\n",orig);
        }
    }
    else
    {
        ParserError(SCase->CaseFile, offnlinha,
                "Block input %s not specificed.\r\n",
                ANSIDTW_DIG_INPUT_BLOCKB);
    }

    // STAGE C -------------------------------------------------------------------------------------
    //Searches for all block input addresses amount the application list of blocks and outputs
    if(GetParameterFromCurrentSection(arq, ANSIDTW_DIG_INPUT_BLOCKC, param, &offnlinha, orig))
    {
        if(SearchBlockOutput(param, BlockTpl, SCase, bnum, &offset)) //looks for the output that should drive the enable input 
        {
            ANSIDTW->BlockC=(void*)offset; 
        }
        else
        {
            ParserError(SCase->CaseFile, offnlinha,
                        "Could not find output specified [%s]\r\n",orig);
        }
    }
    else
    {
        ParserError(SCase->CaseFile, offnlinha,
                "Block input %s not specificed.\r\n",
                ANSIDTW_DIG_INPUT_BLOCKC);
    }

    // STAGE C -------------------------------------------------------------------------------------
    //Searches for all block input addresses amount the application list of blocks and outputs
    if(GetParameterFromCurrentSection(arq, ANSIDTW_DIG_INPUT_BLOCK0, param, &offnlinha, orig))
    {
        if(SearchBlockOutput(param, BlockTpl, SCase, bnum, &offset)) //looks for the output that should drive the enable input 
        {
            ANSIDTW->Block0=(void*)offset; 
        }
        else
        {
            ParserError(SCase->CaseFile, offnlinha,
                        "Could not find output specified [%s]\r\n",orig);
        }
    }
    else
    {
        ParserError(SCase->CaseFile, offnlinha,
                "Block input %s not specificed.\r\n",
                ANSIDTW_DIG_INPUT_BLOCK0);
    }

    strncpy(ANSIDTW->nam, SCase->Block[bnum].InstanceName, ID_MAXLENGTH);

    // Stores block metadata inside structure
    ANSIDTW->BlockType = BlockTpl->Block[SCase->Block[bnum].BlockTemplateNum].BlockID;
    ANSIDTW->BlockSize = BlockTpl->Block[SCase->Block[bnum].BlockTemplateNum].BlockSize;

    // Stores commands inside runtime structure
    RuntimeInsertInitFunction(Rt, BlockTpl, SCase, bnum);
    RuntimeInsertRunFunction(Rt, BlockTpl, SCase, bnum);

#ifdef WIN32
#pragma warning( pop )
#endif

    return OK;
}

void ANSIDTWDeclaration(BlockTemplateData *Template)
{
    long bnum;
    ANSIDTWData aux;

    bnum = TemplateIncludeBlock(Template, NAME_ANSIDTW,
                                ID_ANSIDTW, sizeof(aux), BLOCK_ARITHMETIC);

    //Digital inputs
    BlockIncludeInput(&Template->Block[bnum], DIG_INPUT,
                      ANSIDTW_DIG_INPUT_BLOCKA, (long)(&aux.BlockA)-(long)(&aux));
    BlockIncludeInput(&Template->Block[bnum], DIG_INPUT,
                      ANSIDTW_DIG_INPUT_BLOCKB, (long)(&aux.BlockB)-(long)(&aux));
    BlockIncludeInput(&Template->Block[bnum], DIG_INPUT,
                      ANSIDTW_DIG_INPUT_BLOCKC, (long)(&aux.BlockC)-(long)(&aux));
    BlockIncludeInput(&Template->Block[bnum], DIG_INPUT,
                      ANSIDTW_DIG_INPUT_BLOCK0, (long)(&aux.Block0)-(long)(&aux));
                
    // Digital outputs
    BlockIncludeOutput(&Template->Block[bnum], DIG_OUTPUT,
                       ANSIDTW_DIG_OUTPUT_OPERATE_A, (long)(&aux.Operate_A) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], DIG_OUTPUT,
                       ANSIDTW_DIG_OUTPUT_OPERATE_B, (long)(&aux.Operate_B) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], DIG_OUTPUT,
                       ANSIDTW_DIG_OUTPUT_OPERATE_C, (long)(&aux.Operate_C) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], DIG_OUTPUT,
                       ANSIDTW_DIG_OUTPUT_OPERATE_0, (long)(&aux.Operate_0) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], DIG_OUTPUT,
                       ANSIDTW_DIG_OUTPUT_OPERATE,   (long)(&aux.Operate)   - (long)(&aux));

    // Analog outputs - Operating
    BlockIncludeOutput(&Template->Block[bnum], AN_OUTPUT,
                       ANSIDTW_AN_OUTPUT_OP_A, (long)(&aux.Op_A) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], AN_OUTPUT,
                       ANSIDTW_AN_OUTPUT_OP_B, (long)(&aux.Op_B) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], AN_OUTPUT,
                       ANSIDTW_AN_OUTPUT_OP_C, (long)(&aux.Op_C) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], AN_OUTPUT,
                       ANSIDTW_AN_OUTPUT_OP_0, (long)(&aux.Op_0) - (long)(&aux));

    // Analog outputs - Restraining
    BlockIncludeOutput(&Template->Block[bnum], AN_OUTPUT,
                       ANSIDTW_AN_OUTPUT_RE_A, (long)(&aux.Re_A) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], AN_OUTPUT,
                       ANSIDTW_AN_OUTPUT_RE_B, (long)(&aux.Re_B) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], AN_OUTPUT,
                       ANSIDTW_AN_OUTPUT_RE_C, (long)(&aux.Re_C) - (long)(&aux));
    BlockIncludeOutput(&Template->Block[bnum], AN_OUTPUT,
                       ANSIDTW_AN_OUTPUT_RE_0, (long)(&aux.Re_0) - (long)(&aux));

    // Internal parameters
    BlockIncludeParam(&Template->Block[bnum], DIG_PARAM,
                      ANSIDTW_WIN_SIZE, (long)(&aux.win_size) - (long)(&aux));
    BlockIncludeParam(&Template->Block[bnum], AN_PARAM,
                      ANSIDTW_REST_GAIN_ABC, (long)(&aux.rest_gain_abc) - (long)(&aux));
    BlockIncludeParam(&Template->Block[bnum], AN_PARAM,
                      ANSIDTW_REST_GAIN_0, (long)(&aux.rest_gain_0) - (long)(&aux));
    BlockIncludeParam(&Template->Block[bnum], AN_PARAM,
                      ANSIDTW_TCONF_MS, (long)(&aux.tconf_ms) - (long)(&aux));

    // Block operational details
    BlockIncludeInitializer(&Template->Block[bnum]);
    BlockIncludeRuntime(&Template->Block[bnum]);

    // Stores custom block data parser
    BlockIncludeDataParser(&Template->Block[bnum], &ANSIDTWDataParser);
}

#endif
