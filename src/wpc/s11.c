// license:BSD-3-Clause

/* Williams System 9, 11, and All Data East Hardware */
#include <stdarg.h>
#include <time.h>
#include "driver.h"
#include "cpu/m6800/m6800.h"
#include "machine/6821pia.h"
#include "core.h"
#include "sndbrd.h"
#include "snd_cmd.h"
#include "wmssnd.h"
#include "desound.h"
#include "dedmd.h"
#include "s11.h"
#ifdef PROC_SUPPORT
#include "p-roc/p-roc.h"
#endif
#if defined(PINMAME) && defined(LISY_SUPPORT)
#include "lisy/lisy_w.h"
#endif /* PINMAME && LISY_SUPPORT */

#define FIXMUX // DataEast Playboy 35th fix

#ifdef RTMPIN
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdio.h>
#include <string.h>
#include "led-matrix-c.h"
#endif


// TODO:
// DE display layouts
#define S11_PIA0 0
#define S11_PIA1 1
#define S11_PIA2 2
#define S11_PIA3 3
#define S11_PIA4 4
#define S11_PIA5 5

// MAME: Length of time in cycles between IRQs on the main 6808 CPU
// This length is determined by the settings of the W14 and W15 jumpers
// It can be 0x300, 0x380, 0x700 or 0x780 cycles long.
// IRQ length is always 32 cycles
//#define S11_IRQ_CYCLES     0x380
// PinWiki: 1ms IRQ signal (W14 in) or 2ms IRQ signal (W15 in), Data East seems to be the same (J7a and J7b, where J7b corresponds to W14)
//          Sys11: W14 in, DE: J7b in (except for Laser War CPU Rev 1, where J7a in)
// Ed Cheung: The IRQ on System 3-11 fires every 928 usec, which perfectly matches MAMEs 0x380+32!
#define S11_IRQFREQ     (1000000.0/928.0)
/*-- Smoothing values --*/
#ifdef PROC_SUPPORT || defined RTMPIN
// TODO/PROC: Make variables out of these defines. Values depend on "-proc" switch.
#define S11_SOLSMOOTH      1 /* Don't smooth values on real hardware */
#define S11_LAMPSMOOTH     1
#define S11_DISPLAYSMOOTH  1
#else
#define S11_SOLSMOOTH       2 /* Smooth the Solenoids over this numer of VBLANKS */
#define S11_LAMPSMOOTH      2 /* Smooth the lamps over this number of VBLANKS */
#define S11_DISPLAYSMOOTH   2 /* Smooth the display over this number of VBLANKS */
#endif


static MACHINE_STOP(s11);
static NVRAM_HANDLER(s11);
static NVRAM_HANDLER(de);
//top, left, start, length, type
const struct core_dispLayout s11_dispS9[] = {
  {4, 0, 1,7, CORE_SEG87}, {4,16, 9,7, CORE_SEG87},
  {0, 0,21,7, CORE_SEG87}, {0,16,29,7, CORE_SEG87},
  DISP_SEG_CREDIT(0,8,CORE_SEG7S),DISP_SEG_BALLS(20,28,CORE_SEG7S),{0}
};
const struct core_dispLayout s11_dispS11[] = {
  DISP_SEG_7(0,0,CORE_SEG16),DISP_SEG_7(0,1,CORE_SEG16),
  DISP_SEG_7(1,0,CORE_SEG8), DISP_SEG_7(1,1,CORE_SEG8),
  {2,8,0,1,CORE_SEG7S},{2,10,8,1,CORE_SEG7S}, {2,2,20,1,CORE_SEG7S},{2,4,28,1,CORE_SEG7S}, {0}
};
const struct core_dispLayout s11_dispS11a[] = {
  DISP_SEG_7(0,0,CORE_SEG16),DISP_SEG_7(0,1,CORE_SEG16),
  DISP_SEG_7(1,0,CORE_SEG8), DISP_SEG_7(1,1,CORE_SEG8) ,{0}
};
const struct core_dispLayout s11_dispS11b2[] = {
  DISP_SEG_16(0,CORE_SEG16),DISP_SEG_16(1,CORE_SEG16),{0}
};

#ifdef RTMPIN
static int serbus;
extern struct RGBLedMatrixOptions matrix_options;
extern struct RGBLedRuntimeOptions runtime_options;
extern struct RGBLedMatrix  *matrix;
extern struct LedCanvas *offscreen_canvas;
#endif


/*----------------
/  Local variables
/-----------------*/
static struct {
  int    vblankCount;
  UINT32 solenoids, solsmooth[S11_SOLSMOOTH];
  UINT32 extSol, extSolPulse;
  core_tSeg segments, pseg;
  int    lampRow, lampColumn;
#ifdef PROC_SUPPORT
  int    ac_select, ac_state;
#endif
  int    digSel;
  int    diagnosticLed;
  int    swCol;
  int    ssEn;				/* Special solenoids and flippers enabled ? */
  int    sndCmd;	/* external sound board cmd */
  int    piaIrq;
  int	 deGame;	/*Flag to see if it's a Data East game running*/
#ifdef FIXMUX
  UINT8  solBits1,solBits2;
  UINT8  solBits2prv;
#endif
} locals;

static void s11_irqline(int state) {
  if (state) {
    cpu_set_irq_line(0, M6808_IRQ_LINE, ASSERT_LINE);
	/*Set coin door inputs, differs between S11 & DE*/
    if (locals.deGame) {
      pia_set_input_ca1(S11_PIA2, !core_getSw(DE_SWADVANCE));
      pia_set_input_cb1(S11_PIA2, core_getSw(DE_SWUPDN));
    }
    else {
      pia_set_input_ca1(S11_PIA2, core_getSw(S11_SWADVANCE));
      pia_set_input_cb1(S11_PIA2, core_getSw(S11_SWUPDN));
    }
  }
  else if (!locals.piaIrq) {
    cpu_set_irq_line(0, M6808_IRQ_LINE, CLEAR_LINE);
    pia_set_input_ca1(S11_PIA2, locals.deGame);
    pia_set_input_cb1(S11_PIA2, locals.deGame);
  }
}

static void s11_piaMainIrq(int state) {
  s11_irqline(locals.piaIrq = state);
}

static INTERRUPT_GEN(s11_irq) {
  s11_irqline(1); timer_set(TIME_IN_CYCLES(32,0),0,s11_irqline);
}

static INTERRUPT_GEN(s11_vblank) {
  /*-------------------------------
  /  copy local data to interface
  /--------------------------------*/

  int ii;

#ifdef PROC_SUPPORT
  //TODO/PROC: Check implementation
  UINT64 allSol = core_getAllSol();
  // Keep the P-ROC tickled each time we run around the interrupt
  // so it knows we are still alive
  if (coreGlobals.p_rocEn) {
    procTickleWatchdog();
  }
#endif


  locals.vblankCount++;
  /*-- lamps --*/
  if ((locals.vblankCount % S11_LAMPSMOOTH) == 0) {
#ifdef PROC_SUPPORT
			if (coreGlobals.p_rocEn) {
                    // Loop through the lamp matrix, looking for any which have changed state
                        int col, row, procLamp;
			for(col = 0; col < CORE_STDLAMPCOLS; col++) {
				UINT8 chgLamps = coreGlobals.lampMatrix[col] ^ coreGlobals.tmpLampMatrix[col];
				UINT8 tmpLamps = coreGlobals.tmpLampMatrix[col];
				for (row = 0; row < 8; row++) {
                                        procLamp = 80 + (8 * col) + row;
                                        // If lamp (col,row) has changed state, drive the P-ROC with the new value
					if (chgLamps & 0x01) {
                                            procDriveLamp(procLamp, tmpLamps & 0x01);
                                        }
                                        // If this lamp was defined in the YAML file as one showng kickback status,
                                        // then call to the kickback routine to update information
                                        if (coreGlobals.isKickbackLamp[procLamp]) {
                                                procKickbackCheck(procLamp);
                                        }
					chgLamps >>= 1;
					tmpLamps >>= 1;
				}
			}
			procFlush();
		}
#endif //PROC_SUPPORT
    memcpy(coreGlobals.lampMatrix, coreGlobals.tmpLampMatrix, sizeof(coreGlobals.tmpLampMatrix));
#if defined(LISY_SUPPORT)
    lisy_w_lamp_handler( );
#endif
    memset(coreGlobals.tmpLampMatrix, 0, sizeof(coreGlobals.tmpLampMatrix));
  }
  /*-- solenoids --*/
  if (locals.ssEn) { // set gameon and special solenoids
    locals.solenoids |= CORE_SOLBIT(S11_GAMEONSOL);

#ifdef PROC_SUPPORT
    if (!coreGlobals.p_rocEn) {
#endif
#ifdef LISY_SUPPORT
    if (0) { //not with LISY support
#endif

    /*-- special solenoids updated based on switches         -- */
    /*-- but only when no P-ROC, otherwise special solenoids -- */
    /*-- lock on when controlled by direct switches          -- */
    for (ii = 0; ii < 6; ii++) {
      if (core_gameData->sxx.ssSw[ii] && core_getSw(core_gameData->sxx.ssSw[ii]))
        locals.solenoids |= CORE_SOLBIT(CORE_FIRSTSSSOL+ii);
    }
#if defined(PROC_SUPPORT) || defined(LISY_SUPPORT)
    }
#endif
  }
#ifdef FIXMUX
// mux translation moved
#else
  if ((core_gameData->sxx.muxSol) &&
      (locals.solenoids & CORE_SOLBIT(core_gameData->sxx.muxSol))) {
    if (core_gameData->hw.gameSpecific1 & S11_RKMUX)
      locals.solenoids = (locals.solenoids & 0x00ff8fef) |
                         ((locals.solenoids & 0x00000010)<<20) |
                         ((locals.solenoids & 0x00007000)<<13);
    else
      locals.solenoids = (locals.solenoids & 0x00ffff00) | (locals.solenoids<<24);
  }
#endif
  locals.solsmooth[locals.vblankCount % S11_SOLSMOOTH] = locals.solenoids;
#ifndef PROC_SUPPORT 
 #if S11_SOLSMOOTH != 2
 #  error "Need to update smooth formula"
 #endif
#endif
  coreGlobals.solenoids  = locals.solsmooth[0] | locals.solsmooth[1];
  coreGlobals.solenoids2 = locals.extSol << 8;
#if defined(LISY_SUPPORT)
   lisy_w_solenoid_handler( ); //RTH: need to add solnoids2 ?
#endif
  locals.solenoids = coreGlobals.pulsedSolState;
  locals.extSol = locals.extSolPulse;

#ifdef PROC_SUPPORT
  if (coreGlobals.p_rocEn) {
  /*
   * Now this code is kind of complicated.  Basically on each iteration, we examine the state of the coils
   * as pinmame has them at the moment.  We compare that to the state of the coils on the previous iteration.
   * So in theory we know which coils have changed and need to be updated on the P-ROC.
   * However, sometimes we catch the iteration just as the A/C select solenoid was going to be
   * pulsed and we don't always have quite the right list of coils.
   * So this code tries to ensure that we only switch the A/C select when all the coils it controls
   * are currently inactive.
   * The routine first handles all coils going inactive, then A/C if it's safe, then all the coils
   * going active

   */
 		allSol = (allSol & 0xffffffff00000000) |
		         (coreGlobals.solenoids);
                // The Sys11 code fires all coils at once at the start - FF00 - in order
                // to get the buffers on the MPU to a known state, before the blanking
                // comes on.  However, with the P-ROC the blanking comes on earlier
                // which means we get a lot of clunks.  So we skip the FF00 call.
                // Cannot think of any reason why any valid call for all coils would
                // be made during the game, so this should be fine.
                if (coreGlobals.p_rocEn && allSol != 0xff00) {
			//int ii;
                        UINT64 chgSol1 = (allSol ^ coreGlobals.lastSol) & 0xffffffffffffffff; //vp_getSolMask64();
			UINT64 tmpSol1 = allSol;
                        UINT64 chgSol2 = chgSol1;
                        UINT64 tmpSol2 = tmpSol1;
                        UINT64 onSol = allSol & chgSol1;
                        UINT64 offSol = ~allSol & chgSol1;
                        // Only bother with all this checking if anything actually changed
                        if (chgSol1 || locals.ac_select) {
                            if (mame_debug) {
                                fprintf(stderr,"\nCoil Loop Start");
                                if (locals.ac_select) fprintf(stderr,"\n -Remembered delayed AC from previous loop");
                            }
                            // First loop through for anything going inactive
                            for (ii=0; ii<64; ii++) {
                                    if (chgSol1 & 0x1) {
                                        // If the A/C select has made any state change, just make a note of it
                                        if (ii==core_gameData->sxx.muxSol - 1) {locals.ac_select = 1; locals.ac_state = tmpSol1 & 0x1;}
                                        // otherwise we only want to do this if we are going inactive
                                        else if (!(tmpSol1 & 0x1)) {
                                            if (mame_debug) fprintf(stderr,"\n -Pinmame coil %d changed state",ii);

                                            if (ii<24) procDriveCoil(ii+40,0);
                                            else if (core_gameData->hw.gameSpecific1 & S11_RKMUX) {
                                                if (mame_debug) fprintf(stderr,"\nRK MUX Logic");
                                                if (ii==24) procDriveCoil(44,0);
                                                else procDriveCoil(ii+27,0);
                                                }
                                            else if (ii < 44) procDriveCoil(ii+16, 0);
                                            }
                                    }
                                    chgSol1 >>= 1;
                                    tmpSol1 >>= 1;
                            }
                            // Now that we processed anything going inactive, we take care of the
                            // A/C select if it needs to change
                            
                            if (locals.ac_select)  {
                                // If this is a Road Kings, it has different muxed solenoids
                                // so check if any of them are still active
                                if (core_gameData->hw.gameSpecific1 & S11_RKMUX) {
                                    if ((offSol ^ coreGlobals.lastSol) & 0xff007010) {
                                        // Some are active, so we won't process A/C yet
                                        if (mame_debug) fprintf(stderr,"\n -AC Select change delayed, code %lx %lx", (long unsigned) offSol,(long unsigned) allSol);
                                    }
                                    else {
                                        // No switched coils are active, so it's safe to do the A/C
                                        if (mame_debug) fprintf(stderr,"\n -AC Select changed state");
                                        procDriveCoil(core_gameData->sxx.muxSol + 39,locals.ac_state);
                                        locals.ac_select = 0;
                                    }
                                }
                                else { //Not a Road Kings
                                    // Check if any of the switched solenoids are active
                                    if ((offSol ^ coreGlobals.lastSol) & 0xff0000ff) {
                                        if (mame_debug) fprintf(stderr,"\n -AC Select change delayed, code %lx %lx", (long unsigned) offSol,(long unsigned) allSol);
                                    }
                                    else {
                                        // If not, it's safe to handle the A/C select
                                        if (mame_debug) fprintf(stderr,"\n -AC Select changed state");
                                        procDriveCoil(core_gameData->sxx.muxSol + 39,locals.ac_state);
                                        locals.ac_select = 0;
                                    }
                                }
                            }
                            // Second loop through for anything going active, these will be
                            // safe now as we processed the A/C select already (if applicable)
                            if (onSol) for (ii=0; ii<64; ii++) {
                                    if (chgSol2 & 0x1) {
                                        // If the A/C select has made any state change, skip it because we already
                                        // handled it
                                        if (ii!=core_gameData->sxx.muxSol - 1 && (tmpSol2 & 0x1)) {
                                            if (mame_debug) fprintf(stderr,"\n -Pinmame coil %d changed state",ii);

                                            if (ii<24) procDriveCoil(ii+40,1);
                                            else if (core_gameData->hw.gameSpecific1 & S11_RKMUX) {
                                                if (mame_debug) fprintf(stderr,"\nRK MUX Logic");
                                                if (ii==24) procDriveCoil(44,1);
                                                else procDriveCoil(ii+27,1);
                                                }
                                            else if (ii< 44) procDriveCoil(ii+16, 1);
                                            }
                                    }
                                    chgSol2 >>= 1;
                                    tmpSol2 >>= 1;
                            }
                            if (mame_debug) fprintf(stderr,"\nCoil Loop End");

                            // This doesn't seem to be happening in core.c.  Why not?
                            coreGlobals.lastSol = allSol;
                        }
		}
  }
#endif  //PROC_SUPPORT


  /*-- display --*/
  if ((locals.vblankCount % S11_DISPLAYSMOOTH) == 0) {
    memcpy(coreGlobals.segments, locals.segments, sizeof(coreGlobals.segments));
#if defined(LISY_SUPPORT)
    lisy_w_display_handler( );
#endif
    memcpy(locals.segments, locals.pseg, sizeof(locals.segments));
#ifdef PROC_SUPPORT
    if (coreGlobals.p_rocEn) {
      /* On the Combo-interface for Sys11, driver 63 is the diag LED */
      if (core_gameData->gen & GEN_ALLS11) {
        if (coreGlobals.diagnosticLed != locals.diagnosticLed) {
            procDriveCoil(63,locals.diagnosticLed);
        }
      }
    }
#endif //PROC_SUPPORT

#ifdef RTMPIN
  UINT8 digit,digitline;
  UINT8 x,y;

	for (y = 0; y < 33; y++) {
		for (x = 0; x < 129; x++) {
			coreGlobals.dotCol[y][x] = 0;
		}
	}
  y = 2;
  do
  {
    for (digit = 0; digit < 16; digit++) {
      x = 8*digit;
      if (y > 15)
      {
        digitline = digit + 20;
      } else
      {
        digitline = digit;
      }

      if (coreGlobals.segments[digitline].w & 1)
      {
        coreGlobals.dotCol[y][x] = BRIGHT_CORNER;
        coreGlobals.dotCol[y][x+1] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y][x+2] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y][x+3] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y][x+4] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y][x+5] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y][x+6] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 2)
      {
        coreGlobals.dotCol[y][x+6] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+1][x+6] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+2][x+6] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+3][x+6] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+4][x+6] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+5][x+6] = BRIGHT_CORNER;
      }
      
      if (coreGlobals.segments[digitline].w & 4)
      {
        coreGlobals.dotCol[y+5][x+6] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+6][x+6] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+7][x+6] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+8][x+6] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+9][x+6] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+10][x+6] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 8)
      {
        coreGlobals.dotCol[y+10][x] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+10][x+1] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+10][x+2] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+10][x+3] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+10][x+4] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+10][x+5] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+10][x+6] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 16)
      {
        coreGlobals.dotCol[y+5][x] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+6][x] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+7][x] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+8][x] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+9][x] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+10][x] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 32)
      {
        coreGlobals.dotCol[y][x] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+1][x] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+2][x] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+3][x] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+4][x] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+5][x] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 64)
      {
        coreGlobals.dotCol[y+5][x] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+5][x+1] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+5][x+2] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+5][x+3] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 128)
      {
        coreGlobals.dotCol[y+10][x+7] = BRIGHT_STRAIGHT;
      }
      if (coreGlobals.segments[digitline].w & 256)
      {
        coreGlobals.dotCol[y+0][x+0] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+1][x+1] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+2][x+1] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+2][x+2] = BRIGHT_ALIASING;
        coreGlobals.dotCol[y+3][x+1] = BRIGHT_ALIASING;
        coreGlobals.dotCol[y+3][x+2] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+4][x+2] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+5][x+3] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 512)
      {
        coreGlobals.dotCol[y+1][x+3] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+2][x+3] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+3][x+3] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+4][x+3] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+5][x+3] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 1024)
      {
        coreGlobals.dotCol[y+0][x+6] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+1][x+5] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+2][x+5] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+2][x+4] = BRIGHT_ALIASING;
        coreGlobals.dotCol[y+3][x+4] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+3][x+5] = BRIGHT_ALIASING;
        coreGlobals.dotCol[y+4][x+4] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+5][x+3] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 2048)
      {
        coreGlobals.dotCol[y+5][x+3] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+5][x+4] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+5][x+5] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+5][x+6] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 4096)
      {
        coreGlobals.dotCol[y+10][x+6] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+9][x+5] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+8][x+5] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+8][x+4] = BRIGHT_ALIASING;
        coreGlobals.dotCol[y+7][x+4] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+7][x+5] = BRIGHT_ALIASING;
        coreGlobals.dotCol[y+6][x+4] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+5][x+3] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 8192)
      {
        coreGlobals.dotCol[y+5][x+3] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+6][x+3] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+7][x+3] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+8][x+3] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+9][x+3] = BRIGHT_STRAIGHT;
    }
      if (coreGlobals.segments[digitline].w & 16384)
      {
        coreGlobals.dotCol[y+10][x+0] = BRIGHT_CORNER;
        coreGlobals.dotCol[y+9][x+1] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+8][x+1] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+8][x+2] = BRIGHT_ALIASING;
        coreGlobals.dotCol[y+7][x+2] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+7][x+1] = BRIGHT_ALIASING;
        coreGlobals.dotCol[y+6][x+2] = BRIGHT_ANGULAR;
        coreGlobals.dotCol[y+5][x+3] = BRIGHT_CORNER;
      }
      if (coreGlobals.segments[digitline].w & 32768)
      {
        coreGlobals.dotCol[y+9][x+7] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+11][x+6] = BRIGHT_STRAIGHT;
        coreGlobals.dotCol[y+11][x+7] = BRIGHT_CORNER;
      }
    }
    y += 16;
  }
  while (y < 32);


	for (int yy = 0; yy < 33; yy++) {
		for (int xx = 0; xx < 129; xx++) {
			UINT8 pixel = (coreGlobals.dotCol[yy][xx]) * 1;
			led_canvas_set_pixel(offscreen_canvas, xx, yy, pixel, 0, 0);  // only R no G or B
		}
	}
	offscreen_canvas = led_matrix_swap_on_vsync(matrix, offscreen_canvas);
#endif



    coreGlobals.diagnosticLed = locals.diagnosticLed;
    locals.diagnosticLed = 0;
  }
  core_updateSw(locals.ssEn);


#ifdef RTMPIN
/* This function is called in rhythm 12ms 21ms 12ms 21ms ..., sometimes also values in between 12ms and 21ms 
   ohne X11 alle 8 ms, kaum vorhersagbar. Ohne X11 und ohne sound (per padsp) kommen sogar 2 Aufrufe unmittelbar
   hintereinander, daher darf nur jeder 2. Aufruf zum senden des seriellen Frames genutzt werden. 
   Mit Sound (per padsp) fehlen zwischendurch ein paar frames, ist aber unkritisch
   Hinweis: hatte den Code vorher in "static SWITCH_UPDATE(s11)" untergebracht - aber gleiches Verhalten wie hier */

/* Receive switch status from nodes, write solnoids, flashers and lamps to nodes  */

/* Address nodes with first 4 bits, next 4 bits is length of data packet in bytes
 * excluding first byte (addr&length)
 */

/* Node addressing starts with 0.
 * 15 is reserved for broadcast (not used yet),
 * 14 is used for receiving the node-status (until now only the switch status)
 * other free addresses might be used for special commands like
 * initialising (be aware to adapt the number of MaxNodes)
 * First the receive-buffer is read to get the node-status from last cycle,
 * then data packets are sent per serial line followed by the special address 14 to 
 * make the nodes send their status one after the other (timing managed by nodes,
 * PI has to be quiet until next cycle).Half duplex as we use RS485.
 */
	unsigned char toNode[17]; /* addr&length + data (max 16) */
	unsigned char fromNodes[14*2]; /* MaxNodes * 2 bytes (fix) */
	unsigned char tmpShift;	/* helper for bit-shifting*/
  unsigned char nr_received = 0;

if ((locals.vblankCount % 2) == 0) {    

/* read the serial buffer to get the nodes status form last cycle */
  nr_received = read(serbus, &fromNodes, sizeof(fromNodes));
  if (nr_received > 0) {

/* debug: printf("received byte 4: %u \n", fromNodes[3]);*/

/* Template Dedicated switches:
 * left coin, center coin, right coin, fourth coin, escape, down, up, enter */
        tmpShift = 0;
/*        tmpShift |= ((fromNodes[0]>>2)&1)<<7;
        tmpShift |= ((fromNodes[0]>>2)&1)<<6;
        tmpShift |= ((fromNodes[0]>>2)&1)<<5;
        tmpShift |= ((fromNodes[0]>>2)&1)<<4;*/
/*        tmpShift |= ((fromNodes[0]>>3)&1)<<3;
        tmpShift |= ((fromNodes[0]>>2)&1)<<2;
        tmpShift |= ((fromNodes[0]>>1)&1)<<1;
        tmpShift |= ((fromNodes[0]>>0)&1)<<0;*/
/*        coreGlobals.swMatrix[CORE_COINDOORSWCOL] = tmpShift; */ /* no more node-inputs available -> setup pin per VNC */

/* Column 1: Line 1 = LSB; Line 8 = MSB */
        tmpShift = 0;
        tmpShift |= ((fromNodes[5]>>3)&1); /* Row 1 */
        tmpShift |= ((fromNodes[3]>>4)&1)<<2; /* Row 3 */
        coreGlobals.swMatrix[1] = tmpShift;

/* Column 2: Line 1 = LSB; Line 8 = MSB */
        tmpShift = 0;
        tmpShift |= ((fromNodes[1]>>1)&1); /* Row 1 */
        tmpShift |= ((fromNodes[1]>>3)&1)<<2; /* Row 3 */
        tmpShift |= ((fromNodes[1]>>6)&1)<<3; /* Row 4 */
        tmpShift |= ((fromNodes[1]>>5)&1)<<4; /* Row 5 */
        tmpShift |= ((fromNodes[2]>>2)&1)<<5; /* Row 6 */
        tmpShift |= ((fromNodes[2]>>3)&1)<<7; /* Row 8 */
        coreGlobals.swMatrix[2] = tmpShift;

/* Column 3 */
        tmpShift = 0;
        tmpShift |= ((fromNodes[4]>>1)&1); /* Row 1 */
        tmpShift |= ((fromNodes[1]>>0)&1)<<1; /* Row 2 */
        tmpShift |= ((fromNodes[1]>>4)&1)<<2; /* Row 3 */
        tmpShift |= ((fromNodes[1]>>2)&1)<<3; /* Row 4 */
        tmpShift |= ((fromNodes[0]>>7)&1)<<5; /* Row 6 */
        tmpShift |= ((fromNodes[2]>>4)&1)<<6; /* Row 7 */
        tmpShift |= ((fromNodes[0]>>6)&1)<<7; /* Row 8 */
        coreGlobals.swMatrix[3] = tmpShift;

/* Column 4 */
        tmpShift = 0;
        tmpShift |= ((fromNodes[3]>>5)&1); /* Row 1 */
        tmpShift |= ((fromNodes[3]>>6)&1)<<1; /* Row 2 */
        tmpShift |= ((fromNodes[3]>>7)&1)<<2; /* Row 3 */
        tmpShift |= ((fromNodes[2]>>0)&1)<<3; /* Row 4 */
        tmpShift |= ((fromNodes[2]>>1)&1)<<4; /* Row 5 */
        tmpShift |= ((fromNodes[0]>>5)&1)<<5; /* Row 6 */
        tmpShift |= ((fromNodes[0]>>4)&1)<<6; /* Row 7 */
        tmpShift |= ((fromNodes[0]>>3)&1)<<7; /* Row 8 */
        coreGlobals.swMatrix[4] = tmpShift;

/* Column 5 */
        tmpShift = 0;
        tmpShift |= ((fromNodes[3]>>1)&1); /* Row 1 */
        tmpShift |= ((fromNodes[1]>>7)&1)<<1; /* Row 2 */
        tmpShift |= ((fromNodes[2]>>5)&1)<<2; /* Row 3 */
        tmpShift |= ((fromNodes[2]>>7)&1)<<3; /* Row 4 */
        tmpShift |= ((fromNodes[2]>>6)&1)<<4; /* Row 5 */
        tmpShift |= ((fromNodes[5]>>1)&1)<<5; /* Row 6 */
        tmpShift |= ((fromNodes[5]>>0)&1)<<6; /* Row 7 */
        tmpShift |= ((fromNodes[3]>>2)&1)<<7; /* Row 8 */
        coreGlobals.swMatrix[5] = tmpShift;

/* Column 6 */
        tmpShift = 0;
        tmpShift |= ((fromNodes[4]>>2)&1); /* Row 1 */
        tmpShift |= ((fromNodes[4]>>3)&1)<<1; /* Row 2 */
        tmpShift |= ((fromNodes[4]>>4)&1)<<2; /* Row 3 */
        tmpShift |= ((fromNodes[4]>>0)&1)<<3; /* Row 4 */
        tmpShift |= ((fromNodes[5]>>7)&1)<<5; /* Row 6 */
        tmpShift |= ((fromNodes[5]>>6)&1)<<6; /* Row 7 */
        tmpShift |= ((fromNodes[5]>>5)&1)<<7; /* Row 8 */
        coreGlobals.swMatrix[6] = tmpShift;

/* Column 7 */
        tmpShift = 0;
        tmpShift |= ((fromNodes[4]>>7)&1); /* Row 1 */
        tmpShift |= ((fromNodes[4]>>6)&1)<<1; /* Row 2 */
        tmpShift |= ((fromNodes[4]>>5)&1)<<2; /* Row 3 */
        tmpShift |= ((fromNodes[5]>>2)&1)<<3; /* Row 4 */
        tmpShift |= ((fromNodes[0]>>0)&1)<<4; /* Row 5 */
        tmpShift |= ((fromNodes[0]>>1)&1)<<5; /* Row 6 */
        tmpShift |= ((fromNodes[3]>>0)&1)<<7; /* Row 8 */
        coreGlobals.swMatrix[7] = tmpShift;

/* Column 8 */
        tmpShift = 0;
        tmpShift |= ((fromNodes[0]>>2)&1)<<0; /* Row 1 */
        tmpShift |= ((fromNodes[3]>>3)&1)<<1; /* Row 2 */
        coreGlobals.swMatrix[8] = tmpShift;


    }

/* set outputs */

  tcflush(serbus, TCIOFLUSH);

/* Node 0 */
	toNode[0] = 0x07; /* node 0, 7 data bytes */

	/* Solenoids nr. in manual - 1 */
	tmpShift = 0;
	tmpShift = 1<<7; /* Enable right flipper; tmpShift |= ((coreGlobals.solenoids>>4)&1); */
	tmpShift |= ((coreGlobals.solenoids>>21)&1)<<6; /* magnet */
	tmpShift |= ((coreGlobals.solenoids>>0)&1)<<5; /* outhole kicker */
	tmpShift |= ((coreGlobals.solenoids>>1)&1)<<4; /* shooterlane feeder */
	tmpShift |= 1<<3; /* Enable right Slingshot tmpShift |= ((coreGlobals.solenoids>>15)&1)<<3;*/
	tmpShift |= ((coreGlobals.solenoids>>7)&1)<<2; /* lock diverter */
	tmpShift |= ((coreGlobals.solenoids>>13)&1)<<1; /* lock kickback */
	tmpShift |= ((coreGlobals.solenoids>>5)&1)<<0; /* targets */
	toNode[1] = tmpShift;


/* Template for GI(String) 1, 2, 3, 4, 5, nc, nc, nc */
	tmpShift = 0;
/*	tmpShift |= ((coreGlobals.solenoids2>>7)&1);*/
/*	tmpShift |= ((coreGlobals.solenoids2>>8)&1)<<1;*/
/*	tmpShift |= ((coreGlobals.solenoids2>>9)&1)<<2;*/
/*	tmpShift |= ((coreGlobals.gi[0]>>3)&1)<<3; */ /* if on coreGlobal.gi[0] = 8, off = 0 */
/*	tmpShift |= ((coreGlobals.gi[1]>>3)&1)<<4;*/
/*	tmpShift |= ((coreGlobals.lampMatrix[6]>>0)&1)<<6; nc */
/*	tmpShift |= ((coreGlobals.lampMatrix[7]>>0)&1)<<7; nc */
/*	toNode[5] = tmpShift; */


	/* Lamp[col-1]>>(row-1)  */
	tmpShift = 0;
	tmpShift |= 0;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<6;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<5;
	tmpShift |= ((coreGlobals.lampMatrix[4]>>2)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[4]>>1)&1)<<3;
	tmpShift |= ((coreGlobals.lampMatrix[4]>>0)&1)<<2;
	tmpShift |= 0;
	tmpShift |= ((coreGlobals.lampMatrix[2]>>5)&1);
	toNode[2] = tmpShift;

	/* Lamp[col-1]>>(row-1)  */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.lampMatrix[6]>>6)&1)<<7;
	tmpShift |= ((coreGlobals.lampMatrix[6]>>5)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[6]>>0)&1)<<5;
	tmpShift |= ((coreGlobals.lampMatrix[6]>>1)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[6]>>2)&1)<<3;
	tmpShift |= ((coreGlobals.lampMatrix[6]>>4)&1)<<2;
	tmpShift |= ((coreGlobals.lampMatrix[6]>>3)&1)<<1;
	tmpShift |= ((coreGlobals.solenoids>>23)&1);
	toNode[3] = tmpShift;

	/* Lamp[col-1]>>(row-1) */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.lampMatrix[2]>>7)&1)<<7;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<6;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<5;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<4;
	tmpShift |= 0;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<2;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<1;
	tmpShift |= ((coreGlobals.solenoids>>10)&1);
	toNode[4] = tmpShift;


	/* Lamp[col-1]>>(row-1) */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<7;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[5]>>7)&1)<<5;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[5]>>6)&1)<<3;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<2;
	tmpShift |= ((coreGlobals.lampMatrix[5]>>5)&1)<<1;
	tmpShift |= ((coreGlobals.solenoids>>10)&1);
	toNode[5] = tmpShift;

	/* Lamp[col-1]>>(row-1) */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<7;
	tmpShift |= ((coreGlobals.lampMatrix[5]>>3)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[5]>>2)&1)<<5;
	tmpShift |= ((coreGlobals.lampMatrix[5]>>1)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[5]>>0)&1)<<3;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<2;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<1;
	tmpShift |= ((coreGlobals.solenoids>>10)&1);
	toNode[6] = tmpShift;

	/* Lamp[col-1]>>(row-1) */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<7;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<6;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<5;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<4;
	toNode[7] = tmpShift;

	write(serbus, toNode, 8); /* 7 bytes + addr&length */

/* Node 1 */
	toNode[0] = 0x18; /* node 1, 8 data bytes */

	/* Solenoids:  nr. in manual - 1 */
	tmpShift = 0;
	tmpShift = 1<<7; /* Enable left flipper */
	tmpShift |= ((coreGlobals.solenoids>>19)&1)<<6; /* kick back */
	tmpShift |= 1<<5; /* enable lift sling */
	tmpShift |= ((coreGlobals.solenoids>>12)&1)<<4; /* upper kick back */
	tmpShift |= ((coreGlobals.solenoids>>3)&1)<<3; /* ball popper VOK */
	tmpShift |= 1<<2; /* enable bumper left */
	tmpShift |= 1<<1; /* enable bumper middle */
	tmpShift |= 1<<0; /* enable bumper right */
	toNode[1] = tmpShift;


/* Template for GI(String) 1, 2, 3, 4, 5, nc, nc, nc */
	tmpShift = 0;
/*	tmpShift |= ((coreGlobals.solenoids2>>7)&1);*/
/*	tmpShift |= ((coreGlobals.solenoids2>>8)&1)<<1;*/
/*	tmpShift |= ((coreGlobals.solenoids2>>9)&1)<<2;*/
/*	tmpShift |= ((coreGlobals.gi[0]>>3)&1)<<3; */ /* if on coreGlobal.gi[0] = 8, off = 0 */
/*	tmpShift |= ((coreGlobals.gi[1]>>3)&1)<<4;*/
/*	tmpShift |= ((coreGlobals.lampMatrix[6]>>0)&1)<<6; nc */
/*	tmpShift |= ((coreGlobals.lampMatrix[7]>>0)&1)<<7; nc */
/*	toNode[5] = tmpShift; */


	/* Lamp[col-1]>>(row-1)  */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.lampMatrix[4]>>7)&1)<<7;
	tmpShift |= ((coreGlobals.lampMatrix[5]>>4)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[6]>>7)&1)<<5;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<4;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<3;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<2;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<1;
	tmpShift |= ((coreGlobals.lampMatrix[2]>>2)&1)<<0;
	toNode[2] = tmpShift;

	/* Lamp[col-1]>>(row-1)  */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.lampMatrix[2]>>0)&1)<<7;
	tmpShift |= ((coreGlobals.lampMatrix[1]>>1)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[1]>>0)&1)<<5;
	tmpShift |= ((coreGlobals.lampMatrix[7]>>5)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[2]>>4)&1)<<3;
	tmpShift |= ((coreGlobals.lampMatrix[7]>>7)&1)<<2;
	tmpShift |= ((coreGlobals.lampMatrix[7]>>6)&1)<<1;
	tmpShift |= ((coreGlobals.solenoids>>10)&1);
	toNode[3] = tmpShift;

	/* Lamp[col-1]>>(row-1) */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<7;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[7]>>4)&1)<<5;
	tmpShift |= ((coreGlobals.lampMatrix[1]>>5)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[1]>>7)&1)<<3;
	tmpShift |= ((coreGlobals.lampMatrix[2]>>1)&1)<<2;
	tmpShift |= ((coreGlobals.lampMatrix[2]>>3)&1)<<1;
	tmpShift |= ((coreGlobals.lampMatrix[1]>>4)&1)<<0;
	toNode[4] = tmpShift;


	/* Lamp[col-1]>>(row-1) */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<7;
	tmpShift |= ((coreGlobals.lampMatrix[5]>>4)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[4]>>7)&1)<<5;
	tmpShift |= ((coreGlobals.lampMatrix[3]>>7)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[3]>>6)&1)<<3;
	tmpShift |= ((coreGlobals.lampMatrix[3]>>5)&1)<<2;
	tmpShift |= ((coreGlobals.lampMatrix[7]>>3)&1)<<1;
	tmpShift |= ((coreGlobals.lampMatrix[1]>>6)&1)<<0;
	toNode[5] = tmpShift;

	/* Lamp[col-1]>>(row-1) */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.lampMatrix[1]>>3)&1)<<7;
	tmpShift |= ((coreGlobals.lampMatrix[1]>>2)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[7]>>0)&1)<<5;
	tmpShift |= ((coreGlobals.lampMatrix[7]>>1)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[7]>>2)&1)<<3;
	tmpShift |= ((coreGlobals.lampMatrix[4]>>5)&1)<<2;
	tmpShift |= ((coreGlobals.lampMatrix[4]>>4)&1)<<1;
	tmpShift |= ((coreGlobals.lampMatrix[4]>>3)&1)<<0;
	toNode[6] = tmpShift;

	/* Lamp[col-1]>>(row-1) */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.lampMatrix[3]>>0)&1)<<7;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[3]>>1)&1)<<5;
	tmpShift |= ((coreGlobals.lampMatrix[3]>>2)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[3]>>3)&1)<<3;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<2;
	tmpShift |= ((coreGlobals.solenoids>>10)&1)<<1;
	tmpShift |= ((coreGlobals.lampMatrix[3]>>4)&1)<<0;
	toNode[7] = tmpShift;

	/* Lamp[col-1]>>(row-1) */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.lampMatrix[2]>>6)&1)<<7;
	tmpShift |= ((coreGlobals.lampMatrix[4]>>6)&1)<<6;
	toNode[8] = tmpShift;

	write(serbus, toNode, 9); /* 8 bytes + addr&length */

/* Node 2 */
	toNode[0] = 0x22; /* node 2, 2 data bytes */

	/* Solenoids nr. in manual - 1 */
	tmpShift = 0;
	tmpShift = 1<<7; /* Enable upper flipper */
	tmpShift |= ((coreGlobals.solenoids>>17)&1)<<6; /* ramp diverter */
	tmpShift |= ((coreGlobals.solenoids>>30)&1)<<5; /* Flasher 7 */
	tmpShift |= ((coreGlobals.solenoids>>29)&1)<<4; /* Flasher 6 */
	tmpShift |= ((coreGlobals.solenoids>>28)&1)<<3; /* Flasher 5 */
	tmpShift |= ((coreGlobals.solenoids>>27)&1)<<2; /* Flasher 4 */
	tmpShift |= ((coreGlobals.solenoids>>26)&1)<<1; /* Flasher 3 */
	tmpShift |= ((coreGlobals.solenoids>>25)&1)<<0; /* Flasher 2 */
	toNode[1] = tmpShift;



	/* Lamp[col-1]>>(row-1)  */
	tmpShift = 0;
	tmpShift |= ((coreGlobals.lampMatrix[0]>>0)&1)<<7;
	tmpShift |= ((coreGlobals.lampMatrix[0]>>1)&1)<<6;
	tmpShift |= ((coreGlobals.lampMatrix[0]>>2)&1)<<5;
	tmpShift |= ((coreGlobals.lampMatrix[0]>>3)&1)<<4;
	tmpShift |= ((coreGlobals.lampMatrix[0]>>4)&1)<<3;
	tmpShift |= ((coreGlobals.lampMatrix[0]>>5)&1)<<2;
	tmpShift |= ((coreGlobals.lampMatrix[0]>>6)&1)<<1;
	tmpShift |= ((coreGlobals.lampMatrix[0]>>7)&1)<<0;
	toNode[2] = tmpShift;

	write(serbus, toNode, 3); /* 2 bytes + addr&length */


/* when all data was sent, the special address 14 is used to 
 * make the nodes send their staus
 */
  toNode[0] = 0xe0;
  write(serbus, toNode, 1);  /* 1 byte (addr&length) */

} /* of if ((locals.vblankCount % 2) == 0)  */
#endif

}

/*---------------
/  Lamp handling
/----------------*/
static WRITE_HANDLER(pia1a_w) {
  core_setLamp(coreGlobals.tmpLampMatrix, locals.lampColumn, locals.lampRow = ~data);
}
static WRITE_HANDLER(pia1b_w) {
  core_setLamp(coreGlobals.tmpLampMatrix, locals.lampColumn = data, locals.lampRow);
}

/*-- Jumper W7 --*/
static READ_HANDLER (pia2a_r) { return core_getDip(0)<<7; }

/*-----------------
/ Display handling
/-----------------*/
/*NOTE: Data East DMD Games:     data = 0x01, CN1-Pin 7 (Strobe) goes low
				 data = 0x04, CN2-Pin 1 (Enable) goes low
			(currently we don't need to read these values)*/
static WRITE_HANDLER(pia2a_w) {
  locals.digSel = data & 0x0f;
  if (core_gameData->hw.display & S11_BCDDIAG)
    locals.diagnosticLed |= core_bcd2seg[(data & 0x70)>>4];
  else
    locals.diagnosticLed |= (data & 0x10)>>4;
}

static WRITE_HANDLER(pia2b_w) {
  /* Data East writes auxiliary solenoids here for DMD games
     CN3 Printer Data Lines (Used by various games)
       data = 0x01, CN3-Pin 9 (GNR Magnet 3, inverted for Star Trek 25th chase lights)
       data = 0x02, CN3-Pin 8 (GNR Magnet 2, -"-)
       data = 0x04, CN3-Pin 7 (GNR Magnet 1, -"-)
       ....
       data = 0x80, CN3-Pin 1 (Blinder on Tommy)*/
  if (core_gameData->gen & GEN_DEDMD16) {
    if (core_gameData->hw.gameSpecific1 & S11_PRINTERLINE) locals.extSol |= locals.extSolPulse = (data ^ 0xff);
  } else if (core_gameData->gen & (GEN_DEDMD32|GEN_DEDMD64)) {
    if (core_gameData->hw.gameSpecific1 & S11_PRINTERLINE) locals.extSol |= locals.extSolPulse = data;
  }
  else {
    if (core_gameData->hw.display & S11_DISPINV) data = ~data;
    if (core_gameData->hw.display & S11_BCDDISP) {
      locals.segments[locals.digSel].w |=
           locals.pseg[locals.digSel].w = core_bcd2seg[data&0x0f];
      locals.segments[20+locals.digSel].w |=
           locals.pseg[20+locals.digSel].w = core_bcd2seg[data>>4];
    }
    else
      locals.segments[20+locals.digSel].b.lo |=
          locals.pseg[20+locals.digSel].b.lo = data;
  }
}
static WRITE_HANDLER(pia5a_w) { // Not used for DMD
  if (core_gameData->hw.display & S11_DISPINV) data = ~data;
  if (core_gameData->hw.display & S11_LOWALPHA)
    locals.segments[20+locals.digSel].b.hi |=
         locals.pseg[20+locals.digSel].b.hi = data;
}
static WRITE_HANDLER(pia3a_w) {
  if (core_gameData->hw.display & S11_DISPINV) data = ~data;
  locals.segments[locals.digSel].b.hi |= locals.pseg[locals.digSel].b.hi = data;
}
static WRITE_HANDLER(pia3b_w) {
  if (core_gameData->hw.display & S11_DISPINV) data = ~data;
  locals.segments[locals.digSel].b.lo |= locals.pseg[locals.digSel].b.lo = data;
}

static READ_HANDLER(pia3b_dmd_r) {
  if (core_gameData->gen & GEN_DEDMD32)
    return (sndbrd_0_data_r(0) ? 0x80 : 0x00) | (sndbrd_0_ctrl_r(0)<<3);
  else if (core_gameData->gen & GEN_DEDMD64)
    return sndbrd_0_data_r(0) ? 0x80 : 0x00;
  return 0;
}

//NOTE: Unusued in Data East Alpha Games
static WRITE_HANDLER(pia2ca2_w) {
  data = data ? 0x80 : 0x00;
  if (core_gameData->gen & GEN_S9) {
    locals.segments[1+locals.digSel].b.lo |= data;
    locals.pseg[1+locals.digSel].b.lo = (locals.pseg[1+locals.digSel].b.lo & 0x7f) | data;
  } else {
    locals.segments[20+locals.digSel].b.lo |= data;
    locals.pseg[20+locals.digSel].b.lo = (locals.pseg[20+locals.digSel].b.lo & 0x7f) | data;
  }
}
//NOTE: Pin 10 of CN3 for Data East DMD Games (Currently we don't need to read this value)
static WRITE_HANDLER(pia2cb2_w) {
  data = data ? 0x80 : 0x00;
  if (core_gameData->gen & GEN_S9) {
    locals.segments[21+locals.digSel].b.lo |= data;
    locals.pseg[21+locals.digSel].b.lo = (locals.pseg[21+locals.digSel].b.lo & 0x7f) | data;
  } else {
    locals.segments[locals.digSel].b.lo |= data;
    locals.pseg[locals.digSel].b.lo = (locals.pseg[locals.digSel].b.lo & 0x7f) | data;
  }
}

static READ_HANDLER(pia5a_r) {
  if (core_gameData->gen & GEN_DEDMD64)
    return sndbrd_0_ctrl_r(0);
  else if (core_gameData->gen & GEN_DEDMD16)
    return (sndbrd_0_data_r(0) ? 0x01 : 0x00) | (sndbrd_0_ctrl_r(0)<<1);
  return 0;
}

/*------------
/  Solenoids
/-------------*/
static void setSSSol(int data, int solNo) {
                                    /*    WMS          DE */
  static const int ssSolNo[2][6] = {{5,4,1,2,0,3},{3,4,5,1,0,2}};
  int bit = CORE_SOLBIT(CORE_FIRSTSSSOL + ssSolNo[locals.deGame][solNo]);
  if (locals.ssEn & (~data & 1))
    { coreGlobals.pulsedSolState |= bit;  locals.solenoids |= bit; }
  else
    coreGlobals.pulsedSolState &= ~bit;
}

#ifdef FIXMUX

static void updsol(void) {
  /* set new solenoids, preserve SSSol */
  coreGlobals.pulsedSolState = (coreGlobals.pulsedSolState & 0x00ff0000)
                             | (locals.solBits2 << 8)
                             | (locals.solBits1     );

  /* if game has a MUX and it's active... */
  if ((core_gameData->sxx.muxSol) &&
      (coreGlobals.pulsedSolState & CORE_SOLBIT(core_gameData->sxx.muxSol))) {
    if (core_gameData->hw.gameSpecific1 & S11_RKMUX) /* special case WMS Road Kings */
      coreGlobals.pulsedSolState = (coreGlobals.pulsedSolState & 0x00ff8fef)      |
                                  ((coreGlobals.pulsedSolState & 0x00000010)<<20) |
                                  ((coreGlobals.pulsedSolState & 0x00007000)<<13);
    else
      coreGlobals.pulsedSolState = (coreGlobals.pulsedSolState & 0x00ffff00) |
                                   (coreGlobals.pulsedSolState << 24);
  }

  locals.solenoids |= coreGlobals.pulsedSolState;
}

static WRITE_HANDLER(pia0b_w) {
  // DataEast Playboy35th needs the MUX delayed my one IRQ:
  if (core_gameData->hw.gameSpecific1 & S11_MUXDELAY) {
    // new solbits are stored, previous solbits are processed
    UINT8 h            = locals.solBits2prv;
    locals.solBits2prv = data;
    data               = h;
  }
  if (data != locals.solBits2) {
    locals.solBits2 = data;
    updsol();
  }
}

static WRITE_HANDLER(latch2200) {
  if (data != locals.solBits1) {
    //fprintf(stderr,"\nPIA Data %d",data);
    locals.solBits1 = data;
    updsol();
  }
}

#else
static WRITE_HANDLER(pia0b_w) {
  coreGlobals.pulsedSolState = (coreGlobals.pulsedSolState & 0xffff00ff) | (data<<8);
  locals.solenoids |= (data<<8);
}
static WRITE_HANDLER(latch2200) {
  coreGlobals.pulsedSolState = (coreGlobals.pulsedSolState & 0xffffff00) | data;
  locals.solenoids |= data;
}
#endif

static WRITE_HANDLER(pia0cb2_w) { locals.ssEn = !data;}

static WRITE_HANDLER(pia1ca2_w) { setSSSol(data, 0); }
static WRITE_HANDLER(pia1cb2_w) { setSSSol(data, 1); }
static WRITE_HANDLER(pia3ca2_w) { setSSSol(data, 2); }
static WRITE_HANDLER(pia3cb2_w) { setSSSol(data, 3); }
static WRITE_HANDLER(pia4ca2_w) { setSSSol(data, 4); }
static WRITE_HANDLER(pia4cb2_w) { setSSSol(data, 5); }

/*---------------
/ Switch reading
/----------------*/
static WRITE_HANDLER(pia4b_w)
{
  if (locals.deGame) {
    int x;
    for (x=0; x<8; x++)
      if (data & (1u << x))
        break;
    locals.swCol = data & (1u << x);
  }
  else
    locals.swCol = data;
#if defined(LISY_SUPPORT)
  lisy_w_throttle();
#endif
}

static READ_HANDLER(pia4a_r)
{
#if defined(LISY_SUPPORT)
  //get the switches from LISY_mini
  lisy_w_switch_handler();
#endif
  return core_getSwCol(locals.swCol);
}

/*-------
/  Sound
/--------*/
/*-- Sound board sound command--*/
static WRITE_HANDLER(pia5b_w) {
  //Data East 128x16 games need to eat the 0xfe command (especially Hook)
  if ((core_gameData->gen & GEN_DEDMD16) && (data == 0xfe)) return;
  locals.sndCmd = data; sndbrd_1_data_w(0,data);
}

/*-- Sound board sound command available --*/
static WRITE_HANDLER(pia5cb2_w) {
  /* don't pass to sound board if a sound overlay board is available */
  if ((core_gameData->hw.gameSpecific1 & S11_SNDOVERLAY) &&
      ((locals.sndCmd & 0xe0) == 0)) {
    if (!data) locals.extSol |= locals.extSolPulse = (~locals.sndCmd) & 0x1f;
  }
  else sndbrd_1_ctrl_w(0,data);
}
/*-- reset sound board CPU --*/
static WRITE_HANDLER(pia5ca2_w) { /*
  if (core_gameData->gen & ~(GEN_S11B_3|GEN_S9)) {
    cpu_set_reset_line(S11_SCPU1NO, PULSE_LINE);
    s11cs_reset();
  } */
}
#ifndef PINMAME_NO_UNUSED	// currently unused function (GCC 3.4)
static WRITE_HANDLER(s11_sndCmd_w) {
  static int soundSys = -1; /* 0 = CPU board sound, 1 = Sound board */
  if (soundSys < 0)
    soundSys = (data & 0x01);
  else {
    sndbrd_data_w(soundSys, data); sndbrd_ctrl_w(soundSys,1); sndbrd_ctrl_w(soundSys,0);
    soundSys = -1;
  }
}
#endif

#ifndef PINMAME_NO_UNUSED	// currently unused function (GCC 3.4)
static WRITE_HANDLER(de_sndCmd_w) {
    sndbrd_data_w(1, data); sndbrd_ctrl_w(1,1); sndbrd_ctrl_w(1,0);
}
#endif

//NOTE: Not used for Data East
static WRITE_HANDLER(pia0ca2_w) { sndbrd_0_ctrl_w(0,data); }
//NOTE: Not used for Data East
static READ_HANDLER(pia5b_r) { return sndbrd_1_ctrl_r(0); }

static struct pia6821_interface s11_pia[] = {
{/* PIA 0 (2100) */
 /* PA0 - PA7 Sound Select Outputs (sound latch) */
 /* PB0 - PB7 Solenoid 9-16 (12 is usually for multiplexing) */
  /* CA2       Sound H.S.  */
 /* CB2       Enable Special Solenoids */
 /* in  : A/B,CA/B1,CA/B2 */ 0, 0, 0, 0, 0, 0,
 /* out : A/B,CA/B2       */ sndbrd_0_data_w, pia0b_w, pia0ca2_w, pia0cb2_w,
 /* irq : A/B             */ s11_piaMainIrq, s11_piaMainIrq
},{ /* PIA 1 (2400) */
 /* PA0 - PA7 Lamp Matrix Strobe */
 /* PB0 - PB7 Lamp Matrix Return */
 /* CA2       F SS6 */
 /* CB2       E SS5 */
 /* in  : A/B,CA/B1,CA/B2 */ 0, 0, 0, 0, 0, 0,
 /* out : A/B,CA/B2       */ pia1a_w, pia1b_w, pia1ca2_w, pia1cb2_w,
 /* irq : A/B             */ s11_piaMainIrq, s11_piaMainIrq
},{ /* PIA 2 (2800) */
 /* PA0 - PA3 Digit Select 1-16 */
 /* PA4       Diagnostic LED */
 /* PA5-PA6   NC */
 /* PA7       (I) Jumper W7 */
 /* PB0 - PB7 Digit BCD */
 /* CA1       (I) Diagnostic Advance */
 /* CB1       (I) Diagnostic Up/dn */
 /* CA2       Comma 3+4 */
 /* CB2       Comma 1+2 */
 /* in  : A/B,CA/B1,CA/B2 */ pia2a_r, 0, 0,0, 0, 0,
 /* out : A/B,CA/B2       */ pia2a_w, pia2b_w, pia2ca2_w, pia2cb2_w,
 /* irq : A/B             */ s11_piaMainIrq, s11_piaMainIrq
},{ /* PIA 3 (2c00) */
 /* PA0 - PA7 Display Data (h,j,k,m,n,p,r,dot) */
 /* PB0 - PB7 Display Data (a,b,c,d,e,f,g,com) */
 /* CA1       Widget I/O LCA1 */
 /* CB1       Widget I/O LCB1 */
 /* CA2       (I) B SST2 */
 /* CB2       (I) C SST3 */
 /* in  : A/B,CA/B1,CA/B2 */ 0, 0, 0, 0, 0, 0,
 /* out : A/B,CA/B2       */ pia3a_w, pia3b_w, pia3ca2_w, pia3cb2_w,
 /* irq : A/B             */ s11_piaMainIrq, s11_piaMainIrq
},{ /* PIA 4 (3000) */
 /* PA0 - PA7 Switch Input (row) */
 /* PB0 - PB7 Switch Drivers (column) */
 /* CA1/CB1   GND */
 /* CA2       (I) A SS1 */
 /* CB2       (I) D SS4 */
 /* in  : A/B,CA/B1,CA/B2 */ pia4a_r, 0, 0, 0, 0, 0,
 /* out : A/B,CA/B2       */ 0, pia4b_w, pia4ca2_w, pia4cb2_w,
 /* irq : A/B             */ s11_piaMainIrq, s11_piaMainIrq
},{ /* PIA 5 (3400) */
 /* PA0 - PA7 Display Data' (h,j,k ,m,n,p,r,dot), DMD status */
 /* PB0 - PB7 Widget I/O MD0-MD7 */
 /* CA1       Widget I/O MCA1 */
 /* CB1       Widget I/O MCB1 */
 /* CA2       Widget I/O MCA2 */
 /* CB2       Widget I/O MCB2 */
 /* in  : A/B,CA/B1,CA/B2 */ pia5a_r, pia5b_r, 0, 0, 0, 0,
 /* out : A/B,CA/B2       */ pia5a_w, pia5b_w, pia5ca2_w, pia5cb2_w,
 /* irq : A/B             */ s11_piaMainIrq, s11_piaMainIrq
},{ /* PIA 3 (2c00) for DMD games*/
 /* PA0 - PA7 DMD data */
 /* PB0 - PB7 DMD ctrl */
 /* CA1       NC */
 /* CB1       NC */
 /* CA2       (O) B SST2 */
 /* CB2       (O) C SST3 */
 /* in  : A/B,CA/B1,CA/B2 */ 0, pia3b_dmd_r, 0, 0, 0, 0,
 /* out : A/B,CA/B2       */ sndbrd_0_data_w, sndbrd_0_ctrl_w, pia3ca2_w, pia3cb2_w,
 /* irq : A/B             */ s11_piaMainIrq, s11_piaMainIrq
}
};

static SWITCH_UPDATE(s11) {
#ifdef PROC_SUPPORT
	// Go read the switches from the P-ROC
	if (coreGlobals.p_rocEn) 
		procGetSwitchEvents();
#endif


#ifndef LISY_SUPPORT
//if we have LISY, all switches come from LISY (Matrix[0] has e.g. ADVANCE Button!
  if (inports) {
    coreGlobals.swMatrix[0] = (inports[S11_COMINPORT] & 0x7f00)>>8;
    // All the matrix switches come from the P-ROC, so we only want to read
    // the first column from the keyboard if we are not using the P-ROC
#ifndef RTMPIN
#ifndef PROC_SUPPORT
    coreGlobals.swMatrix[1] = inports[S11_COMINPORT];
#endif
#endif
  }
#endif

  /*-- Generate interupts for diganostic keys --*/
  cpu_set_nmi_line(0, core_getSw(S11_SWCPUDIAG) ? ASSERT_LINE : CLEAR_LINE);
  sndbrd_0_diag(core_getSw(S11_SWSOUNDDIAG));
  if ((core_gameData->hw.gameSpecific1 & S11_MUXSW2) && core_gameData->sxx.muxSol)
    core_setSw(2, core_getSol(core_gameData->sxx.muxSol));
  if (locals.deGame) {
    /* Show Status of Black Advance Switch */
    core_textOutf(40,20,BLACK,core_getSw(DE_SWADVANCE) ? "B-Down" : "B-Up  ");
    /* Show Status of Green Up/Down Switch */
    core_textOutf(40,30,BLACK,core_getSw(DE_SWUPDN)    ? "G-Down" : "G-Up  ");
  }
  else {
    /* Show Status of Advance Switch */
    core_textOutf(40,20,BLACK,core_getSw(S11_SWADVANCE) ? "A-Down" : "A-Up  ");
    /* Show Status of Green Up/Down Switch */
    core_textOutf(40,30,BLACK,core_getSw(S11_SWUPDN)    ? "G-Down" : "G-Up  ");
  }
}

// convert lamp and switch numbers
// S11 is 1-64
// convert to 0-64 (+8)
// i.e. 1=8, 2=9...
#ifndef PINMAME_NO_UNUSED	// currently unused function (GCC 3.4)
static int s11_sw2m(int no) { return no+7; }
#endif
#ifdef PROC_SUPPORT
int s11_m2sw(int col, int row) { return col*8+row-7; } // needed to map
#endif

static MACHINE_INIT(s11) {
  if (core_gameData->gen & (GEN_DE | GEN_DEDMD16 | GEN_DEDMD32 | GEN_DEDMD64))
    locals.deGame = 1;
  else
    locals.deGame = 0;
  pia_config(S11_PIA0, PIA_STANDARD_ORDERING, &s11_pia[0]);
  pia_config(S11_PIA1, PIA_STANDARD_ORDERING, &s11_pia[1]);
  pia_config(S11_PIA2, PIA_STANDARD_ORDERING, &s11_pia[2]);
  pia_config(S11_PIA3, PIA_STANDARD_ORDERING, &s11_pia[3]);
  pia_config(S11_PIA4, PIA_STANDARD_ORDERING, &s11_pia[4]);
  pia_config(S11_PIA5, PIA_STANDARD_ORDERING, &s11_pia[5]);

  /*Additional hardware dependent init code*/
  switch (core_gameData->gen) {
    case GEN_S9:
      sndbrd_0_init(SNDBRD_S9S, 1, NULL, NULL, NULL);
      break;
    case GEN_S11:
      sndbrd_0_init(SNDBRD_S11S,  1, memory_region(S11S_ROMREGION), NULL, NULL);
      break;
    case GEN_S11X:
      sndbrd_0_init(SNDBRD_S11XS, 2, memory_region(S11XS_ROMREGION), NULL, NULL);
      sndbrd_1_init(SNDBRD_S11CS, 1, memory_region(S11CS_ROMREGION), pia_5_cb1_w, NULL);
      break;
    case GEN_S11B2:
      sndbrd_0_init(SNDBRD_S11BS, 2, memory_region(S11XS_ROMREGION), NULL, NULL);
      sndbrd_1_init(SNDBRD_S11JS, 1, memory_region(S11JS_ROMREGION), pia_5_cb1_w, NULL);
      break;
    case GEN_S11C:
      sndbrd_1_init(SNDBRD_S11CS, 1, memory_region(S11CS_ROMREGION), pia_5_cb1_w, NULL);
      break;
    case GEN_DE:
      sndbrd_1_init(SNDBRD_DE1S,  1, memory_region(DE1S_ROMREGION), pia_5_cb1_w, NULL);
      break;
    case GEN_DEDMD16:
    case GEN_DEDMD32:
    case GEN_DEDMD64:
      pia_config(S11_PIA3, PIA_STANDARD_ORDERING, &s11_pia[6]); // PIA 3 is different for DMD games
      sndbrd_0_init(core_gameData->hw.display,    2, memory_region(DE_DMD16ROMREGION),NULL,NULL);
      sndbrd_1_init(core_gameData->hw.soundBoard, 1, memory_region(DE1S_ROMREGION), pia_5_cb1_w, NULL);
      break;
  }


#ifdef RTMPIN

/* init RGB panel */
memset(&matrix_options, 0, sizeof(matrix_options));
	matrix_options.disable_hardware_pulsing = 1;
	matrix_options.hardware_mapping = "regular";
	matrix_options.rows = 32;
	matrix_options.cols = 64;
	matrix_options.chain_length = 2;
	matrix_options.parallel = 1;
	matrix_options.led_rgb_sequence = "RBG";
	matrix_options.limit_refresh_rate_hz = 60;
	matrix_options.panel_type = "FM6126A";

	runtime_options.gpio_slowdown = 4;

	/* matrix = led_matrix_create(32, 128, 1); */
	matrix = led_matrix_create_from_options_and_rt_options(&matrix_options, &runtime_options);
	if (matrix == NULL) {
		fprintf(stderr, "Failed to create matrix\n");
		exit(1);
	}

	offscreen_canvas = led_matrix_create_offscreen_canvas(matrix);
	if (offscreen_canvas == NULL)	{
		fprintf(stderr, "Failed to allocate LED canvas\n");
		exit(1);
	}
	int width, height;
	led_canvas_get_size(offscreen_canvas, &width, &height);
	printf("DMD width=%d height=%d\n", width, height);


/* init UART */
/* siehe https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#examples */

/*    char *portname = "/dev/serial0"; */
    char *portname = "/dev/ttyUSB0";

    /* open interface */
    serbus = open(portname, O_RDWR);

    if (serbus < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        exit(1);
    }


    struct termios tty;

    if (tcgetattr(serbus, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        exit(1);
    }
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
    tty.c_cflag &= ~CSIZE; // Clear all the size bits
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON; // disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 0;    // No blocking, return immediately with what is available.
    tty.c_cc[VMIN] = 0;
    // possible in Linux:B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    // Save tty settings, also checking for error
    if (tcsetattr(serbus, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
/* sleep bevore flush
 * see https://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
 */
    usleep (10000);
    tcflush(serbus, TCIOFLUSH);

#endif
}

static MACHINE_INIT(s9pf) {
  pia_config(S11_PIA0, PIA_STANDARD_ORDERING, &s11_pia[0]);
  pia_config(S11_PIA1, PIA_STANDARD_ORDERING, &s11_pia[1]);
  pia_config(S11_PIA2, PIA_STANDARD_ORDERING, &s11_pia[2]);
  pia_config(S11_PIA3, PIA_STANDARD_ORDERING, &s11_pia[3]);
  pia_config(S11_PIA4, PIA_STANDARD_ORDERING, &s11_pia[4]);
  pia_config(S11_PIA5, PIA_STANDARD_ORDERING, &s11_pia[5]);
  sndbrd_0_init(SNDBRD_S9S, 1, NULL, NULL, NULL);



}
static MACHINE_RESET(s11) {
  pia_reset();
}
static MACHINE_STOP(s11) {
  sndbrd_0_exit(); sndbrd_1_exit();

#ifdef RTMPIN
	close(serbus);
#endif


}

/*---------------------------
/  Memory map for main CPU
/----------------------------*/
static MEMORY_READ_START(s11_readmem)
  { 0x0000, 0x1fff, MRA_RAM},
  { 0x2100, 0x2103, pia_r(S11_PIA0) },
  { 0x2400, 0x2403, pia_r(S11_PIA1) },
  { 0x2800, 0x2803, pia_r(S11_PIA2) },
  { 0x2c00, 0x2c03, pia_r(S11_PIA3) },
  { 0x3000, 0x3003, pia_r(S11_PIA4) },
  { 0x3400, 0x3403, pia_r(S11_PIA5) },
  { 0x4000, 0xffff, MRA_ROM },
MEMORY_END

static MEMORY_WRITE_START(s11_writemem)
  { 0x0000, 0x1fff, MWA_RAM }, /* CMOS */
  { 0x2100, 0x2103, pia_w(S11_PIA0) },
  { 0x2200, 0x2200, latch2200},
  { 0x2400, 0x2403, pia_w(S11_PIA1) },
  { 0x2800, 0x2803, pia_w(S11_PIA2) },
  { 0x2c00, 0x2c03, pia_w(S11_PIA3) },
  { 0x3000, 0x3003, pia_w(S11_PIA4) },
  { 0x3400, 0x3403, pia_w(S11_PIA5) },
  { 0x4000, 0xffff, MWA_ROM },
MEMORY_END

/*-----------------
/  Machine drivers
/------------------*/
static MACHINE_DRIVER_START(s11)
  MDRV_IMPORT_FROM(PinMAME)
  MDRV_CORE_INIT_RESET_STOP(s11,s11,s11)
  MDRV_CPU_ADD(M6808, 1000000) // Early system 11 games used a M6802
  MDRV_CPU_MEMORY(s11_readmem, s11_writemem)
  MDRV_CPU_VBLANK_INT(s11_vblank, 1)
  MDRV_CPU_PERIODIC_INT(s11_irq, S11_IRQFREQ)
  MDRV_DIPS(1) /* (actually a jumper) */
  MDRV_SWITCH_UPDATE(s11)
MACHINE_DRIVER_END

/* System 9 */
MACHINE_DRIVER_START(s11_s9S)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(wmssnd_s9s)
  MDRV_NVRAM_HANDLER(s11)
  MDRV_DIAGNOSTIC_LED7
  MDRV_SOUND_CMD(s11_sndCmd_w)
  MDRV_SOUND_CMDHEADING("s11")
MACHINE_DRIVER_END

/* System 11 with S11C sound board, diagnostic digit */
MACHINE_DRIVER_START(s11_s11XS)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(wmssnd_s11xs)
  MDRV_NVRAM_HANDLER(s11)
  MDRV_DIAGNOSTIC_LED7
  MDRV_SOUND_CMD(s11_sndCmd_w)
  MDRV_SOUND_CMDHEADING("s11")
MACHINE_DRIVER_END

/* Pennant Fever */
MACHINE_DRIVER_START(s11_s9PS)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(wmssnd_s9ps)
  MDRV_CORE_INIT_RESET_STOP(s9pf,s11,s11)
  MDRV_NVRAM_HANDLER(s11)
  MDRV_DIAGNOSTIC_LED7
  MDRV_SOUND_CMD(s11_sndCmd_w)
  MDRV_SOUND_CMDHEADING("s11")
MACHINE_DRIVER_END

/* System 11 with S11C sound board, diagnostic LED only */
MACHINE_DRIVER_START(s11_s11XSL)
  MDRV_IMPORT_FROM(s11_s11XS)
  MDRV_DIAGNOSTIC_LEDH(1)
MACHINE_DRIVER_END

/* System 11a without external sound board*/
MACHINE_DRIVER_START(s11_s11S)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(wmssnd_s11s)
  MDRV_NVRAM_HANDLER(s11)
  MDRV_DIAGNOSTIC_LEDH(1)
  MDRV_SOUND_CMD(s11_sndCmd_w)
  MDRV_SOUND_CMDHEADING("s11")
MACHINE_DRIVER_END

/* System 11a with S11C sound board */
MACHINE_DRIVER_START(s11_s11aS)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(wmssnd_s11xs)
  MDRV_NVRAM_HANDLER(s11)
  MDRV_DIAGNOSTIC_LEDH(1)
  MDRV_SOUND_CMD(s11_sndCmd_w)
  MDRV_SOUND_CMDHEADING("s11")
MACHINE_DRIVER_END

/* System 11B with Jokerz! sound board*/
MACHINE_DRIVER_START(s11_s11b2S)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(wmssnd_s11b2s)
  MDRV_NVRAM_HANDLER(s11)
  MDRV_DIAGNOSTIC_LEDH(1)
  MDRV_SOUND_CMD(s11_sndCmd_w)
  MDRV_SOUND_CMDHEADING("s11")
MACHINE_DRIVER_END

/* System 11C */
MACHINE_DRIVER_START(s11_s11cS)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(wmssnd_s11cs)
  MDRV_NVRAM_HANDLER(s11)
  MDRV_DIAGNOSTIC_LEDH(1)
  MDRV_SOUND_CMD(s11_sndCmd_w)
  MDRV_SOUND_CMDHEADING("s11")
MACHINE_DRIVER_END

/* DE alpa numeric No Sound */
MACHINE_DRIVER_START(de_a)
  MDRV_IMPORT_FROM(s11)
  MDRV_NVRAM_HANDLER(de)
  MDRV_DIAGNOSTIC_LEDH(1)
MACHINE_DRIVER_END

/* DE alphanumeric Sound 1 */
MACHINE_DRIVER_START(de_a1S)
  MDRV_IMPORT_FROM(de_a)
  MDRV_IMPORT_FROM(de1s)
  MDRV_SOUND_CMD(de_sndCmd_w)
  MDRV_SOUND_CMDHEADING("DE")
MACHINE_DRIVER_END

/* DE 128x16 Sound 1 */
MACHINE_DRIVER_START(de_dmd161S)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(de1s)
  MDRV_IMPORT_FROM(de_dmd16)
  MDRV_NVRAM_HANDLER(de)
  MDRV_DIAGNOSTIC_LEDH(1)
  MDRV_SOUND_CMD(de_sndCmd_w)
  MDRV_SOUND_CMDHEADING("DE")
MACHINE_DRIVER_END

/* DE 128x16 Sound 2a */
MACHINE_DRIVER_START(de_dmd162aS)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(de2aas)
  MDRV_IMPORT_FROM(de_dmd16)
  MDRV_NVRAM_HANDLER(de)
  MDRV_DIAGNOSTIC_LEDH(1)
  MDRV_SOUND_CMD(de_sndCmd_w)
  MDRV_SOUND_CMDHEADING("DE")
MACHINE_DRIVER_END

/* DE 128x32 Sound 2a with hacky/tweaked DMD IRQ (LW3/AS/R&B/SW) */
MACHINE_DRIVER_START(de_dmd322aS_hack)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(de2aas)
  MDRV_IMPORT_FROM(de_dmd32_hack)
  MDRV_NVRAM_HANDLER(de)
  MDRV_DIAGNOSTIC_LEDH(1)
  MDRV_SOUND_CMD(de_sndCmd_w)
  MDRV_SOUND_CMDHEADING("DE")
MACHINE_DRIVER_END

/* DE 128x32 Sound 2a */
MACHINE_DRIVER_START(de_dmd322aS)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(de2aas)
  MDRV_IMPORT_FROM(de_dmd32)
  MDRV_NVRAM_HANDLER(de)
  MDRV_DIAGNOSTIC_LEDH(1)
  MDRV_SOUND_CMD(de_sndCmd_w)
  MDRV_SOUND_CMDHEADING("DE")
MACHINE_DRIVER_END

/* DE 192x64 Sound 2a */
MACHINE_DRIVER_START(de_dmd642aS)
  MDRV_IMPORT_FROM(s11)
  MDRV_IMPORT_FROM(de2aas)
  MDRV_IMPORT_FROM(de_dmd64)
  MDRV_NVRAM_HANDLER(de)
  MDRV_DIAGNOSTIC_LEDH(1)
  MDRV_SOUND_CMD(de_sndCmd_w)
  MDRV_SOUND_CMDHEADING("DE")
MACHINE_DRIVER_END

/*-----------------------------------------------
/ Load/Save static ram
/-------------------------------------------------*/
static NVRAM_HANDLER(s11) {
  core_nvram(file, read_or_write, memory_region(S11_CPUREGION), 0x0800, 0xff);
}
static NVRAM_HANDLER(de) {
  core_nvram(file, read_or_write, memory_region(S11_CPUREGION), 0x2000, 0xff);
}
