/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) yzh_expl_vde_adj_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_c1 CASADI_PREFIX(c1)
#define casadi_c2 CASADI_PREFIX(c2)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[18] = {17, 1, 0, 14, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

static const casadi_real casadi_c0[4] = {-5.0000000000000003e-02, 5.0000000000000003e-02, -5.0000000000000003e-02, 5.0000000000000003e-02};
static const casadi_real casadi_c1[4] = {2.8284271247461905e-02, -2.8284271247461905e-02, -2.8284271247461905e-02, 2.8284271247461905e-02};
static const casadi_real casadi_c2[4] = {-2.8284271247461905e-02, -2.8284271247461905e-02, 2.8284271247461905e-02, 2.8284271247461905e-02};

/* yzh_expl_vde_adj:(i0[13],i1[13],i2[4],i3[])->(o0[17x1,14nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real *w0=w+4, *w1=w+8, w2, *w3=w+13, *w4=w+22, *w5=w+35, *w6=w+38, *w7=w+42, w8, w9, w10, w11, w12, w13, w14, w15, w16, *w17=w+54, *w18=w+58, *w19=w+61, *w20=w+70, *w21=w+73, w22, w23, w24, w25, w26, w27, w28, w29, *w30=w+84, *w31=w+88, *w32=w+92, *w33=w+96, *w34=w+100, *w35=w+116;
  /* #0: @0 = zeros(4x1) */
  casadi_clear(w0, 4);
  /* #1: @1 = input[0][1] */
  casadi_copy(arg[0] ? arg[0]+3 : 0, 4, w1);
  /* #2: @2 = @1[1] */
  for (rr=(&w2), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #3: @3 = zeros(3x3) */
  casadi_clear(w3, 9);
  /* #4: @4 = input[1][0] */
  casadi_copy(arg[1], 13, w4);
  /* #5: {@5, @6, @7, @8, @9, @10} = vertsplit(@4) */
  casadi_copy(w4, 3, w5);
  casadi_copy(w4+3, 4, w6);
  casadi_copy(w4+7, 3, w7);
  w8 = w4[10];
  w9 = w4[11];
  w10 = w4[12];
  /* #6: @11 = 0 */
  w11 = 0.;
  /* #7: @12 = 0 */
  w12 = 0.;
  /* #8: @13 = input[2][0] */
  w13 = arg[2] ? arg[2][0] : 0;
  /* #9: @14 = input[2][1] */
  w14 = arg[2] ? arg[2][1] : 0;
  /* #10: @15 = input[2][2] */
  w15 = arg[2] ? arg[2][2] : 0;
  /* #11: @16 = input[2][3] */
  w16 = arg[2] ? arg[2][3] : 0;
  /* #12: @17 = vertcat(@13, @14, @15, @16) */
  rr=w17;
  *rr++ = w13;
  *rr++ = w14;
  *rr++ = w15;
  *rr++ = w16;
  /* #13: @17 = (2.*@17) */
  for (i=0, rr=w17, cs=w17; i<4; ++i) *rr++ = (2.* *cs++ );
  /* #14: @13 = @17[0] */
  for (rr=(&w13), ss=w17+0; ss!=w17+1; ss+=1) *rr++ = *ss;
  /* #15: @14 = @17[1] */
  for (rr=(&w14), ss=w17+1; ss!=w17+2; ss+=1) *rr++ = *ss;
  /* #16: @13 = (@13+@14) */
  w13 += w14;
  /* #17: @14 = @17[2] */
  for (rr=(&w14), ss=w17+2; ss!=w17+3; ss+=1) *rr++ = *ss;
  /* #18: @13 = (@13+@14) */
  w13 += w14;
  /* #19: @14 = @17[3] */
  for (rr=(&w14), ss=w17+3; ss!=w17+4; ss+=1) *rr++ = *ss;
  /* #20: @13 = (@13+@14) */
  w13 += w14;
  /* #21: @18 = vertcat(@11, @12, @13) */
  rr=w18;
  *rr++ = w11;
  *rr++ = w12;
  *rr++ = w13;
  /* #22: @11 = 0.083 */
  w11 = 8.3000000000000004e-02;
  /* #23: @18 = (@18/@11) */
  for (i=0, rr=w18; i<3; ++i) (*rr++) /= w11;
  /* #24: @18 = @18' */
  /* #25: @3 = mac(@7,@18,@3) */
  for (i=0, rr=w3; i<3; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w7+j, tt=w18+i*1; k<1; ++k) *rr += ss[k*3]**tt++;
  /* #26: @19 = @3' */
  for (i=0, rr=w19, cs=w3; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #27: {@18, @20, @21} = horzsplit(@19) */
  casadi_copy(w19, 3, w18);
  casadi_copy(w19+3, 3, w20);
  casadi_copy(w19+6, 3, w21);
  /* #28: @21 = @21' */
  /* #29: {@11, @12, @13} = horzsplit(@21) */
  w11 = w21[0];
  w12 = w21[1];
  w13 = w21[2];
  /* #30: @12 = (2.*@12) */
  w12 = (2.* w12 );
  /* #31: @14 = (@2*@12) */
  w14  = (w2*w12);
  /* #32: @15 = @1[2] */
  for (rr=(&w15), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #33: @11 = (2.*@11) */
  w11 = (2.* w11 );
  /* #34: @16 = (@15*@11) */
  w16  = (w15*w11);
  /* #35: @14 = (@14-@16) */
  w14 -= w16;
  /* #36: @20 = @20' */
  /* #37: {@16, @22, @23} = horzsplit(@20) */
  w16 = w20[0];
  w22 = w20[1];
  w23 = w20[2];
  /* #38: @23 = (2.*@23) */
  w23 = (2.* w23 );
  /* #39: @24 = (@2*@23) */
  w24  = (w2*w23);
  /* #40: @14 = (@14-@24) */
  w14 -= w24;
  /* #41: @24 = @1[3] */
  for (rr=(&w24), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #42: @16 = (2.*@16) */
  w16 = (2.* w16 );
  /* #43: @25 = (@24*@16) */
  w25  = (w24*w16);
  /* #44: @14 = (@14+@25) */
  w14 += w25;
  /* #45: @18 = @18' */
  /* #46: {@25, @26, @27} = horzsplit(@18) */
  w25 = w18[0];
  w26 = w18[1];
  w27 = w18[2];
  /* #47: @27 = (2.*@27) */
  w27 = (2.* w27 );
  /* #48: @28 = (@15*@27) */
  w28  = (w15*w27);
  /* #49: @14 = (@14+@28) */
  w14 += w28;
  /* #50: @26 = (2.*@26) */
  w26 = (2.* w26 );
  /* #51: @28 = (@24*@26) */
  w28  = (w24*w26);
  /* #52: @14 = (@14-@28) */
  w14 -= w28;
  /* #53: (@0[0] += @14) */
  for (rr=w0+0, ss=(&w14); rr!=w0+1; rr+=1) *rr += *ss++;
  /* #54: @14 = (2.*@2) */
  w14 = (2.* w2 );
  /* #55: @13 = (-@13) */
  w13 = (- w13 );
  /* #56: @13 = (2.*@13) */
  w13 = (2.* w13 );
  /* #57: @14 = (@14*@13) */
  w14 *= w13;
  /* #58: @28 = @1[0] */
  for (rr=(&w28), ss=w1+0; ss!=w1+1; ss+=1) *rr++ = *ss;
  /* #59: @29 = (@28*@12) */
  w29  = (w28*w12);
  /* #60: @14 = (@14+@29) */
  w14 += w29;
  /* #61: @29 = (@24*@11) */
  w29  = (w24*w11);
  /* #62: @14 = (@14+@29) */
  w14 += w29;
  /* #63: @29 = (@28*@23) */
  w29  = (w28*w23);
  /* #64: @14 = (@14-@29) */
  w14 -= w29;
  /* #65: @29 = (2.*@2) */
  w29 = (2.* w2 );
  /* #66: @22 = (-@22) */
  w22 = (- w22 );
  /* #67: @22 = (2.*@22) */
  w22 = (2.* w22 );
  /* #68: @29 = (@29*@22) */
  w29 *= w22;
  /* #69: @14 = (@14+@29) */
  w14 += w29;
  /* #70: @29 = (@15*@16) */
  w29  = (w15*w16);
  /* #71: @14 = (@14+@29) */
  w14 += w29;
  /* #72: @29 = (@24*@27) */
  w29  = (w24*w27);
  /* #73: @14 = (@14+@29) */
  w14 += w29;
  /* #74: @29 = (@15*@26) */
  w29  = (w15*w26);
  /* #75: @14 = (@14+@29) */
  w14 += w29;
  /* #76: (@0[1] += @14) */
  for (rr=w0+1, ss=(&w14); rr!=w0+2; rr+=1) *rr += *ss++;
  /* #77: @14 = (@15*@12) */
  w14  = (w15*w12);
  /* #78: @29 = (@2*@11) */
  w29  = (w2*w11);
  /* #79: @14 = (@14+@29) */
  w14 += w29;
  /* #80: @29 = (@15*@23) */
  w29  = (w15*w23);
  /* #81: @14 = (@14+@29) */
  w14 += w29;
  /* #82: @29 = (2.*@24) */
  w29 = (2.* w24 );
  /* #83: @29 = (@29*@22) */
  w29 *= w22;
  /* #84: @14 = (@14+@29) */
  w14 += w29;
  /* #85: @29 = (@28*@16) */
  w29  = (w28*w16);
  /* #86: @14 = (@14+@29) */
  w14 += w29;
  /* #87: @29 = (@2*@27) */
  w29  = (w2*w27);
  /* #88: @14 = (@14+@29) */
  w14 += w29;
  /* #89: @29 = (@28*@26) */
  w29  = (w28*w26);
  /* #90: @14 = (@14-@29) */
  w14 -= w29;
  /* #91: @29 = (2.*@24) */
  w29 = (2.* w24 );
  /* #92: @25 = (-@25) */
  w25 = (- w25 );
  /* #93: @25 = (2.*@25) */
  w25 = (2.* w25 );
  /* #94: @29 = (@29*@25) */
  w29 *= w25;
  /* #95: @14 = (@14+@29) */
  w14 += w29;
  /* #96: (@0[3] += @14) */
  for (rr=w0+3, ss=(&w14); rr!=w0+4; rr+=1) *rr += *ss++;
  /* #97: @14 = (2.*@15) */
  w14 = (2.* w15 );
  /* #98: @14 = (@14*@13) */
  w14 *= w13;
  /* #99: @12 = (@24*@12) */
  w12  = (w24*w12);
  /* #100: @14 = (@14+@12) */
  w14 += w12;
  /* #101: @11 = (@28*@11) */
  w11  = (w28*w11);
  /* #102: @14 = (@14-@11) */
  w14 -= w11;
  /* #103: @23 = (@24*@23) */
  w23  = (w24*w23);
  /* #104: @14 = (@14+@23) */
  w14 += w23;
  /* #105: @16 = (@2*@16) */
  w16  = (w2*w16);
  /* #106: @14 = (@14+@16) */
  w14 += w16;
  /* #107: @27 = (@28*@27) */
  w27  = (w28*w27);
  /* #108: @14 = (@14+@27) */
  w14 += w27;
  /* #109: @26 = (@2*@26) */
  w26  = (w2*w26);
  /* #110: @14 = (@14+@26) */
  w14 += w26;
  /* #111: @26 = (2.*@15) */
  w26 = (2.* w15 );
  /* #112: @26 = (@26*@25) */
  w26 *= w25;
  /* #113: @14 = (@14+@26) */
  w14 += w26;
  /* #114: (@0[2] += @14) */
  for (rr=w0+2, ss=(&w14); rr!=w0+3; rr+=1) *rr += *ss++;
  /* #115: @17 = zeros(4x1) */
  casadi_clear(w17, 4);
  /* #116: @14 = 0 */
  w14 = 0.;
  /* #117: @18 = input[0][3] */
  casadi_copy(arg[0] ? arg[0]+10 : 0, 3, w18);
  /* #118: @26 = @18[0] */
  for (rr=(&w26), ss=w18+0; ss!=w18+1; ss+=1) *rr++ = *ss;
  /* #119: @26 = (-@26) */
  w26 = (- w26 );
  /* #120: @25 = @18[1] */
  for (rr=(&w25), ss=w18+1; ss!=w18+2; ss+=1) *rr++ = *ss;
  /* #121: @25 = (-@25) */
  w25 = (- w25 );
  /* #122: @27 = @18[2] */
  for (rr=(&w27), ss=w18+2; ss!=w18+3; ss+=1) *rr++ = *ss;
  /* #123: @27 = (-@27) */
  w27 = (- w27 );
  /* #124: @30 = horzcat(@14, @26, @25, @27) */
  rr=w30;
  *rr++ = w14;
  *rr++ = w26;
  *rr++ = w25;
  *rr++ = w27;
  /* #125: @30 = @30' */
  /* #126: @14 = @18[0] */
  for (rr=(&w14), ss=w18+0; ss!=w18+1; ss+=1) *rr++ = *ss;
  /* #127: @26 = 0 */
  w26 = 0.;
  /* #128: @25 = @18[2] */
  for (rr=(&w25), ss=w18+2; ss!=w18+3; ss+=1) *rr++ = *ss;
  /* #129: @27 = @18[1] */
  for (rr=(&w27), ss=w18+1; ss!=w18+2; ss+=1) *rr++ = *ss;
  /* #130: @27 = (-@27) */
  w27 = (- w27 );
  /* #131: @31 = horzcat(@14, @26, @25, @27) */
  rr=w31;
  *rr++ = w14;
  *rr++ = w26;
  *rr++ = w25;
  *rr++ = w27;
  /* #132: @31 = @31' */
  /* #133: @14 = @18[1] */
  for (rr=(&w14), ss=w18+1; ss!=w18+2; ss+=1) *rr++ = *ss;
  /* #134: @26 = @18[2] */
  for (rr=(&w26), ss=w18+2; ss!=w18+3; ss+=1) *rr++ = *ss;
  /* #135: @26 = (-@26) */
  w26 = (- w26 );
  /* #136: @25 = 0 */
  w25 = 0.;
  /* #137: @27 = @18[0] */
  for (rr=(&w27), ss=w18+0; ss!=w18+1; ss+=1) *rr++ = *ss;
  /* #138: @32 = horzcat(@14, @26, @25, @27) */
  rr=w32;
  *rr++ = w14;
  *rr++ = w26;
  *rr++ = w25;
  *rr++ = w27;
  /* #139: @32 = @32' */
  /* #140: @14 = @18[2] */
  for (rr=(&w14), ss=w18+2; ss!=w18+3; ss+=1) *rr++ = *ss;
  /* #141: @26 = @18[1] */
  for (rr=(&w26), ss=w18+1; ss!=w18+2; ss+=1) *rr++ = *ss;
  /* #142: @25 = @18[0] */
  for (rr=(&w25), ss=w18+0; ss!=w18+1; ss+=1) *rr++ = *ss;
  /* #143: @25 = (-@25) */
  w25 = (- w25 );
  /* #144: @27 = 0 */
  w27 = 0.;
  /* #145: @33 = horzcat(@14, @26, @25, @27) */
  rr=w33;
  *rr++ = w14;
  *rr++ = w26;
  *rr++ = w25;
  *rr++ = w27;
  /* #146: @33 = @33' */
  /* #147: @34 = horzcat(@30, @31, @32, @33) */
  rr=w34;
  for (i=0, cs=w30; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w31; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w32; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w33; i<4; ++i) *rr++ = *cs++;
  /* #148: @14 = 0.5 */
  w14 = 5.0000000000000000e-01;
  /* #149: @6 = (@14*@6) */
  for (i=0, rr=w6, cs=w6; i<4; ++i) (*rr++)  = (w14*(*cs++));
  /* #150: @17 = mac(@34,@6,@17) */
  for (i=0, rr=w17; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w34+j, tt=w6+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #151: @0 = (@0+@17) */
  for (i=0, rr=w0, cs=w17; i<4; ++i) (*rr++) += (*cs++);
  /* #152: output[0][0] = @0 */
  casadi_copy(w0, 4, res[0]);
  /* #153: output[0][1] = @5 */
  if (res[0]) casadi_copy(w5, 3, res[0]+4);
  /* #154: @5 = zeros(3x1) */
  casadi_clear(w5, 3);
  /* #155: @34 = zeros(4x4) */
  casadi_clear(w34, 16);
  /* #156: @1 = @1' */
  /* #157: @34 = mac(@6,@1,@34) */
  for (i=0, rr=w34; i<4; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w6+j, tt=w1+i*1; k<1; ++k) *rr += ss[k*4]**tt++;
  /* #158: @35 = @34' */
  for (i=0, rr=w35, cs=w34; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #159: {@6, @1, @0, @17} = horzsplit(@35) */
  casadi_copy(w35, 4, w6);
  casadi_copy(w35+4, 4, w1);
  casadi_copy(w35+8, 4, w0);
  casadi_copy(w35+12, 4, w17);
  /* #160: @17 = @17' */
  /* #161: {@14, @26, @25, NULL} = horzsplit(@17) */
  w14 = w17[0];
  w26 = w17[1];
  w25 = w17[2];
  /* #162: @25 = (-@25) */
  w25 = (- w25 );
  /* #163: (@5[0] += @25) */
  for (rr=w5+0, ss=(&w25); rr!=w5+1; rr+=1) *rr += *ss++;
  /* #164: (@5[1] += @26) */
  for (rr=w5+1, ss=(&w26); rr!=w5+2; rr+=1) *rr += *ss++;
  /* #165: (@5[2] += @14) */
  for (rr=w5+2, ss=(&w14); rr!=w5+3; rr+=1) *rr += *ss++;
  /* #166: @0 = @0' */
  /* #167: {@14, @26, NULL, @25} = horzsplit(@0) */
  w14 = w0[0];
  w26 = w0[1];
  w25 = w0[3];
  /* #168: (@5[0] += @25) */
  for (rr=w5+0, ss=(&w25); rr!=w5+1; rr+=1) *rr += *ss++;
  /* #169: @26 = (-@26) */
  w26 = (- w26 );
  /* #170: (@5[2] += @26) */
  for (rr=w5+2, ss=(&w26); rr!=w5+3; rr+=1) *rr += *ss++;
  /* #171: (@5[1] += @14) */
  for (rr=w5+1, ss=(&w14); rr!=w5+2; rr+=1) *rr += *ss++;
  /* #172: @1 = @1' */
  /* #173: {@14, NULL, @26, @25} = horzsplit(@1) */
  w14 = w1[0];
  w26 = w1[2];
  w25 = w1[3];
  /* #174: @25 = (-@25) */
  w25 = (- w25 );
  /* #175: (@5[1] += @25) */
  for (rr=w5+1, ss=(&w25); rr!=w5+2; rr+=1) *rr += *ss++;
  /* #176: (@5[2] += @26) */
  for (rr=w5+2, ss=(&w26); rr!=w5+3; rr+=1) *rr += *ss++;
  /* #177: (@5[0] += @14) */
  for (rr=w5+0, ss=(&w14); rr!=w5+1; rr+=1) *rr += *ss++;
  /* #178: @6 = @6' */
  /* #179: {NULL, @14, @26, @25} = horzsplit(@6) */
  w14 = w6[1];
  w26 = w6[2];
  w25 = w6[3];
  /* #180: @25 = (-@25) */
  w25 = (- w25 );
  /* #181: (@5[2] += @25) */
  for (rr=w5+2, ss=(&w25); rr!=w5+3; rr+=1) *rr += *ss++;
  /* #182: @26 = (-@26) */
  w26 = (- w26 );
  /* #183: (@5[1] += @26) */
  for (rr=w5+1, ss=(&w26); rr!=w5+2; rr+=1) *rr += *ss++;
  /* #184: @14 = (-@14) */
  w14 = (- w14 );
  /* #185: (@5[0] += @14) */
  for (rr=w5+0, ss=(&w14); rr!=w5+1; rr+=1) *rr += *ss++;
  /* #186: output[0][2] = @5 */
  if (res[0]) casadi_copy(w5, 3, res[0]+7);
  /* #187: @14 = 628.931 */
  w14 = 6.2893081761006283e+02;
  /* #188: @14 = (@14*@10) */
  w14 *= w10;
  /* #189: @6 = [-0.05, 0.05, -0.05, 0.05] */
  casadi_copy(casadi_c0, 4, w6);
  /* #190: @6 = @6' */
  /* #191: @6 = (@14*@6) */
  for (i=0, rr=w6, cs=w6; i<4; ++i) (*rr++)  = (w14*(*cs++));
  /* #192: @6 = @6' */
  /* #193: @14 = 628.931 */
  w14 = 6.2893081761006283e+02;
  /* #194: @14 = (@14*@9) */
  w14 *= w9;
  /* #195: @1 = [0.0282843, -0.0282843, -0.0282843, 0.0282843] */
  casadi_copy(casadi_c1, 4, w1);
  /* #196: @1 = @1' */
  /* #197: @1 = (@14*@1) */
  for (i=0, rr=w1, cs=w1; i<4; ++i) (*rr++)  = (w14*(*cs++));
  /* #198: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<4; ++i) *rr++ = (- *cs++ );
  /* #199: @1 = @1' */
  /* #200: @6 = (@6+@1) */
  for (i=0, rr=w6, cs=w1; i<4; ++i) (*rr++) += (*cs++);
  /* #201: @14 = 628.931 */
  w14 = 6.2893081761006283e+02;
  /* #202: @14 = (@14*@8) */
  w14 *= w8;
  /* #203: @1 = [-0.0282843, -0.0282843, 0.0282843, 0.0282843] */
  casadi_copy(casadi_c2, 4, w1);
  /* #204: @1 = @1' */
  /* #205: @1 = (@14*@1) */
  for (i=0, rr=w1, cs=w1; i<4; ++i) (*rr++)  = (w14*(*cs++));
  /* #206: @1 = @1' */
  /* #207: @6 = (@6+@1) */
  for (i=0, rr=w6, cs=w1; i<4; ++i) (*rr++) += (*cs++);
  /* #208: @6 = (2.*@6) */
  for (i=0, rr=w6, cs=w6; i<4; ++i) *rr++ = (2.* *cs++ );
  /* #209: @1 = zeros(4x1) */
  casadi_clear(w1, 4);
  /* #210: @14 = 12.0482 */
  w14 = 1.2048192771084336e+01;
  /* #211: @5 = zeros(3x1) */
  casadi_clear(w5, 3);
  /* #212: @8 = 1 */
  w8 = 1.;
  /* #213: @9 = sq(@15) */
  w9 = casadi_sq( w15 );
  /* #214: @10 = sq(@24) */
  w10 = casadi_sq( w24 );
  /* #215: @9 = (@9+@10) */
  w9 += w10;
  /* #216: @9 = (2.*@9) */
  w9 = (2.* w9 );
  /* #217: @8 = (@8-@9) */
  w8 -= w9;
  /* #218: @9 = (@2*@15) */
  w9  = (w2*w15);
  /* #219: @10 = (@28*@24) */
  w10  = (w28*w24);
  /* #220: @9 = (@9-@10) */
  w9 -= w10;
  /* #221: @9 = (2.*@9) */
  w9 = (2.* w9 );
  /* #222: @10 = (@2*@24) */
  w10  = (w2*w24);
  /* #223: @26 = (@28*@15) */
  w26  = (w28*w15);
  /* #224: @10 = (@10+@26) */
  w10 += w26;
  /* #225: @10 = (2.*@10) */
  w10 = (2.* w10 );
  /* #226: @18 = horzcat(@8, @9, @10) */
  rr=w18;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #227: @18 = @18' */
  /* #228: @8 = (@2*@15) */
  w8  = (w2*w15);
  /* #229: @9 = (@28*@24) */
  w9  = (w28*w24);
  /* #230: @8 = (@8+@9) */
  w8 += w9;
  /* #231: @8 = (2.*@8) */
  w8 = (2.* w8 );
  /* #232: @9 = 1 */
  w9 = 1.;
  /* #233: @10 = sq(@2) */
  w10 = casadi_sq( w2 );
  /* #234: @26 = sq(@24) */
  w26 = casadi_sq( w24 );
  /* #235: @10 = (@10+@26) */
  w10 += w26;
  /* #236: @10 = (2.*@10) */
  w10 = (2.* w10 );
  /* #237: @9 = (@9-@10) */
  w9 -= w10;
  /* #238: @10 = (@15*@24) */
  w10  = (w15*w24);
  /* #239: @26 = (@28*@2) */
  w26  = (w28*w2);
  /* #240: @10 = (@10-@26) */
  w10 -= w26;
  /* #241: @10 = (2.*@10) */
  w10 = (2.* w10 );
  /* #242: @20 = horzcat(@8, @9, @10) */
  rr=w20;
  *rr++ = w8;
  *rr++ = w9;
  *rr++ = w10;
  /* #243: @20 = @20' */
  /* #244: @8 = (@2*@24) */
  w8  = (w2*w24);
  /* #245: @9 = (@28*@15) */
  w9  = (w28*w15);
  /* #246: @8 = (@8-@9) */
  w8 -= w9;
  /* #247: @8 = (2.*@8) */
  w8 = (2.* w8 );
  /* #248: @24 = (@15*@24) */
  w24  = (w15*w24);
  /* #249: @28 = (@28*@2) */
  w28 *= w2;
  /* #250: @24 = (@24+@28) */
  w24 += w28;
  /* #251: @24 = (2.*@24) */
  w24 = (2.* w24 );
  /* #252: @28 = 1 */
  w28 = 1.;
  /* #253: @2 = sq(@2) */
  w2 = casadi_sq( w2 );
  /* #254: @15 = sq(@15) */
  w15 = casadi_sq( w15 );
  /* #255: @2 = (@2+@15) */
  w2 += w15;
  /* #256: @2 = (2.*@2) */
  w2 = (2.* w2 );
  /* #257: @28 = (@28-@2) */
  w28 -= w2;
  /* #258: @21 = horzcat(@8, @24, @28) */
  rr=w21;
  *rr++ = w8;
  *rr++ = w24;
  *rr++ = w28;
  /* #259: @21 = @21' */
  /* #260: @19 = horzcat(@18, @20, @21) */
  rr=w19;
  for (i=0, cs=w18; i<3; ++i) *rr++ = *cs++;
  for (i=0, cs=w20; i<3; ++i) *rr++ = *cs++;
  for (i=0, cs=w21; i<3; ++i) *rr++ = *cs++;
  /* #261: @5 = mac(@19,@7,@5) */
  for (i=0, rr=w5; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w19+j, tt=w7+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #262: @5 = (@14*@5) */
  for (i=0, rr=w5, cs=w5; i<3; ++i) (*rr++)  = (w14*(*cs++));
  /* #263: {NULL, NULL, @14} = vertsplit(@5) */
  w14 = w5[2];
  /* #264: (@1[3] += @14) */
  for (rr=w1+3, ss=(&w14); rr!=w1+4; rr+=1) *rr += *ss++;
  /* #265: (@1[2] += @14) */
  for (rr=w1+2, ss=(&w14); rr!=w1+3; rr+=1) *rr += *ss++;
  /* #266: (@1[1] += @14) */
  for (rr=w1+1, ss=(&w14); rr!=w1+2; rr+=1) *rr += *ss++;
  /* #267: (@1[0] += @14) */
  for (rr=w1+0, ss=(&w14); rr!=w1+1; rr+=1) *rr += *ss++;
  /* #268: @1 = (2.*@1) */
  for (i=0, rr=w1, cs=w1; i<4; ++i) *rr++ = (2.* *cs++ );
  /* #269: @6 = (@6+@1) */
  for (i=0, rr=w6, cs=w1; i<4; ++i) (*rr++) += (*cs++);
  /* #270: {@14, @8, @24, @28} = vertsplit(@6) */
  w14 = w6[0];
  w8 = w6[1];
  w24 = w6[2];
  w28 = w6[3];
  /* #271: output[0][3] = @14 */
  if (res[0]) res[0][10] = w14;
  /* #272: output[0][4] = @8 */
  if (res[0]) res[0][11] = w8;
  /* #273: output[0][5] = @24 */
  if (res[0]) res[0][12] = w24;
  /* #274: output[0][6] = @28 */
  if (res[0]) res[0][13] = w28;
  return 0;
}

CASADI_SYMBOL_EXPORT int yzh_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int yzh_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int yzh_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void yzh_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int yzh_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void yzh_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void yzh_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void yzh_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int yzh_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int yzh_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real yzh_expl_vde_adj_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* yzh_expl_vde_adj_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* yzh_expl_vde_adj_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* yzh_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* yzh_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int yzh_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 8;
  if (sz_res) *sz_res = 7;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 132;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
