#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pidx/pid_autotune.h"
static int pass=0,fail=0;
#define CK(c,m) do{ if(c){pass++;} else {fail++;printf("  FAIL: %s\n",m);} }while(0)
#define DB 8192
static double bf[DB]; static int wi; static double yy;
static double plant(double u,double K,double T,double L,double dt){
  int nd=(int)(L/dt+0.5); bf[wi]=u; int ri=(wi-nd+DB)%DB; double ud=bf[ri];
  wi=(wi+1)%DB; yy+=dt*((K*ud-yy)/T); return yy; }
static void reset(void){memset(bf,0,sizeof bf);wi=0;yy=0;}

int main(void){
  const double dt=0.01;
  printf("== 1. model/rule mismatch is rejected at Init ==\n");
  {
    PID_AutoTuneConfig tc; PID_AutoTune_ConfigDefault(&tc,PID_IDENT_RELAY);
    tc.rule=PID_RULE_COHEN_COON;   /* needs FOPDT, relay gives FREQ */
    tc.output_step=0.2f; tc.eval_cycles=4;
    PID_AutoTune t; int rc=PID_AutoTune_Init(&t,&tc);
    CK(rc==PID_ERR_TUNE_MODEL_MISMATCH,"relay+CohenCoon must be MODEL_MISMATCH");
    printf("   relay + Cohen-Coon  -> rc=%d\n",rc);
    PID_AutoTune_ConfigDefault(&tc,PID_IDENT_STEP);
    tc.rule=PID_RULE_ZN;           /* ZN needs FREQ, step gives FOPDT */
    tc.output_step=1.0f; tc.eval_cycles=4;
    rc=PID_AutoTune_Init(&t,&tc);
    CK(rc==PID_ERR_TUNE_MODEL_MISMATCH,"step+ZN must be MODEL_MISMATCH");
    printf("   step  + Ziegler-Nichols -> rc=%d\n",rc);
    PID_AutoTune_ConfigDefault(&tc,PID_IDENT_STEP);
    tc.rule=PID_RULE_COHEN_COON;
    tc.output_step=1.0f; tc.eval_cycles=4;
    rc=PID_AutoTune_Init(&t,&tc);
    CK(rc==PID_OK,"step+CohenCoon must be OK");
    printf("   step  + Cohen-Coon  -> rc=%d (ok)\n",rc);
  }

  printf("\n== 2. NULL / uninitialised handling ==\n");
  {
    PID_AutoTune t; PID_AutoTuneConfig tc;
    PID_AutoTune_ConfigDefault(&tc,PID_IDENT_RELAY);
    CK(PID_AutoTune_Init(NULL,&tc)==PID_ERR_NULL,"Init(NULL,cfg)");
    CK(PID_AutoTune_Init(&t,NULL)==PID_ERR_NULL,"Init(t,NULL)");
    CK(PID_AutoTune_Start(NULL,NULL,0)==PID_ERR_NULL,"Start(NULL)");
    memset(&t,0,sizeof t);   /* no magic */
    CK(PID_AutoTune_GetState(&t)==PID_TUNE_IDLE,"zeroed handle reads IDLE");
    PID_AutoTuneResult r;
    CK(PID_AutoTune_GetResult(&t,&r)!=PID_OK,"result from zeroed handle rejected");
  }

  printf("\n== 3. abort restores the controller ==\n");
  {
    reset();
    PID_Handle h; PID_Config c; PID_ConfigDefault(&c);
    c.core.sample_time=(PID_Float)dt; PID_Init(&h,&c);
    PID_SetSetpoint(&h,7.0f);
    PID_SetMode(&h,PID_MODE_AUTOMATIC);
    PID_Update(&h,0.0f);
    PID_Mode m0=PID_GetMode(&h); PID_Float sp0=PID_GetSetpoint(&h);
    PID_AutoTuneConfig tc; PID_AutoTune_ConfigDefault(&tc,PID_IDENT_RELAY);
    tc.output_step=0.2f;tc.hysteresis=0.002f;tc.output_min=-5;tc.output_max=5;
    tc.timeout_s=100;tc.stab_time=0.2f;tc.rule=PID_RULE_ZN;
    PID_AutoTune t; PID_AutoTune_Init(&t,&tc);
    PID_AutoTune_Start(&t,&h,0.0f);
    CK(PID_GetMode(&h)==PID_MODE_MANUAL,"tuner forces MANUAL");
    double y=0;
    for(int i=0;i<200;i++){ double u=PID_AutoTune_Update(&t,(PID_Float)y,(PID_Float)dt);
                            y=plant(u,2,1,0.3,dt); }
    PID_AutoTune_Abort(&t);
    CK(PID_AutoTune_GetState(&t)==PID_TUNE_FAILED ||
       PID_AutoTune_GetState(&t)==PID_TUNE_IDLE,"abort leaves terminal state");
    CK(PID_GetMode(&h)==m0,"mode restored after abort");
    CK(PID_GetSetpoint(&h)==sp0,"setpoint restored after abort");
    printf("   mode %d->%d  sp %.2f->%.2f  err=%d\n",m0,PID_GetMode(&h),
           (double)sp0,(double)PID_GetSetpoint(&h),PID_AutoTune_GetError(&t));
  }

  printf("\n== 4. timeout is honoured ==\n");
  {
    reset();
    PID_Handle h; PID_Config c; PID_ConfigDefault(&c);
    c.core.sample_time=(PID_Float)dt; PID_Init(&h,&c);
    PID_SetMode(&h,PID_MODE_MANUAL); PID_SetManualOutput(&h,0.5f);
    PID_AutoTuneConfig tc; PID_AutoTune_ConfigDefault(&tc,PID_IDENT_RELAY);
    tc.output_step=0.2f;tc.hysteresis=0.002f;tc.output_min=-5;tc.output_max=5;
    tc.timeout_s=3.0f; tc.stab_time=0.2f; tc.rule=PID_RULE_ZN; tc.eval_cycles=12;
    PID_AutoTune t; PID_AutoTune_Init(&t,&tc); PID_AutoTune_Start(&t,&h,0.0f);
    int n=0; double y=0;
    while(PID_AutoTune_IsRunning(&t)&&n<100000){
      double u=PID_AutoTune_Update(&t,(PID_Float)y,(PID_Float)dt);
      y=plant(u,2,1,0.3,dt); n++; }
    CK(PID_AutoTune_GetError(&t)==PID_ERR_TUNE_TIMEOUT,"timeout code");
    CK(n*dt<=3.2,"stopped at the budget");
    printf("   stopped after %.2fs (budget 3.0s) err=%d\n",n*dt,PID_AutoTune_GetError(&t));
  }

  printf("\n== 5. dead plant -> NO_OSCILLATION, not a hang ==\n");
  {
    PID_Handle h; PID_Config c; PID_ConfigDefault(&c);
    c.core.sample_time=(PID_Float)dt; PID_Init(&h,&c);
    PID_SetMode(&h,PID_MODE_MANUAL); PID_SetManualOutput(&h,0.5f);
    PID_AutoTuneConfig tc; PID_AutoTune_ConfigDefault(&tc,PID_IDENT_RELAY);
    tc.output_step=0.2f;tc.hysteresis=0.002f;tc.output_min=-5;tc.output_max=5;
    tc.timeout_s=20; tc.stab_time=0.2f; tc.rule=PID_RULE_ZN;
    PID_AutoTune t; PID_AutoTune_Init(&t,&tc); PID_AutoTune_Start(&t,&h,0.0f);
    int n=0;
    while(PID_AutoTune_IsRunning(&t)&&n<100000){
      (void)PID_AutoTune_Update(&t,1.0f,(PID_Float)dt); n++; }
    int e=PID_AutoTune_GetError(&t);
    CK(e==PID_ERR_TUNE_TIMEOUT||e==PID_ERR_TUNE_NO_OSCILLATION,"terminates");
    printf("   frozen sensor -> err=%d after %.1fs\n",e,n*dt);
  }

  printf("\n== 6. gains actually stabilise the plant (closed loop) ==\n");
  {
    reset();
    PID_Handle h; PID_Config c; PID_ConfigDefault(&c);
    c.core.sample_time=(PID_Float)dt; PID_Init(&h,&c);
    PID_SetMode(&h,PID_MODE_MANUAL); PID_SetManualOutput(&h,0.0f);
    PID_AutoTuneConfig tc; PID_AutoTune_ConfigDefault(&tc,PID_IDENT_STEP);
    tc.output_step=1.0f;tc.output_min=-10;tc.output_max=10;
    tc.timeout_s=200;tc.stab_time=0.2f;tc.rule=PID_RULE_AMIGO_STEP;
    PID_AutoTune t; PID_AutoTune_Init(&t,&tc); PID_AutoTune_Start(&t,&h,0.0f);
    double y=0;
    while(PID_AutoTune_IsRunning(&t)){
      double u=PID_AutoTune_Update(&t,(PID_Float)y,(PID_Float)dt);
      y=plant(u,2,1,0.3,dt); }
    CK(PID_AutoTune_GetState(&t)==PID_TUNE_COMPLETE,"tune completed");
    int rc=PID_AutoTune_Apply(&t,&h);
    CK(rc==PID_OK,"Apply ok");
    PID_Float kp,ki,kd; PID_GetGains(&h,&kp,&ki,&kd);
    printf("   applied Kp=%.4f Ki=%.4f Kd=%.4f\n",(double)kp,(double)ki,(double)kd);
    reset();
    PID_SetMode(&h,PID_MODE_AUTOMATIC); PID_SetSetpoint(&h,1.0f);
    double peak=0,last=0;
    for(int i=0;i<20000;i++){
      double u=PID_Update(&h,(PID_Float)y);
      y=plant(u,2,1,0.3,dt);
      if(y>peak){peak=y;}
      last=y;
    }
    double over=100*(peak-1.0);
    CK(fabs(last-1.0)<0.02,"closed loop reaches setpoint");
    CK(over<30.0,"overshoot below 30%");
    printf("   closed loop: final=%.5f overshoot=%.1f%%\n",last,over);
  }

  printf("\n%d passed, %d failed\n",pass,fail);
  return fail!=0;
}
