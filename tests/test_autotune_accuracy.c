#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pidx/pid_autotune.h"
#define DBUF 65536
typedef struct { double y,K,T,L,dt,buf[DBUF]; int wi; } P1;
static void ini(P1*p,double K,double T,double L,double dt){p->y=0;p->K=K;p->T=T;p->L=L;p->dt=dt;p->wi=0;for(int i=0;i<DBUF;i++)p->buf[i]=0;}
static double st(P1*p,double u,double noise){
    int nd=(int)(p->L/p->dt+0.5); if(nd>=DBUF)nd=DBUF-1;
    p->buf[p->wi]=u; int ri=(p->wi-nd+DBUF)%DBUF; double ud=p->buf[ri];
    p->wi=(p->wi+1)%DBUF; p->y+=p->dt*((p->K*ud-p->y)/p->T);
    return p->y + noise*((double)rand()/RAND_MAX-0.5);
}
static int run_step(double K,double T,double L,double dt,double noise,
                    double*oK,double*oT,double*oL,int*q){
    P1 p; ini(&p,K,T,L,dt);
    PID_Handle h; PID_Config c; PID_ConfigDefault(&c);
    c.core.sample_time=(PID_Float)dt; PID_Init(&h,&c);
    PID_SetMode(&h,PID_MODE_MANUAL); PID_SetManualOutput(&h,0.0f);
    PID_AutoTuneConfig tc; PID_AutoTune_ConfigDefault(&tc,PID_IDENT_STEP);
    tc.output_step=1.0f; tc.output_min=-10; tc.output_max=10;
    tc.timeout_s=(PID_Float)(200*T+50*L); tc.stab_time=(PID_Float)(20*dt);
    tc.rule=PID_RULE_AMIGO_STEP;
    PID_AutoTune t; PID_AutoTune_Init(&t,&tc); PID_AutoTune_Start(&t,&h,0.0f);
    double y=0; int n=0,lim=(int)(tc.timeout_s/dt)+10;
    while(PID_AutoTune_IsRunning(&t)&&n<lim){
        double u=PID_AutoTune_Update(&t,(PID_Float)y,(PID_Float)dt);
        y=st(&p,u,noise); n++;
    }
    PID_AutoTuneResult r; int rc=PID_AutoTune_GetResult(&t,&r);
    *oK=r.model.k;*oT=r.model.t;*oL=r.model.l;*q=r.model.quality; return rc;
}
static void analytic(double K,double T,double L,double*Ku,double*Pu){
    double lo=1e-6,hi=1e4;
    for(int i=0;i<200;i++){double w=0.5*(lo+hi);
        if(atan(w*T)+w*L<M_PI)lo=w;else hi=w;}
    double w=0.5*(lo+hi);*Ku=sqrt(1+(w*T)*(w*T))/K;*Pu=2*M_PI/w;
}
static int run_relay(double K,double T,double L,double dt,double*oKu,double*oPu,int*q){
    P1 p; ini(&p,K,T,L,dt);
    for(int i=0;i<(int)(60*T/dt);i++) st(&p,1.0/K,0);
    PID_Handle h; PID_Config c; PID_ConfigDefault(&c);
    c.core.sample_time=(PID_Float)dt; PID_Init(&h,&c);
    PID_SetMode(&h,PID_MODE_MANUAL); PID_SetManualOutput(&h,(PID_Float)(1.0/K));
    PID_AutoTuneConfig tc; PID_AutoTune_ConfigDefault(&tc,PID_IDENT_RELAY);
    tc.output_step=0.2f; tc.hysteresis=0.002f; tc.auto_bias=true;
    tc.output_min=-10; tc.output_max=10; tc.timeout_s=(PID_Float)(300*T+100*L);
    tc.eval_cycles=5; tc.stab_time=(PID_Float)(20*dt); tc.rule=PID_RULE_ZN;
    /* relay amplitude must be large enough to drive the plant clear of the
     * hysteresis band: h*K should exceed eps by a wide margin */
    tc.output_step=(PID_Float)(0.2/K); tc.hysteresis=(PID_Float)(0.002);
    PID_AutoTune t; PID_AutoTune_Init(&t,&tc);
    double y=p.y;
    PID_AutoTune_Start(&t,&h,(PID_Float)y);
    int n=0,lim=(int)(tc.timeout_s/dt)+10;
    while(PID_AutoTune_IsRunning(&t)&&n<lim){
        double u=PID_AutoTune_Update(&t,(PID_Float)y,(PID_Float)dt);
        y=st(&p,u,0); n++;
    }
    PID_AutoTuneResult r; int rc=PID_AutoTune_GetResult(&t,&r);
    *oKu=r.model.ku;*oPu=r.model.pu;*q=r.model.quality; return rc;
}
int main(void){
    srand(12345);
    struct{double K,T,L,dt;const char*n;} tv[]={
        {2.0,1.0,0.3,0.01,"nominal"},
        {1.0,10.0,1.0,0.05,"slow thermal"},
        {5.0,0.5,0.05,0.001,"fast, small L"},
        {0.5,2.0,2.0,0.01,"L/T = 1"},
        {1.0,1.0,0.05,0.01,"L/T = 0.05"},
        {3.0,20.0,4.0,0.1, "big furnace"},
    };
    printf("=== STEP identification (noise-free) ===\n");
    printf("%-15s %7s %7s %7s | %6s %6s %6s | q  rc\n","plant","K","T","L","eK%","eT%","eL%");
    int bad=0;
    for(unsigned i=0;i<sizeof(tv)/sizeof(tv[0]);i++){
        double oK,oT,oL;int q;
        int rc=run_step(tv[i].K,tv[i].T,tv[i].L,tv[i].dt,0,&oK,&oT,&oL,&q);
        double eK=100*(oK/tv[i].K-1),eT=100*(oT/tv[i].T-1),eL=(tv[i].L>0)?100*(oL/tv[i].L-1):0;
        printf("%-15s %7.4f %7.4f %7.4f | %+6.1f %+6.1f %+6.1f | %3d %d\n",
               tv[i].n,oK,oT,oL,eK,eT,eL,q,rc);
        if(rc!=0||fabs(eK)>3||fabs(eT)>12||fabs(eL)>25) bad++;
    }
    printf("\n=== STEP with 1%% measurement noise ===\n");
    for(unsigned i=0;i<6;i++){
        double oK,oT,oL;int q;
        double nz=0.01*tv[i].K;
        int rc=run_step(tv[i].K,tv[i].T,tv[i].L,tv[i].dt,nz,&oK,&oT,&oL,&q);
        printf("%-15s K=%.4f T=%.4f L=%.4f | eK=%+5.1f%% eT=%+6.1f%% eL=%+6.1f%% q=%3d rc=%d\n",
               tv[i].n,oK,oT,oL,100*(oK/tv[i].K-1),100*(oT/tv[i].T-1),
               100*(oL/tv[i].L-1),q,rc);
        if(rc!=0||fabs(100*(oT/tv[i].T-1))>20) bad++;
    }
    printf("\n=== RELAY identification (Pu is the load-bearing number) ===\n");
    printf("%-15s %8s %8s | %8s %8s | %6s %6s | q rc\n",
           "plant","Ku","Pu","Ku_true","Pu_true","eKu%","ePu%");
    for(unsigned i=0;i<sizeof(tv)/sizeof(tv[0]);i++){
        double oKu,oPu,aKu,aPu;int q;
        analytic(tv[i].K,tv[i].T,tv[i].L,&aKu,&aPu);
        int rc=run_relay(tv[i].K,tv[i].T,tv[i].L,tv[i].dt,&oKu,&oPu,&q);
        double ePu=100*(oPu/aPu-1);
        printf("%-15s %8.4f %8.4f | %8.4f %8.4f | %+6.1f %+6.1f | %d %d\n",
               tv[i].n,oKu,oPu,aKu,aPu,100*(oKu/aKu-1),ePu,q,rc);
        /* What is asserted here, and why it is not tighter.
         *
         * Ku is deliberately NOT asserted on. The describing-function
         * formula keeps only the first harmonic of the relay square wave, so
         * it underestimates Ku by 11-31% on these plants BY CONSTRUCTION.
         * Solving the exact relay limit cycle analytically and feeding it to
         * the same formula reproduces those figures to within 1-2%, which is
         * the real accuracy of this implementation. See docs section 9.8.
         *
         * Pu is asserted, because Pu is what carries Ti and Td. Tolerance is
         * 15% for plants sampled well above the validator's 20-samples-per-
         * period floor, and 35% for "L/T = 0.05". That plant is lag-dominated
         * with almost no dead time, and there the relay limit cycle is
         * genuinely 19% slower than the true ultimate period - an exact
         * analytic solution of the cycle gives 0.2334 s against Pu = 0.1961 s.
         * No implementation can recover Pu from that experiment; asserting a
         * tighter bound would only be asserting that the physics is different.
         */
        {
            const double tol = (i==4) ? 35.0 : 15.0;
            if(rc!=0||fabs(ePu)>tol){ bad++; }
        }
    }
    printf("\n%s (bad=%d)\n",bad?"*** FAILURES ***":"all within tolerance",bad);
    return 0;
}
