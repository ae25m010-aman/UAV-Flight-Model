function ms=FM1_Mission(ac,in,pf)
g=9.81;
ms.V_loiter_ideal=in.k_loiter_ideal*in.V_cruise;
n_turn=1/cosd(in.bank_loiter_deg);
ms.V_loiter_floor=in.k_stall_margin*pf.V_stall*sqrt(n_turn);
ms.V_loiter=ms.V_loiter_ideal;
if in.loiter_floor,ms.V_loiter=max(ms.V_loiter_ideal,ms.V_loiter_floor); end
ms.loiter_limited=ms.V_loiter > ms.V_loiter_ideal;
ms.loiter_radius=ms.V_loiter^2/(g*tand(in.bank_loiter_deg));
ms.gamma_descent=in.k_descent*pf.gamma_glide;
ms.ROD=in.V_descent*sind(ms.gamma_descent);
s.t=0; s.x=0; s.h=0; s.V=0.01; s.m=ac.m;
s.Ah=0; s.Wh=0; s.Q=in.batt.Q_Ah;
L=emptylog();
ms.phase={};
rho0=FM1_Atmosphere(0);
Vs0=sqrt(2*s.m*g/(rho0*ac.S*ac.CL_max));
V_lof=in.k_liftoff*Vs0;
CL_roll=ac.CL0 + ac.CLa*in.alpha_roll_deg*pi/180;
aux=FM1_Aux('takeoff',in);
dt=in.dt_ground; st=s;
while s.V < V_lof && s.t < 120
    q=0.5*rho0*s.V^2;
    K=ac.K*ground_eff(0,ac,in);
    D=q*ac.S*(ac.CD0 + K*CL_roll^2);
    Lf=q*ac.S*CL_roll;
    r=FM1_Propulsion(Inf,s.V,rho0,soc_now(s,in),aux,in);
    a=(r.T - D - in.mu_roll*max(s.m*g - Lf,0))/s.m;
    [s,L]=adv(s,L,r,a,0,dt,CL_roll,D,1);
end
ms=endphase(ms,'Ground roll',st,s,L);
ms.V_liftoff=V_lof; ms.S_roll=s.x;
if s.V < V_lof,ms.takeoff_failed=true; else,ms.takeoff_failed=false; end
aux=FM1_Aux('takeoff',in); st=s; roc_f=0; dt=in.dt_ground;
while s.h < in.h_screen && s.t < 300
    rho=FM1_Atmosphere(s.h);
    q=0.5*rho*s.V^2;
    r=FM1_Propulsion(Inf,s.V,rho,soc_now(s,in),aux,in);
    D0=q*ac.S*(ac.CD0 + ac.K*ground_eff(s.h,ac,in)*(s.m*g/(q*ac.S))^2);
    roc_c=max(min((r.T - D0)*s.V/(s.m*g) + in.K_air*(s.V - in.V_airborne),in.ROC_air_max),0.5);
    roc_f=roc_f + max(min(roc_c - roc_f,in.ROC_slew*dt),-in.ROC_slew*dt);
    sg=roc_f/s.V; cg=sqrt(1 - sg^2);
    CL=s.m*g*cg/(q*ac.S);
    D=q*ac.S*(ac.CD0 + ac.K*ground_eff(s.h,ac,in)*CL^2);
    a=(r.T - D - s.m*g*sg)/s.m;
    [s,L]=adv(s,L,r,a,roc_f,dt,CL,D,2);
end
ms=endphase(ms,'Airborne',st,s,L);
ms.S_to_screen=s.x; ms.S_airborne=s.x - st.x; ms.V_screen=s.V;
aux=FM1_Aux('climb',in);
st=s;
while s.h < in.h_cruise && s.t < 3000
    ROC=min(in.ROC_climb,(in.h_cruise - s.h)/in.dt_air + 0.01);
    [s,L]=airstep(s,L,in.V_climb,ROC,1,aux,in.dt_air,ac,in,3);
end
ms=endphase(ms,'Climb',st,s,L);
ms.h_reached=s.h;
aux=FM1_Aux('cruise',in); st=s;
[s,L]=hold_phase(s,L,in.V_cruise,1,aux,in.t_cruise,ac,in,4);
ms=endphase(ms,'Cruise out',st,s,L);
aux=FM1_Aux('loiter',in); st=s;
[s,L]=hold_phase(s,L,ms.V_loiter,n_turn,aux,in.t_loiter,ac,in,5);
ms=endphase(ms,'Loiter',st,s,L);
if in.drop_payload,s.m=s.m - ac.m_payload; end
aux=FM1_Aux('cruise',in); st=s;
[s,L]=hold_phase(s,L,in.V_cruise,1,aux,in.t_cruise,ac,in,6);
ms=endphase(ms,'Cruise back',st,s,L);
aux=FM1_Aux('descent',in); st=s;
while s.h > in.h_screen && s.t < 20000
    [s,L]=airstep(s,L,in.V_descent,-ms.ROD,1,aux,in.dt_air,ac,in,7);
end
ms=endphase(ms,'Descent',st,s,L);
aux=FM1_Aux('landing',in); st=s;
Vs0=sqrt(2*s.m*g/(rho0*ac.S*ac.CL_max));
V_app=in.k_approach*Vs0; V_td=in.k_touchdown*Vs0;
ms.V_approach=V_app; ms.V_touchdown=V_td;
R_fl=V_app^2/((in.n_flare-1)*g);
h_fl=R_fl*(1-cosd(in.gamma_app_deg));
x0=s.x;
while s.h > h_fl && s.t < 21000
    [s,L]=airstep(s,L,V_app,-V_app*sind(in.gamma_app_deg),1,aux,0.2,ac,in,8);
end
S_fl=R_fl*sind(in.gamma_app_deg);
t_fl=S_fl/(0.5*(s.V+V_td));
r0=FM1_Propulsion(0,s.V,rho0,soc_now(s,in),aux,in);
nf=max(ceil(t_fl/0.05),1); dtf=t_fl/nf; a_fl=(V_td - s.V)/t_fl;
hrate=-s.h/t_fl;
for i=1:nf
    q_fl=0.5*rho0*s.V^2;
    CL_fl=s.m*g*in.n_flare/(q_fl*ac.S);
    D_fl=q_fl*ac.S*(ac.CD0 + ac.K*ground_eff(s.h,ac,in)*CL_fl^2);
    [s,L]=adv(s,L,r0,a_fl,hrate,dtf,CL_fl,D_fl,8);
end
s.h=0;
ms=endphase(ms,'Approach + flare',st,s,L);
S_air_land=s.x - x0;
st=s; dt=in.dt_ground;
x1=s.x;
while s.V > 0.2 && s.t < 22000
    q=0.5*rho0*s.V^2;
    K=ac.K*ground_eff(0,ac,in);
    D=q*ac.S*(ac.CD0 + K*CL_roll^2);
    Lf=q*ac.S*CL_roll;
    r0=FM1_Propulsion(0,s.V,rho0,soc_now(s,in),aux,in);
    a=(-D - in.mu_landing*max(s.m*g - Lf,0))/s.m;
    [s,L]=adv(s,L,r0,a,0,dt,CL_roll,D,9);
end
ms=endphase(ms,'Landing roll',st,s,L);
ms.S_land_roll=s.x - x1; ms.S_land_air=S_air_land;
ms.log=L;
ms.t_total=s.t; ms.x_total=s.x; ms.Wh=s.Wh; ms.Ah=s.Ah;
ms.SoC_end=1 - s.Ah/in.batt.Q_Ah;
ms.feasible_energy=ms.SoC_end >= in.batt.SoC_min;
ms.Ah_required=s.Ah/(1 - in.batt.SoC_min);
ms.t_at_reserve=NaN;
k=find(L.SoC < in.batt.SoC_min,1);
if ~isempty(k),ms.t_at_reserve=L.t(k); ms.phase_at_reserve=L.phase(k); end
ms.t_empty=NaN;
k=find(L.SoC < 0,1);
if ~isempty(k),ms.t_empty=L.t(k); end
ms.stall_flag=any(L.CL > ac.CL_max + 1e-6);
ms.thrust_limited_frac=mean(L.limited(L.phase>=2 & L.phase<=7));
ms.I_peak=max(L.I_b); ms.I_motor_peak=max(L.I_m);
end
function L=emptylog()
L=struct('t',[],'x',[],'h',[],'V',[],'SoC',[],'T',[],'D',[],'CL',[],'P',[],'I_b',[],...
    'I_m',[],'V_b',[],'RPM',[],'d',[],'eta_p',[],'ROC',[],'limited',[],'phase',[],...
    'J',[],'P_shaft',[],'m',[],'Ah_',[],'Wh_',[]);
end
function v=soc_now(s,in)
v=1 - s.Ah/in.batt.Q_Ah;
end
function f=ground_eff(h,ac,in)
f=1;
if in.use_ground_effect
    hw=h + ac.h_wing_ground;
    x=16*hw/ac.b;
    f=x^2/(1+x^2);
end
end
function [s,L]=hold_phase(s,L,Vt,n,aux,T_phase,ac,in,id)
t_end=s.t + T_phase;
while s.t < t_end - 1e-9
    [s,L]=airstep(s,L,Vt,0,n,aux,min(in.dt_air,t_end - s.t),ac,in,id);
end
end
function [s,L]=airstep(s,L,Vt,ROC,n,aux,dt,ac,in,id)
g=9.81;
rho=FM1_Atmosphere(max(s.h,0));
sg=max(min(ROC/max(s.V,1),0.9),-0.9);
cg=sqrt(1-sg^2);
a=max(min((Vt - s.V)/in.tau_V,in.a_max),-in.a_max);
q=0.5*rho*s.V^2;
CL=s.m*g*cg*n/(q*ac.S);
K=ac.K*ground_eff(s.h,ac,in);
D=q*ac.S*(ac.CD0 + K*CL^2);
T_req=D + s.m*g*sg + s.m*a;
r=FM1_Propulsion(T_req,s.V,rho,soc_now(s,in),aux,in);
if T_req <= 0
    a=(-D - s.m*g*sg)/s.m;
elseif r.limited
    sg=max(min((r.T - D - s.m*a)/(s.m*g),0.9),-0.9);
    ROC=sg*s.V;
end
[s,L]=adv(s,L,r,a,ROC,dt,CL,D,id);
end
function [s,L]=adv(s,L,r,a,hdot,dt,CL,D,id)
k=numel(L.t)+1;
L.t(k)=s.t; L.x(k)=s.x; L.h(k)=s.h; L.V(k)=s.V;
L.T(k)=r.T; L.D(k)=D; L.CL(k)=CL; L.P(k)=r.P_elec; L.I_b(k)=r.I_b; L.I_m(k)=r.I_m;
L.V_b(k)=r.V_b; L.RPM(k)=r.RPM; L.d(k)=r.d; L.eta_p(k)=r.eta_p; L.ROC(k)=hdot;
L.limited(k)=r.limited; L.phase(k)=id; L.J(k)=r.J; L.P_shaft(k)=r.P_shaft; L.m(k)=s.m;
s.Ah=s.Ah + r.I_b*dt/3600;
s.Wh=s.Wh + r.P_elec*dt/3600;
L.Ah_(k)=s.Ah; L.Wh_(k)=s.Wh;
L.SoC(k)=1 - s.Ah/s.Q;
xd=sqrt(max(s.V^2 - hdot^2,0));
s.x=s.x + xd*dt;
s.h=s.h + hdot*dt;
s.V=max(s.V + a*dt,0.01);
s.t=s.t + dt;
end
function ms=endphase(ms,name,st,s,L)
i=numel(ms.phase)+1;
ms.phase{i}=name;
ms.ph_t(i)=s.t - st.t;
ms.ph_x(i)=s.x - st.x;
ms.ph_h(i)=s.h;
ms.ph_V(i)=s.V;
ms.ph_Wh(i)=s.Wh - st.Wh;
ms.ph_Ah(i)=s.Ah - st.Ah;
idx=L.t >= st.t - 1e-9;
ms.ph_Pavg(i)=(s.Wh - st.Wh)*3600/max(s.t - st.t,1e-6);
ms.ph_Iavg(i)=(s.Ah - st.Ah)*3600/max(s.t - st.t,1e-6);
ms.ph_T(i)=mean(L.T(idx));
ms.ph_RPM(i)=mean(L.RPM(idx));
ms.ph_CL(i)=mean(L.CL(idx));
ms.ph_SoC(i)=s.Ah;
end
