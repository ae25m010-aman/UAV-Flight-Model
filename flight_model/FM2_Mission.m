function ms=FM2_Mission(ac,c,in,pf)
g=9.81; rho0=FM1_Atmosphere(0);
names={'Ground roll','Airborne','Climb','Cruise out','Loiter','Cruise back','Descent','Approach + flare','Landing roll'};
Vs0=sqrt(2*ac.m*g/(rho0*ac.S*ac.CL_max));
ms.V_R=in.k_liftoff*Vs0;
ms.V_approach=in.k_approach*Vs0;
ms.V_touchdown=NaN;
n_turn=1/cosd(in.bank_loiter_deg);
ms.V_loiter_ideal=in.k_loiter_ideal*in.V_cruise;
ms.V_loiter_floor=in.k_stall_margin*pf.V_stall*sqrt(n_turn);
ms.V_loiter=ms.V_loiter_ideal;
if in.loiter_floor,ms.V_loiter=max(ms.V_loiter_ideal,ms.V_loiter_floor); end
ms.loiter_limited=ms.V_loiter > ms.V_loiter_ideal;
ms.loiter_radius=ms.V_loiter^2/(g*tand(in.bank_loiter_deg));
ms.gamma_descent=in.k_descent*pf.gamma_glide;
ms.ROD=in.V_descent*sind(ms.gamma_descent);
theta_lof=((ac.CL_max/in.k_liftoff^2 - ac.CL0)/ac.CLa)*180/pi;
ms.theta_lof=theta_lof;
th0=ac.theta_ground*pi/180;
rz_m=ac.xb_main*sin(th0) + ac.zb_main*cos(th0);
frac_t=ac.xb_main/(ac.xb_main - ac.xb_tail);
s=[0; -rz_m - ac.m*g*(1-frac_t)/c.k_main; 0; 0; th0; 0];
de=0; d_act=0; phi=0;
int_th=0; int_V=0.6; int_gam=0;
ph=1; t=0; t_ph=0; path=0; Ah=0; Wh=0; n_prop=200; th_cmd_rot=c.theta_roll;
t_prop=-1; t_log=-1; pr=struct('T',0,'P_elec',0,'I_b',0,'I_m',0,'V_b',0,'n',200,'eta_p',0,'J',0,'P_shaft',0); first_prop=true; hdot_f=0;
ms.t_tail_up=NaN; ms.V_tail_up=NaN; ms.S_tail_up=NaN; ms.S_roll=NaN; ms.V_liftoff=NaN;
ms.S_to_screen=NaN; x_td=NaN; x_ld=NaN; ms.t_touch=NaN; ms.sink_touch=NaN;
L=struct('t',[],'x',[],'X',[],'h',[],'V',[],'T',[],'D',[],'CL',[],'CD',[],'Cm',[],'P',[],'I_b',[],'I_m',[],'V_b',[],...
    'RPM',[],'d',[],'eta_p',[],'J',[],'P_shaft',[],'ROC',[],'theta',[],'alpha',[],'gamma',[],'q',[],'de',[],'de_cmd',[],...
    'theta_cmd',[],'V_cmd',[],'h_cmd',[],'phi',[],'Nm',[],'Nt',[],'stalled',[],'de_sat',[],'limited',[],'phase',[],'m',[],'Ah_',[],'Wh_',[],'qb',[]);
ms.fail_reason=''; edges=0; k=0;
t_max=2*(in.t_cruise*2 + in.t_loiter) + 2000;
while ph <= 9 && t < t_max
    [~,a]=FM2_Deriv(s,de,0,phi,ac.m,ac,c,in);
    V=a.V; hw=a.hw; onG=a.Nm > 0 || a.Nt > 0;
    hdot=s(4);
    if ph==1 || ph==9 || hw < 3,dt=c.dt_ground; else,dt=c.dt_air; end
    d_cmd=NaN; th_force=NaN; hdot_c=0; V_c=in.V_cruise; h_c=in.h_cruise; ph_t=0;
    switch ph
        case 1
            d_cmd=1;
            if V >= ms.V_R,th_cmd_rot=min(th_cmd_rot + c.rot_rate*dt,theta_lof); end
            th_force=th_cmd_rot;
            V_c=ms.V_R; h_c=0;
            if isnan(ms.t_tail_up) && a.Nt <= 0 && V > 1
            ms.t_tail_up=t; ms.V_tail_up=V; ms.S_tail_up=path;
            end
            if ~onG && hw > 0.05
            ms.S_roll=path; ms.V_liftoff=V; ms.t_liftoff=t;
            ph=2; t_ph=0; edges(end+1)=t; int_gam=0; int_V=0.75;
            end
        case 2
            d_cmd=1; V_c=in.V_airborne; h_c=in.h_screen;
            hdot_c=max(min((pr.T - a.D)*V/(ac.m*g) + in.K_air*(V - in.V_airborne),in.ROC_air_max),0.5);
            if hw >= in.h_screen
            ms.S_to_screen=path; ms.S_airborne=path - ms.S_roll;
            ms.V_screen=V; ms.t_screen=t;
            ph=3; t_ph=0; edges(end+1)=t; int_V=0.68;
            end
        case 3
            V_c=in.V_climb; h_c=in.h_cruise;
            hdot_c=min(in.ROC_climb,c.K_h*(h_c - hw));
            if hw >= in.h_cruise - 0.5
            ph=4; t_ph=0; edges(end+1)=t;
            end
        case 4
            hdot_c=c.K_h*(h_c - hw);
            if t_ph >= in.t_cruise,ph=5; t_ph=0; edges(end+1)=t; end
        case 5
            V_c=ms.V_loiter; hdot_c=c.K_h*(h_c - hw);
            ph_t=in.bank_loiter_deg;
            if t_ph >= in.t_loiter
            ph=6; t_ph=0; edges(end+1)=t;
            end
        case 6
            hdot_c=c.K_h*(h_c - hw);
            if t_ph >= in.t_cruise,ph=7; t_ph=0; edges(end+1)=t; end
        case 7
            V_c=in.V_descent; hdot_c=-ms.ROD;
            if hw <= in.h_screen,ph=8; t_ph=0; edges(end+1)=t; x_ld=path; end
        case 8
            V_c=ms.V_approach;
            hf=c.tau_flare*(V*sind(in.gamma_app_deg) - c.sink_td);
            hdot_c=-V*sind(in.gamma_app_deg);
            if hw <= hf
            hdot_c=-(hw/c.tau_flare + c.sink_td); d_cmd=0;
            end
            if onG
            ms.V_touchdown=V; ms.sink_touch=-hdot; ms.t_touch=t; x_td=path;
            ms.S_land_air=path - x_ld;
            ph=9; t_ph=0; edges(end+1)=t;
            th_cmd_rot=s(5)*180/pi;
            end
        case 9
            d_cmd=0; V_c=0; h_c=0;
            th_force=th_cmd_rot;
            if V < 0.2
            ms.S_land_roll=path - x_td;
            ph=10;
            end
    end
    if ph > 9,break; end
    if ph == 2 || ph == 3
        hdot_f=hdot_f + max(min(hdot_c - hdot_f,in.ROC_slew*dt),-in.ROC_slew*dt);
        hdot_c=hdot_f;
    else
        hdot_f=hdot;
    end
    phi_tgt=ph_t*pi/180;
    phi=phi + max(min(phi_tgt - phi,(in.bank_loiter_deg*pi/180/c.t_bank)*dt),-(in.bank_loiter_deg*pi/180/c.t_bank)*dt);
    sch=min(c.q_ref/max(a.qb,1),c.gain_max);
    if isnan(th_force)
        Vs=max(V,5);
        gam_c=asin(max(min(hdot_c/Vs,0.5),-0.5));
        CLn=ac.m*g*cos(gam_c)/(0.5*a.rho*Vs^2*ac.S*cos(phi));
        alpha_ff=(min(CLn,ac.CL_max) - ac.CL0)/ac.CLa;
        int_gam=int_gam + c.Ki_gamma*(hdot_c - hdot)/Vs*dt;
        int_gam=max(min(int_gam,0.2),-0.2);
        th_c=gam_c + alpha_ff + int_gam + c.K_gamma*(hdot_c - hdot)/Vs;
    else
        th_c=th_force*pi/180;
    end
    e_th=s(5) - th_c;
    if ~onG || ph==1 && a.Nt<=0
        int_th=int_th + c.Ki_theta*e_th*dt*sch;
        int_th=max(min(int_th,ac.de_max),-ac.de_max);
    end
    de_c=int_th + sch*(c.K_theta*e_th + c.K_q*s(6));
    if ph == 1 && a.Nt > 0 && e_th > 0,de_c=ac.de_max; end
    sat=abs(de_c) >= ac.de_max;
    de_c=max(min(de_c,ac.de_max),-ac.de_max);
    rl=c.de_rate*pi/180*dt;
    de=de + max(min(de_c - de,rl),-rl);
    if isnan(d_cmd)
        e_V=V_c - V;
        int_V=int_V + c.Ki_V*e_V*dt;
        int_V=max(min(int_V,1),0);
        d_cmd=max(min(int_V + c.K_V*e_V,1),0);
    end
    d_act=d_act + (d_cmd - d_act)*min(dt/c.tau_thr,1);
    if t - t_prop >= c.dt_prop - 1e-9 || first_prop
        switch ph
            case {1,2},aux='takeoff'; case 3,aux='climb'; case {4,6},aux='cruise';
            case 5,aux='loiter'; case 7,aux='descent'; otherwise,aux='landing';
        end
        pr=FM2_Propulsion(d_act,V,a.rho,1-Ah/in.batt.Q_Ah,FM1_Aux(aux,in),in,n_prop);
        n_prop=pr.n; t_prop=t; first_prop=false;
    end
    if t - t_log >= (c.dt_log - 1e-9)*(1 - 0.5*(dt < c.dt_air))
        k=k + 1; t_log=t;
        L.t(k)=t; L.x(k)=path; L.X(k)=s(1); L.h(k)=max(hw,0); L.V(k)=V; L.T(k)=pr.T; L.D(k)=a.D; L.CL(k)=a.CL; L.CD(k)=a.CD; L.Cm(k)=a.Cm;
        L.P(k)=pr.P_elec; L.I_b(k)=pr.I_b; L.I_m(k)=pr.I_m; L.V_b(k)=pr.V_b; L.RPM(k)=pr.n*60*(pr.T>0); L.d(k)=d_act; L.eta_p(k)=pr.eta_p; L.J(k)=pr.J; L.P_shaft(k)=pr.P_shaft;
        L.ROC(k)=hdot; L.theta(k)=s(5)*180/pi; L.alpha(k)=a.alpha*180/pi; L.gamma(k)=a.gam*180/pi; L.q(k)=s(6)*180/pi; L.de(k)=de*180/pi; L.de_cmd(k)=de_c*180/pi;
        L.theta_cmd(k)=th_c*180/pi; L.V_cmd(k)=V_c; L.h_cmd(k)=h_c; L.phi(k)=phi*180/pi; L.Nm(k)=a.Nm; L.Nt(k)=a.Nt; L.stalled(k)=a.stalled; L.de_sat(k)=sat; L.limited(k)=d_act>=0.98;
        L.phase(k)=ph; L.m(k)=ac.m; L.Ah_(k)=Ah; L.Wh_(k)=Wh; L.qb(k)=a.qb;
    end
    T=pr.T;
    k1=FM2_Deriv(s,de,T,phi,ac.m,ac,c,in);
    k2=FM2_Deriv(s + dt/2*k1,de,T,phi,ac.m,ac,c,in);
    k3=FM2_Deriv(s + dt/2*k2,de,T,phi,ac.m,ac,c,in);
    k4=FM2_Deriv(s + dt*k3,de,T,phi,ac.m,ac,c,in);
    s=s + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    path=path + V*dt;
    Ah=Ah + pr.I_b*dt/3600; Wh=Wh + pr.P_elec*dt/3600;
    t=t + dt; t_ph=t_ph + dt;
    if ph == 1 && t > 60
        ms.fail_reason=sprintf('Did not lift off in 60 s: highest speed reached %.1f m/s',max(L.V)); break
    end
    if abs(s(5)) > 75*pi/180 || any(~isfinite(s))
        ms.fail_reason=sprintf('Lost control at t = %.1f s in phase %d: pitch %.0f deg',t,ph,s(5)*180/pi); break
    end
end
edges(end+1)=t;
L.SoC=1 - L.Ah_/in.batt.Q_Ah;
ms.finished=ph > 9;
ms.log=L; ms.phase=names(1:numel(edges)-1);
ms.ph_t=diff(edges);
for i=1:numel(ms.ph_t)
    idx=L.phase==i;
    ii=find(idx);
    a1=L.Ah_(ii(1)); w1=L.Wh_(ii(1));
    if i < numel(ms.ph_t)
        jj=find(L.phase==i+1,1); a2=L.Ah_(jj); w2=L.Wh_(jj);
    else
        a2=Ah; w2=Wh;
    end
    ms.ph_Wh(i)=w2-w1; ms.ph_Ah(i)=a2-a1;
    ms.ph_Pavg(i)=ms.ph_Wh(i)*3600/max(ms.ph_t(i),1e-6);
    ms.ph_Iavg(i)=ms.ph_Ah(i)*3600/max(ms.ph_t(i),1e-6);
    ms.ph_x(i)=L.x(find(idx,1,'last')) - L.x(ii(1));
    ms.ph_T(i)=mean(L.T(idx)); ms.ph_V(i)=L.V(find(idx,1,'last')); ms.ph_h(i)=L.h(find(idx,1,'last'));
    ms.ph_alpha(i)=mean(L.alpha(idx)); ms.ph_de(i)=mean(L.de(idx)); ms.ph_theta(i)=mean(L.theta(idx));
    ms.ph_demax(i)=max(abs(L.de(idx)));
end
ms.t_total=t; ms.x_total=path; ms.Wh=Wh; ms.Ah=Ah;
ms.SoC_end=1 - Ah/in.batt.Q_Ah;
ms.feasible_energy=ms.SoC_end >= in.batt.SoC_min;
ms.I_peak=max(L.I_b); ms.I_motor_peak=max(L.I_m);
ms.stall_flag=any(L.stalled);
ms.t_stall=sum(diff([L.t t]).*L.stalled);
ms.de_sat_air=sum(L.de_sat(L.phase>=2 & L.phase<=8));
ms.t_at_reserve=NaN; j=find(L.SoC < in.batt.SoC_min,1);
if ~isempty(j),ms.t_at_reserve=L.t(j); ms.phase_at_reserve=L.phase(j); end
end
