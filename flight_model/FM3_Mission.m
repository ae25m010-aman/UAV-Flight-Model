function ms=FM3_Mission(ac,c,cc,in,pf,fix)
g=9.81; rho0=FM1_Atmosphere(0);
names={'Ground roll','Airborne','Climb','Cruise out','Loiter','Cruise back','Descent','Approach + flare','Landing roll'};
Vs0=sqrt(2*ac.m*g/(rho0*ac.S*ac.CL_max));
ms.V_R=in.k_liftoff*Vs0; ms.V_approach=in.k_approach*Vs0;
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
S_app=in.h_screen/tand(in.gamma_app_deg);
X_ap=cc.X_touchdown + S_app;
th0=ac.theta_ground*pi/180; psi_c=0;
Wm=ac.wheels(1,:);
frac_t=ac.xb_main/(ac.xb_main - ac.xb_tail);
h0=-(Wm(1)*(-sin(th0)) + Wm(3)*cos(th0))*(-1) - ac.m*g*(1-frac_t)/c.k_main;
s=[0;0;-h0;0;0;0;0;0;0;0;th0;0];
de=0; da=0; dr=fix.rudder; d_act=0; phi_c=0;
int_th=0; int_V=0.6; int_gam=0; int_phi=0; int_beta=0;
ph=1; t=0; t_ph=0; path=0; Ah=0; Wh=0; n_prop=200; th_cmd=c.theta_roll;
t_prop=-1; t_log=-1; first=true; hdot_f=0;
pp=FM3_Slip(0,0,0,0,rho0,ac,cc); P_el=0; I_b=0; I_m=0; V_b=0; eta_p=0;
ms.V_tail_up=NaN; ms.S_tail_up=NaN; ms.S_roll=NaN; ms.V_liftoff=NaN; ms.S_to_screen=NaN;
ms.y_max_roll=0; x_td=NaN; x_ld=NaN; ms.R_d=NaN; ms.Nd=0;
orbit=false; orb_ang=0; orb_tgt=0; orb_c=[0 0]; chi_prev=0; loi_c=[0 0];
L=struct('t',[],'x',[],'X',[],'Y',[],'h',[],'V',[],'T',[],'D',[],'CL',[],'CD',[],'CY',[],'Cm',[],'P',[],'I_b',[],'I_m',[],'V_b',[],...
    'RPM',[],'d',[],'eta_p',[],'P_shaft',[],'Q',[],'ROC',[],'theta',[],'alpha',[],'beta',[],'gamma',[],'phi',[],'psi',[],'chi',[],...
    'p',[],'q',[],'r',[],'de',[],'da',[],'dr',[],'theta_cmd',[],'phi_cmd',[],'V_cmd',[],'Nr',[],'Nl',[],'Nt',[],'stalled',[],...
    'de_sat',[],'phase',[],'m',[],'Ah_',[],'Wh_',[],'qb',[],'eps',[],'fv',[],'orbit',[],'ey',[]);
ms.fail_reason=''; edges=0; k=0; t_max=2*(in.t_cruise*2 + in.t_loiter) + 2500;
de_max=ac.de_max; da_max=ac.da_max; dr_max=ac.dr_max;
cm=c; cm.mu_side=cc.mu_side; cm.prop_inertia=cc.prop_inertia; cm.k_pfactor=cc.k_pfactor;
while ph <= 9 && t < t_max
    u0=struct('de',de,'da',da,'dr',dr,'hub',fix.hub,'dt',cc.tailwheel_k*(dr-fix.rudder));
    [~,a]=FM3_Deriv(s,u0,pp,ac.m,ac,cm,in);
    V=a.V; hw=a.hw; onG=any(a.Nw > 0); hdot=a.hdot;
    X=s(1); Y=s(2); phi=s(10); theta=s(11); psi=s(12);
    Ntail=a.Nw(3);
    chi=psi; if V > 5,chi=atan2(a.vy,a.vx); end
    if ph==1 || ph==9 || hw < 3,dt=c.dt_ground; else,dt=c.dt_air; end
    d_cmd=NaN; th_force=NaN; hdot_c=0; V_c=in.V_cruise; h_c=in.h_cruise;
    guide='line0'; ff=0; ey=Y;
    switch ph
        case 1
            d_cmd=1;
            if V >= ms.V_R,th_cmd=min(th_cmd + c.rot_rate*dt,theta_lof); end
            th_force=th_cmd; V_c=ms.V_R;
            if isnan(ms.V_tail_up) && Ntail <= 0 && V > 1,ms.V_tail_up=V; ms.S_tail_up=path; end
            if ~onG && hw > 0.05
            ms.S_roll=path; ms.V_liftoff=V; ms.y_off_lof=Y;
            ph=2; t_ph=0; edges(end+1)=t; int_gam=0; int_V=0.75;
            end
        case 2
            d_cmd=1; V_c=in.V_airborne; h_c=in.h_screen;
            hdot_c=max(min((pp.T - a.D)*V/(ac.m*g) + in.K_air*(V - in.V_airborne),in.ROC_air_max),0.5);
            if hw >= in.h_screen
            ms.S_to_screen=path; ms.S_airborne=path - ms.S_roll; ms.V_screen=V;
            ph=3; t_ph=0; edges(end+1)=t; int_V=0.68;
            end
        case 3
            V_c=in.V_climb;
            hdot_c=min(in.ROC_climb,c.K_h*(h_c - hw));
            if hw >= in.h_cruise - 0.5,ph=4; t_ph=0; edges(end+1)=t; end
        case 4
            hdot_c=c.K_h*(h_c - hw);
            if t_ph >= in.t_cruise
            ph=5; t_ph=0; edges(end+1)=t;
            loi_c=[X,Y + ms.loiter_radius];
            end
        case 5
            V_c=ms.V_loiter; hdot_c=c.K_h*(h_c - hw);
            guide='loiter';
            if t_ph >= in.t_loiter,ph=6; t_ph=0; edges(end+1)=t; end
        case 6
            hdot_c=c.K_h*(h_c - hw); guide='lineB';
            if t_ph >= in.t_cruise
            ph=7; t_ph=0; edges(end+1)=t;
            L_d=(hw - in.h_screen)/tand(ms.gamma_descent);
            excess=L_d - (X - X_ap);
            if excess > 0
            ms.Nd=max(1,round(excess/(2*pi*cc.R_orbit)));
            ms.R_d=excess/(2*pi*ms.Nd);
            orbit=true; orb_ang=0; orb_tgt=2*pi*ms.Nd;
            orb_c=[X,Y - ms.R_d]; chi_prev=chi;
            end
            end
        case 7
            V_c=in.V_descent;
            if orbit
            orb_ang=orb_ang + abs(wrap_pi(chi - chi_prev)); chi_prev=chi;
            if orb_ang >= orb_tgt,orbit=false; end
            end
            if orbit
            guide='orbit';
            S_rem=(orb_tgt - orb_ang)*ms.R_d + (orb_c(1) - X_ap);
            else
            guide='lineB';
            S_rem=max(X - X_ap,20);
            end
            hdot_c=-min(V*max(hw - in.h_screen,0)/max(S_rem,20),1.5*ms.ROD);
            if hw <= in.h_screen
            ph=8; t_ph=0; edges(end+1)=t; x_ld=path;
            ms.X_gate=X; ms.Y_gate=Y;
            end
        case 8
            V_c=ms.V_approach; guide='lineB';
            hf=c.tau_flare*(V*sind(in.gamma_app_deg) - c.sink_td);
            hdot_c=-V*sind(in.gamma_app_deg);
            if hw <= hf,hdot_c=-(hw/c.tau_flare + c.sink_td); d_cmd=0; end
            if onG
            ms.V_touchdown=V; ms.sink_touch=-hdot; x_td=path;
            ms.S_land_air=path - x_ld; ms.X_touch=X; ms.Y_touch=Y;
            ph=9; t_ph=0; edges(end+1)=t; th_cmd=theta*180/pi;
            end
        case 9
            d_cmd=0; V_c=0; h_c=0; guide='lineB';
            CL_keep=cc.k_land_lift*ac.m*g/(max(a.qb,1)*ac.S);
            th_tgt=min(((min(CL_keep,ac.CL_max) - ac.CL0)/ac.CLa)*180/pi,ac.theta_ground + 3);
            th_cmd=th_cmd + max(min(th_tgt - th_cmd,cc.land_pitch_rate*dt),-cc.land_pitch_rate*dt);
            th_force=th_cmd;
            if V < 0.2,ms.S_land_roll=path - x_td; ms.X_stop=X; ms.Y_stop=Y; ph=10; end
    end
    if ph > 9,break; end
    switch guide
        case 'line0'
            chi_c=-atan(max(min(cc.K_track*Y,1.2),-1.2));
        case 'lineB'
            chi_c=pi + atan(max(min(cc.K_track*Y,1.2),-1.2));
        case 'loiter'
            dx=X - loi_c(1); dy=Y - loi_c(2); d=sqrt(dx^2+dy^2);
            chi_c=atan2(dy,dx) + pi/2 + atan(max(min(cc.K_track*3*(d - ms.loiter_radius),1.2),-1.2));
            ff=V^2/(g*ms.loiter_radius); ey=d - ms.loiter_radius;
        case 'orbit'
            dx=X - orb_c(1); dy=Y - orb_c(2); d=sqrt(dx^2+dy^2);
            chi_c=atan2(dy,dx) + pi/2 + atan(max(min(cc.K_track*3*(d - ms.R_d),1.2),-1.2));
            ff=V^2/(g*ms.R_d); ey=d - ms.R_d;
    end
    phi_t=atan(ff + cc.K_chi*V/g*wrap_pi(chi_c - chi));
    pm=cc.phi_max*pi/180; phi_t=max(min(phi_t,pm),-pm);
    if ph == 1 || ph == 9 || hw < 2,phi_t=0; end
    rl=cc.phi_rate*pi/180*dt; phi_c=phi_c + max(min(phi_t - phi_c,rl),-rl);
    if ph == 2 || ph == 3
        hdot_f=hdot_f + max(min(hdot_c - hdot_f,in.ROC_slew*dt),-in.ROC_slew*dt); hdot_c=hdot_f;
    else
        hdot_f=hdot;
    end
    sch=min(c.q_ref/max(a.qb,1),c.gain_max);
    if isnan(th_force)
        Vs=max(V,5);
        gam_c=asin(max(min(hdot_c/Vs,0.5),-0.5));
        CLn=ac.m*g*cos(gam_c)/(0.5*a.rho*Vs^2*ac.S*cos(phi));
        alpha_ff=(min(CLn,ac.CL_max) - ac.CL0)/ac.CLa;
        int_gam=max(min(int_gam + c.Ki_gamma*(hdot_c - hdot)/Vs*dt,0.2),-0.2);
        th_c=gam_c + alpha_ff + int_gam + c.K_gamma*(hdot_c - hdot)/Vs;
    else
        th_c=th_force*pi/180;
    end
    e_th=theta - th_c;
    if ~onG || (ph==1 && Ntail<=0)
        int_th=max(min(int_th + c.Ki_theta*e_th*dt*sch,de_max),-de_max);
    end
    de_c=int_th + sch*(c.K_theta*e_th + c.K_q*s(8));
    if ph == 1 && Ntail > 0 && e_th > 0,de_c=de_max; end
    sat=abs(de_c) >= de_max;
    de_c=max(min(de_c,de_max),-de_max);
    rl=c.de_rate*pi/180*dt; de=de + max(min(de_c - de,rl),-rl);
    if onG
        da_c=0;
        if V > 6,da_c=sch*(cc.K_phi*phi + cc.K_p*s(7)); end
        if ph == 1,psi_ref=-cc.K_gnd_y*Y; else,psi_ref=pi + cc.K_gnd_y*Y; end
        dr_c=fix.rudder + sch*(cc.K_gnd_psi*wrap_pi(psi - psi_ref) + cc.K_gnd_r*s(9));
        int_phi=0; int_beta=0;
    else
        int_phi=max(min(int_phi + cc.Ki_phi*(phi - phi_c)*dt*sch,da_max),-da_max);
        da_c=int_phi + sch*(cc.K_phi*(phi - phi_c) + cc.K_p*s(7));
        rw=s(9) - g/max(V,5)*sin(phi)*cos(theta);
        int_beta=max(min(int_beta - cc.Ki_beta*a.beta*dt*sch,dr_max),-dr_max);
        dr_c=fix.rudder + int_beta + sch*(-cc.K_beta*a.beta + cc.K_r*rw);
    end
    da_c=max(min(da_c,da_max),-da_max); dr_c=max(min(dr_c,dr_max),-dr_max);
    rl=cc.da_rate*pi/180*dt; da=da + max(min(da_c - da,rl),-rl);
    rl=cc.dr_rate*pi/180*dt; dr=dr + max(min(dr_c - dr,rl),-rl);
    if isnan(d_cmd)
        e_V=V_c - V;
        int_V=max(min(int_V + c.Ki_V*e_V*dt,1),0);
        d_cmd=max(min(int_V + c.K_V*e_V,1),0);
    end
    d_act=d_act + (d_cmd - d_act)*min(dt/c.tau_thr,1);
    if t - t_prop >= c.dt_prop - 1e-9 || first
        switch ph
            case {1,2},ax='takeoff'; case 3,ax='climb'; case {4,6},ax='cruise';
            case 5,ax='loiter'; case 7,ax='descent'; otherwise,ax='landing';
        end
        pr=FM2_Propulsion(d_act,V,a.rho,1-Ah/in.batt.Q_Ah,FM1_Aux(ax,in),in,n_prop);
        n_prop=pr.n; t_prop=t; first=false;
        pp=FM3_Slip(pr.T,pr.P_shaft,pr.n*(pr.T>0),V,a.rho,ac,cc);
        P_el=pr.P_elec; I_b=pr.I_b; I_m=pr.I_m; V_b=pr.V_b; eta_p=pr.eta_p;
    end
    if t - t_log >= (c.dt_log - 1e-9)*(1 - 0.5*(dt < c.dt_air))
        k=k + 1; t_log=t;
        L.t(k)=t; L.x(k)=path; L.X(k)=X; L.Y(k)=Y; L.h(k)=max(hw,0); L.V(k)=V; L.T(k)=pp.T; L.D(k)=a.D; L.CL(k)=a.CL; L.CD(k)=a.CD; L.CY(k)=a.CY; L.Cm(k)=a.Cm;
        L.P(k)=P_el; L.I_b(k)=I_b; L.I_m(k)=I_m; L.V_b(k)=V_b; L.RPM(k)=pp.n*60; L.d(k)=d_act; L.eta_p(k)=eta_p; L.P_shaft(k)=pp.Q*2*pi*pp.n; L.Q(k)=pp.Q;
        L.ROC(k)=hdot; L.theta(k)=theta*180/pi; L.alpha(k)=a.alpha*180/pi; L.beta(k)=a.beta*180/pi; L.gamma(k)=asind(max(min(hdot/max(V,1),1),-1));
        L.phi(k)=phi*180/pi; L.psi(k)=psi*180/pi; L.chi(k)=chi*180/pi; L.p(k)=s(7)*180/pi; L.q(k)=s(8)*180/pi; L.r(k)=s(9)*180/pi;
        L.de(k)=de*180/pi; L.da(k)=da*180/pi; L.dr(k)=dr*180/pi; L.theta_cmd(k)=th_c*180/pi; L.phi_cmd(k)=phi_c*180/pi; L.V_cmd(k)=V_c;
        L.Nr(k)=a.Nw(1); L.Nl(k)=a.Nw(2); L.Nt(k)=a.Nw(3); L.stalled(k)=a.stalled; L.de_sat(k)=sat; L.phase(k)=ph; L.m(k)=ac.m; L.Ah_(k)=Ah; L.Wh_(k)=Wh; L.qb(k)=a.qb;
        L.eps(k)=pp.eps*180/pi; L.fv(k)=pp.fv; L.orbit(k)=orbit; L.ey(k)=ey;
    end
    u0=struct('de',de,'da',da,'dr',dr,'hub',fix.hub,'dt',cc.tailwheel_k*(dr-fix.rudder));
    k1=FM3_Deriv(s,u0,pp,ac.m,ac,cm,in);
    k2=FM3_Deriv(s + dt/2*k1,u0,pp,ac.m,ac,cm,in);
    k3=FM3_Deriv(s + dt/2*k2,u0,pp,ac.m,ac,cm,in);
    k4=FM3_Deriv(s + dt*k3,u0,pp,ac.m,ac,cm,in);
    s=s + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    s(12)=wrap_0(s(12));
    path=path + V*dt;
    Ah=Ah + I_b*dt/3600; Wh=Wh + P_el*dt/3600;
    t=t + dt; t_ph=t_ph + dt;
    if ph == 1 && t > cc.t_takeoff_limit
        ms.fail_reason=sprintf('Did not lift off in %.0f s: highest speed reached %.1f m/s',cc.t_takeoff_limit,max(L.V)); break
    end
    if abs(s(10)) > 75*pi/180 || abs(s(11)) > 75*pi/180 || any(~isfinite(s))
        ms.fail_reason=sprintf('Lost control at t = %.1f s in phase %d: bank %.0f deg, pitch %.0f deg',t,ph,s(10)*180/pi,s(11)*180/pi); break
    end
end
edges(end+1)=t;
L.SoC=1 - L.Ah_/in.batt.Q_Ah;
ms.finished=ph > 9;
ms.log=L; ms.phase=names(1:numel(edges)-1); ms.ph_t=diff(edges);
for i=1:numel(ms.ph_t)
    idx=L.phase==i; ii=find(idx);
    a1=L.Ah_(ii(1)); w1=L.Wh_(ii(1));
    if i < numel(ms.ph_t),jj=find(L.phase==i+1,1); a2=L.Ah_(jj); w2=L.Wh_(jj); else,a2=Ah; w2=Wh; end
    ms.ph_Wh(i)=w2-w1; ms.ph_Ah(i)=a2-a1;
    ms.ph_Pavg(i)=ms.ph_Wh(i)*3600/max(ms.ph_t(i),1e-6);
    ms.ph_Iavg(i)=ms.ph_Ah(i)*3600/max(ms.ph_t(i),1e-6);
    ms.ph_x(i)=L.x(ii(end)) - L.x(ii(1));
    ms.ph_T(i)=mean(L.T(idx)); ms.ph_V(i)=L.V(ii(end)); ms.ph_h(i)=L.h(ii(end));
    ms.ph_alpha(i)=mean(L.alpha(idx)); ms.ph_theta(i)=mean(L.theta(idx)); ms.ph_de(i)=mean(L.de(idx));
    ms.ph_da(i)=mean(L.da(idx)); ms.ph_dr(i)=mean(L.dr(idx)); ms.ph_beta(i)=mean(L.beta(idx)); ms.ph_phi(i)=mean(L.phi(idx));
    ms.ph_betamax(i)=max(abs(L.beta(idx))); ms.ph_drmax(i)=max(abs(L.dr(idx))); ms.ph_damax(i)=max(abs(L.da(idx))); ms.ph_demax(i)=max(abs(L.de(idx)));
end
ms.t_total=t; ms.x_total=path; ms.Wh=Wh; ms.Ah=Ah;
ms.SoC_end=1 - Ah/in.batt.Q_Ah; ms.feasible_energy=ms.SoC_end >= in.batt.SoC_min;
ms.I_peak=max(L.I_b); ms.I_motor_peak=max(L.I_m);
ms.stall_flag=any(L.stalled); ms.t_stall=sum(diff([L.t t]).*L.stalled);
ms.t_at_reserve=NaN; j=find(L.SoC < in.batt.SoC_min,1);
if ~isempty(j),ms.t_at_reserve=L.t(j); end
ms.y_max_ground=max(abs(L.Y(L.phase==1 | L.phase==9)));
end
function a=wrap_pi(a)
a=atan2(sin(a),cos(a));
end
function a=wrap_0(a)
a=atan2(sin(a),cos(a));
end
