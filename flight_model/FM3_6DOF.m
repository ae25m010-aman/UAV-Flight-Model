clc; clear; close all;
evalc('Flight_Model_Wing_Detail');
ac.m=Sum_W/1000; ac.m_payload=W6/1000; ac.S=S/1e6; ac.b=b/1000; ac.c=c/1000;
ac.K=K; ac.CD0=C_D0; ac.CL_max=C_L_max; ac.CL0=C_L0; ac.CLa=C_L_alpha_total;
ac.CLq=C_Lq1; ac.CLde=C_L_de; ac.Cm0=C_M_0; ac.Cma=C_M_alpha; ac.Cmq=C_mq1; ac.Cmde=C_m_de;
ac.Iyy=Iyy; ac.Ixx=Ixx; ac.Izz=Izz; ac.Ixz=sum((W/1000).*((X-Xcg)/1000).*((Z-Zcg)/1000));
ac.CYb=C_Y_beta; ac.CYp=C_Y_p2; ac.CYr=C_Y_r1_VT; ac.CYdr=C_Y_dr;
ac.Clb=C_l_beta; ac.Clp=C_l_p2; ac.Clr_VT=C_l_r1_VT; ac.Clda=C_l_da; ac.Cldr=C_l_dr;
ac.Cnb=C_n_beta; ac.Cnp_VT=C_n_p2_VT; ac.Cnr=C_n_r1 + C_n_r2; ac.Cnda=C_n_da; ac.Cndr=C_n_dr;
ac.de_max=delta_e_max; ac.da_max=delta_a_max; ac.dr_max=delta_r_max;
ac.theta_ground=theta_ground; ac.alpha_stall=(C_L_max - C_L0)/C_L_alpha_total*180/pi;
ac.alpha_trim=alpha_trim; ac.CL_trim=C_L_star; ac.V_trim=V_star; ac.Xcg=Xcg; ac.Zcg=Zcg;
ac.arm_T=(Zcg - H_fuse/2)/1000;
ac.x_prop=(Xcg + 40)/1000;
ac.prop_R=13*0.0254/2; ac.hv=h_v/1000; ac.bt=b_t/1000; ac.Sv=S_v/1e6;
ac.CLa_VT=C_L_alpha_VT; ac.l_v=l_v/1000;
ac.dz_tail_hub=(H_fuse-26/2 - H_fuse/2)/1000;
ac.dz_hub_cg_up=(H_fuse/2 - Zcg)/1000;
ac.kCY=(S_v/S)*pi*e_t*AR_v;
y_wh=(W_f/2 + 192.6 + 12)/1000;
ac.wheels=[(Xcg-170.3)/1000 y_wh (Zcg-z_main_contact)/1000; ...
            (Xcg-170.3)/1000 -y_wh (Zcg-z_main_contact)/1000; ...
            (Xcg-(L_total-l_tail_lg/2))/1000 0 (Zcg-z_tail_contact)/1000];
ac.xb_main=(Xcg-170.3)/1000; ac.zb_main=(z_main_contact-Zcg)/1000;
ac.xb_tail=(Xcg-(L_total-l_tail_lg/2))/1000; ac.zb_tail=(z_tail_contact-Zcg)/1000;
ac.h_wing_cg=(H_fuse + z_chord*c - z_main_contact)/1000; ac.h_wing_ground=ac.h_wing_cg;
code.V=V_star; code.sp=complex(sigma_SP,omega_d_SP); code.ph=complex(sigma_Ph,omega_d_Ph);
el=eig_vals_lat(:); ic=find(abs(imag(el))>1e-6); ir=find(abs(imag(el))<=1e-6);
code.dr=complex(real(el(ic(1))),abs(imag(el(ic(1)))));
[~,j]=sort(abs(el(ir)),'descend'); code.roll=el(ir(j(1))); code.spiral=el(ir(j(2)));
g=struct('H',H_fuse,'W_f',W_f,'Nr',Nose_Hori_run,'Rr',rear_Hori_run,'L_f',L_f,'nose_low',58,'rear_h',60,...
    'c',c,'b',b,'x_le',x_w_le,'x_spar',x_w_load,'z_chord',z_chord,'ct',c_t,'bt',b_t,'cv',c_v,'hv',h_v,'Xt',X_tail,...
    'Xb',X_boom_start,'L_total',L_total,'boom_w',12.7,'boom_h',26,'z_boom',H_fuse-26/2,...
    'x_main',170.3,'lg_x',26,'lg_t',12.7,'block',15,'y_leg',y_main_leg,'leg_drop',leg_drop,'L_leg',L_main_leg,...
    'z_axle',z_main_axle,'rw',r_main_wheel,'x_tw',L_total-l_tail_lg/2,'L_strut',L_tail_strut,'rt',r_tail_wheel,...
    'z_tw_contact',z_tail_contact,'z_main_contact',z_main_contact,'theta_ground',theta_ground,'prop_D',13*25.4,...
    'Xcg',Xcg,'Zcg',Zcg,'Xn',h_n*c);
clearvars -except ac g code
in=FM1_Inputs(); c2=FM2_Inputs(); cc=FM3_Inputs();
in.out.folder=cc.folder; in.out.list_name=cc.list_name;
ac.CL_neg=in.vn.CL_max_neg;
cm=c2; cm.mu_side=cc.mu_side; cm.prop_inertia=cc.prop_inertia; cm.k_pfactor=cc.k_pfactor;
in.out.folder=FM_OutFolder(in);
fid=fopen(fullfile(in.out.folder,in.out.list_name),'w');
fprintf(fid,'FILES SAVED BY FM3_6DOF.m\nS = 6-DOF results, R = rudder and hub, C = comparison with the other models\n\n'); fclose(fid);
summary_file=fullfile(in.out.folder,'FM3_Results_Summary.txt');
if exist(summary_file,'file'),delete(summary_file); end
diary(summary_file);
dx_gear=g.x_tw - g.x_main;
fprintf('GROUND ATTITUDE \n');
fprintf('Main leg drop %.2f mm, tail strut %.2f mm: sits %.2f deg nose-up. Stall angle %.2f deg, margin %.2f deg\n',g.leg_drop,g.L_strut,g.theta_ground,ac.alpha_stall,ac.alpha_stall - g.theta_ground);
if g.theta_ground >= ac.alpha_stall
    fprintf('STALL WARNING: the wing is stalled at the ground attitude. Take-off is likely to fail.\n');
end
for th_try=[10 8]
    fprintf('  For %.0f deg with this main leg, use tail strut %.0f mm\n',th_try,g.z_boom - g.rt - (g.z_main_contact + dx_gear*tand(th_try)));
end
fprintf('\n');
h=in.h_cruise; h2=h/2;
base=FM3_PropTrim(in.V_cruise,0,0,h,ac,cc,in);
fix_r=base.dr; fix_h=base.hub;
if ~isempty(cc.rudder_fix_deg),fix_r=cc.rudder_fix_deg; end
if ~isempty(cc.hub_fix_deg),fix_h=cc.hub_fix_deg; end
gam_cl=asind(in.ROC_climb/in.V_climb);
pf=FM1_Performance(ac,in);
msA=FM1_Mission(ac,in,pf);
gam_ds=-msA.gamma_descent; V_L=msA.V_loiter;
rows={'Airborne, full power',20,15,0,50; 'Climb',in.V_climb,gam_cl,0,h2; 'Cruise',in.V_cruise,0,0,h; 'Loiter',V_L,0,in.bank_loiter_deg,h; 'Descent',in.V_descent,gam_ds,0,h2};
for i=1:size(rows,1)
    TB(i)=FM3_PropTrim(rows{i,2},rows{i,3},rows{i,4},rows{i,5},ac,cc,in,fix_r,fix_h);
end
fprintf('PROPELLER EFFECTS AND CORRECTION (clockwise from behind) \n');
fprintf('Yaw and roll from the propeller, N.m. Negative yaw = nose left, negative roll = left wing down.\n');
fprintf('%-22s %6s %7s %7s %8s %8s %8s %7s\n','Phase','V','T (N)','RPM','P-factor','Swirl','Torque','Swirl deg');
for i=1:numel(TB)
    fprintf('%-22s %6.2f %7.2f %7.0f %8.3f %8.3f %8.3f %7.2f\n',rows{i,1},TB(i).V,TB(i).T,TB(i).RPM,TB(i).N_pf,TB(i).N_sw,TB(i).L_tq,TB(i).eps);
end
fprintf('\nAngle needed in each phase for zero sideslip (rudder + = yaw left; hub + = thrust towards right wing):\n');
fprintf('%-22s %10s %10s %10s %12s\n','Phase','Rudder deg','Hub deg','Aileron','Beta if none');
for i=1:numel(TB)
    fprintf('%-22s %10.2f %10.2f %10.2f %12.2f\n',rows{i,1},TB(i).dr,TB(i).hub,TB(i).da_a,TB(i).beta_none);
end
fprintf('\nIf set once at cruise: rudder %.2f deg, or hub %.2f deg. Rudder angle the autopilot must still add:\n',fix_r,fix_h);
fprintf('%-22s %14s %14s\n','Phase','With rudder fix','With hub fix');
for i=1:numel(TB)
    fprintf('%-22s %14.2f %14.2f\n',rows{i,1},TB(i).dr_left_rudder,TB(i).dr_left_hub);
end
fprintf('The torque roll is not removed by either option: aileron trim is needed in both.\n');
cases={'none',0,0; 'rudder',fix_r,0; 'hub',0,fix_h};
fprintf('\nMISSION WITH EACH OPTION \n');
fprintf('%-8s %8s %10s %10s %10s %10s %10s %10s\n','Option','Wh','Cruise dr','Cruise da','Cruise beta','Climb dr','Max |beta| air','SoC end');
for i=1:3
    fx=struct('rudder',cases{i,2}*pi/180,'hub',cases{i,3}*pi/180);
    m3=FM3_Mission(ac,c2,cc,in,pf,fx);
    if ~m3.finished
    fprintf('\nMISSION STOPPED: %s\n',m3.fail_reason);
    fprintf('Ground attitude %.2f deg, stall angle %.2f deg. Main leg drop %.2f mm, tail strut %.2f mm.\n',g.theta_ground,ac.alpha_stall,g.leg_drop,g.L_strut);
    fprintf('Change leg_drop and L_tail_strut in Flight_Model_Wing_Detail.m (lines 67 and 68), then run again.\n');
    diary off;
    return
    end
    fprintf('%-8s %8.2f %10.2f %10.2f %10.2f %10.2f %10.2f %9.1f%%\n',cases{i,1},m3.Wh,m3.ph_dr(4),m3.ph_da(4),m3.ph_beta(4),m3.ph_dr(3),max(m3.ph_betamax(3:7)),m3.SoC_end*100);
    CMP(i)=struct('name',cases{i,1},'Wh',m3.Wh,'dr',m3.ph_dr,'beta',m3.ph_beta);
    if strcmp(cases{i,1},cc.yaw_fix),ms=m3; fix=fx; end
end
ms.E_usable=pf.E_usable_Wh; ms.fix_name=cc.yaw_fix;
ms2=FM2_Mission(ac,c2,in,pf); ms2.E_usable=pf.E_usable_Wh;
vn=FM1_Vn(ac,in,pf,ms);
fprintf('\n6-DOF DATA (from stability code) \n');
fprintf('Ixx %.3f, Iyy %.3f, Izz %.3f, Ixz %.3f kg.m2\n',ac.Ixx,ac.Iyy,ac.Izz,ac.Ixz);
fprintf('CYb %.3f, Clb %.4f, Cnb %.4f, Clp %.3f, Cnr %.3f, Clda %.3f, Cnda %.4f, Cndr %.4f, CYdr %.4f /rad\n',ac.CYb,ac.Clb,ac.Cnb,ac.Clp,ac.Cnr,ac.Clda,ac.Cnda,ac.Cndr,ac.CYdr);
fprintf('\nMODES \n');
mc=FM3_Modes(in.V_cruise,h,ac,cm,in); mt=FM3_Modes(code.V,h,ac,cm,in);
nm={sprintf('6-DOF at cruise %.2f m/s',in.V_cruise),sprintf('6-DOF at trim V* %.2f m/s',code.V),sprintf('Stability code at V* %.2f m/s',code.V)};
S={mc,mt,code};
ttl={'SHORT PERIOD','sp'; 'PHUGOID','ph'; 'DUTCH ROLL','dr'};
for q=1:3
    fprintf('%s\n%-34s %24s %12s %10s\n',ttl{q,1},'','Eigenvalue (1/s)','wn (rad/s)','zeta');
    for i=1:3
        ev=S{i}.(ttl{q,2});
        fprintf('%-34s %24s %12.3f %10.3f\n',nm{i},sprintf('%.4f +/- j%.4f',real(ev),abs(imag(ev))),abs(ev),-real(ev)/abs(ev));
    end
end
fprintf('ROLL AND SPIRAL (real eigenvalues)\n%-34s %12s %12s %16s\n','','Roll (1/s)','Spiral (1/s)','Spiral doubles (s)');
for i=1:3
    fprintf('%-34s %12.3f %12.3f %16.1f\n',nm{i},real(S{i}.roll),real(S{i}.spiral),log(2)/max(real(S{i}.spiral),1e-6));
end
fprintf('\n6-DOF MISSION, correction: %s \n',cc.yaw_fix);
fprintf('%-18s %7s %8s %7s %6s %6s %7s %7s %7s %7s %7s %7s\n','Phase','t (s)','x (m)','h end','V end','T (N)','P (W)','Wh','elev','ail','rud','beta');
for i=1:numel(ms.phase)
    fprintf('%-18s %7.1f %8.0f %7.1f %6.2f %6.2f %7.1f %7.2f %7.2f %7.2f %7.2f %7.2f\n',ms.phase{i},ms.ph_t(i),ms.ph_x(i),ms.ph_h(i),ms.ph_V(i),ms.ph_T(i),ms.ph_Pavg(i),ms.ph_Wh(i),ms.ph_de(i),ms.ph_da(i),ms.ph_dr(i),ms.ph_beta(i));
end
fprintf('%-18s %7.1f %8.0f %42s %7.2f\n','TOTAL',ms.t_total,ms.x_total,'',ms.Wh);
fprintf('Mission finished in the simulation: %d\n',ms.finished);
fprintf('\nTake-off: tail lifts at %.2f m/s (%.1f m); lift-off %.2f m/s, ground roll %.1f m\n',ms.V_tail_up,ms.S_tail_up,ms.V_liftoff,ms.S_roll);
fprintf('Airborne: to %.0f m in %.1f s and %.1f m; speed there %.2f m/s. Total take-off distance %.1f m\n',in.h_screen,ms.ph_t(2),ms.S_airborne,ms.V_screen,ms.S_to_screen);
fprintf('Descent : %d orbit(s) of %.0f m radius, then straight to the runway\n',ms.Nd,ms.R_d);
fprintf('Landing : touchdown %.2f m/s, sink %.2f m/s at X = %.0f m, Y = %.1f m; stops at X = %.0f m, Y = %.1f m; ground roll %.1f m\n',ms.V_touchdown,ms.sink_touch,ms.X_touch,ms.Y_touch,ms.X_stop,ms.Y_stop,ms.S_land_roll);
fprintf('Largest sideways distance from the runway line on the ground: %.2f m (runway half width 15 m)\n',ms.y_max_ground);
fprintf('Largest in each phase (deg):\n');
fprintf('  Elevator'); fprintf(' %.1f',ms.ph_demax); fprintf('  (limit %.0f)\n',ac.de_max*180/pi);
fprintf('  Aileron '); fprintf(' %.1f',ms.ph_damax); fprintf('  (limit %.0f)\n',ac.da_max*180/pi);
fprintf('  Rudder  '); fprintf(' %.1f',ms.ph_drmax); fprintf('  (limit %.0f)\n',ac.dr_max*180/pi);
fprintf('  Beta    '); fprintf(' %.1f',ms.ph_betamax); fprintf('\n');
if ms.stall_flag
    fprintf('STALL WARNING: wing beyond stall for %.1f s\n',ms.t_stall);
else
    fprintf('Stall check: the wing stays below stall for the whole mission\n');
end
fprintf('\nMISSION CHECK \n');
fprintf('Energy used %.1f Wh, usable %.1f Wh. State of charge at landing %.1f%% (limit %.0f%%)\n',ms.Wh,pf.E_usable_Wh,ms.SoC_end*100,in.batt.SoC_min*100);
t_loiter_max=in.t_loiter + (in.batt.Q_Ah*(1-in.batt.SoC_min) - ms.Ah)*3600/ms.ph_Iavg(5);
fprintf('Longest loiter this battery allows with these cruise legs: %.0f s\n',t_loiter_max);
if ms.feasible_energy,fprintf('RESULT: MISSION FEASIBLE ON BATTERY ENERGY\n');
else,fprintf('RESULT: MISSION NOT FEASIBLE ON THIS BATTERY (reserve reached at t = %.0f s)\n',ms.t_at_reserve); end
fprintf('\nAGAINST THE OTHER MODELS \n');
fprintf('%-18s %9s %9s %9s %9s %9s %9s\n','Phase','t PM','t 3DOF','t 6DOF','Wh PM','Wh 3DOF','Wh 6DOF');
for i=1:numel(ms.phase)
    fprintf('%-18s %9.1f %9.1f %9.1f %9.2f %9.2f %9.2f\n',ms.phase{i},msA.ph_t(i),ms2.ph_t(i),ms.ph_t(i),msA.ph_Wh(i),ms2.ph_Wh(i),ms.ph_Wh(i));
end
fprintf('%-18s %9.1f %9.1f %9.1f %9.2f %9.2f %9.2f\n','TOTAL',msA.t_total,ms2.t_total,ms.t_total,msA.Wh,ms2.Wh,ms.Wh);
fprintf('\nV-n: largest load factor flown %.2f. Outside the envelope: %d samples\n',max(vn.n_log),sum(vn.outside));
diary off;
FM3_Figures(ms,ms2,ac,in,vn,TB,rows,CMP,fix_r,fix_h,cc,code,mc,mt);
if in.out.make_video,FM3_Video(ms,ac,in,g,vn); end
fprintf('\nAll results saved in folder: %s\n',in.out.folder);
