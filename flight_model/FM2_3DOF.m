clc; clear; close all;
evalc('Flight_Model_Wing_Detail');
ac.m=Sum_W/1000; ac.m_payload=W6/1000; ac.S=S/1e6; ac.b=b/1000; ac.c=c/1000;
ac.K=K; ac.CD0=C_D0; ac.CL_max=C_L_max; ac.CL0=C_L0; ac.CLa=C_L_alpha_total;
ac.CLq=C_Lq1; ac.CLde=C_L_de; ac.Cm0=C_M_0; ac.Cma=C_M_alpha; ac.Cmq=C_mq1; ac.Cmde=C_m_de;
ac.Iyy=Iyy; ac.de_max=delta_e_max; ac.theta_ground=theta_ground;
ac.alpha_trim=alpha_trim; ac.CL_trim=C_L_star; ac.V_trim=V_star;
ac.Xcg=Xcg; ac.Zcg=Zcg; ac.alpha_stall=(C_L_max - C_L0)/C_L_alpha_total*180/pi;
ac.arm_T=(Zcg - H_fuse/2)/1000;
ac.xb_main=(Xcg - 170.3)/1000;
ac.zb_main=(z_main_contact - Zcg)/1000;
ac.xb_tail=(Xcg - (L_total - l_tail_lg/2))/1000;
ac.zb_tail=(z_tail_contact - Zcg)/1000;
ac.h_wing_cg=(H_fuse + z_chord*c - z_main_contact)/1000;
ac.h_wing_ground=ac.h_wing_cg;
code.w_sp=omega_n_SP; code.z_sp=zeta_SP; code.w_ph=omega_n_Ph; code.z_ph=zeta_Ph; code.V=V_star;
code.re_sp=sigma_SP; code.im_sp=omega_d_SP; code.re_ph=sigma_Ph; code.im_ph=omega_d_Ph;
g=struct('H',H_fuse,'W_f',W_f,'Nr',Nose_Hori_run,'Rr',rear_Hori_run,'L_f',L_f,'nose_low',58,'rear_h',60,...
    'c',c,'b',b,'x_le',x_w_le,'x_spar',x_w_load,'z_chord',z_chord,'ct',c_t,'bt',b_t,'cv',c_v,'hv',h_v,'Xt',X_tail,...
    'Xb',X_boom_start,'L_total',L_total,'boom_w',12.7,'boom_h',26,'z_boom',H_fuse-26/2,...
    'x_main',170.3,'lg_x',26,'lg_t',12.7,'block',15,'y_leg',y_main_leg,'leg_drop',leg_drop,'L_leg',L_main_leg,...
    'z_axle',z_main_axle,'rw',r_main_wheel,'x_tw',L_total-l_tail_lg/2,'L_strut',L_tail_strut,'rt',r_tail_wheel,...
    'z_tw_contact',z_tail_contact,'z_main_contact',z_main_contact,'theta_ground',theta_ground,'prop_D',13*25.4,...
    'Xcg',Xcg,'Zcg',Zcg,'Xn',h_n*c);
clearvars -except ac g code
in=FM1_Inputs();
cc=FM2_Inputs();
in.out.folder=cc.folder; in.out.list_name=cc.list_name;
ac.CL_neg=in.vn.CL_max_neg;
in.out.folder=FM_OutFolder(in);
fid=fopen(fullfile(in.out.folder,in.out.list_name),'w');
fprintf(fid,'FILES SAVED BY FM2_3DOF.m\nD = 3-DOF results, C = comparison with the point-mass model\n\n'); fclose(fid);
summary_file=fullfile(in.out.folder,'FM2_Results_Summary.txt');
if exist(summary_file,'file'),delete(summary_file); end
diary(summary_file);
pf=FM1_Performance(ac,in);
msA=FM1_Mission(ac,in,pf); msA.E_usable=pf.E_usable_Wh;
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
ms=FM2_Mission(ac,cc,in,pf); ms.E_usable=pf.E_usable_Wh;
if ~ms.finished
    fprintf('\nMISSION STOPPED: %s\n',ms.fail_reason);
    fprintf('Ground attitude %.2f deg, stall angle %.2f deg. Main leg drop %.2f mm, tail strut %.2f mm.\n',g.theta_ground,ac.alpha_stall,g.leg_drop,g.L_strut);
    fprintf('Change leg_drop and L_tail_strut in Flight_Model_Wing_Detail.m (lines 67 and 68), then run again.\n');
    diary off;
    return
end
vn=FM1_Vn(ac,in,pf,ms);
fprintf('3-DOF AIRCRAFT DATA (from stability code) \n');
fprintf('Mass %.3f kg | Iyy %.3f kg.m2 | CL0 %.3f | CLa %.3f | CLde %.3f | CLq %.2f /rad\n',ac.m,ac.Iyy,ac.CL0,ac.CLa,ac.CLde,ac.CLq);
fprintf('Cm0 %.4f | Cma %.3f | Cmde %.3f | Cmq %.2f /rad | elevator limit %.0f deg | thrust line %.1f mm below CG\n',ac.Cm0,ac.Cma,ac.Cmde,ac.Cmq,ac.de_max*180/pi,ac.arm_T*1000);
fprintf('\nTRIM IN LEVEL FLIGHT at %.0f m \n',in.h_cruise);
fprintf('%-10s %7s %8s %8s %8s %8s\n','','V (m/s)','alpha','elevator','thrust N','CL');
tc=FM2_Trim(in.V_cruise,in.h_cruise,0,0,ac.m,ac); tl=FM2_Trim(ms.V_loiter,in.h_cruise,0,in.bank_loiter_deg,ac.m,ac); tv=FM2_Trim(vn.V_D,in.h_cruise,0,0,ac.m,ac);
fprintf('%-10s %7.2f %8.2f %8.2f %8.2f %8.3f\n','Cruise',tc.V,tc.alpha,tc.de,tc.T,tc.CL);
fprintf('%-10s %7.2f %8.2f %8.2f %8.2f %8.3f\n','Loiter',tl.V,tl.alpha,tl.de,tl.T,tl.CL);
fprintf('%-10s %7.2f %8.2f %8.2f %8.2f %8.3f\n','Dive speed',tv.V,tv.alpha,tv.de,tv.T,tv.CL);
fprintf('Positive elevator = trailing edge down = nose down.\n');
fprintf('\nLONGITUDINAL MODES \n');
mdc=FM2_Modes(in.V_cruise,in.h_cruise,ac.m,ac,cc,in); mdt=FM2_Modes(code.V,in.h_cruise,ac.m,ac,cc,in);
mode_rows={sprintf('3-DOF at cruise %.2f m/s',in.V_cruise),real(mdc.eig(1)),abs(imag(mdc.eig(1))),mdc.wn_sp,mdc.z_sp,real(mdc.eig(3)),abs(imag(mdc.eig(3))),mdc.wn_ph,mdc.z_ph; ...
            sprintf('3-DOF at trim V* %.2f m/s',code.V),real(mdt.eig(1)),abs(imag(mdt.eig(1))),mdt.wn_sp,mdt.z_sp,real(mdt.eig(3)),abs(imag(mdt.eig(3))),mdt.wn_ph,mdt.z_ph; ...
            sprintf('Stability code at V* %.2f m/s',code.V),code.re_sp,code.im_sp,code.w_sp,code.z_sp,code.re_ph,code.im_ph,code.w_ph,code.z_ph};
fprintf('SHORT PERIOD MODE\n%-34s %24s %12s %10s\n','','Eigenvalue (1/s)','wn (rad/s)','zeta');
for i=1:3
    fprintf('%-34s %24s %12.3f %10.3f\n',mode_rows{i,1},sprintf('%.4f +/- j%.4f',mode_rows{i,2},mode_rows{i,3}),mode_rows{i,4},mode_rows{i,5});
end
fprintf('\nPHUGOID MODE\n%-34s %24s %12s %10s\n','','Eigenvalue (1/s)','wn (rad/s)','zeta');
for i=1:3
    fprintf('%-34s %24s %12.3f %10.3f\n',mode_rows{i,1},sprintf('%.4f +/- j%.4f',mode_rows{i,6},mode_rows{i,7}),mode_rows{i,8},mode_rows{i,9});
end
fprintf('\n3-DOF MISSION \n');
fprintf('%-18s %7s %8s %7s %6s %6s %7s %7s %7s %7s %7s\n','Phase','t (s)','x (m)','h end','V end','T (N)','P (W)','Wh','alpha','theta','elev');
for i=1:numel(ms.phase)
    fprintf('%-18s %7.1f %8.0f %7.1f %6.2f %6.2f %7.1f %7.2f %7.2f %7.2f %7.2f\n',ms.phase{i},ms.ph_t(i),ms.ph_x(i),ms.ph_h(i),ms.ph_V(i),ms.ph_T(i),ms.ph_Pavg(i),ms.ph_Wh(i),ms.ph_alpha(i),ms.ph_theta(i),ms.ph_de(i));
end
fprintf('%-18s %7.1f %8.0f %42s %7.2f\n','TOTAL',ms.t_total,ms.x_total,'',ms.Wh);
fprintf('Mission finished in the simulation: %d\n',ms.finished);
fprintf('\nTake-off: tail lifts at %.2f m/s (%.1f m); lift-off %.2f m/s, ground roll %.1f m\n',ms.V_tail_up,ms.S_tail_up,ms.V_liftoff,ms.S_roll);
fprintf('Airborne: lift-off to %.0f m height in %.1f s and %.1f m; speed there %.2f m/s. Total take-off distance %.1f m\n',in.h_screen,ms.ph_t(2),ms.S_airborne,ms.V_screen,ms.S_to_screen);
fprintf('Landing : approach %.2f m/s, touchdown %.2f m/s, sink %.2f m/s, air %.1f m, ground roll %.1f m\n',ms.V_approach,ms.V_touchdown,ms.sink_touch,ms.S_land_air,ms.S_land_roll);
fprintf('Largest elevator in each phase (deg):'); fprintf(' %.1f',ms.ph_demax); fprintf('  (limit %.0f)\n',ac.de_max*180/pi);
fprintf('Time with elevator at its limit in the air: %d samples\n',ms.de_sat_air);
fprintf('Peak current: battery %.1f A, motor %.1f A (limit %.0f A)\n',ms.I_peak,ms.I_motor_peak,in.mot.I_max);
if ms.stall_flag
    fprintf('STALL WARNING: wing beyond stall for %.1f s of the mission, phases:',ms.t_stall); fprintf(' %d',unique(ms.log.phase(ms.log.stalled>0))); fprintf('\n');
else
    fprintf('Stall check: the wing stays below stall for the whole mission\n');
end
fprintf('\nMISSION CHECK \n');
fprintf('Energy used %.1f Wh, usable %.1f Wh. State of charge at landing %.1f%% (limit %.0f%%)\n',ms.Wh,pf.E_usable_Wh,ms.SoC_end*100,in.batt.SoC_min*100);
t_loiter_max=in.t_loiter + (in.batt.Q_Ah*(1-in.batt.SoC_min) - ms.Ah)*3600/ms.ph_Iavg(5);
fprintf('Longest loiter this battery allows with these cruise legs: %.0f s\n',t_loiter_max);
if ms.feasible_energy,fprintf('RESULT: MISSION FEASIBLE ON BATTERY ENERGY\n');
else,fprintf('RESULT: MISSION NOT FEASIBLE ON THIS BATTERY (reserve reached at t = %.0f s)\n',ms.t_at_reserve); end
fprintf('\n3-DOF AGAINST POINT MASS \n');
fprintf('%-18s %9s %9s %9s %9s\n','Phase','t A (s)','t B (s)','Wh A','Wh B');
for i=1:numel(ms.phase)
    fprintf('%-18s %9.1f %9.1f %9.2f %9.2f\n',ms.phase{i},msA.ph_t(i),ms.ph_t(i),msA.ph_Wh(i),ms.ph_Wh(i));
end
fprintf('%-18s %9.1f %9.1f %9.2f %9.2f\n','TOTAL',msA.t_total,ms.t_total,msA.Wh,ms.Wh);
fprintf('Ground roll: A %.1f m, B %.1f m. Landing roll: A %.1f m, B %.1f m\n',msA.S_roll,ms.S_roll,msA.S_land_roll,ms.S_land_roll);
fprintf('\nV-n \n');
fprintf('Largest load factor flown %.2f, smallest %.2f. Outside the envelope: %d samples\n',max(vn.n_log),min(vn.n_log),sum(vn.outside));
diary off;
FM2_Figures(ms,msA,ac,cc,in,vn);
if in.out.make_video,FM2_Video(ms,ac,in,g,vn,cc); end
fprintf('\nAll results saved in folder: %s\n',in.out.folder);
