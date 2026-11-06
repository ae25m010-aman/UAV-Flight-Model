clc; clear; close all;
evalc('Flight_Model_Wing_Detail');
ac.m=Sum_W/1000; ac.m_payload=W6/1000;
ac.S=S/1e6; ac.b=b/1000; ac.c=c/1000; ac.AR=AR_w; ac.e=e;
ac.K=K; ac.CD0=C_D0; ac.CL_max=C_L_max;
ac.CL0=C_L0; ac.CLa=C_L_alpha_total;
ac.alpha_trim=alpha_trim; ac.CL_trim=C_L_star; ac.V_trim=V_star;
ac.Xcg=Xcg; ac.Zcg=Zcg;
ac.h_wing_ground=(H_fuse + z_chord*c - z_main_contact)/1000;
g=struct('H',H_fuse,'W_f',W_f,'Nr',Nose_Hori_run,'Rr',rear_Hori_run,'L_f',L_f,'nose_low',58,'rear_h',60,...
    'c',c,'b',b,'x_le',x_w_le,'x_spar',x_w_load,'z_chord',z_chord,'ct',c_t,'bt',b_t,'cv',c_v,'hv',h_v,'Xt',X_tail,...
    'Xb',X_boom_start,'L_total',L_total,'boom_w',12.7,'boom_h',26,'z_boom',H_fuse-26/2,...
    'x_main',170.3,'lg_x',26,'lg_t',12.7,'block',15,'y_leg',y_main_leg,'leg_drop',leg_drop,'L_leg',L_main_leg,...
    'z_axle',z_main_axle,'rw',r_main_wheel,'x_tw',L_total-l_tail_lg/2,'L_strut',L_tail_strut,'rt',r_tail_wheel,...
    'z_tw_contact',z_tail_contact,'z_main_contact',z_main_contact,'theta_ground',theta_ground,'prop_D',13*25.4,...
    'Xcg',Xcg,'Zcg',Zcg,'Xn',h_n*c);
g.str_x=[X56 X57 X58 X59 X61]; g.str_z=[Z56 Z57 Z58 Z59 Z61];
g.alpha_stall=(C_L_max - C_L0)/C_L_alpha_total*180/pi + i_w*180/pi;
g.W=W; g.X=X; g.Y=Y; g.Z=Z;
clearvars -except ac g
in=FM1_Inputs();
in.out.folder=FM_OutFolder(in);
fid=fopen(fullfile(in.out.folder,'FM1_Figure_List.txt'),'w');
fprintf(fid,'FILES SAVED BY FM1_PointMass.m\nG = geometry, P = performance, M = mission\n\n'); fclose(fid);
summary_file=fullfile(in.out.folder,'FM1_Results_Summary.txt');
if exist(summary_file,'file'),delete(summary_file); end
diary(summary_file);
pf=FM1_Performance(ac,in);
ms=FM1_Mission(ac,in,pf);
ms.E_usable=pf.E_usable_Wh;
fprintf('AIRCRAFT (from stability code) \n');
fprintf('Mass %.3f kg | S %.4f m2 | W/S %.1f N/m2 | CD0 %.3f | K %.4f | CLmax %.2f\n',...
    ac.m,ac.S,ac.m*9.81/ac.S,ac.CD0,ac.K,ac.CL_max);
fprintf('Trim from stability code: V* %.2f m/s, CL* %.3f, alpha %.2f deg\n',ac.V_trim,ac.CL_trim,ac.alpha_trim);
fprintf('\nPERFORMANCE at %.0f m (rho %.3f) \n',in.h_cruise,pf.rho);
fprintf('Stall speed                 %6.2f m/s  (ground level %.2f m/s)\n',pf.V_stall,pf.V_stall_0);
fprintf('Minimum-thrust speed V_md   %6.2f m/s  CL %.3f  (L/D max %.2f)\n',pf.V_md,pf.CL_md,pf.LD_max);
fprintf('Minimum-power speed V_mp    %6.2f m/s  CL %.3f\n',pf.V_mp,pf.CL_mp);
fprintf('Max level speed             %6.2f m/s\n',pf.V_max);
fprintf('Best climb rate             %6.2f m/s at %.2f m/s\n',pf.ROC_max,pf.V_y);
fprintf('Best climb angle            %6.2f deg at %.2f m/s\n',pf.gamma_max,pf.V_x);
fprintf('Best glide angle            %6.2f deg\n',pf.gamma_glide);
fprintf('Static thrust               %6.2f N (T/W %.2f), %.0f RPM, motor %.1f A, %.0f W\n',...
    pf.T_static,pf.TW_static,pf.RPM_static,pf.I_static,pf.P_static);
fprintf('Battery energy              %6.1f Wh total, %.1f Wh usable\n',pf.E_total_Wh,pf.E_usable_Wh);
fprintf('Best endurance              %6.1f min at %.2f m/s (%.0f W)\n',pf.Endurance_min,pf.V_best_end,pf.P_elec_min);
fprintf('Best range                  %6.1f km at %.2f m/s\n',pf.Range_km,pf.V_best_range);
fprintf('\nLOITER SPEED \n');
fprintf('Relation: (1/3)^(1/4) x %.1f = %.2f m/s\n',in.V_cruise,ms.V_loiter_ideal);
fprintf('Lowest safe speed (%.2f x stall in a %.0f deg bank) = %.2f m/s\n',in.k_stall_margin,in.bank_loiter_deg,ms.V_loiter_floor);
fprintf('At %.2f m/s the wing would need %.0f%% of its maximum lift in the turn\n',ms.V_loiter_ideal,100*(pf.V_stall/ms.V_loiter_ideal)^2/cosd(in.bank_loiter_deg));
fprintf('Used: %.2f m/s, circle radius %.0f m\n',ms.V_loiter,ms.loiter_radius);
fprintf('\nMISSION \n');
fprintf('%-18s %7s %8s %7s %6s %6s %7s %7s %7s %7s\n','Phase','t (s)','x (m)','h end','V end','T (N)','P (W)','I (A)','Wh','mAh');
for i=1:numel(ms.phase)
    fprintf('%-18s %7.1f %8.0f %7.1f %6.2f %6.2f %7.1f %7.2f %7.2f %7.0f\n',ms.phase{i},ms.ph_t(i),ms.ph_x(i),...
        ms.ph_h(i),ms.ph_V(i),ms.ph_T(i),ms.ph_Pavg(i),ms.ph_Iavg(i),ms.ph_Wh(i),ms.ph_Ah(i)*1000);
end
fprintf('%-18s %7.1f %8.0f %7s %6s %6s %7s %7s %7.2f %7.0f\n','TOTAL',ms.t_total,ms.x_total,'','','','','',ms.Wh,ms.Ah*1000);
fprintf('\nTake-off: lift-off %.2f m/s, ground roll %.1f m\n',ms.V_liftoff,ms.S_roll);
fprintf('Airborne: lift-off to %.0f m height in %.1f s and %.1f m; speed there %.2f m/s. Total take-off distance %.1f m\n',in.h_screen,ms.ph_t(2),ms.S_airborne,ms.V_screen,ms.S_to_screen);
fprintf('Landing : approach %.2f m/s, touchdown %.2f m/s, air %.1f m, ground roll %.1f m\n',ms.V_approach,ms.V_touchdown,ms.S_land_air,ms.S_land_roll);
fprintf('Descent : %.2f deg, %.2f m/s sink\n',ms.gamma_descent,ms.ROD);
fprintf('Peak current: battery %.1f A (limit %.0f A), motor %.1f A (limit %.0f A)\n',ms.I_peak,in.batt.I_cont,ms.I_motor_peak,in.mot.I_max);
fprintf('\nMISSION CHECK \n');
fprintf('Energy used %.1f Wh, usable %.1f Wh. State of charge at landing %.1f%% (limit %.0f%%)\n',ms.Wh,pf.E_usable_Wh,ms.SoC_end*100,in.batt.SoC_min*100);
t_loiter_max=in.t_loiter + (in.batt.Q_Ah*(1-in.batt.SoC_min) - ms.Ah)*3600/ms.ph_Iavg(5);
fprintf('Longest loiter this battery allows with these cruise legs: %.0f s\n',t_loiter_max);
if ms.feasible_energy
    fprintf('RESULT: MISSION FEASIBLE ON BATTERY ENERGY\n');
else
    fprintf('RESULT: MISSION NOT FEASIBLE ON THIS BATTERY\n');
    fprintf('  Reserve reached at t = %.0f s, in phase: %s\n',ms.t_at_reserve,ms.phase{ms.phase_at_reserve});
    if ~isnan(ms.t_empty),fprintf('  Battery empty at t = %.0f s\n',ms.t_empty); end
end
if ms.loiter_limited
    fprintf('NOTE: the ideal loiter speed is below the safe minimum; the safe minimum was used.\n');
end
if ms.stall_flag,fprintf('WARNING: CL went above CLmax somewhere in the mission.\n'); end
fprintf('\nGROUND ATTITUDE \n');
fprintf('INPUTS in Flight_Model_Wing_Detail.m: main leg drop %.2f mm, tail strut %.2f mm\n',g.leg_drop,g.L_strut);
fprintf('Result: sits %.2f deg nose-up, main leg length %.1f mm, main axle z %.1f mm\n',g.theta_ground,g.L_leg,g.z_axle);
CL_rest=ac.CL0 + ac.CLa*g.theta_ground*pi/180;
fprintf('Stall angle %.2f deg (CLmax %.2f). At rest: CL %.3f, margin to stall %.2f deg\n',g.alpha_stall,ac.CL_max,CL_rest,g.alpha_stall - g.theta_ground);
if g.theta_ground >= g.alpha_stall
    fprintf('STALL WARNING: at this ground attitude the wing is at or beyond its stall angle\n');
else
    fprintf('Stall check: OK, the wing is below its stall angle on the ground\n');
end
gap=(g.H/2 - g.prop_D/2) - g.z_main_contact;
fprintf('Propeller tip to ground, fuselage level: %.1f mm\n',gap);
if gap <= 0,fprintf('PROPELLER STRIKE WARNING: the tip reaches the ground with the fuselage level\n'); end
if g.leg_drop < 0 || g.L_strut < 0,fprintf('INPUT WARNING: a gear length is negative\n'); end
fprintf('CG is %.1f mm behind the main axle; tail wheel carries %.1f%% of the weight\n',g.Xcg-g.x_main,(g.Xcg-g.x_main)/(g.x_tw-g.x_main)*100);
vn=FM1_Vn(ac,in,pf,ms);
fprintf('\nV-n DIAGRAM at %.0f m \n',in.h_cruise);
fprintf('Stall V_S %.2f | Loiter V_L %.2f | Cruise V_C %.2f | Maneuvering V_A %.2f | Dive V_D %.2f m/s\n',vn.V_S,vn.V_L,vn.V_C,vn.V_A,vn.V_D);
fprintf('Stall curve reaches n+ at %.2f m/s (envelope corner)\n',vn.V_corner);
fprintf('Load limits n+ %.2f, n- %.2f. Negative stall %.2f m/s, reaches n- at %.2f m/s\n',vn.n_pos,vn.n_neg,vn.V_Sneg,vn.V_G);
fprintf('Largest load factor flown in the mission: %.2f; highest speed flown %.2f m/s\n',max(vn.n_log),max(ms.log.V));
if any(vn.outside)
    fprintf('%d mission points are outside the envelope\n',sum(vn.outside));
else
    fprintf('The whole mission stays inside the envelope\n');
end
if in.vn.show_gust
    fprintf('Gust load factor: %.2f at V_C (%.2f m/s gust), %.2f at V_D (%.2f m/s gust)\n',vn.n_gC,in.vn.U_gust_C,vn.n_gD,in.vn.U_gust_D);
    if max(vn.n_gC,vn.n_gD) > vn.n_pos,fprintf('GUST NOTE: gust load factor is above the %.2f limit\n',vn.n_pos); end
end
if pf.V_max > vn.V_D,fprintf('SPEED NOTE: max level speed %.2f m/s at full power is above V_D %.2f m/s\n',pf.V_max,vn.V_D); end
diary off;
FM1_GeometryFigures(g,in);
FM1_PerformanceFigures(pf,ac,in,ms);
FM1_MissionFigures(ms,ac,in);
FM1_VnFigures(vn,ms,in);
if in.out.make_video,FM1_Video(ms,ac,in,g,vn); end
fprintf('\nAll results saved in folder: %s\n',in.out.folder);
