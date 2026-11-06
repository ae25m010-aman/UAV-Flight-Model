clc; clear; close all;
evalc('Flight_Model_Wing_Detail');
ac0=struct('m',Sum_W/1000,'m_payload',W6/1000,'S',S/1e6,'b',b/1000,'c',c/1000,'K',K,'CD0',C_D0,...
    'CL_max',C_L_max,'CL0',C_L0,'CLa',C_L_alpha_total,'h_wing_ground',(H_fuse+0.03*c-z_main_contact)/1000);
clearvars -except ac0
in0=FM1_Inputs();
in0.dt_air=2;
kg_per_Ah=0.621/in0.batt.Q_Ah;
fprintf('%-28s %9s %8s %9s %8s\n','Case','Pack mAh','Pack g','AUW kg','Wh');
cases={'Full mission',false};
for ic=1:size(cases,1)
    in=in0; in.drop_payload=cases{ic,2};
    Q=8;
    for it=1:4
        in.batt.Q_Ah=Q;
        in.batt.R_pack=in0.batt.R_pack*in0.batt.Q_Ah/Q;
        ac=ac0; ac.m=ac0.m + kg_per_Ah*(Q - in0.batt.Q_Ah);
        rho=FM1_Atmosphere(in.h_cruise);
        pf.V_stall=sqrt(2*ac.m*9.81/(rho*ac.S*ac.CL_max));
        pf.gamma_glide=atand(2*sqrt(ac.CD0*ac.K));
        ms=FM1_Mission(ac,in,pf);
        Q=Q*(1 - ms.SoC_end)/(1 - in.batt.SoC_min);
    end
    fprintf('%-28s %9.0f %8.0f %9.3f %8.1f\n',cases{ic,1},Q*1000,kg_per_Ah*Q*1000,ac.m,ms.Wh);
end
