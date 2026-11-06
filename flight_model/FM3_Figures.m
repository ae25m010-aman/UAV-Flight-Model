function FM3_Figures(ms,ms2,ac,in,vn,TB,rows,CMP,fix_r,fix_h,cc,code,mc,mt)
L=ms.log; t=L.t; soc=L.SoC*100;
LD=L.CL./max(L.CD,1e-3); LD(L.V<3)=NaN;
list={ ...
 'S01_Altitude',{L.h},{'Height'},'Height above ground (m)','Altitude'; ...
 'S02_Airspeed',{L.V,L.V_cmd},{'Airspeed','Command'},'Airspeed (m/s)','Airspeed against command'; ...
 'S03_Pitch_Attitude',{L.theta,L.theta_cmd},{'Pitch','Command'},'Pitch (deg)','Pitch attitude against command'; ...
 'S04_Bank_Angle',{L.phi,L.phi_cmd},{'Bank','Command'},'Bank (deg), + = right wing down','Bank angle against command'; ...
 'S05_Heading',{L.psi},{'Heading'},'Heading (deg)','Heading'; ...
 'S06_Course',{L.chi},{'Course'},'Course (deg)','Course over the ground'; ...
 'S07_Angle_of_Attack',{L.alpha,ac.alpha_stall},{'Alpha','Stall angle'},'Angle of attack (deg)','Angle of attack against stall angle'; ...
 'S08_Sideslip',{L.beta},{'Beta'},'Sideslip (deg)','Sideslip angle'; ...
 'S09_Flight_Path',{L.gamma},{'Gamma'},'Flight path angle (deg)','Flight path angle'; ...
 'S10_Roll_Rate',{L.p},{'p'},'Roll rate (deg/s)','Roll rate'; ...
 'S11_Pitch_Rate',{L.q},{'q'},'Pitch rate (deg/s)','Pitch rate'; ...
 'S12_Yaw_Rate',{L.r},{'r'},'Yaw rate (deg/s)','Yaw rate'; ...
 'S13_Elevator',{L.de,ac.de_max*180/pi,-ac.de_max*180/pi},{'Elevator','Limit','Limit'},'Elevator (deg), + = nose down','Elevator and limits'; ...
 'S14_Aileron',{L.da,ac.da_max*180/pi,-ac.da_max*180/pi},{'Aileron','Limit','Limit'},'Aileron (deg), + = roll left','Aileron and limits'; ...
 'S15_Rudder',{L.dr,ac.dr_max*180/pi,-ac.dr_max*180/pi},{'Rudder','Limit','Limit'},'Rudder (deg), + = yaw left','Rudder and limits'; ...
 'S16_Rate_of_Climb',{L.ROC},{'ROC'},'Rate of climb (m/s)','Rate of climb'; ...
 'S17_Thrust_and_Drag',{L.T,L.D},{'Thrust','Drag'},'Force (N)','Thrust and drag'; ...
 'S18_Lift_Coefficient',{L.CL,ac.CL_max},{'CL','CLmax'},'CL','Lift coefficient against CLmax'; ...
 'S19_Drag_Coefficient',{L.CD},{'CD'},'CD','Drag coefficient'; ...
 'S20_Side_Force_Coefficient',{L.CY},{'CY'},'CY','Side-force coefficient'; ...
 'S21_Lift_to_Drag',{LD},{'L/D'},'L/D','Lift-to-drag ratio'; ...
 'S22_Dynamic_Pressure',{L.qb},{'q'},'Dynamic pressure (Pa)','Dynamic pressure'; ...
 'S23_Throttle',{L.d*100},{'Throttle'},'Throttle (%)','Throttle'; ...
 'S24_Motor_RPM',{L.RPM},{'RPM'},'RPM','Motor and propeller RPM'; ...
 'S25_Propeller_Torque',{L.Q},{'Torque'},'Torque (N.m)','Propeller torque (rolls the aircraft left)'; ...
 'S26_Slipstream_Swirl',{L.eps},{'Swirl'},'Swirl angle at the tail (deg)','Slipstream swirl angle at the tail'; ...
 'S27_Tail_in_Slipstream',{L.fv*100},{'Share'},'Vertical tail in slipstream (%)','Share of the vertical tail in the slipstream'; ...
 'S28_Battery_Power',{L.P},{'Battery power'},'Power (W)','Battery power'; ...
 'S29_Battery_Current',{L.I_b},{'Current'},'Current (A)','Battery current'; ...
 'S30_Motor_Current',{L.I_m,in.mot.I_max},{'Motor current','Limit'},'Current (A)','Motor current against its limit'; ...
 'S31_Battery_Voltage',{L.V_b},{'Voltage'},'Voltage (V)','Battery voltage under load'; ...
 'S32_State_of_Charge',{soc,in.batt.SoC_min*100},{'SoC','Reserve limit'},'State of charge (%)','State of charge'; ...
 'S33_Energy_Used',{L.Wh_,ms.E_usable},{'Energy used','Usable'},'Energy (Wh)','Energy used against usable energy'; ...
 'S34_Load_Factor',{vn.n_log,vn.n_pos},{'Load factor','n+ limit'},'n','Load factor (lift / weight)'; ...
 'S35_Wheel_Loads',{L.Nr,L.Nl,L.Nt},{'Right main','Left main','Tail'},'Wheel load (N)','Wheel loads'; ...
 'S36_Track_Error',{L.ey},{'Offset'},'Offset from path (m)','Offset from the planned path'};
for k=1:size(list,1)
    FM1_TimePlot(t,list{k,2},list{k,3},ms,list{k,5},list{k,4},list{k,1},list{k,5},in);
end
e=[0 cumsum(ms.ph_t)];
zm={'Takeoff',[0 e(3)+8],37; 'Landing',[e(8)-8 e(end)],43};
zv={{L.V},{'Airspeed'},'Airspeed (m/s)','Speed'; {L.h},{'Height'},'Height (m)','Height'; {L.theta,L.alpha},{'Pitch','Alpha'},'Angle (deg)','Pitch'; ...
      {L.dr,L.da,L.de},{'Rudder','Aileron','Elevator'},'Control (deg)','Controls'; {L.Y},{'Y'},'Sideways from runway line (m)','Runway_Offset'; {L.Nr,L.Nl,L.Nt},{'Right main','Left main','Tail'},'Wheel load (N)','Wheel_Loads'};
for z=1:2
    for k=1:6
        nm=sprintf('S%d_%s_%s',zm{z,3}+k-1,zm{z,1},zv{k,4});
        FM1_TimePlot(t,zv{k,1},zv{k,2},ms,[zm{z,1} ' close-up: ' zv{k,3}],zv{k,3},nm,[zm{z,1} ' close-up: ' zv{k,3}],in,zm{z,2});
    end
end
col=FM1_PhaseColors()*0.75; hh=[];
f=figure('Color','w','Position',[60 60 1200 700]); hold on;
for k=1:9,i=L.phase==k; hh(k)=plot(L.X(i)/1000,L.Y(i)/1000,'Color',col(k,:),'LineWidth',2); end
plot([-100 800 800 -100 -100]/1000,[-15 -15 15 15 -15]/1000,'k-');
axis equal; grid on; box on; xlabel('X (km)'); ylabel('Y (km), + = right of take-off direction'); title('Ground track from the 6-DOF simulation');
legend(hh,ms.phase,'Location','eastoutside'); FM1_SaveFig(f,'S49_Ground_Track','Ground track, by phase',in);
f=figure('Color','w','Position',[60 60 1200 760]); hold on; hh=[];
for k=1:9,i=L.phase==k; hh(k)=plot3(L.X(i)/1000,L.Y(i)/1000,L.h(i),'Color',col(k,:),'LineWidth',2); end
view([-35 28]); grid on; box on; xlabel('X (km)'); ylabel('Y (km)'); zlabel('Height (m)'); title('3D mission path from the 6-DOF simulation');
legend(hh,ms.phase,'Location','eastoutside'); FM1_SaveFig(f,'S50_3D_Path','3D mission path, by phase',in);
f=figure('Color','w','Position',[60 60 1200 560]); hold on; hh=[];
for k=1:9,i=L.phase==k; hh(k)=plot(L.x(i)/1000,L.h(i),'Color',col(k,:),'LineWidth',3); end
grid on; box on; xlabel('Path distance (km)'); ylabel('Height (m)'); title('Mission profile'); legend(hh,ms.phase,'Location','eastoutside');
FM1_SaveFig(f,'S51_Mission_Profile','Mission profile by phase',in);
f=figure('Color','w','Position',[60 60 1250 720]); ax=axes(); FM1_VnDraw(ax,vn,false); set(ax,'FontSize',10); hold(ax,'on'); hh=[];
for k=1:9,i=L.phase==k; hh(k)=plot(ax,L.V(i),vn.n_log(i),'-','Color',col(k,:),'LineWidth',2.2); end
legend(hh,ms.phase,'Location','eastoutside','Interpreter','none'); title(ax,'6-DOF mission on the V-n diagram');
FM1_SaveFig(f,'S52_Vn_Mission','6-DOF mission on the V-n diagram, by phase',in);
n=numel(TB); nm=rows(:,1)';
f=barf([[TB.dr]; [TB.hub]]',nm,{'Rudder (deg)','Hub (deg)'},'Angle (deg)','Angle needed in each phase for zero sideslip');
FM1_SaveFig(f,'R01_Rudder_and_Hub_by_Phase','Rudder or hub angle needed in each phase',in);
f=barf([[TB.dr_left_rudder]; [TB.dr_left_hub]]',nm,{sprintf('Rudder fixed at %.2f deg',fix_r),sprintf('Hub fixed at %.2f deg',fix_h)},'Extra rudder (deg)','Rudder the autopilot must still add when set once at cruise');
FM1_SaveFig(f,'R02_Remaining_Rudder','Rudder still needed after a fixed correction',in);
f=barf([[TB.N_pf]; [TB.N_sw]; [TB.L_tq]]',nm,{'P-factor yaw','Swirl yaw','Torque roll'},'Moment (N.m)','Propeller moments in each phase');
FM1_SaveFig(f,'R03_Propeller_Moments','Propeller yaw and roll moments by phase',in);
f=barf([TB.beta_none]',nm,{'Sideslip'},'Sideslip (deg)','Sideslip if nothing corrects it (neutral rudder)');
FM1_SaveFig(f,'R04_Sideslip_No_Correction','Sideslip with no correction',in);
V=15:0.5:28; [dr,hb,bt,da]=deal(zeros(size(V)));
for k=1:numel(V)
    r=FM3_PropTrim(V(k),0,0,in.h_cruise,ac,cc,in); dr(k)=r.dr; hb(k)=r.hub; bt(k)=r.beta_none; da(k)=r.da_a;
end
f=figure('Color','w','Position',[60 60 1050 580]); hold on;
plot(V,dr,'LineWidth',2,'Color',[0.08 0.32 0.62]); plot(V,hb,'LineWidth',2,'Color',[0.82 0.25 0.1]); plot(V,bt,'LineWidth',2,'Color',[0.1 0.5 0.25]); plot(V,da,'LineWidth',2,'Color',[0.45 0.2 0.6]);
yl=ylim; plot(in.V_cruise*[1 1],yl,'k--'); ylim(yl); grid on; box on; xlabel('Airspeed (m/s)'); ylabel('Angle (deg)');
legend('Rudder needed','Hub needed','Sideslip if none','Aileron needed','Cruise','Location','eastoutside'); title('Level flight: correction against speed');
FM1_SaveFig(f,'R05_Correction_vs_Speed','Rudder, hub, sideslip and aileron against speed',in);
f=barf([CMP.Wh]',{CMP.name},{'Energy'},'Energy (Wh)','Mission energy with each correction');
FM1_SaveFig(f,'R06_Energy_by_Option','Mission energy with each correction',in);
A=reshape([CMP.dr],9,[]);
f=barf(A(3:7,:),ms.phase(3:7),{CMP.name},'Average rudder (deg)','Average rudder in each phase, by correction');
FM1_SaveFig(f,'R07_Rudder_by_Option','Average rudder in each phase, by correction',in);
f=figure('Color','w','Position',[60 60 1050 620]); hold on;
S={mc,mt,code}; mk={'o','s','d'}; cl={[0.08 0.32 0.62],[0.82 0.25 0.1],[0.1 0.5 0.25]}; h=[];
for i=1:3
    pts=[S{i}.sp conj(S{i}.sp) S{i}.ph conj(S{i}.ph) S{i}.dr conj(S{i}.dr) S{i}.roll S{i}.spiral];
    h(i)=plot(real(pts),imag(pts),mk{i},'Color',cl{i},'MarkerSize',9,'LineWidth',1.6);
end
yl=ylim; plot([0 0],yl,'k-'); ylim(yl); grid on; box on; xlabel('Real (1/s)'); ylabel('Imaginary (rad/s)');
legend(h,{'6-DOF at cruise','6-DOF at V*','Stability code at V*'},'Location','northwest'); title('Eigenvalues: short period, phugoid, Dutch roll, roll, spiral');
FM1_SaveFig(f,'R08_Eigenvalue_Map','Eigenvalues of all modes',in);
LA=ms2.log;
cmp={L.h,LA.h,'Height (m)','C01_Altitude_vs_3DOF'; L.V,LA.V,'Airspeed (m/s)','C02_Speed_vs_3DOF'; L.theta,LA.theta,'Pitch (deg)','C03_Pitch_vs_3DOF'; L.Wh_,LA.Wh_,'Energy used (Wh)','C04_Energy_vs_3DOF'};
for k=1:4
    f=figure('Color','w','Position',[60 60 1100 520]);
    plot(t,cmp{k,1},'LineWidth',1.8,'Color',[0.08 0.32 0.62]); hold on; plot(LA.t,cmp{k,2},'--','LineWidth',1.5,'Color',[0.82 0.25 0.1]);
    grid on; xlabel('Mission time (s)'); ylabel(cmp{k,3}); legend('6-DOF','3-DOF'); title(['6-DOF against 3-DOF: ' cmp{k,3}]);
    FM1_SaveFig(f,cmp{k,4},['6-DOF against 3-DOF: ' cmp{k,3}],in);
end
end
function f=barf(Y,labels,names,yl,ttl)
f=figure('Color','w','Position',[60 60 1050 580]); hold on;
c=[0.08 0.32 0.62; 0.82 0.25 0.1; 0.1 0.5 0.25];
[nr,ng]=size(Y); w=0.8/ng; h=[];
for g=1:ng
    for r=1:nr
        x=r - 0.4 + (g-1)*w;
        p=patch([x x+w x+w x],[0 0 Y(r,g) Y(r,g)],c(g,:),'EdgeColor',[.3 .3 .3]);
        if r == 1,h(g)=p; end
        text(x+w/2,Y(r,g),sprintf('%.2f',Y(r,g)),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',9);
    end
end
set(gca,'XTick',1:nr,'XTickLabel',labels,'XTickLabelRotation',15); xlim([0.4 nr+0.6]);
ylabel(yl); title(ttl); grid on; box on;
legend(h,names,'Location','eastoutside','Interpreter','none');
end
