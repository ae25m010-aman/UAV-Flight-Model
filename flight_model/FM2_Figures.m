function FM2_Figures(ms,msA,ac,c,in,vn)
L=ms.log; t=L.t;
Pth=L.T.*L.V;
soc=L.SoC*100;
LD=L.CL./max(L.CD,1e-3); LD(L.V<3)=NaN;
list={ ...
 'D01_Altitude',{L.h},{'Height'},'Height above ground (m)','Altitude'; ...
 'D02_Airspeed',{L.V,L.V_cmd},{'Airspeed','Command'},'Airspeed (m/s)','Airspeed against command'; ...
 'D03_Pitch_Attitude',{L.theta,L.theta_cmd},{'Pitch','Command'},'Pitch attitude (deg)','Pitch attitude against command'; ...
 'D04_Angle_of_Attack',{L.alpha,ac.alpha_stall},{'Alpha','Stall angle'},'Angle of attack (deg)','Angle of attack against stall angle'; ...
 'D05_Flight_Path',{L.gamma},{'Gamma'},'Flight path angle (deg)','Flight path angle'; ...
 'D06_Pitch_Rate',{L.q},{'q'},'Pitch rate (deg/s)','Pitch rate'; ...
 'D07_Elevator',{L.de,ac.de_max*180/pi,-ac.de_max*180/pi},{'Elevator','Limit','Limit'},'Elevator (deg), + = nose down','Elevator deflection and limits'; ...
 'D08_Rate_of_Climb',{L.ROC},{'ROC'},'Rate of climb (m/s)','Rate of climb'; ...
 'D09_Thrust_and_Drag',{L.T,L.D},{'Thrust','Drag'},'Force (N)','Thrust and drag'; ...
 'D10_Lift_Coefficient',{L.CL,ac.CL_max},{'CL','CLmax'},'CL','Lift coefficient against CLmax'; ...
 'D11_Drag_Coefficient',{L.CD},{'CD'},'CD','Drag coefficient'; ...
 'D12_Lift_to_Drag',{LD},{'L/D'},'L/D','Lift-to-drag ratio'; ...
 'D13_Pitching_Moment',{L.Cm},{'Cm'},'Cm (aerodynamic, about CG)','Aerodynamic pitching moment coefficient'; ...
 'D14_Dynamic_Pressure',{L.qb},{'q'},'Dynamic pressure (Pa)','Dynamic pressure'; ...
 'D15_Throttle',{L.d*100},{'Throttle'},'Throttle (%)','Throttle'; ...
 'D16_Motor_RPM',{L.RPM},{'RPM'},'RPM','Motor and propeller RPM'; ...
 'D17_Battery_Power',{L.P},{'Battery power'},'Power (W)','Battery power'; ...
 'D18_Power_Flow',{L.P,L.P_shaft,Pth},{'Battery','Shaft','Thrust power (T.V)'},'Power (W)','Power from battery to thrust'; ...
 'D19_Battery_Current',{L.I_b},{'Battery current'},'Current (A)','Battery current'; ...
 'D20_Motor_Current',{L.I_m,in.mot.I_max},{'Motor current','Limit'},'Current (A)','Motor current against its limit'; ...
 'D21_Battery_Voltage',{L.V_b},{'Voltage'},'Voltage (V)','Battery voltage under load'; ...
 'D22_State_of_Charge',{soc,in.batt.SoC_min*100},{'SoC','Reserve limit'},'State of charge (%)','State of charge'; ...
 'D23_Energy_Used',{L.Wh_,ms.E_usable},{'Energy used','Usable'},'Energy (Wh)','Energy used against usable energy'; ...
 'D24_Propeller_Efficiency',{L.eta_p},{'eta_p'},'Efficiency','Propeller efficiency, with installation loss'; ...
 'D25_Load_Factor',{vn.n_log,vn.n_pos},{'Load factor','n+ limit'},'n','Load factor (lift / weight)'; ...
 'D26_Bank_Angle',{L.phi},{'Bank'},'Bank (deg)','Bank angle used in the loiter'; ...
 'D27_Wheel_Loads',{L.Nm,L.Nt},{'Main wheels','Tail wheel'},'Wheel load (N)','Wheel loads'};
for k=1:size(list,1)
    FM1_TimePlot(t,list{k,2},list{k,3},ms,list{k,5},list{k,4},list{k,1},list{k,5},in);
end
e=[0 cumsum(ms.ph_t)];
tA=[0 e(3)+8]; tB=[e(8)-8 e(end)];
zm={'Takeoff',tA; 'Landing',tB};
for z=1:2
    nm=zm{z,1}; tl=zm{z,2};
    FM1_TimePlot(t,{L.V},{'Airspeed'},ms,[nm ' close-up: airspeed'],'Airspeed (m/s)',['D' num2str(26+2+5*(z-1)) '_' nm '_Speed'],[nm ' airspeed'],in,tl);
    FM1_TimePlot(t,{L.h},{'Height'},ms,[nm ' close-up: height'],'Height (m)',['D' num2str(29+5*(z-1)) '_' nm '_Height'],[nm ' height'],in,tl);
    FM1_TimePlot(t,{L.theta,L.alpha},{'Pitch','Alpha'},ms,[nm ' close-up: pitch and alpha'],'Angle (deg)',['D' num2str(30+5*(z-1)) '_' nm '_Pitch'],[nm ' pitch and alpha'],in,tl);
    FM1_TimePlot(t,{L.de},{'Elevator'},ms,[nm ' close-up: elevator'],'Elevator (deg), + = nose down',['D' num2str(31+5*(z-1)) '_' nm '_Elevator'],[nm ' elevator'],in,tl);
    FM1_TimePlot(t,{L.Nm,L.Nt},{'Main wheels','Tail wheel'},ms,[nm ' close-up: wheel loads'],'Wheel load (N)',['D' num2str(32+5*(z-1)) '_' nm '_Wheel_Loads'],[nm ' wheel loads'],in,tl);
end
V=14:0.5:30; [al,de,th,tt]=deal(zeros(size(V)));
for k=1:numel(V)
    r=FM2_Trim(V(k),in.h_cruise,0,0,ac.m,ac); al(k)=r.alpha; de(k)=r.de; th(k)=r.CL; tt(k)=r.T;
end
s={al,'Angle of attack (deg)','D38_Trim_Alpha'; de,'Elevator (deg), + = nose down','D39_Trim_Elevator'; tt,'Thrust (N)','D40_Trim_Thrust'};
for k=1:3
    f=figure('Color','w','Position',[60 60 1000 560]); plot(V,s{k,1},'LineWidth',2,'Color',[0.08 0.32 0.62]); hold on; yl=ylim;
    plot(in.V_cruise*[1 1],yl,'--','Color',[0.1 0.5 0.25]); plot(ms.V_loiter*[1 1],yl,'--','Color',[0.45 0.2 0.6]); ylim(yl);
    grid on; xlabel('Airspeed (m/s)'); ylabel(s{k,2}); title(['Level-flight trim: ' s{k,2}]); legend('Trim','Cruise','Loiter');
    FM1_SaveFig(f,s{k,3},['Level-flight trim: ' s{k,2}],in);
end
tr=FM2_Trim(in.V_cruise,in.h_cruise,0,0,ac.m,ac);
x=[0; in.h_cruise; in.V_cruise; 0; tr.alpha*pi/180; 0];
dt=0.02; N=round(180/dt); OL=zeros(N,5);
for k=1:N
    tk=(k-1)*dt; d0=tr.de*pi/180;
    if tk >= 1 && tk < 1.5,d0=d0 + 3*pi/180; elseif tk >= 1.5 && tk < 2,d0=d0 - 3*pi/180; end
    [k1,a]=FM2_Deriv(x,d0,tr.T,0,ac.m,ac,c,in);
    k2=FM2_Deriv(x+dt/2*k1,d0,tr.T,0,ac.m,ac,c,in);
    k3=FM2_Deriv(x+dt/2*k2,d0,tr.T,0,ac.m,ac,c,in);
    k4=FM2_Deriv(x+dt*k3,d0,tr.T,0,ac.m,ac,c,in);
    OL(k,:)=[tk a.V x(5)*180/pi a.alpha*180/pi x(2)];
    x=x + dt/6*(k1+2*k2+2*k3+k4);
end
f=figure('Color','w','Position',[60 60 1100 620]);
subplot(2,1,1); i=OL(:,1)<=10; plot(OL(i,1),OL(i,4),'LineWidth',1.8,'Color',[0.08 0.32 0.62]); grid on; ylabel('Alpha (deg)'); title('Open-loop elevator doublet: short period');
subplot(2,1,2); plot(OL(:,1),OL(:,2),'LineWidth',1.8,'Color',[0.82 0.25 0.1]); grid on; ylabel('Airspeed (m/s)'); xlabel('Time (s)'); title('Same run, no autopilot: phugoid');
FM1_SaveFig(f,'D41_Open_Loop_Modes','Open-loop elevator doublet at cruise: short period and phugoid',in);
f=figure('Color','w','Position',[60 60 1250 720]); ax=axes();
FM1_VnDraw(ax,vn,false); set(ax,'FontSize',10); hold(ax,'on');
c8=FM1_PhaseColors(9)*0.7; hh=[];
for k=1:9
    i=L.phase==k; hh(k)=plot(ax,L.V(i),vn.n_log(i),'-','Color',c8(k,:),'LineWidth',2.2);
end
legend(hh,ms.phase,'Location','eastoutside','Interpreter','none'); title(ax,'3-DOF mission on the V-n diagram, by phase');
FM1_SaveFig(f,'D42_Vn_Mission','3-DOF mission on the V-n diagram, by phase',in);
tr3=FM2_Track(ms,in); col=FM1_PhaseColors(9)*0.75; hh=[];
f=figure('Color','w','Position',[60 60 1200 560]); hold on;
for k=1:9,i=L.phase==k; hh(k)=plot(L.x(i)/1000,L.h(i),'Color',col(k,:),'LineWidth',3); end
grid on; box on; xlabel('Path distance (km)'); ylabel('Height (m)'); title('Mission profile'); legend(hh,ms.phase,'Location','eastoutside');
FM1_SaveFig(f,'D43_Mission_Profile','Mission profile by phase',in);
f=figure('Color','w','Position',[60 60 1200 760]); hold on; hh=[];
for k=1:9,i=L.phase==k; hh(k)=plot3(tr3.X(i)/1000,tr3.Y(i)/1000,L.h(i),'Color',col(k,:),'LineWidth',2); end
view([-35 28]); grid on; box on; xlabel('X (km)'); ylabel('Y (km)'); zlabel('Height (m)'); title('3D mission path (sideways shape is illustrative until the 6-DOF model)'); legend(hh,ms.phase,'Location','eastoutside');
FM1_SaveFig(f,'D44_3D_Path','3D mission path by phase',in);
c0=FM1_PhaseColors(9);
bars={ms.ph_Wh,'Energy (Wh)','Energy used in each phase','D45_Energy_by_Phase'; ms.ph_t,'Time (s)','Time in each phase','D46_Time_by_Phase'; ms.ph_de,'Elevator (deg)','Average elevator in each phase','D47_Elevator_by_Phase'};
for b=1:3
    f=figure('Color','w','Position',[60 60 1100 520]); hold on; v=bars{b,1};
    for k=1:9
        patch(k+[-.38 .38 .38 -.38],[0 0 v(k) v(k)],c0(k,:),'EdgeColor',[.3 .3 .3]);
        text(k,v(k),sprintf('%.2f',v(k)),'HorizontalAlignment','center','VerticalAlignment','bottom');
    end
    set(gca,'XTick',1:9,'XTickLabel',ms.phase,'XTickLabelRotation',20); xlim([0.4 9.6]);
    ylabel(bars{b,2}); title(bars{b,3}); grid on; box on;
    FM1_SaveFig(f,bars{b,4},bars{b,3},in);
end
LA=msA.log;
cmp={L.h,LA.h,'Height (m)','C01_Altitude_vs_Point_Mass'; L.V,LA.V,'Airspeed (m/s)','C02_Speed_vs_Point_Mass'; L.P,LA.P,'Battery power (W)','C03_Power_vs_Point_Mass'; L.Wh_,LA.Wh_,'Energy used (Wh)','C04_Energy_vs_Point_Mass'};
for k=1:4
    f=figure('Color','w','Position',[60 60 1100 520]);
    plot(t,cmp{k,1},'LineWidth',1.8,'Color',[0.08 0.32 0.62]); hold on; plot(LA.t,cmp{k,2},'--','LineWidth',1.5,'Color',[0.82 0.25 0.1]);
    grid on; xlabel('Mission time (s)'); ylabel(cmp{k,3}); legend('3-DOF','Point mass'); title(['3-DOF against point mass: ' cmp{k,3}]);
    FM1_SaveFig(f,cmp{k,4},['3-DOF against point mass: ' cmp{k,3}],in);
end
end
