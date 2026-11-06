function FM1_PerformanceFigures(pf,ac,in,ms)
V=pf.V; b=[0.08 0.32 0.62]; r=[0.82 0.25 0.1]; g=[0.1 0.5 0.25]; k=[0 0 0];
W=ac.m*9.81;
f=perf_fig(); plot(V,pf.T_req,'Color',b,'LineWidth',2); hold on; plot(V,pf.T_av,'Color',r,'LineWidth',2);
speed_marks(pf,[0 max(pf.T_av)]); ax_labels('Airspeed (m/s)','Thrust (N)','Thrust required and available');
legend('Required','Available','Stall','V min thrust','V min power','Location','northeast');
FM1_SaveFig(f,'P01_Thrust_vs_Speed','Thrust required and available against speed',in);
f=perf_fig(); plot(V,pf.P_req,'Color',b,'LineWidth',2); hold on; plot(V,pf.P_av,'Color',r,'LineWidth',2);
speed_marks(pf,[0 max(pf.P_av)]); ax_labels('Airspeed (m/s)','Thrust power (W)','Power required and available');
legend('Required','Available','Stall','V min thrust','V min power','Location','northwest');
FM1_SaveFig(f,'P02_Power_vs_Speed','Thrust power required and available against speed',in);
f=perf_fig(); plot(V,pf.P_elec,'Color',g,'LineWidth',2); hold on; speed_marks(pf,[0 700]); ylim([0 700]);
ax_labels('Airspeed (m/s)','Battery power (W)','Battery power in level flight');
legend('Battery power','Stall','V min thrust','V min power','Location','northwest');
FM1_SaveFig(f,'P03_Battery_Power_vs_Speed','Battery power in level flight against speed',in);
f=perf_fig(); plot(V,pf.ROC,'Color',b,'LineWidth',2); hold on; plot(pf.V_y,pf.ROC_max,'o','Color',r,'MarkerFaceColor',r);
plot(V,in.ROC_climb*ones(size(V)),'k--'); ylim([0 pf.ROC_max*1.15]); speed_marks(pf,[0 pf.ROC_max*1.15]);
ax_labels('Airspeed (m/s)','Rate of climb (m/s)',sprintf('Rate of climb at full power: best %.2f m/s at %.1f m/s',pf.ROC_max,pf.V_y));
legend('ROC','Best','Commanded climb','Stall','V min thrust','V min power','Location','northeast');
FM1_SaveFig(f,'P04_Rate_of_Climb','Rate of climb at full power against speed',in);
f=perf_fig(); ang=asind(max(min(pf.ROC./V,1),-1)); plot(V,ang,'Color',b,'LineWidth',2); hold on; ylim([0 max(ang)*1.15]);
speed_marks(pf,[0 max(ang)*1.15]); ax_labels('Airspeed (m/s)','Climb angle (deg)','Climb angle at full power');
legend('Climb angle','Stall','V min thrust','V min power');
FM1_SaveFig(f,'P05_Climb_Angle','Climb angle at full power against speed',in);
f=perf_fig(); ld=pf.CL./pf.CD; plot(V,ld,'Color',b,'LineWidth',2); hold on; speed_marks(pf,[0 12]); ylim([0 12]);
ax_labels('Airspeed (m/s)','L/D',sprintf('Lift-to-drag ratio: max %.2f',pf.LD_max));
legend('L/D','Stall','V min thrust','V min power');
FM1_SaveFig(f,'P06_LD_vs_Speed','Lift-to-drag ratio against speed',in);
f=perf_fig(); cl=linspace(-0.2,ac.CL_max,100); cd=ac.CD0 + ac.K*cl.^2; plot(cd,cl,'Color',b,'LineWidth',2); hold on;
plot(ac.CD0+ac.K*pf.CL_md^2,pf.CL_md,'o','Color',r,'MarkerFaceColor',r);
ax_labels('CD','CL','Drag polar'); legend('Polar','Best L/D','Location','southeast');
FM1_SaveFig(f,'P07_Drag_Polar','Drag polar',in);
f=perf_fig(); plot(V,pf.eta_p,'Color',b,'LineWidth',2); hold on; speed_marks(pf,[0 0.8]); ylim([0 0.8]);
ax_labels('Airspeed (m/s)','Efficiency','Propeller efficiency in level flight');
legend('eta_p','Stall','V min thrust','V min power','Location','southeast','Interpreter','none');
FM1_SaveFig(f,'P08_Propeller_Efficiency','Propeller efficiency in level flight against speed',in);
f=perf_fig(); plot(V,pf.CL,'Color',b,'LineWidth',2); hold on; plot(V,ac.CL_max*ones(size(V)),'k--');
xlim([V(1) V(end)]); ylim([0 2]); ax_labels('Airspeed (m/s)','CL','CL needed in level flight');
legend('CL needed','CLmax');
FM1_SaveFig(f,'P09_CL_vs_Speed','Lift coefficient needed against speed',in);
f=perf_fig(); plot(pf.ROC_h,pf.hs,'Color',b,'LineWidth',2); hold on; plot(0.5*[1 1],[0 pf.hs(end)],'k--');
ax_labels('Best rate of climb (m/s)','Height (m)','Best rate of climb against height'); legend('Best ROC','0.5 m/s');
FM1_SaveFig(f,'P10_Climb_vs_Altitude','Best rate of climb against height',in);
f=perf_fig(); dx=V.*sqrt(max(1-(pf.ROC./V).^2,0)); plot(dx,pf.ROC,'Color',b,'LineWidth',2);
axis equal; xlim([0 40]); ylim([0 10]); ax_labels('Horizontal speed (m/s)','Vertical speed (m/s)','Climb hodograph at full power');
FM1_SaveFig(f,'P11_Climb_Hodograph','Climb hodograph at full power',in);
f=perf_fig(); sk=pf.CD./pf.CL.*V; plot(V,sk,'Color',b,'LineWidth',2); hold on; set(gca,'YDir','reverse'); ylim([0 6]);
speed_marks(pf,[0 6]); ax_labels('Airspeed (m/s)','Sink rate (m/s)','Motor-off glide');
legend('Sink rate','Stall','V min thrust','V min power','Location','southwest');
FM1_SaveFig(f,'P12_Glide_Sink_Rate','Motor-off sink rate against speed',in);
end
function f=perf_fig()
f=figure('Color','w','Position',[60 60 1000 600]);
end
function ax_labels(xl,yl,ttl)
xlabel(xl); ylabel(yl); title(ttl); grid on; box on;
end
function speed_marks(pf,yl)
plot(pf.V_stall*[1 1],yl,'k--'); plot(pf.V_md*[1 1],yl,':','Color',[.1 .5 .25],'LineWidth',1.5);
plot(pf.V_mp*[1 1],yl,':','Color',[.8 .25 .1],'LineWidth',1.5);
end
