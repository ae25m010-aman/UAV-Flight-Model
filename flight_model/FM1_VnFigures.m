function FM1_VnFigures(vn,ms,in)
f=figure('Color','w','Position',[60 60 1250 720]); ax=axes();
h=FM1_VnDraw(ax,vn,true);
names={'Maneuver envelope'};
if vn.show_gust,names{end+1}='Gust lines'; end
names=[names {sprintf('V_S stall = %.2f m/s',vn.V_S),sprintf('V_L loiter = %.2f m/s',vn.V_L),...
    sprintf('V_C cruise = %.2f m/s',vn.V_C),sprintf('V_A maneuvering = %.2f m/s',vn.V_A),...
    sprintf('V_D dive = %.2f m/s',vn.V_D)}];
legend(h,names,'Location','eastoutside','Interpreter','none');
text('Parent',ax,'Position',[0.3 vn.n_pos],'String',sprintf('n+ = %.2f',vn.n_pos),'VerticalAlignment','bottom');
text('Parent',ax,'Position',[0.3 vn.n_neg],'String',sprintf('n- = %.2f',vn.n_neg),'VerticalAlignment','top');
title(ax,sprintf('V-n diagram at %.0f m (rho %.3f kg/m3)',in.h_cruise,vn.rho));
FM1_SaveFig(f,'P13_Vn_Diagram','V-n diagram with stall, loiter, cruise, maneuvering and dive speeds',in);
FM1_TimePlot(ms.log.t,{vn.n_log,vn.n_pos},{'Load factor','n+ limit'},ms,'Load factor (lift / weight)','n',...
    'M37_Load_Factor','Load factor against time',in);
f=figure('Color','w','Position',[60 60 1250 720]); ax=axes();
FM1_VnDraw(ax,vn,false); set(ax,'FontSize',10); hold(ax,'on');
c=FM1_PhaseColors()*0.7; hh=[];
for k=1:9
    i=ms.log.phase==k;
    hh(k)=plot(ax,ms.log.V(i),vn.n_log(i),'-','Color',c(k,:),'LineWidth',2.5);
end
legend(hh,ms.phase,'Location','eastoutside','Interpreter','none');
title(ax,'The mission flown on the V-n diagram, by phase');
FM1_SaveFig(f,'M38_Vn_Mission','The mission flown on the V-n diagram, by phase',in);
end
