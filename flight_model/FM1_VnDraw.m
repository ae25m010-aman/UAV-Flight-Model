function h=FM1_VnDraw(ax,vn,large)
axes(ax); hold(ax,'on');
h=[];
if large,lw=2; fs=10; else,lw=1.2; fs=7; end
patch('Parent',ax,'XData',vn.env_V,'YData',vn.env_n,'FaceColor',[0.86 0.92 0.98],'EdgeColor','none');
h(1)=plot(ax,vn.env_V,vn.env_n,'-','Color',[0.08 0.32 0.62],'LineWidth',lw);
vmax=1.12*vn.V_D;
plot(ax,[0 vmax],[0 0],'k-','LineWidth',0.6);
plot(ax,[0 vmax],[1 1],':','Color',[0.4 0.4 0.4]);
if vn.show_gust
    gc=[0.8 0.4 0.1];
    h(2)=plot(ax,[0 vn.V_C vn.V_D vn.V_D],[1 vn.n_gC vn.n_gD 1],'--','Color',gc,'LineWidth',lw*0.7);
    plot(ax,[0 vn.V_C vn.V_D vn.V_D],[1 vn.n_gCm vn.n_gDm 1],'--','Color',gc,'LineWidth',lw*0.7);
    ymax=1.12*max([vn.n_pos vn.n_gC vn.n_gD]);
    ymin=1.12*min([vn.n_neg vn.n_gCm vn.n_gDm]);
else
    ymax=1.12*vn.n_pos; ymin=1.12*vn.n_neg;
end
sp=[vn.V_S vn.V_L vn.V_C vn.V_A vn.V_D];
tags={'V_S','V_L','V_C','V_A','V_D'};
cols=[0.75 0.1 0.1; 0.45 0.2 0.6; 0.1 0.5 0.25; 0.8 0.45 0.05; 0.1 0.1 0.1];
lev=[0.97 0.9 0.97 0.97 0.97];
for k=1:5
    p=plot(ax,sp(k)*[1 1],[ymin ymax],'--','Color',cols(k,:),'LineWidth',lw*0.6);
    if large,h(end+1)=p; end
    text('Parent',ax,'Position',[sp(k) ymin+lev(k)*(ymax-ymin)],'String',tags{k},'Color',cols(k,:),...
        'FontSize',fs,'FontWeight','bold','HorizontalAlignment','center','BackgroundColor','w','Interpreter','none');
end
xlim(ax,[0 vmax]); ylim(ax,[ymin ymax]); grid(ax,'on'); box(ax,'on'); set(ax,'Layer','top');
xlabel(ax,'V (m/s)','FontSize',fs+1); ylabel(ax,'n','FontSize',fs+1); set(ax,'FontSize',fs);
end
