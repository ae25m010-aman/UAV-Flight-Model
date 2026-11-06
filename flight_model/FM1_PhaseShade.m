function hp=FM1_PhaseShade(ms,yl,tlim,numbered)
edges=[0 cumsum(ms.ph_t)];
n=numel(ms.phase);
col=FM1_PhaseColors(n); hp=[];
for k=1:n
    a=edges(k); b=edges(k+1);
    hp(k)=patch([a b b a],[yl(1) yl(1) yl(2) yl(2)],col(k,:),'EdgeColor','none','FaceAlpha',0.45);
    if numbered && b >= tlim(1) && a <= tlim(2)
        m=(max(a,tlim(1))+min(b,tlim(2)))/2;
        lev=yl(2) - (0.04 + 0.05*mod(k,2)*(b-a < 0.04*(tlim(2)-tlim(1))))*(yl(2)-yl(1));
        text(m,lev,sprintf('%d',k),'HorizontalAlignment','center','FontWeight','bold','FontSize',10);
    end
end
end
