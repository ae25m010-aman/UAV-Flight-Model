function f=FM1_TimePlot(t,Y,names,ms,ttl,ylab,name,descr,in,tlim)
if nargin < 10,tlim=[t(1) t(end)]; end
f=figure('Color','w','Position',[60 60 1200 560]); hold on;
if ~iscell(Y),Y={Y}; end
idx=t >= tlim(1) & t <= tlim(2);
all_y=[]; for k=1:numel(Y),yk=Y{k}; if numel(yk)==1,all_y=[all_y yk]; else,all_y=[all_y yk(idx)]; end; end
all_y=all_y(isfinite(all_y));
lo=min(all_y); hi=max(all_y); if hi <= lo,hi=lo + 1; end
pad=0.12*(hi-lo); yl=[lo - 0.04*(hi-lo),hi + pad];
hp=FM1_PhaseShade(ms,yl,tlim,true);
col={[0.08 0.32 0.62],[0.82 0.25 0.1],[0.1 0.5 0.25],[0.45 0.2 0.6]};
hl=[];
for k=1:numel(Y)
    yk=Y{k}; st='-';
    if numel(yk)==1,yk=yk*ones(size(t)); st='--'; end
    hl(k)=plot(t(idx),yk(idx),st,'Color',col{k},'LineWidth',1.8);
end
xlim(tlim); ylim(yl); box on; grid on; set(gca,'Layer','top');
xlabel('Mission time (s)'); ylabel(ylab); title(ttl);
legend([hl hp],[names ms.phase],'Location','eastoutside','Interpreter','none');
FM1_SaveFig(f,name,descr,in);
end
