clear; clc; close all;
FUSELAGE_ONLY=false;
BUCKLING_METHOD=1;
SAVE_FIGURES=true;
FIG_FOLDER='Fuselage_Figures';
FIG_FORMATS={'png'};
FIG_DPI=300;
noseLen=109.53;
boxLen=225.47;
tailTop=170.22;
tailDrop=165-60;
tailTaper=200;
width=97.5;
heightBox=165;
plateFrameDepth=5;
noseTipDia=49;
noseTipHalfH=noseTipDia/2;
noseBaseHalfH=heightBox/2;
xNoseEnd=noseLen;
xBoxEnd=xNoseEnd + boxLen;
xFusEnd=xBoxEnd + tailTop;
xUAVEnd=978.35;
tailTaperCheck=hypot(tailTop,tailDrop);
fprintf('Rear taper length check: given %.3f mm, geometric %.3f mm\n',...
        tailTaper,tailTaperCheck);
skin_t=0.5;
skin_rho=2.71e-3;
m_skin=375.252;
g_acc=9.81;
comp={ ...
 -40,23.7,'Propeller' ; ...
 -35,150,'Motor' ; ...
 0,78.576,'Motor Mount + Bulkhead' ; ...
 109.53,900,'Payload' ; ...
 129.53,641,'Battery' ; ...
 129.53,123.129,'Nose Bulkhead' ; ...
 134.53,15,'Camera' ; ...
 170.3,150,'ESC' ; ...
 170.3,75,'PDB + Tx' ; ...
 170.3,123.129,'LG Bulkhead' ; ...
 170.3,301.011,'Main LG' ; ...
 250.44,2525.878,'Wing' ; ...
 253.67,188.229,'Longerons' ; ...
 265,123.129,'Wing Bulkhead' ; ...
 273.66,25,'Boom Joints' ; ...
 294.81,0,'Neutral Point' ; ...
 375,150,'Wing Avionics' ; ...
 502.69,47.208,'Rear Bulkhead' ; ...
 539.33,277.159,'Boom' ; ...
 825.3,75.569,'Vertical Tail' ; ...
 825.3,322.939,'Horizontal Tail' ; ...
 934.3,150,'Tail Servos' ; ...
 953.35,39,'Tail LG' ...
};
x_all=cell2mat(comp(:,1));
w_all_g=cell2mat(comp(:,2));
lbl_all=comp(:,3);
F_all=w_all_g * 1e-3 * g_acc;
if FUSELAGE_ONLY
    excludeSet={'Boom','Vertical Tail','Horizontal Tail','Tail Servos','Tail LG'};
    keep=~ismember(lbl_all,excludeSet);
    x_load=x_all(keep);
    F_load=F_all(keep);
    lbl_load=lbl_all(keep);
    xBeamMax=xFusEnd;
else
    x_load=x_all;
    F_load=F_all;
    lbl_load=lbl_all;
    xBeamMax=xUAVEnd;
end
xBeamMin=min(x_load);
xFixed=294.81;
figure('Name','Fuselage Loading Diagram','Color','w');
hold on; grid on; axis equal;
xNose=linspace(0,xNoseEnd,200);
halfH_nose=noseTipHalfH + (noseBaseHalfH - noseTipHalfH).*(xNose./xNoseEnd);
zNoseTop=heightBox/2 + halfH_nose;
zNoseBot=heightBox/2 - halfH_nose;
xBox=[xNoseEnd,xBoxEnd];
zBoxTop=[heightBox,heightBox];
zBoxBot=[0,0];
xTail=[xBoxEnd,xFusEnd];
zTailTop=[heightBox,heightBox];
zTailBot=[0,tailDrop];
xProfTop=[xNose,xBox,xTail];
zProfTop=[zNoseTop,zBoxTop,zTailTop];
xProfBot=[xNose,xBox,xTail];
zProfBot=[zNoseBot,zBoxBot,zTailBot];
plot(xProfTop,zProfTop,'k-','LineWidth',1.6);
plot(xProfBot,zProfBot,'k-','LineWidth',1.6);
plot([xFusEnd xFusEnd],[tailDrop heightBox],'k-','LineWidth',1.6);
plot([0 0],[heightBox/2-noseTipHalfH,heightBox/2+noseTipHalfH],...
     'Color',[0.85 0.33 0.1],'LineWidth',4,'HandleVisibility','off');
plot([xFusEnd xFusEnd],[tailDrop,heightBox],...
     'Color',[0.1 0.5 0.2],'LineWidth',4,'HandleVisibility','off');
patch([0 xFusEnd xFusEnd 0],[-20 -20 heightBox+20 heightBox+20],...
      [0.85 0.93 1],'FaceAlpha',0.35,'EdgeColor','none');
xline(xFusEnd,'--',sprintf(' = %.3f mm',xFusEnd),...
      'Color',[0.2 0.3 0.7],'FontSize',20);
if ~FUSELAGE_ONLY
    plot([xFusEnd xUAVEnd],[heightBox/2 heightBox/2],'k:','LineWidth',1.2);
    xline(xUAVEnd,'-.',sprintf('Overall UAV length = %.2f mm',xUAVEnd),...
          'Color',[0.6 0 0],'LabelVerticalAlignment','top','FontSize',20);
end
markerColors=lines(numel(x_load));
h=gobjects(numel(x_load),1);
for i=1:numel(x_load)
    zPlot=heightBox/2;
    h(i)=plot(x_load(i),zPlot,'o','MarkerSize',7,...
        'MarkerFaceColor',markerColors(i,:),'MarkerEdgeColor','k');
    text(x_load(i),zPlot+8,num2str(i),'FontSize',30,...
        'HorizontalAlignment','center','FontWeight','bold');
end
legendStr=arrayfun(@(i) sprintf('%d: %s (%.1f mm, %.2f N)',...
    i,lbl_load{i},x_load(i),F_load(i)),1:numel(x_load),'UniformOutput',false);
legend(h,legendStr,'Location','southoutside','FontSize',20,'NumColumns',3);
xlabel('x - longitudinal station (mm)','FontSize',30); ylabel('z - height (mm)','FontSize',30);
title('Fuselage x-z Profile with Component Point Loads','FontSize',20);
ylim([-20,heightBox+40]);
hold off;
dxSW=0.5;
xSW=(0:dxSW:xFusEnd)';
hSW=fuselage_height(xSW,xNoseEnd,xBoxEnd,xFusEnd,heightBox,tailDrop,noseTipHalfH);
perimSW=2*(width + hSW);
wPerLen=perimSW * skin_t * skin_rho * 1e-3 * g_acc;
F_sw=wPerLen * dxSW;
F_sw=F_sw*(m_skin*1e-3*g_acc)/sum(F_sw);
inRange=xSW >= xBeamMin & xSW <= xBeamMax;
x_sw_load=xSW(inRange);
F_sw_load=F_sw(inRange);
fprintf('Total skin self-weight (fuselage shell) = %.3f N (%.1f g)\n',...
        sum(F_sw),sum(F_sw)/g_acc*1e3);
fprintf('\n Data for Fig 1: component point loads \n');
fprintf('  %-4s %-18s %10s %10s %11s\n','No.','Component','x (mm)','Mass (g)','Weight (N)');
for i=1:numel(x_load)
    fprintf('  %-4d %-18s %10.2f %10.1f %11.3f\n',i,lbl_load{i},x_load(i),...
            F_load(i)/g_acc*1e3,F_load(i));
end
fprintf('  Total point-load weight = %.3f N (%.1f g)\n',sum(F_load),sum(F_load)/g_acc*1e3);
fprintf('  Beam domain: x = %.2f to %.2f mm, fixed support at x = %.2f mm\n',...
        xBeamMin,xBeamMax,xFixed);
x_all_load=[x_load(:); x_sw_load(:)];
F_all_load=[F_load(:); F_sw_load(:)];
xBeam=linspace(xBeamMin,xBeamMax,4000);
V=zeros(size(xBeam));
M=zeros(size(xBeam));
for k=1:numel(xBeam)
    xc=xBeam(k);
    if xc <= xFixed
        sel=x_all_load <= xc;
        V(k)=sum(F_all_load(sel));
        M(k)=sum(F_all_load(sel) .* (xc - x_all_load(sel)));
    else
        sel=x_all_load >= xc;
        V(k)=-sum(F_all_load(sel));
        M(k)=sum(F_all_load(sel) .* (x_all_load(sel) - xc));
    end
end
[~,iVpk]=max(abs(V));
[~,iMpk]=max(abs(M));
colV=[0 0.447 0.741];
colM=[0.85 0.325 0.098];
figure('Name','SFD and BMD','Color','w');
yyaxis left
hV=plot(xBeam,V,'-','Color',colV,'LineWidth',1.5); hold on; grid on;
hVpk=plot(xBeam(iVpk),V(iVpk),'o','MarkerSize',11,...
    'MarkerFaceColor',colV,'MarkerEdgeColor','k');
ylabel('Shear force V(x) (N)','FontSize',20);
yyaxis right
hM=plot(xBeam,M,'-','Color',colM,'LineWidth',1.5);
hMpk=plot(xBeam(iMpk),M(iMpk),'s','MarkerSize',11,...
    'MarkerFaceColor',colM,'MarkerEdgeColor','k');
ylabel('Bending moment M(x) (N.mm)','FontSize',20);
axSB=gca;
axSB.YAxis(1).Color=colV;
axSB.YAxis(2).Color=colM;
xline(xFixed,'k--','Fixed support','FontSize',20);
xlabel('x (mm)','FontSize',20);
title('Shear Force and Bending Moment Diagrams (downward loading = positive M)','FontSize',20);
legend([hV hVpk hM hMpk],...
    {'Shear force V(x)',...
     sprintf('Max |V| = %.3f N at x = %.1f mm',abs(V(iVpk)),xFixed),...
     'Bending moment M(x)',...
     sprintf('Max |M| = %.1f N.mm at x = %.1f mm',abs(M(iMpk)),xFixed)},...
    'Location','best','FontSize',20);
hold off;
Vmax=max(abs(V));
fprintf('Max |shear force| along beam = %.4f N (used for section shear flow)\n',Vmax);
fprintf('\n Data for Fig 2: SFD and BMD \n');
[Vhi,iVhi]=max(V); [Vlo,iVlo]=min(V);
[Mhi,iMhi]=max(M); [Mlo,iMlo]=min(M);
[~,iFix]=min(abs(xBeam - xFixed));
fprintf('  Max positive shear V = %10.4f N     at x = %8.2f mm\n',Vhi,xBeam(iVhi));
fprintf('  Max negative shear V = %10.4f N     at x = %8.2f mm\n',Vlo,xBeam(iVlo));
fprintf('  Max |V|              = %10.4f N     at x = %8.2f mm\n',abs(V(iVpk)),xBeam(iVpk));
fprintf('  Max positive moment M = %12.3f N.mm at x = %8.2f mm\n',Mhi,xBeam(iMhi));
fprintf('  Max negative moment M = %12.3f N.mm at x = %8.2f mm\n',Mlo,xBeam(iMlo));
fprintf('  Max |M|               = %12.3f N.mm at x = %8.2f mm\n',abs(M(iMpk)),xBeam(iMpk));
fprintf('  Moment at the fixed support (x = %.2f mm) = %.3f N.mm\n',xFixed,M(iFix));
fprintf('  Total load on the beam (point loads + skin) = %.4f N = support reaction\n',sum(F_all_load));
fprintf('  Centroid of the total load: x = %.2f mm\n',sum(F_all_load.*x_all_load)/sum(F_all_load));
h_sec=heightBox;
b_sec=width;
t_sec=skin_t;
Aweb=t_sec*h_sec;
Aflange=t_sec*b_sec;
zNA=(2*Aweb*(h_sec/2) + Aflange*0) / (2*Aweb + Aflange);
Iweb_own=t_sec*h_sec^3/12;
Iweb=Iweb_own + Aweb*(h_sec/2 - zNA)^2;
Iflange_own=t_sec^3*b_sec/12;
Iflange=Iflange_own + Aflange*zNA^2;
Iyy_sec=2*Iweb + Iflange;
fprintf('\nSection properties (open U-channel):\n');
fprintf('Neutral axis height, zNA = %.4f mm (from bottom)\n',zNA);
fprintf('I_yy = %.2f mm^4\n',Iyy_sec);
V_design=Vmax;
s_web=linspace(0,h_sec,400);
q_web=-(V_design*t_sec/Iyy_sec) .* ( (h_sec - zNA).*s_web - s_web.^2/2 );
s_flange=linspace(0,b_sec/2,200);
q_corner=q_web(end);
q_flange=q_corner + (V_design*t_sec/Iyy_sec) * zNA .* s_flange;
s_star=h_sec - zNA;
q_max=-(V_design*t_sec/Iyy_sec) * (s_star^2/2);
fprintf('  q_max = %.4f N/mm at s = %.2f mm along the web (from top)\n',q_max,s_star);
fprintf('  q at centreline (should be ~0) = %.6f N/mm\n',q_flange(end));
areaVal=trapz(s_web,q_web);
areaValF=trapz(s_flange,q_flange);
colWeb=[0 0.447 0.741];
colFlange=[0.85 0.325 0.098];
qScale=max(abs([q_web,q_flange]));
offScale=0.35*b_sec/qScale;
zWeb=h_sec - s_web;
arrowLen=14;
figure('Name','Shear Flow on U-Section','Color','w');
hold on; grid on; axis equal;
hWall=plot([-b_sec/2 -b_sec/2 b_sec/2 b_sec/2],[h_sec 0 0 h_sec],'k-','LineWidth',3);
idxW=round(linspace(1,numel(s_web),25));
for side=[-1 1]
    yWall=side*b_sec/2*ones(size(s_web));
    yOrd=yWall + side*offScale*abs(q_web);
    hWebP=patch([yWall fliplr(yOrd)],[zWeb fliplr(zWeb)],colWeb,...
        'FaceAlpha',0.35,'EdgeColor',colWeb,'LineWidth',1.5);
    hx=[yWall(idxW); yOrd(idxW); nan(1,numel(idxW))];
    hz=[zWeb(idxW); zWeb(idxW); nan(1,numel(idxW))];
    plot(hx(:),hz(:),'-','Color',colWeb,'LineWidth',0.8,'HandleVisibility','off');
end
idxF=round(linspace(1,numel(s_flange),12));
zOrdF=-offScale*abs(q_flange);
for side=[-1 1]
    yF=side*(b_sec/2 - s_flange);
    hFlangeP=patch([yF fliplr(yF)],[zeros(size(yF)) fliplr(zOrdF)],colFlange,...
        'FaceAlpha',0.35,'EdgeColor',colFlange,'LineWidth',1.5);
    hx=[yF(idxF); yF(idxF); nan(1,numel(idxF))];
    hz=[zeros(1,numel(idxF)); zOrdF(idxF); nan(1,numel(idxF))];
    plot(hx(:),hz(:),'-','Color',colFlange,'LineWidth',0.8,'HandleVisibility','off');
end
sArrW=[0.12 0.32 0.52 0.72 0.92]*h_sec;
qArrW=interp1(s_web,q_web,sArrW);
sArrF=[0.15 0.45 0.75]*(b_sec/2);
qArrF=interp1(s_flange,q_flange,sArrF);
for side=[-1 1]
    quiver(side*(b_sec/2 - 6)*ones(size(sArrW)),h_sec - sArrW,...
        zeros(size(sArrW)),-sign(qArrW)*arrowLen,0,...
        'Color',colWeb,'LineWidth',2,'MaxHeadSize',1.5,'HandleVisibility','off');
    quiver(side*(b_sec/2 - sArrF),6*ones(size(sArrF)),...
        -side*sign(qArrF)*arrowLen,zeros(size(sArrF)),0,...
        'Color',colFlange,'LineWidth',2,'MaxHeadSize',1.5,'HandleVisibility','off');
end
xLab=b_sec/2 + offScale*qScale + 4;
for side=[-1 1]
    if side < 0
        ha='right';
    else
        ha='left';
    end
    plot([side*(b_sec/2 + offScale*abs(q_max)),side*xLab],[zNA zNA],':',...
        'Color',[0.4 0.4 0.4],'HandleVisibility','off');
    text(side*xLab,zNA,sprintf('q_{max} = %.4f N/mm',abs(q_max)),...
        'HorizontalAlignment',ha,'FontSize',20);
    plot([side*(b_sec/2 + offScale*abs(q_corner)),side*xLab],[0 0],':',...
        'Color',[0.4 0.4 0.4],'HandleVisibility','off');
    text(side*xLab,0,sprintf('q_{corner} = %.4f N/mm',abs(q_corner)),...
        'HorizontalAlignment',ha,'FontSize',20);
end
zCtr=-offScale*abs(q_corner) - 10;
plot([0 0],[0 zCtr],':','Color',[0.4 0.4 0.4],'HandleVisibility','off');
text(0,zCtr,'q = 0 at centreline','HorizontalAlignment','center',...
    'VerticalAlignment','top','FontSize',20);
xlim([-(xLab + 150),(xLab + 150)]);
ylim([zCtr-30,h_sec+15]);
xlabel('y (mm)','FontSize',20); ylabel('z (mm)','FontSize',20);
title(sprintf(['Shear Flow Distribution on the U-Section (V_{design} = %.3f N)\n',...
    'Ordinates show |q|, arrows show the flow direction'],V_design),'FontSize',20);
legend([hWall hWebP hFlangeP],...
    {'Open U-section (symmetric)',...
     sprintf('Web shear flow (each web), area under q(s) = %.3f N',abs(areaVal)),...
     sprintf('Flange shear flow (each half), area under q(s) = %.3f N',abs(areaValF))},...
    'Location','southoutside','FontSize',20);
hold off;
fprintf('\n Data for Fig 3 and Fig 4: open U-section shear flow \n');
fprintf('  Design shear force V_design = %.4f N\n',V_design);
fprintf('  Shear flow at the top free edge of the web = %.4f N/mm\n',q_web(1));
fprintf('  Shear flow at the bottom corner (web/flange junction) = %.4f N/mm\n',q_corner);
fprintf('  q_max = %.4f N/mm at z = %.2f mm from the bottom (neutral axis)\n',q_max,zNA);
fprintf('  Maximum shear stress |q_max|/t = %.4f MPa\n',abs(q_max)/t_sec);
fprintf('  Area under q(s): each web = %.4f N, each flange half = %.4f N\n',areaVal,areaValF);
fprintf('  Vertical resultant of the two web flows = %.4f N (%.4f x V_design)\n',...
        2*abs(areaVal),2*abs(areaVal)/V_design);
yWebL=-b_sec/2 * ones(size(s_web));
zWebL=h_sec - s_web;
yWebR=b_sec/2 * ones(size(s_web));
zWebR=h_sec - s_web;
yFlangeL=-b_sec/2 + s_flange;
zFlangeL=zeros(size(s_flange));
yFlangeR=b_sec/2 - s_flange;
zFlangeR=zeros(size(s_flange));
figure('Name','Section Shear Flow Map','Color','w');
Yfull=[yWebL,yFlangeL,fliplr(yFlangeR),fliplr(yWebR)];
Zfull=[zWebL,zFlangeL,fliplr(zFlangeR),fliplr(zWebR)];
Qfull=[q_web,q_flange,fliplr(q_flange),fliplr(q_web)];
scatter(Yfull,Zfull,18,Qfull,'filled'); hold on; grid on; axis equal;
colormap(jet); cb=colorbar; cb.Label.String='Shear flow q (N/mm)';
plot(Yfull,Zfull,'k-','LineWidth',0.5);
xlabel('y (mm)','FontSize',20); ylabel('z (mm)','FontSize',20);
title('Normal Shear Flow Distribution on Fuselage Cross Section (y-z plane)','FontSize',20);
hold off;
E_skin=69e3;
mu_skin=0.33;
E_bh=68.9e3;
mu_bh=0.33;
sigma_allow=276/6.75;
bulkheads=[xNoseEnd,170.3,265];
topStart=xNoseEnd; topEnd=xFusEnd;
topBreaks=sort(unique([topStart,bulkheads(bulkheads>topStart & bulkheads<topEnd),topEnd]));
botStart=xNoseEnd; botEnd=xBoxEnd;
botBreaks=sort(unique([botStart,bulkheads(bulkheads>botStart & bulkheads<botEnd),botEnd]));
tapStart=xBoxEnd; tapEnd=xFusEnd;
tapBreaks=sort(unique([tapStart,bulkheads(bulkheads>tapStart & bulkheads<tapEnd),tapEnd]));
noseStart=0; noseEnd=xNoseEnd;
noseBreaks=sort(unique([noseStart,bulkheads(bulkheads>noseStart & bulkheads<noseEnd),noseEnd]));
panelDefs={};
runs={topBreaks,'Top longeron'; ...
        botBreaks,'Bottom-corner longeron (box)'; ...
        tapBreaks,'Bottom-corner longeron (tapered)'; ...
        noseBreaks,'Nose taper longeron (top)'; ...
        noseBreaks,'Nose taper longeron (bottom)'};
for r=1:size(runs,1)
    brks=runs{r,1};
    for p=1:numel(brks)-1
        panelDefs(end+1,:)={runs{r,2},brks(p),brks(p+1),brks(p+1)-brks(p)};
    end
end
tau_applied=abs(q_max) / t_sec;
nPanels=size(panelDefs,1);
PanelName=strings(nPanels,1);
a_len=zeros(nPanels,1);
b_len=width * ones(nPanels,1);
Pcr_or_Ncr=zeros(nPanels,1);
tauCrit=zeros(nPanels,1);
SafetyMargin=zeros(nPanels,1);
for i=1:nPanels
    PanelName(i)=string(panelDefs{i,1});
    a_len(i)=panelDefs{i,4};
    [criticalLoad,criticalStress]=longeron_buckling(BUCKLING_METHOD,...
        E_skin,skin_t,mu_skin,a_len(i),b_len(i));
    Pcr_or_Ncr(i)=criticalLoad;
    tauCrit(i)=criticalStress;
    SafetyMargin(i)=criticalStress / tau_applied;
end
PanelStart=cellfun(@(c) c,panelDefs(:,2));
PanelEnd=cellfun(@(c) c,panelDefs(:,3));
Status=strings(nPanels,1);
for i=1:nPanels
    if SafetyMargin(i) >= 1
        Status(i)="PASS (sustains,SM=" + num2str(SafetyMargin(i),'%.2f') + ")";
    else
        Status(i)="FAIL (buckles,SM=" + num2str(SafetyMargin(i),'%.2f') + ")";
    end
end
LongeronTable=table(PanelName,PanelStart,PanelEnd,a_len,b_len,...
    Pcr_or_Ncr,tauCrit,repmat(tau_applied,nPanels,1),SafetyMargin,Status,...
    'VariableNames',{'Longeron_Run','x_start_mm','x_end_mm','PanelLength_a_mm',...
    'PanelWidth_b_mm','CriticalLoad_or_Ncr','CriticalStress_MPa',...
    'AppliedShearStress_MPa','SafetyMargin','Status'});
disp(' ');
disp('Longeron buckling summary (Status = PASS if CriticalStress >= AppliedStress):');
disp(LongeronTable);
fprintf('\n Nose taper longeron sustain check (Pcr/CriticalStress vs Applied) \n');
noseRows=contains(LongeronTable.Longeron_Run,"Nose taper longeron");
noseSub=LongeronTable(noseRows,:);
for i=1:height(noseSub)
    fprintf('  %-30s x=[%.2f, %.2f] mm | Critical=%.3f MPa vs Applied=%.3f MPa -> %s\n',...
        noseSub.Longeron_Run(i),noseSub.x_start_mm(i),noseSub.x_end_mm(i),...
        noseSub.CriticalStress_MPa(i),noseSub.AppliedShearStress_MPa(i),noseSub.Status(i));
end
if all(noseSub.SafetyMargin >= 1)
    fprintf('  ==> All 4 nose taper longerons SUSTAIN the applied shear.\n');
else
    fprintf('  ==> WARNING: at least one nose taper longeron FAILS (SM < 1) — resize/thicken.\n');
end
t_longeron=1;
b_sweep=linspace(2,25,200);
isTop=(LongeronTable.Longeron_Run == "Top longeron") | ...
        (LongeronTable.Longeron_Run == "Nose taper longeron (top)");
isBot=contains(LongeronTable.Longeron_Run,"Bottom-corner longeron") | ...
        (LongeronTable.Longeron_Run == "Nose taper longeron (bottom)");
a_top_gov=max(LongeronTable.PanelLength_a_mm(isTop));
a_bot_gov=max(LongeronTable.PanelLength_a_mm(isBot));
K_endfix=1;
Le_top=K_endfix * a_top_gov;
Le_bot=K_endfix * a_bot_gov;
M_max=max(abs(M));
c_top=h_sec - zNA;
c_bot=zNA;
sigma_applied_top=M_max*c_top/Iyy_sec;
sigma_applied_bot=M_max*c_bot/Iyy_sec;
I_min_sweep=t_longeron*b_sweep.^3/12;
A_sweep=2*b_sweep*t_longeron;
Pcr_top_sweep=pi^2*E_bh*I_min_sweep/Le_top^2;
Pcr_bot_sweep=pi^2*E_bh*I_min_sweep/Le_bot^2;
sigma_cr_top=Pcr_top_sweep./A_sweep;
sigma_cr_bot=Pcr_bot_sweep./A_sweep;
[~,iTopCross]=min(abs(sigma_cr_top - sigma_applied_top));
[~,iBotCross]=min(abs(sigma_cr_bot - sigma_applied_bot));
b_design_top=b_sweep(iTopCross);
b_design_bot=b_sweep(iBotCross);
b_design=12.5;
feasMask=(sigma_cr_top >= sigma_applied_top) & (sigma_cr_bot >= sigma_applied_bot);
iFeas=find(feasMask,1,'first');
strengthOK=(sigma_applied_top <= sigma_allow) && (sigma_applied_bot <= sigma_allow);
figure('Name','Longeron L-Section Sizing','Color','w');
hCrTop=plot(b_sweep,sigma_cr_top,'b-','LineWidth',1.8); hold on; grid on;
hCrBot=plot(b_sweep,sigma_cr_bot,'r-','LineWidth',1.8);
hAppTop=yline(sigma_applied_top,'b--',sprintf('\\sigma_{applied,top} = %.2f MPa',sigma_applied_top),...
    'FontSize',20,'LabelHorizontalAlignment','center');
hAppBot=yline(sigma_applied_bot,'r--',sprintf('\\sigma_{applied,bot} = %.2f MPa',sigma_applied_bot),...
    'FontSize',20,'LabelHorizontalAlignment','right');
hAllow=yline(sigma_allow,'k:',sprintf('\\sigma_{allow} (material) = %.2f MPa',sigma_allow),'FontSize',20);
hMkTop=plot(b_design_top,sigma_cr_top(iTopCross),'bp','MarkerSize',14,'MarkerFaceColor','b');
hMkBot=plot(b_design_bot,sigma_cr_bot(iBotCross),'rp','MarkerSize',14,'MarkerFaceColor','r');
xline(b_design,'k-.',sprintf('adopted b = %.2f mm',b_design),'FontSize',20,'HandleVisibility','off');
xlabel('Leg length, b (mm)','FontSize',20);
ylabel('Stress, \sigma (MPa)','FontSize',20);
title('Longeron L-Section: Buckling Stress vs Leg Length (t = 1 mm, pinned-pinned, incl. nose panels)','FontSize',20);
legHandles=[hCrTop hCrBot hAppTop hAppBot hAllow hMkTop hMkBot];
legLabels={'\sigma_{cr}(b) - top longeron (incl. nose)',...
            '\sigma_{cr}(b) - bottom longeron (incl. nose)',...
            '\sigma_{applied,top}','\sigma_{applied,bot}','\sigma_{allow}',...
            'crossing - top','crossing - bottom'};
if strengthOK && ~isempty(iFeas)
    yl=ylim;
    hFeas=patch([b_sweep(iFeas) b_sweep(end) b_sweep(end) b_sweep(iFeas)],...
            [yl(1) yl(1) yl(2) yl(2)],[0.55 0.85 0.55],...
            'FaceAlpha',0.3,'EdgeColor','none');
    uistack(hFeas,'bottom');
    ylim(yl);
    legHandles(end+1)=hFeas;
    legLabels{end+1}=sprintf('Feasible region (b \\geq %.2f mm)',b_sweep(iFeas));
end
legend(legHandles,legLabels,'Location','best','FontSize',20,'Box','off');
styleAxes(gca); hold off;
A_longeron=2*b_design*t_longeron;
I_longeron=t_longeron*b_design^3/12;
Pcr_top=pi^2*E_bh*I_longeron/Le_top^2;
Pcr_bot=pi^2*E_bh*I_longeron/Le_bot^2;
SM_top=Pcr_top/(sigma_applied_top*A_longeron);
SM_bot=Pcr_bot/(sigma_applied_bot*A_longeron);
fprintf('\n Section 8b: Longeron L-section sizing (incl. nose taper panels) \n');
fprintf('Governing panel length: top(+nose) = %.3f mm, bottom(+nose) = %.3f mm\n',a_top_gov,a_bot_gov);
fprintf('Applied stress (M_max envelope): top = %.3f MPa, bottom = %.3f MPa (allow = %.3f MPa)\n',...
        sigma_applied_top,sigma_applied_bot,sigma_allow);
fprintf('Adopted leg length b = %.3f mm (t = %.1f mm) -> A = %.3f mm^2, I_min = %.3f mm^4\n',...
        b_design,t_longeron,A_longeron,I_longeron);
fprintf('  Top longeron:    Pcr = %.2f N, applied = %.2f N, SM = %.2f\n',...
        Pcr_top,sigma_applied_top*A_longeron,SM_top);
fprintf('  Bottom longeron: Pcr = %.2f N, applied = %.2f N, SM = %.2f\n',...
        Pcr_bot,sigma_applied_bot*A_longeron,SM_bot);
fprintf('\n Data for Fig 5: longeron L-section feasible region \n');
if strengthOK
    strengthTxt='satisfied';
else
    strengthTxt='NOT satisfied';
end
fprintf('  Strength check (applied <= allowable %.3f MPa): %s\n',sigma_allow,strengthTxt);
fprintf('  Leg length at sigma_cr = sigma_applied (nearest sweep point): top = %.3f mm, bottom = %.3f mm\n',...
        b_design_top,b_design_bot);
if isempty(iFeas)
    fprintf('  Feasible region: none inside the sweep (b = %.1f to %.1f mm)\n',b_sweep(1),b_sweep(end));
else
    fprintf('  Feasible region (sigma_cr >= sigma_applied for both longerons): b >= %.3f mm, up to %.1f mm (end of sweep)\n',...
            b_sweep(iFeas),b_sweep(end));
    if iFeas == 1
        fprintf('  The whole sweep is feasible; the lower limit is at or below b = %.1f mm\n',b_sweep(1));
    end
end
bAllowTop=interp1(sigma_cr_top,b_sweep,sigma_allow);
bAllowBot=interp1(sigma_cr_bot,b_sweep,sigma_allow);
fprintf('  Leg length where sigma_cr reaches sigma_allow: top = %.3f mm, bottom = %.3f mm (NaN = outside the sweep)\n',...
        bAllowTop,bAllowBot);
if strengthOK && ~isempty(iFeas) && b_design >= b_sweep(iFeas)
    adoptedTxt='inside the feasible region';
else
    adoptedTxt='OUTSIDE the feasible region';
end
fprintf('  Adopted b = %.3f mm is %s; sigma_cr = %.3f MPa (top), %.3f MPa (bottom)\n',...
        b_design,adoptedTxt,Pcr_top/A_longeron,Pcr_bot/A_longeron);
zTopNoseFun=@(xq) heightBox/2 + (noseTipHalfH + (noseBaseHalfH-noseTipHalfH).*(xq./xNoseEnd));
zBotNoseFun=@(xq) heightBox/2 - (noseTipHalfH + (noseBaseHalfH-noseTipHalfH).*(xq./xNoseEnd));
xNoseLine=[0 xNoseEnd];
fprintf('\nDone. FUSELAGE_ONLY = %d, BUCKLING_METHOD = %d\n',FUSELAGE_ONLY,BUCKLING_METHOD);
t_bh=1;
L_lift=67.491;
zCentroid_UAV=113.475;
qWebFun=@(s) -(V_design*t_sec/Iyy_sec) .* ( (h_sec - zNA).*s - s.^2/2 );
qFlangeFun=@(s) qWebFun(h_sec) + (V_design*t_sec/Iyy_sec)*zNA .* s;
nSeg=1000;
r_sweep=linspace(5,40,60);
sigma_max_r=zeros(size(r_sweep));
for ir=1:numel(r_sweep)
    res=bulkhead_ring_solve(r_sweep(ir),t_bh,qWebFun,qFlangeFun,h_sec,L_lift,nSeg);
    sigma_max_r(ir)=res.sigma_max;
end
[~,idxCross]=min(abs(sigma_max_r - sigma_allow));
r_design=15;
res_design=bulkhead_ring_solve(r_design,t_bh,qWebFun,qFlangeFun,h_sec,L_lift,nSeg);
figure('Name','Bulkhead: r vs Max Combined Stress','Color','w');
plot(r_sweep,sigma_max_r,'b-','LineWidth',1.8); hold on; grid on;
yline(sigma_allow,'r--',sprintf('\\sigma_{allow} = %.2f MPa',sigma_allow),'FontSize',20);
plot(r_design,sigma_max_r(idxCross),'rp','MarkerSize',14,'MarkerFaceColor','r');
text(r_design,sigma_max_r(idxCross),sprintf('  r \\approx %.2f mm',r_design),'FontSize',20);
xlabel('Flange depth, r (mm)','FontSize',20);
ylabel('Max combined stress in ring, \sigma (MPa)','FontSize',20);
title('Bulkhead Ring (x = 265 mm): r vs Maximum Combined Stress','FontSize',20);
legend('\sigma_{max}(r)','\sigma_{allow}','Nearest to allowable','Location','best','FontSize',20,'Box','off');
styleAxes(gca);
hold off;
fprintf('\n Bulkhead ring (x = 265 mm) summary \n');
fprintf('Lift applied (point load, top): %.3f N\n',L_lift);
fprintf('r nearest sigma_allow (%.2f MPa): r = %.2f mm (sigma = %.2f MPa)\n',...
        sigma_allow,r_design,sigma_max_r(idxCross));
fprintf('\n Data for Fig 6: wing-AC bulkhead r sweep \n');
fprintf('  Sweep: r = %.1f to %.1f mm, sigma_max ranges from %.3f to %.3f MPa\n',...
        r_sweep(1),r_sweep(end),min(sigma_max_r),max(sigma_max_r));
fprintf('  Sweep point nearest sigma_allow: r = %.3f mm (sigma = %.3f MPa)\n',...
        r_sweep(idxCross),sigma_max_r(idxCross));
fprintf('  Adopted r = %.3f mm: sigma_max = %.3f MPa (allow = %.3f MPa, margin = %.2f%%)\n',...
        r_design,res_design.sigma_max,sigma_allow,...
        100*(sigma_allow - res_design.sigma_max)/sigma_allow);
tau_bh_applied=max(abs(qWebFun(linspace(0,h_sec,200))))/t_bh;
bhPanelDefs={ 'Bulkhead web (depth r)',2*res_design.h_r,r_design; ...
            'Bulkhead top/bottom (depth r)',width-r_design,r_design };
nBH=size(bhPanelDefs,1);
BH_Name=strings(nBH,1); BH_a=zeros(nBH,1); BH_b=zeros(nBH,1);
BH_critStress=zeros(nBH,1); BH_SM=zeros(nBH,1);
for i=1:nBH
    BH_Name(i)=string(bhPanelDefs{i,1});
    BH_a(i)=bhPanelDefs{i,2}; BH_b(i)=bhPanelDefs{i,3};
    [~,critStress]=longeron_buckling(BUCKLING_METHOD,E_bh,t_bh,mu_bh,BH_a(i),BH_b(i));
    BH_critStress(i)=critStress;
    BH_SM(i)=critStress/tau_bh_applied;
end
BulkheadPanelTable=table(BH_Name,BH_a,BH_b,BH_critStress,...
    repmat(tau_bh_applied,nBH,1),BH_SM,'VariableNames',...
    {'Panel','a_mm','b_mm','CriticalStress_MPa','AppliedShear_MPa','SafetyMargin'});
disp(' '); disp('Bulkhead panel buckling summary:'); disp(BulkheadPanelTable);
fprintf('\nLongeron check (Section 8b, incl. nose panels): top SM = %.2f, bottom SM = %.2f\n',SM_top,SM_bot);
r_plot=r_design;
eta_R=res_design.eta; zeta_R=res_design.zeta;
qeta_R=res_design.qeta; qzeta_R=res_design.qzeta;
Feta_R=res_design.Feta; Fzeta_R=res_design.Fzeta;
M_R=res_design.M; N_R=res_design.N;
sigB_R=res_design.sigma_bend; sigA_R=res_design.sigma_axial;
sigC_R=res_design.sigma_comb; s_R=res_design.sigma;
eta_L=-fliplr(eta_R); zeta_L=fliplr(zeta_R);
qeta_L=-fliplr(qeta_R); qzeta_L=fliplr(qzeta_R);
Feta_L=-fliplr(Feta_R); Fzeta_L=fliplr(Fzeta_R);
M_L=fliplr(M_R); N_L=fliplr(N_R);
sigB_L=fliplr(sigB_R); sigA_L=fliplr(sigA_R);
sigC_L=fliplr(sigC_R);
s_L=s_R(end) + (s_R(end) - fliplr(s_R));
eta_full=[eta_R,eta_L(2:end)];
zeta_full=[zeta_R,zeta_L(2:end)];
qeta_full=[qeta_R,qeta_L(2:end)];
qzeta_full=[qzeta_R,qzeta_L(2:end)];
Feta_full=[Feta_R,Feta_L(2:end)];
Fzeta_full=[Fzeta_R,Fzeta_L(2:end)];
M_full=[M_R,M_L(2:end)];
N_full=[N_R,N_L(2:end)];
sigB_full=[sigB_R,sigB_L(2:end)];
sigA_full=[sigA_R,sigA_L(2:end)];
sigC_full=[sigC_R,sigC_L(2:end)];
s_full=[s_R,s_L(2:end)];
figure('Name','Bulkhead Ring Cross-Section','Color','w');
plot(eta_full,zeta_full,'k-','LineWidth',2); hold on; grid on; axis equal;
plot(0,res_design.h_r,'k^','MarkerSize',8,'MarkerFaceColor','y');
text(0,res_design.h_r+6,'L (lift) applied here','HorizontalAlignment','center','FontSize',20);
labelRingSides(gca,res_design.b_r,res_design.h_r);
xlabel('\eta (mm)','FontSize',20); ylabel('\zeta (mm)','FontSize',20);
title(sprintf('Bulkhead Ring Cross-Section (r = %.2f mm)',r_plot),'FontSize',20);
styleAxes(gca);
hold off;
figure('Name','Bulkhead Ring - Shear Flow q(s)','Color','w');
plot(s_full,qzeta_full,'b--','LineWidth',1.5); hold on; grid on;
plot(s_full,qeta_full,'r--','LineWidth',1.5);
xlabel('Arc length s (mm), from top-load point around the ring','FontSize',20);
ylabel('Shear flow q (N/mm)','FontSize',20);
title('Applied Shear Flow Components along Bulkhead Ring','FontSize',20);
legend('q_{\zeta} (vertical)','q_{\eta} (horizontal)','Location','best','FontSize',20,'Box','off');
markRingStations(gca,res_design.b_r,res_design.h_r);
styleAxes(gca);
hold off;
figure('Name','Bulkhead Ring - 3D Shear Flow','Color','w');
plot3(eta_full,zeta_full,qzeta_full,'b--','LineWidth',2); hold on; grid on;
plot3(eta_full,zeta_full,qeta_full,'r--','LineWidth',2);
labelRingSides(gca,res_design.b_r,res_design.h_r,0);
xlabel('\eta (mm)','FontSize',20); ylabel('\zeta (mm)','FontSize',20); zlabel('Shear flow q (N/mm)','FontSize',20);
title('3D Shear Flow Distribution over Bulkhead Ring','FontSize',20);
legend('q_{\zeta}','q_{\eta}','Location','best','FontSize',20,'Box','off');
view(30,30); styleAxes(gca); hold off;
figure('Name','Bulkhead Ring - Section Shear Flow Map','Color','w');
qmag_full=hypot(qeta_full,qzeta_full);
scatter(eta_full,zeta_full,22,qmag_full,'filled'); hold on; grid on; axis equal;
colormap(parula); cb=colorbar; cb.Label.String='Shear flow magnitude |q| (N/mm)';
plot(eta_full,zeta_full,'k-','LineWidth',0.5);
labelRingSides(gca,res_design.b_r,res_design.h_r);
xlabel('\eta (mm)','FontSize',20); ylabel('\zeta (mm)','FontSize',20);
title('Shear Flow Magnitude on Bulkhead Ring Cross Section','FontSize',20);
styleAxes(gca);
hold off;
plot_ring_internal_combined('Bulkhead Ring - Internal Forces and Moment',...
    'Internal Forces and Bending Moment around Bulkhead Ring',...
    s_full,Feta_full,Fzeta_full,N_full,M_full,res_design.b_r,res_design.h_r);
figure('Name','Bulkhead Ring - Stress Distribution','Color','w');
plot(s_full,sigB_full,'b--','LineWidth',1.4); hold on; grid on;
plot(s_full,sigA_full,'r--','LineWidth',1.4);
plot(s_full,sigC_full,'k--','LineWidth',1.8);
xlabel('Arc length s (mm) - shaded bands mark location on the ring','FontSize',20);
ylabel('Stress (MPa)','FontSize',20);
title(sprintf('Stress Distribution around Bulkhead Ring (r = %.2f mm)',r_plot),'FontSize',20);
legend('\sigma_{bend}','\sigma_{axial}','\sigma_{combined}',...
    'Location','best','FontSize',20,'Box','off');
markRingStations(gca,res_design.b_r,res_design.h_r);
styleAxes(gca);
hold off;
fprintf('\n');
fprintf('\n');
fprintf(' BULKHEAD DESIGN SUMMARY - WING AC (fuselage x = 265 mm)\n');
fprintf('\n');
fprintf('\n Inherited from fuselage cross-section (Section 7) \n');
fprintf('  Design shear force, V_design = %.4f N\n',V_design);
fprintf('  Fuselage section I_yy = %.2f mm^4, neutral axis z_NA = %.4f mm\n',Iyy_sec,zNA);
fprintf('  Fuselage skin thickness, t_sec = %.2f mm\n',t_sec);
fprintf('\n Inputs \n');
fprintf('  Material: E = %.1f MPa, mu = %.2f\n',E_bh,mu_bh);
fprintf('  Allowable stress: sigma_allow = 276/6.75 = %.3f MPa\n',sigma_allow);
fprintf('  Bulkhead thickness, t_bh = %.2f mm\n',t_bh);
fprintf('  Applied lift, L = %.3f N\n',L_lift);
fprintf('  Outer envelope: %.1f mm (y) x 165 mm (z)\n',width);
fprintf('  Flange depth sweep: r = %.1f to %.1f mm (%d points)\n',...
        r_sweep(1),r_sweep(end),numel(r_sweep));
fprintf('\n Sizing result \n');
fprintf('  r_design = %.3f mm  (closest match to sigma_allow)\n',r_design);
fprintf('  sigma_max at r_design = %.3f MPa (allow = %.3f MPa, margin = %.2f%%)\n',...
        sigma_max_r(idxCross),sigma_allow,...
        100*(sigma_allow - sigma_max_r(idxCross))/sigma_allow);
fprintf('\n Section properties at r_design \n');
fprintf('  Half-width of flat member, b_r = %.3f mm\n',res_design.b_r);
fprintf('  Half-height of web, h_r = %.3f mm\n',res_design.h_r);
fprintf('  Cross-sectional area, A_r = %.3f mm^2\n',res_design.A_r);
fprintf('  Second moment of area, I_r = %.3f mm^4\n',res_design.I_r);
fprintf('\n Redundants at the symmetry cut \n');
fprintf('  M0 = %.3f N.mm\n',res_design.M0);
fprintf('  N0 = %.3f N\n',res_design.N0);
fprintf('\n Governing internal loads/stresses (with location on the ring) \n');
report_quantity('Bending moment, M(s)',M_full,s_full,res_design.b_r,res_design.h_r,'N.mm');
report_quantity('Axial force, N(s)',N_full,s_full,res_design.b_r,res_design.h_r,'N');
report_quantity('Horizontal shear, F_eta(s)',Feta_full,s_full,res_design.b_r,res_design.h_r,'N');
report_quantity('Vertical shear, F_zeta(s)',Fzeta_full,s_full,res_design.b_r,res_design.h_r,'N');
report_quantity('Bending stress, sigma_bend',sigB_full,s_full,res_design.b_r,res_design.h_r,'MPa');
report_quantity('Axial stress, sigma_axial',sigA_full,s_full,res_design.b_r,res_design.h_r,'MPa');
report_quantity('Combined stress, sigma_comb',sigC_full,s_full,res_design.b_r,res_design.h_r,'MPa');
report_quantity('Shear flow, q_eta(s)',qeta_full,s_full,res_design.b_r,res_design.h_r,'N/mm');
report_quantity('Shear flow, q_zeta(s)',qzeta_full,s_full,res_design.b_r,res_design.h_r,'N/mm');
fprintf('\n Panel buckling (bulkhead flat members) \n');
disp(BulkheadPanelTable);
[~,gIdx]=min(BH_SM);
fprintf('  Governing panel: %s (safety margin = %.2f)\n',BulkheadPanelTable.Panel(gIdx),BH_SM(gIdx));
fprintf('\n Longeron check (constant L-section, Section 8b, incl. nose) \n');
fprintf('  b = %.2f mm, t = %.1f mm -> A = %.3f mm^2, I_min = %.3f mm^4\n',...
        b_design,t_longeron,A_longeron,I_longeron);
fprintf('  Top longeron SM = %.2f   |   Bottom longeron SM = %.2f\n',SM_top,SM_bot);
fprintf('\n\n');
fprintf(' END OF BULKHEAD SUMMARY\n');
fprintf('\n\n');
Iyy_box=2*(t_sec*h_sec^3/12) + 2*(t_sec*b_sec)*(h_sec/2)^2 + 2*(t_sec^3*b_sec/12);
kbox=V_design*t_sec/Iyy_box;
nSegBox=300;
s_flange_box=linspace(0,b_sec/2,nSegBox);
q_flange_box=-kbox*(h_sec/2).*s_flange_box;
qCorner_box=q_flange_box(end);
s_web_box=linspace(0,h_sec,nSegBox);
q_web_box=qCorner_box - kbox*((h_sec/2).*s_web_box - s_web_box.^2/2);
closure_check=qCorner_box + kbox*(h_sec/2)*(b_sec/2);
qFlangeFun_box=@(s) -kbox*(h_sec/2).*s;
qWebFun_box=@(s) qCorner_box - kbox*((h_sec/2).*s - s.^2/2);
s_quarter=[s_flange_box,b_sec/2+s_web_box(2:end),b_sec/2+h_sec+s_flange_box(2:end)];
q_bottomflange_box=qCorner_box + kbox*(h_sec/2).*s_flange_box(2:end);
q_quarter=[q_flange_box,q_web_box(2:end),q_bottomflange_box];
qmax_box=max(abs(q_quarter));
fprintf('\n Section 13: Closed rectangular box shear flow \n');
fprintf('I_yy (closed box)   = %.2f mm^4  (open-channel I_yy for comparison = %.2f mm^4)\n',Iyy_box,Iyy_sec);
fprintf('q at top/bot corner = %.4f N/mm\n',qCorner_box);
fprintf('Closure check (bottom mid-span, should be ~0) = %.6f N/mm\n',closure_check);
fprintf('q_max (closed box)  = %.4f N/mm\n',qmax_box);
[~,iQm]=max(abs(q_quarter));
fprintf('\n Data for Fig 13: closed box shear flow \n');
fprintf('  Shear flow at the flange mid-span = %.4f N/mm\n',q_flange_box(1));
fprintf('  Shear flow at the top corner = %.4f N/mm\n',qCorner_box);
fprintf('  Shear flow at web mid-height = %.4f N/mm\n',qWebFun_box(h_sec/2));
fprintf('  Governing |q| = %.4f N/mm at s = %.2f mm along the quarter path\n',qmax_box,s_quarter(iQm));
fprintf('  Maximum shear stress |q|max/t = %.4f MPa\n',qmax_box/t_sec);
figure('Name','Closed Box Shear Flow (quarter section)','Color','w');
plot(s_quarter,q_quarter,'b-','LineWidth',1.5); hold on; grid on;
xline(b_sec/2,':','Top corner','FontSize',20); xline(b_sec/2+h_sec,':','Bottom corner','FontSize',20);
xlabel('Arc length s (mm), top mid-span -> right side -> bottom mid-span','FontSize',20);
ylabel('Shear flow q (N/mm)','FontSize',20);
title(sprintf('Closed-Box Shear Flow (V_{design} = %.3f N), q_{max} = %.4f N/mm',...
      V_design,qmax_box),'FontSize',20);
styleAxes(gca); hold off;
x_LG=170.3;
r_LG_solve=r_design;
nSegLG=300;
res_LG=bulkhead_ring_closed_box(r_LG_solve,t_bh,qFlangeFun_box,qWebFun_box,...
            h_sec,0,'none',nSegLG);
tau_LG_applied=qmax_box / t_bh;
LGPanelDefs={ 'LG bulkhead web (depth r_lg)',2*res_LG.h_r,r_design; ...
            'LG bulkhead top/bottom (depth r_lg)',width - 2*r_design,r_design };
nBH_LG=size(LGPanelDefs,1);
BH_LG_Name=strings(nBH_LG,1); BH_LG_a=zeros(nBH_LG,1); BH_LG_b=zeros(nBH_LG,1);
BH_LG_critStress=zeros(nBH_LG,1); BH_LG_SM=zeros(nBH_LG,1);
for i=1:nBH_LG
    BH_LG_Name(i)=string(LGPanelDefs{i,1});
    BH_LG_a(i)=LGPanelDefs{i,2}; BH_LG_b(i)=LGPanelDefs{i,3};
    [~,critStress]=longeron_buckling(BUCKLING_METHOD,E_bh,t_bh,mu_bh,BH_LG_a(i),BH_LG_b(i));
    BH_LG_critStress(i)=critStress;
    BH_LG_SM(i)=critStress/tau_LG_applied;
end
LGPanelTable=table(BH_LG_Name,BH_LG_a,BH_LG_b,BH_LG_critStress,...
    repmat(tau_LG_applied,nBH_LG,1),BH_LG_SM,'VariableNames',...
    {'Panel','a_mm','b_mm','CriticalStress_MPa','AppliedShear_MPa','SafetyMargin'});
plot_bulkhead_full(res_LG,'Landing Gear Bulkhead (x=170.3mm)',0,'none');
fprintf('\n\n');
fprintf(' BULKHEAD DESIGN SUMMARY - LANDING GEAR (fuselage x = %.2f mm)\n',x_LG);
fprintf('\n');
fprintf('\n Section 13 inheritance \n');
fprintf('  V_design = %.4f N (global max, reused)\n',V_design);
fprintf('  Closed-box I_yy = %.2f mm^4, q_max = %.4f N/mm\n',Iyy_box,qmax_box);
fprintf('\n Inputs \n');
fprintf('  Material: E = %.1f MPa, mu = %.2f, t_bh = %.2f mm\n',E_bh,mu_bh,t_bh);
fprintf('  r_lg (= wing-bulkhead r_design) = %.3f mm\n',r_design);
fprintf('  Bulkhead envelope = %.2f x %.2f mm\n',...
        width-2*r_design,165-2*r_design);
fprintf('  External point load: NONE\n');
fprintf('\n Section properties \n');
fprintf('  b_r=%.3f mm, h_r=%.3f mm, A_r=%.3f mm^2, I_r=%.3f mm^4\n',...
        res_LG.b_r,res_LG.h_r,res_LG.A_r,res_LG.I_r);
fprintf('\n Redundants \n  M0=%.3f N.mm, N0=%.3f N\n',res_LG.M0,res_LG.N0);
fprintf('\n Governing stress \n');
fprintf('  sigma_max = %.3f MPa (allow=%.3f MPa, margin=%.2f%%)\n',...
        res_LG.sigma_max,sigma_allow,100*(sigma_allow-res_LG.sigma_max)/sigma_allow);
fprintf('\n Panel buckling \n'); disp(LGPanelTable);
[~,gIdxLG]=min(BH_LG_SM);
fprintf('  Governing panel: %s (SM=%.2f)\n',LGPanelTable.Panel(gIdxLG),BH_LG_SM(gIdxLG));
fprintf('\n Longeron check (constant L-section, Section 8b) \n');
fprintf('  Top longeron SM = %.2f   |   Bottom longeron SM = %.2f\n',SM_top,SM_bot);
fprintf('\n\n');
x_nose_bh=129.53;
P_nose=0.9*9.81;
r_nose_solve=r_design;
res_Nose=bulkhead_ring_closed_box(r_nose_solve,t_bh,qFlangeFun_box,qWebFun_box,...
            h_sec,P_nose,'bottom',nSegLG);
tau_Nose_applied=qmax_box / t_bh;
NosePanelDefs={ 'Nose bulkhead web (depth r_lg)',2*res_Nose.h_r,r_design; ...
            'Nose bulkhead top/bottom (depth r_lg)',width - 2*r_design,r_design };
nBH_N=size(NosePanelDefs,1);
BH_N_Name=strings(nBH_N,1); BH_N_a=zeros(nBH_N,1); BH_N_b=zeros(nBH_N,1);
BH_N_critStress=zeros(nBH_N,1); BH_N_SM=zeros(nBH_N,1);
for i=1:nBH_N
    BH_N_Name(i)=string(NosePanelDefs{i,1});
    BH_N_a(i)=NosePanelDefs{i,2}; BH_N_b(i)=NosePanelDefs{i,3};
    [~,critStress]=longeron_buckling(BUCKLING_METHOD,E_bh,t_bh,mu_bh,BH_N_a(i),BH_N_b(i));
    BH_N_critStress(i)=critStress;
    BH_N_SM(i)=critStress/tau_Nose_applied;
end
NosePanelTable=table(BH_N_Name,BH_N_a,BH_N_b,BH_N_critStress,...
    repmat(tau_Nose_applied,nBH_N,1),BH_N_SM,'VariableNames',...
    {'Panel','a_mm','b_mm','CriticalStress_MPa','AppliedShear_MPa','SafetyMargin'});
plot_bulkhead_full(res_Nose,'Nose Bulkhead (x=129.53mm)',P_nose,'bottom');
fprintf('\n\n');
fprintf(' BULKHEAD DESIGN SUMMARY - NOSE (fuselage x = %.2f mm)\n',x_nose_bh);
fprintf('\n');
fprintf('\n Section 13 inheritance \n');
fprintf('  V_design = %.4f N (global max, reused)\n',V_design);
fprintf('  Closed-box I_yy = %.2f mm^4, q_max = %.4f N/mm\n',Iyy_box,qmax_box);
fprintf('\n Inputs \n');
fprintf('  Material: E = %.1f MPa, mu = %.2f, t_bh = %.2f mm\n',E_bh,mu_bh,t_bh);
fprintf('  r_lg (= wing-bulkhead r_design) = %.3f mm\n',r_design);
fprintf('  Bulkhead envelope = %.2f x %.2f mm\n',width-2*r_design,165-2*r_design);
fprintf('  External point load: P = %.4f N, downward, at BOTTOM node\n',P_nose);
fprintf('\n Section properties \n');
fprintf('  b_r=%.3f mm, h_r=%.3f mm, A_r=%.3f mm^2, I_r=%.3f mm^4\n',...
        res_Nose.b_r,res_Nose.h_r,res_Nose.A_r,res_Nose.I_r);
fprintf('\n Redundants \n  M0=%.3f N.mm, N0=%.3f N\n',res_Nose.M0,res_Nose.N0);
fprintf('\n Governing stress \n');
fprintf('  sigma_max = %.3f MPa (allow=%.3f MPa, margin=%.2f%%)\n',...
        res_Nose.sigma_max,sigma_allow,100*(sigma_allow-res_Nose.sigma_max)/sigma_allow);
fprintf('\n Panel buckling \n'); disp(NosePanelTable);
[~,gIdxN]=min(BH_N_SM);
fprintf('  Governing panel: %s (SM=%.2f)\n',NosePanelTable.Panel(gIdxN),BH_N_SM(gIdxN));
fprintf('\n Longeron check (constant L-section, Section 8b) \n');
fprintf('  Top longeron SM = %.2f   |   Bottom longeron SM = %.2f\n',SM_top,SM_bot);
fprintf('\n\n');
r_lg_sweep=r_sweep/2;
sigma_max_r_LG=zeros(size(r_lg_sweep));
sigma_max_r_Nose=zeros(size(r_lg_sweep));
for ir=1:numel(r_lg_sweep)
    r_try=2*r_lg_sweep(ir);
    resTmp_LG=bulkhead_ring_closed_box(r_try,t_bh,qFlangeFun_box,qWebFun_box,h_sec,0,'none',nSegLG);
    resTmp_Nose=bulkhead_ring_closed_box(r_try,t_bh,qFlangeFun_box,qWebFun_box,h_sec,P_nose,'bottom',nSegLG);
    sigma_max_r_LG(ir)=resTmp_LG.sigma_max;
    sigma_max_r_Nose(ir)=resTmp_Nose.sigma_max;
end
[~,idxLG_cross]=min(abs(sigma_max_r_LG - sigma_allow));
[~,idxNose_cross]=min(abs(sigma_max_r_Nose - sigma_allow));
figure('Name','LG Bulkhead: r_lg vs Max Combined Stress','Color','w');
plot(r_lg_sweep,sigma_max_r_LG,'b-','LineWidth',1.8); hold on; grid on;
yline(sigma_allow,'r--',sprintf('\\sigma_{allow} = %.2f MPa',sigma_allow),'FontSize',20);
xline(r_design,'k:',sprintf('adopted r_{lg} = %.2f mm',r_design),'FontSize',20);
plot(r_lg_sweep(idxLG_cross),sigma_max_r_LG(idxLG_cross),'rp','MarkerSize',14,'MarkerFaceColor','r');
xlabel('r_{lg} (mm)','FontSize',20); ylabel('Max combined stress, \sigma (MPa)','FontSize',20);
title('Landing Gear Bulkhead (x=170.3mm): r_{lg} vs Maximum Combined Stress','FontSize',20);
legend('\sigma_{max}(r_{lg})','\sigma_{allow}','adopted r_{lg}','own nearest crossing','Location','best','FontSize',20,'Box','off');
styleAxes(gca); hold off;
figure('Name','Nose Bulkhead: r_nose vs Max Combined Stress','Color','w');
plot(r_lg_sweep,sigma_max_r_Nose,'b-','LineWidth',1.8); hold on; grid on;
yline(sigma_allow,'r--',sprintf('\\sigma_{allow} = %.2f MPa',sigma_allow),'FontSize',20);
xline(r_design,'k:',sprintf('adopted r_{nose} = %.2f mm',r_design),'FontSize',20);
plot(r_lg_sweep(idxNose_cross),sigma_max_r_Nose(idxNose_cross),'rp','MarkerSize',14,'MarkerFaceColor','r');
xlabel('r_{nose} (mm)','FontSize',20); ylabel('Max combined stress, \sigma (MPa)','FontSize',20);
title('Nose Bulkhead (x=129.53mm): r_{lg} vs Maximum Combined Stress','FontSize',20);
legend('\sigma_{max}(r_{nose})','\sigma_{allow}','adopted r_{nose}','own nearest crossing','Location','best','FontSize',20,'Box','off');
styleAxes(gca); hold off;
fprintf('\n Section 16: r_lg sweep cross-check \n');
fprintf('LG bulkhead:   own nearest-crossing r_lg = %.3f mm (sigma=%.3f MPa) vs adopted r_lg = %.3f mm (sigma=%.3f MPa)\n',...
        r_lg_sweep(idxLG_cross),sigma_max_r_LG(idxLG_cross),r_design,res_LG.sigma_max);
fprintf('Nose bulkhead: own nearest-crossing r_nose = %.3f mm (sigma=%.3f MPa) vs adopted r_nose = %.3f mm (sigma=%.3f MPa)\n',...
        r_lg_sweep(idxNose_cross),sigma_max_r_Nose(idxNose_cross),r_design,res_Nose.sigma_max);
fprintf('\n Data for Fig 26 and Fig 27: r_lg sweep limits \n');
sweepNames={'LG bulkhead','Nose bulkhead'};
sweepSigma={sigma_max_r_LG,sigma_max_r_Nose};
for k=1:2
    sg=sweepSigma{k};
    iOK=find(sg <= sigma_allow,1,'first');
    fprintf('  %-13s sweep r_lg = %.2f to %.2f mm, sigma_max ranges from %.3f to %.3f MPa\n',...
            sweepNames{k},r_lg_sweep(1),r_lg_sweep(end),min(sg),max(sg));
    if isempty(iOK)
        fprintf('  %-13s no r_lg in the sweep brings sigma_max below sigma_allow (%.3f MPa)\n',...
            sweepNames{k},sigma_allow);
    else
        fprintf('  %-13s smallest r_lg with sigma_max <= sigma_allow: %.3f mm (sigma = %.3f MPa)\n',...
            sweepNames{k},r_lg_sweep(iOK),sg(iOK));
    end
end
figure('Name','Fuselage Structure View','Color','w');
hold on; grid on; axis equal;
xStations2=linspace(0,xFusEnd,150);
hStations2=fuselage_height(xStations2,xNoseEnd,xBoxEnd,xFusEnd,heightBox,tailDrop,noseTipHalfH);
for k=1:numel(xStations2)-1
    z1=hStations2(k); z2=hStations2(k+1);
    if isnan(z1) || isnan(z2); continue; end
    Xp=[xStations2(k) xStations2(k+1) xStations2(k+1) xStations2(k)];
    patch(Xp,[-width/2 -width/2 width/2 width/2],[0 0 0 0],[0.6 0.8 1],'FaceAlpha',0.06,'EdgeColor','none');
    patch(Xp,[-width/2 -width/2 -width/2 -width/2],[z1 z2 z2 z1],[0.6 0.8 1],'FaceAlpha',0.06,'EdgeColor','none');
    patch(Xp,[ width/2 width/2 width/2 width/2],[z1 z2 z2 z1],[0.6 0.8 1],'FaceAlpha',0.06,'EdgeColor','none');
end
plot3([xNoseEnd xFusEnd],[-width/2 -width/2],[heightBox heightBox],'r-','LineWidth',2.2);
plot3([xNoseEnd xFusEnd],[ width/2 width/2],[heightBox heightBox],'r-','LineWidth',2.2);
plot3([xNoseEnd xBoxEnd],[-width/2 -width/2],[0 0],'g-','LineWidth',2.2);
plot3([xNoseEnd xBoxEnd],[ width/2 width/2],[0 0],'g-','LineWidth',2.2);
plot3([xBoxEnd xFusEnd],[-width/2 -width/2],[0 tailDrop],'m-','LineWidth',2.2);
plot3([xBoxEnd xFusEnd],[ width/2 width/2],[0 tailDrop],'m-','LineWidth',2.2);
plot3(xNoseLine,[-width/2 -width/2],zTopNoseFun(xNoseLine),'c-','LineWidth',2.2);
plot3(xNoseLine,[ width/2 width/2],zTopNoseFun(xNoseLine),'c-','LineWidth',2.2);
plot3(xNoseLine,[-width/2 -width/2],zBotNoseFun(xNoseLine),'Color',[1 0.55 0],'LineWidth',2.2);
plot3(xNoseLine,[ width/2 width/2],zBotNoseFun(xNoseLine),'Color',[1 0.55 0],'LineWidth',2.2);
plot_plate_ring_3d(0,width/2,heightBox/2-noseTipHalfH,heightBox/2+noseTipHalfH,...
    plateFrameDepth,[0.85 0.33 0.1],'Motor-mount plate');
plot_plate_ring_3d(xFusEnd,width/2,tailDrop,heightBox,...
    plateFrameDepth,[0.1 0.5 0.2],'End plate');
zTaperFun=@(xq) tailDrop*(xq-xBoxEnd)/(xFusEnd-xBoxEnd);
for rowIdx=1:height(LongeronTable)
    xs=LongeronTable.x_start_mm(rowIdx); xe=LongeronTable.x_end_mm(rowIdx);
    runName=LongeronTable.Longeron_Run(rowIdx);
    for xq=[xs,xe]
        if contains(runName,'Top') && ~contains(runName,'Nose')
            zq=heightBox;
        elseif contains(runName,'box')
            zq=0;
        elseif contains(runName,'Nose taper longeron (top)')
            zq=zTopNoseFun(xq);
        elseif contains(runName,'Nose taper longeron (bottom)')
            zq=zBotNoseFun(xq);
        else
            zq=zTaperFun(xq);
        end
        scatter3(xq,-width/2,zq,45,'ko','filled','MarkerEdgeColor','k');
        scatter3(xq,width/2,zq,45,'ko','filled','MarkerEdgeColor','k');
    end
end
plot_bulkhead_ring_3d(x_nose_bh,r_design,width,heightBox,'Nose BH');
plot_bulkhead_ring_3d(x_LG,r_design,width,heightBox,'LG BH');
plot_bulkhead_ring_3d(265,r_design,width,heightBox,'Wing BH');
xlabel('x (mm)','FontSize',20); ylabel('y (mm)','FontSize',20); zlabel('z (mm)','FontSize',20);
title('Fuselage Structure: Skin, Longerons, Bulkheads and Plates','FontSize',20);
view(30,30);
hold off;
fprintf('\n Data for Fig 28: geometry, materials and structural layout \n');
fprintf('  Nose: x = 0 to %.2f mm, tip dia = %.1f mm, base height = %.1f mm\n',...
        xNoseEnd,noseTipDia,heightBox);
fprintf('  Box: x = %.2f to %.2f mm (%.2f mm long), width = %.1f mm, height = %.1f mm\n',...
        xNoseEnd,xBoxEnd,boxLen,width,heightBox);
fprintf('  Tail taper: x = %.2f to %.2f mm, rear drop = %.1f mm\n',xBoxEnd,xFusEnd,tailDrop);
fprintf('  Fuselage shell end = %.3f mm, overall UAV length = %.2f mm\n',xFusEnd,xUAVEnd);
fprintf('  Skin: t = %.2f mm, density = %.3e g/mm^3, E = %.1f MPa, mu = %.2f\n',...
        skin_t,skin_rho,E_skin,mu_skin);
fprintf('  Bulkhead / stringer material: E = %.1f MPa, mu = %.2f, allowable = %.3f MPa\n',...
        E_bh,mu_bh,sigma_allow);
fprintf('  Bulkheads (t = %.2f mm): nose x = %.2f mm (r = %.1f mm in the analysis), LG x = %.2f mm (r = %.1f mm), wing x = 265 mm (r = %.1f mm)\n',...
        t_bh,x_nose_bh,r_nose_solve,x_LG,r_design,r_design);
fprintf('  Longeron L-section: leg b = %.2f mm, t = %.1f mm\n',b_design,t_longeron);
fprintf('  Longeron panels analysed: %d (see the longeron buckling table above)\n',height(LongeronTable));
format_all_figures(20);
FIG_NAMES={ ...
    'fig01_loading_diagram',...
    'fig02_sfd_bmd',...
    'fig03_u_section_shear_flow',...
    'fig04_section_shear_flow_map',...
    'fig05_longeron_l_section_sizing',...
    'fig06_wing_bulkhead_r_sweep',...
    'fig07_wing_bulkhead_cross_section',...
    'fig08_wing_bulkhead_shear_flow_q',...
    'fig09_wing_bulkhead_3d_shear_flow',...
    'fig10_wing_bulkhead_shear_flow_map',...
    'fig11_wing_bulkhead_internal_forces_moment',...
    'fig12_wing_bulkhead_stress_distribution',...
    'fig13_closed_box_shear_flow',...
    'fig14_lg_bulkhead_cross_section',...
    'fig15_lg_bulkhead_shear_flow_q',...
    'fig16_lg_bulkhead_3d_shear_flow',...
    'fig17_lg_bulkhead_shear_flow_map',...
    'fig18_lg_bulkhead_internal_forces_moment',...
    'fig19_lg_bulkhead_stress_distribution',...
    'fig20_nose_bulkhead_cross_section',...
    'fig21_nose_bulkhead_shear_flow_q',...
    'fig22_nose_bulkhead_3d_shear_flow',...
    'fig23_nose_bulkhead_shear_flow_map',...
    'fig24_nose_bulkhead_internal_forces_moment',...
    'fig25_nose_bulkhead_stress_distribution',...
    'fig26_lg_bulkhead_r_lg_sweep',...
    'fig27_nose_bulkhead_r_lg_sweep',...
    'fig28_comprehensive_structure_3d'};
if SAVE_FIGURES
    save_all_figures(FIG_FOLDER,FIG_NAMES,FIG_FORMATS,FIG_DPI);
end
function h=fuselage_height(x,xNoseEnd,xBoxEnd,xFusEnd,heightBox,tailDrop,noseTipHalfH)
    h=nan(size(x));
    for k=1:numel(x)
        xi=x(k);
        if xi < 0 || xi > xFusEnd
            continue;
        elseif xi <= xNoseEnd
            h(k)=2*noseTipHalfH + (heightBox - 2*noseTipHalfH)*(xi/xNoseEnd);
        elseif xi <= xBoxEnd
            h(k)=heightBox;
        else
            frac=(xi - xBoxEnd) / (xFusEnd - xBoxEnd);
            h(k)=heightBox - frac*tailDrop;
        end
    end
end
function [critLoad,critStress]=longeron_buckling(method,E,t,mu,a,b)
    if method == 1
        Ra=1; Rb=1;
        Kss=5.34 + 4*(b/a)^2;
        Pcr=Kss * (pi^2*E/(12*(1-mu^2))) * (t/b)^2 * ...
            (Ra + ((Rb-Ra)/2)*(b/a)^2);
        critLoad=Pcr;
        critStress=Pcr / t;
    else
        D=E*t^3*(1-mu^2)/12;
        m=1; n=1;
        Ncr=D * (((m*pi/a)^2 + (n*pi/b)^2)^2) / ((m*pi/a)^2);
        critLoad=Ncr;
        critStress=Ncr / t;
    end
end
function res=bulkhead_ring_solve(r,t_bh,qWebFun,qFlangeFun,h_sec,L_lift,nSeg)
    b_r=(97.5 - r)/2; h_r=(165 - r)/2;
    A_r=r*t_bh; I_r=t_bh*r^3/12;
    s1=linspace(0,b_r,nSeg);
    eta1=s1; zeta1=h_r*ones(size(s1));
    qeta1=zeros(size(s1)); qzeta1=zeros(size(s1));
    s2=linspace(0,2*h_r,nSeg);
    eta2=b_r*ones(size(s2)); zeta2=h_r - s2;
    qzeta2=-qWebFun(s2 + r/2);
    qeta2=zeros(size(s2));
    s3=linspace(0,b_r,nSeg);
    eta3=b_r - s3; zeta3=-h_r*ones(size(s3));
    qeta3=qFlangeFun(s3 + r/2);
    qzeta3=zeros(size(s3));
    eta=[eta1,eta2(2:end),eta3(2:end)];
    zeta=[zeta1,zeta2(2:end),zeta3(2:end)];
    qeta=[qeta1,qeta2(2:end),qeta3(2:end)];
    qzeta=[qzeta1,qzeta2(2:end),qzeta3(2:end)];
    dseg=[0,hypot(diff(eta),diff(zeta))];
    sigma=cumsum(dseg);
    dFeta=qeta .* dseg;
    dFzeta=qzeta.* dseg;
    CFz=cumsum(dFzeta); CFe=cumsum(dFeta);
    CMz=cumsum(dFzeta.*eta); CMe=cumsum(dFeta.*zeta);
    M_dist=eta.*CFz - CMz - zeta.*CFe + CMe;
    M_L=-(L_lift/2) .* eta;
    M_known=M_dist + M_L;
    f2=(h_r - zeta);
    A11=trapz(sigma,ones(size(sigma)));
    A12=trapz(sigma,f2);
    A22=trapz(sigma,f2.^2);
    b1=-trapz(sigma,M_known);
    b2=-trapz(sigma,M_known.*f2);
    sol=[A11 A12; A12 A22] \ [b1; b2];
    M0=sol(1); N0=sol(2);
    M_total=M0 + N0.*f2 + M_known;
    Feta_total=N0 + CFe;
    Fzeta_total=-(L_lift/2) + CFz;
    tEta=gradient(eta,sigma); tZeta=gradient(zeta,sigma);
    tN=hypot(tEta,tZeta); tEta=tEta./tN; tZeta=tZeta./tN;
    N_axial=Feta_total.*tEta + Fzeta_total.*tZeta;
    c_fib=r/2;
    sigma_bend=M_total*c_fib/I_r;
    sigma_axial=N_axial/A_r;
    sigma_comb=abs(sigma_bend) + abs(sigma_axial);
    res.r=r; res.b_r=b_r; res.h_r=h_r; res.A_r=A_r; res.I_r=I_r;
    res.sigma=sigma; res.M=M_total; res.N=N_axial;
    res.sigma_comb=sigma_comb; res.sigma_max=max(sigma_comb);
    res.M0=M0; res.N0=N0;
    res.eta=eta; res.zeta=zeta; res.qeta=qeta; res.qzeta=qzeta;
    res.Feta=Feta_total; res.Fzeta=Fzeta_total;
    res.sigma_bend=sigma_bend; res.sigma_axial=sigma_axial;
end
function styleAxes(ax)
    box(ax,'on');
    ax.LineWidth=1;
    ax.FontSize=20;
    ax.GridAlpha=0.25;
    ax.TickDir='out';
    ax.XMinorTick='on';
    ax.YMinorTick='on';
end
function labelRingSides(ax,b_r,h_r,z0)
    if nargin < 4,z0=0; end
    xoff=0.55*b_r;
    text(ax,-xoff,h_r,z0,'Top','FontSize',20,'FontWeight','bold',...
        'VerticalAlignment','bottom','HorizontalAlignment','center');
    text(ax,-xoff,-h_r,z0,'Bottom','FontSize',20,'FontWeight','bold',...
        'VerticalAlignment','top','HorizontalAlignment','center');
    text(ax,b_r,0,z0,' Right web','FontSize',20,'FontWeight','bold',...
        'HorizontalAlignment','left');
    text(ax,-b_r,0,z0,'Left web ','FontSize',20,'FontWeight','bold',...
        'HorizontalAlignment','right');
end
function hSt=markRingStations(ax,b_r,h_r,startZone)
    if nargin < 4,startZone='Top'; end
    halfPerim=2*(b_r + h_r);
    c1=b_r;
    c2=b_r + 2*h_r;
    c3=halfPerim + b_r;
    c4=halfPerim + b_r + 2*h_r;
    sEnd=2*halfPerim;
    if strcmpi(startZone,'Bottom')
        labelTxt={'Bottom','Right web','Top','Left web','Bottom'};
    else
        labelTxt={'Top','Right web','Bottom','Left web','Top'};
    end
    yl=ylim(ax);
    hold(ax,'on');
    top=[1 0.93 0.8];
    web=[0.85 0.93 1];
    bottom=[0.87 1 0.87];
    edges=[0 c1 c2 c3 c4 sEnd];
    colors=cell(1,5);
    for k=1:5
        switch labelTxt{k}
            case 'Top'
            colors{k}=top;
            case 'Bottom'
            colors{k}=bottom;
            otherwise
            colors{k}=web;
        end
    end
    hBands=gobjects(1,5);
    for k=1:5
        hBands(k)=patch(ax,edges([k k+1 k+1 k]),yl([1 1 2 2]),colors{k},...
            'FaceAlpha',0.35,'EdgeColor','none','HandleVisibility','off');
    end
    xcs=[c1 c2 c3 c4];
    hDiv=gobjects(1,4);
    for k=1:4
        hDiv(k)=xline(ax,xcs(k),':','Color',[0.4 0.4 0.4],'LineWidth',0.75,...
            'HandleVisibility','off');
    end
    hEnd=gobjects(1,2);
    hEnd(1)=xline(ax,0,'-','Color',[0.6 0 0],'LineWidth',1,'HandleVisibility','off');
    hEnd(2)=xline(ax,sEnd,'-','Color',[0.6 0 0],'LineWidth',1,'HandleVisibility','off');
    yLab=yl(1) + 0.93*(yl(2) - yl(1));
    labelPos=[mean([0 c1]),mean([c1 c2]),mean([c2 c3]),mean([c3 c4]),mean([c4 sEnd])];
    for k=1:5
        text(ax,labelPos(k),yLab,labelTxt{k},'FontSize',20,...
            'HorizontalAlignment','center','Color',[0.3 0.3 0.3],...
            'BackgroundColor','w','Margin',1);
    end
    ylim(ax,yl);
    uistack(findobj(ax,'Type','patch'),'bottom');
    hSt.bands=hBands;
    hSt.dividers=hDiv;
    hSt.ends=hEnd;
    hSt.zones=labelTxt;
end
function report_quantity(name,vals,s,b_r,h_r,unit,startZone)
    if nargin < 7,startZone='Top'; end
    [vmax,imax]=max(vals);
    [vmin,imin]=min(vals);
    fprintf('  %s:\n',name);
    fprintf('    max = %10.4f %s at s = %8.3f mm (%s)\n',...
            vmax,unit,s(imax),ringZoneAt(s(imax),b_r,h_r,startZone));
    fprintf('    min = %10.4f %s at s = %8.3f mm (%s)\n',...
            vmin,unit,s(imin),ringZoneAt(s(imin),b_r,h_r,startZone));
end
function zone=ringZoneAt(s,b_r,h_r,startZone)
    if nargin < 4,startZone='Top'; end
    halfPerim=2*(b_r + h_r);
    c1=b_r; c2=b_r + 2*h_r; c3=halfPerim + b_r; c4=halfPerim + b_r + 2*h_r;
    s=mod(s,2*halfPerim);
    if s <= c1
        zone='Top';
    elseif s <= c2
        zone='Right web';
    elseif s <= c3
        zone='Bottom';
    else
        zone='Left web';
    end
    if strcmpi(startZone,'Bottom')
        if strcmp(zone,'Top')
            zone='Bottom';
        elseif strcmp(zone,'Bottom')
            zone='Top';
        end
    end
end
function res=bulkhead_ring_closed_box(r,t_bh,qFlangeFun,qWebFun,h_sec,Pload,loadNode,nSeg)
    b_r=(97.5-r)/2; h_r=(165-r)/2;
    A_r=r*t_bh; I_r=t_bh*r^3/12;
    if strcmpi(loadNode,'bottom')
        z1=-h_r; z3=h_r;
    else
        z1=h_r; z3=-h_r;
    end
    s1=linspace(0,b_r,nSeg);
    eta1=s1; zeta1=z1*ones(size(s1));
    qeta1=qFlangeFun(s1 + r/2); qzeta1=zeros(size(s1));
    s2=linspace(0,2*h_r,nSeg);
    eta2=b_r*ones(size(s2)); zeta2=z1 - sign(z1)*s2;
    qzeta2=-sign(z1)*qWebFun(s2 + r/2); qeta2=zeros(size(s2));
    s3=linspace(0,b_r,nSeg);
    eta3=b_r - s3; zeta3=z3*ones(size(s3));
    qeta3=qFlangeFun((b_r - s3) + r/2); qzeta3=zeros(size(s3));
    eta=[eta1,eta2(2:end),eta3(2:end)];
    zeta=[zeta1,zeta2(2:end),zeta3(2:end)];
    qeta=[qeta1,qeta2(2:end),qeta3(2:end)];
    qzeta=[qzeta1,qzeta2(2:end),qzeta3(2:end)];
    dseg=[0,hypot(diff(eta),diff(zeta))];
    sigma=cumsum(dseg);
    dFeta=qeta.*dseg; dFzeta=qzeta.*dseg;
    CFe=cumsum(dFeta); CFz=cumsum(dFzeta);
    CMz=cumsum(dFzeta.*eta); CMe=cumsum(dFeta.*zeta);
    M_dist=eta.*CFz - CMz - zeta.*CFe + CMe;
    if strcmpi(loadNode,'none')
        M_L=zeros(size(eta)); Fz0=0;
    else
        M_L=-(Pload/2).*eta; Fz0=-(Pload/2);
    end
    M_known=M_dist + M_L;
    f2=(h_r - zeta);
    A11=trapz(sigma,ones(size(sigma)));
    A12=trapz(sigma,f2);
    A22=trapz(sigma,f2.^2);
    b1=-trapz(sigma,M_known);
    b2=-trapz(sigma,M_known.*f2);
    sol=[A11 A12; A12 A22] \ [b1; b2];
    M0=sol(1); N0=sol(2);
    M_total=M0 + N0.*f2 + M_known;
    Feta_total=N0 + CFe;
    Fzeta_total=Fz0 + CFz;
    tEta=gradient(eta,sigma); tZeta=gradient(zeta,sigma);
    tN=hypot(tEta,tZeta); tEta=tEta./tN; tZeta=tZeta./tN;
    N_axial=Feta_total.*tEta + Fzeta_total.*tZeta;
    c_fib=r/2;
    sigma_bend=M_total*c_fib/I_r;
    sigma_axial=N_axial/A_r;
    sigma_comb=abs(sigma_bend) + abs(sigma_axial);
    res.r=r; res.b_r=b_r; res.h_r=h_r; res.A_r=A_r; res.I_r=I_r;
    res.sigma=sigma; res.M=M_total; res.N=N_axial;
    res.sigma_comb=sigma_comb; res.sigma_max=max(sigma_comb);
    res.M0=M0; res.N0=N0;
    res.eta=eta; res.zeta=zeta; res.qeta=qeta; res.qzeta=qzeta;
    res.Feta=Feta_total; res.Fzeta=Fzeta_total;
    res.sigma_bend=sigma_bend; res.sigma_axial=sigma_axial;
end
function plot_bulkhead_full(res,label,Pload,loadNode)
    if strcmpi(loadNode,'bottom')
        startZone='Bottom';
    else
        startZone='Top';
    end
    eta_R=res.eta; zeta_R=res.zeta; qeta_R=res.qeta; qzeta_R=res.qzeta;
    Feta_R=res.Feta; Fzeta_R=res.Fzeta; M_R=res.M; N_R=res.N;
    sigB_R=res.sigma_bend; sigA_R=res.sigma_axial; sigC_R=res.sigma_comb; s_R=res.sigma;
    eta_L=-fliplr(eta_R); zeta_L=fliplr(zeta_R);
    qeta_L=-fliplr(qeta_R); qzeta_L=fliplr(qzeta_R);
    Feta_L=-fliplr(Feta_R); Fzeta_L=fliplr(Fzeta_R);
    M_L=fliplr(M_R); N_L=fliplr(N_R);
    sigB_L=fliplr(sigB_R); sigA_L=fliplr(sigA_R); sigC_L=fliplr(sigC_R);
    s_L=s_R(end) + (s_R(end)-fliplr(s_R));
    eta_full=[eta_R,eta_L(2:end)]; zeta_full=[zeta_R,zeta_L(2:end)];
    qeta_full=[qeta_R,qeta_L(2:end)]; qzeta_full=[qzeta_R,qzeta_L(2:end)];
    Feta_full=[Feta_R,Feta_L(2:end)]; Fzeta_full=[Fzeta_R,Fzeta_L(2:end)];
    M_full=[M_R,M_L(2:end)]; N_full=[N_R,N_L(2:end)];
    sigB_full=[sigB_R,sigB_L(2:end)]; sigA_full=[sigA_R,sigA_L(2:end)];
    sigC_full=[sigC_R,sigC_L(2:end)]; s_full=[s_R,s_L(2:end)];
    figure('Name',[label ' - Cross-Section'],'Color','w');
    plot(eta_full,zeta_full,'k-','LineWidth',2); hold on; grid on; axis equal;
    if ~strcmpi(loadNode,'none')
        if strcmpi(loadNode,'top'),zL=res.h_r; else,zL=-res.h_r; end
        plot(0,zL,'k^','MarkerSize',8,'MarkerFaceColor','y');
        text(0,zL+sign(zL)*6,sprintf('P = %.3f N',Pload),'HorizontalAlignment','center','FontSize',20);
    end
    labelRingSides(gca,res.b_r,res.h_r);
    xlabel('\eta (mm)','FontSize',20); ylabel('\zeta (mm)','FontSize',20);
    title(sprintf('%s (r = %.3f mm)',label,res.r),'FontSize',20);
    styleAxes(gca); hold off;
    figure('Name',[label ' - Shear Flow q(s)'],'Color','w');
    plot(s_full,qzeta_full,'b--','LineWidth',1.5); hold on; grid on;
    plot(s_full,qeta_full,'r--','LineWidth',1.5);
    xlabel('Arc length s (mm)','FontSize',20); ylabel('Shear flow q (N/mm)','FontSize',20);
    title(['Applied Shear Flow - ' label],'FontSize',20);
    legend('q_{\zeta}','q_{\eta}','Location','best','FontSize',20,'Box','off');
    markRingStations(gca,res.b_r,res.h_r,startZone); styleAxes(gca); hold off;
    figure('Name',[label ' - 3D Shear Flow'],'Color','w');
    plot3(eta_full,zeta_full,qzeta_full,'b--','LineWidth',2); hold on; grid on;
    plot3(eta_full,zeta_full,qeta_full,'r--','LineWidth',2);
    labelRingSides(gca,res.b_r,res.h_r,0);
    xlabel('\eta (mm)','FontSize',20); ylabel('\zeta (mm)','FontSize',20); zlabel('q (N/mm)','FontSize',20);
    title(['3D Shear Flow - ' label],'FontSize',20);
    legend('q_{\zeta}','q_{\eta}','Location','best','FontSize',20,'Box','off');
    view(30,30); styleAxes(gca); hold off;
    figure('Name',[label ' - Section Shear Flow Map'],'Color','w');
    qmag_full=hypot(qeta_full,qzeta_full);
    scatter(eta_full,zeta_full,22,qmag_full,'filled'); hold on; grid on; axis equal;
    colormap(parula); cb=colorbar; cb.Label.String='|q| (N/mm)';
    plot(eta_full,zeta_full,'k-','LineWidth',0.5);
    labelRingSides(gca,res.b_r,res.h_r);
    xlabel('\eta (mm)','FontSize',20); ylabel('\zeta (mm)','FontSize',20);
    title(['Shear Flow Magnitude - ' label],'FontSize',20);
    styleAxes(gca); hold off;
    plot_ring_internal_combined([label ' - Internal Forces and Moment'],...
        ['Internal Forces and Bending Moment - ' label],...
        s_full,Feta_full,Fzeta_full,N_full,M_full,res.b_r,res.h_r,startZone);
    figure('Name',[label ' - Stress Distribution'],'Color','w');
    plot(s_full,sigB_full,'b--','LineWidth',1.4); hold on; grid on;
    plot(s_full,sigA_full,'r--','LineWidth',1.4);
    plot(s_full,sigC_full,'k--','LineWidth',1.8);
    xlabel('Arc length s (mm)','FontSize',20); ylabel('Stress (MPa)','FontSize',20);
    title(sprintf('Stress Distribution - %s (r=%.2f mm)',label,res.r),'FontSize',20);
    legend('\sigma_{bend}','\sigma_{axial}','\sigma_{comb}','Location','best','FontSize',20,'Box','off');
    markRingStations(gca,res.b_r,res.h_r,startZone); styleAxes(gca); hold off;
    fprintf('\n-- %s: governing internal loads/stresses \n',label);
    report_quantity('Bending moment, M(s)',M_full,s_full,res.b_r,res.h_r,'N.mm',startZone);
    report_quantity('Axial force, N(s)',N_full,s_full,res.b_r,res.h_r,'N',startZone);
    report_quantity('Horizontal shear, F_eta(s)',Feta_full,s_full,res.b_r,res.h_r,'N',startZone);
    report_quantity('Vertical shear, F_zeta(s)',Fzeta_full,s_full,res.b_r,res.h_r,'N',startZone);
    report_quantity('Bending stress, sigma_bend',sigB_full,s_full,res.b_r,res.h_r,'MPa',startZone);
    report_quantity('Axial stress, sigma_axial',sigA_full,s_full,res.b_r,res.h_r,'MPa',startZone);
    report_quantity('Combined stress, sigma_comb',sigC_full,s_full,res.b_r,res.h_r,'MPa',startZone);
    report_quantity('Shear flow, q_eta(s)',qeta_full,s_full,res.b_r,res.h_r,'N/mm',startZone);
    report_quantity('Shear flow, q_zeta(s)',qzeta_full,s_full,res.b_r,res.h_r,'N/mm',startZone);
end
function plot_ring_internal_combined(figName,titleStr,s_full,Feta_full,Fzeta_full,N_full,M_full,b_r,h_r,startZone)
    if nargin < 10,startZone='Top'; end
    colEta=[0 0.447 0.741];
    colZeta=[0.85 0.325 0.098];
    colN=[0.466 0.674 0.188];
    colM=[0.494 0.184 0.556];
    figure('Name',figName,'Color','w');
    ax=gca;
    yyaxis left
    yF=[Feta_full(:); Fzeta_full(:); N_full(:)];
    pad=0.05*(max(yF) - min(yF));
    ylim([min(yF) - pad,max(yF) + pad]);
    hSt=markRingStations(ax,b_r,h_r,startZone);
    hEta=plot(s_full,Feta_full,'-','Color',colEta,'LineWidth',1.6); grid on;
    hZeta=plot(s_full,Fzeta_full,'-','Color',colZeta,'LineWidth',1.6);
    hN=plot(s_full,N_full,'-','Color',colN,'LineWidth',1.6);
    ylabel('Internal force (N)','FontSize',20);
    yyaxis right
    hM=plot(s_full,M_full,'-','Color',colM,'LineWidth',1.6);
    ylabel('Bending moment M(s) (N.mm)','FontSize',20);
    ax.YAxis(1).Color=[0 0 0];
    ax.YAxis(2).Color=colM;
    xlabel('Arc length s (mm) - shaded bands mark location on the ring','FontSize',20);
    title(titleStr,'FontSize',20);
    legH=[hSt.dividers(1),hSt.ends(1),hSt.bands(1),hSt.bands(2),hSt.bands(3)];
    set(legH,'HandleVisibility','on');
    legend([hEta hZeta hN hM legH],...
        {'F_{\eta}(s): horizontal internal force (N)',...
         'F_{\zeta}(s): vertical internal force (N)',...
         'N(s): axial (tangential) force (N)',...
         'M(s): bending moment (N.mm), right axis',...
         'Zone boundaries (member junctions)',...
         'Ring start / end (s = 0 and s = s_{end})',...
         sprintf('%s member (shaded band)',hSt.zones{1}),...
         'Web members, left / right (shaded band)',...
         sprintf('%s member (shaded band)',hSt.zones{3})},...
        'Location','eastoutside','FontSize',20,'Box','off');
    styleAxes(ax);
    hold off;
end
function plot_bulkhead_ring_3d(xStation,r,width,heightBox,labelStr)
    yOuter=[-width/2 width/2 width/2 -width/2 -width/2];
    zOuter=[0 0 heightBox heightBox 0];
    xOuter=xStation*ones(size(yOuter));
    plot3(xOuter,yOuter,zOuter,'k-','LineWidth',2);
    yInner=[-(width/2-r) (width/2-r) (width/2-r) -(width/2-r) -(width/2-r)];
    zInner=[r r (heightBox-r) (heightBox-r) r];
    xInner=xStation*ones(size(yInner));
    plot3(xInner,yInner,zInner,'c-','LineWidth',2);
    text(xStation,0,heightBox+12,sprintf('%s\nr=%.2fmm',labelStr,r),...
        'HorizontalAlignment','center','FontSize',20,'FontWeight','bold');
end
function plot_plate_ring_3d(xStation,yHalf,zLow,zHigh,frameDepth,faceColor,labelStr)
    y1=-yHalf; y2=yHalf;
    z1=zLow; z2=zHigh;
    y1i=y1 + frameDepth; y2i=y2 - frameDepth;
    z1i=z1 + frameDepth; z2i=z2 - frameDepth;
    xc=xStation;
    patch(xc*ones(1,4),[y1 y2 y2 y1],[z2i z2i z2 z2],faceColor,...
          'FaceAlpha',0.55,'EdgeColor','none','HandleVisibility','off');
    patch(xc*ones(1,4),[y1 y2 y2 y1],[z1 z1 z1i z1i],faceColor,...
          'FaceAlpha',0.55,'EdgeColor','none','HandleVisibility','off');
    patch(xc*ones(1,4),[y1 y1i y1i y1],[z1i z1i z2i z2i],faceColor,...
          'FaceAlpha',0.55,'EdgeColor','none','HandleVisibility','off');
    patch(xc*ones(1,4),[y2i y2 y2 y2i],[z1i z1i z2i z2i],faceColor,...
          'FaceAlpha',0.55,'EdgeColor','none','HandleVisibility','off');
    yOuter=[y1 y2 y2 y1 y1];
    zOuter=[z1 z1 z2 z2 z1];
    plot3(xc*ones(size(yOuter)),yOuter,zOuter,'k-','LineWidth',2,...
          'HandleVisibility','off');
    yInner=[y1i y2i y2i y1i y1i];
    zInner=[z1i z1i z2i z2i z1i];
    plot3(xc*ones(size(yInner)),yInner,zInner,'b-','LineWidth',2,...
          'HandleVisibility','off');
end
function format_all_figures(fontSize)
    figs=findall(0,'Type','figure');
    for k=1:numel(figs)
        f=figs(k);
        set(f,'Units','normalized','OuterPosition',[0 0 1 1]);
        axs=findall(f,'Type','axes');
        set(axs,'TitleFontSizeMultiplier',1,'LabelFontSizeMultiplier',1);
        set(findall(f,'-property','FontSize'),'FontSize',fontSize);
        cbs=findall(f,'Type','colorbar');
        for c=1:numel(cbs)
            cbs(c).Label.FontSize=fontSize;
        end
    end
end
function save_all_figures(folder,names,formats,dpi)
    if ~exist(folder,'dir')
        mkdir(folder);
    end
    figs=findall(0,'Type','figure');
    [~,order]=sort([figs.Number]);
    figs=figs(order);
    if numel(figs) ~= numel(names)
        warning('save_all_figures:count',...
            '%d figures are open but %d file names are defined; the names are taken from the figure titles instead.',...
            numel(figs),numel(names));
        names=cell(1,numel(figs));
        for k=1:numel(figs)
            names{k}=sprintf('fig%02d_%s',k,regexprep(lower(figs(k).Name),'[^a-z0-9]+','_'));
        end
    end
    drawnow;
    fprintf('\n Saving figures \n');
    for k=1:numel(figs)
        f=figs(k);
        for j=1:numel(formats)
            fmt=lower(formats{j});
            target=fullfile(folder,[names{k} '.' fmt]);
            switch fmt
            case 'fig'
            savefig(f,target);
            case 'png'
            if exist('exportgraphics','file') == 2
            exportgraphics(f,target,'Resolution',dpi);
            else
            set(f,'PaperPositionMode','auto');
            print(f,target,'-dpng',sprintf('-r%d',dpi));
            end
            case 'pdf'
            if exist('exportgraphics','file') == 2
            exportgraphics(f,target,'ContentType','vector');
            else
            set(f,'PaperPositionMode','auto');
            print(f,target,'-dpdf');
            end
            end
        end
        fprintf('  Fig %2d -> %s\n',k,names{k});
    end
    [~,info]=fileattrib(folder);
    fprintf('Saved %d figures (%s) to: %s\n',numel(figs),strjoin(formats,', '),info.Name);
end
