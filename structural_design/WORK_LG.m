clear; clc; close all;
scriptDir=fileparts(mfilename('fullpath'));
if isempty(scriptDir),scriptDir=pwd; end
figDir=fullfile(scriptDir,'figures');
if ~exist(figDir,'dir')
    [ok,msg]=mkdir(figDir);
    if ~ok
        warning('Could not create "%s" (%s). Saving figures to the current folder instead.',figDir,msg);
        figDir=pwd;
    end
end
FS=20;
topWidth=97.5;
y_main_leg=212.3049;
axle_drop=300;
legAngle=57.3;
baseWidth=topWidth+2*axle_drop/tand(legAngle);
leg_drop=y_main_leg*tand(legAngle);
L_leg_total=y_main_leg/cosd(legAngle);
beamFullyFixed=false;
W_total=6.87991*9.81;
w_udl=W_total/topWidth;
P=1.393*W_total;
eps_=0.05;
H=eps_ * P;
M_app=10*P;
H_dir_A=-1; H_dir_B=+1;
M_dir_A=+1; M_dir_B=-1;
E=68.9e3;
nu=0.33;
G=E / (2*(1+nu));
sigma_allow=276/3;
t=1;
b_sel=26;
h_sel=12.7;
D_wheel=100;
R_wheel=D_wheel/2;
e_P=R_wheel;
e_H=R_wheel;
k_plate=4;
bh_vec=0:50:200;
bh_vec=bh_vec(bh_vec > 2*t);
loadScale=linspace(0,2,40);
nPerMember=60;
nShapePts=160;
fprintf('\n');
fprintf(' LANDING GEAR LEG FIXED FUSELAGE ATTACHMENT (C,D),\n');
fprintf(' OVERHANGING WHEEL LEGS (A,B FREE)\n');
fprintf('\n');
theta=deg2rad(legAngle);
dx=(baseWidth - topWidth)/2;
legLen=dx/cos(theta);
height=dx*tan(theta);
A=[0,0];
B=[baseWidth,0];
C=[dx,height];
D=[dx+topWidth,height];
fprintf('\n Geometry \n');
fprintf('Leg length up to axle = %.4f mm\n',legLen);
fprintf('Leg angle = %.2f deg\n',legAngle);
fprintf('Frame height = %.4f mm\n',height);
fprintf('Full leg: drop = %.2f mm, sideways run = %.2f mm, length = %.2f mm, part below the axle = %.2f mm\n',leg_drop,y_main_leg,L_leg_total,L_leg_total-legLen);
fprintf('Top beam length = %.4f mm (fuselage attachment width)\n',topWidth);
fprintf('Base width = %.4f mm\n',baseWidth);
fprintf('Fixed supports at C=(%.3f,%.3f) and D=(%.3f,%.3f)\n',C(1),C(2),D(1),D(2));
fprintf('Top beam C-D support: %s\n',ternary(beamFullyFixed,...
    sprintf('EVERY discretisation point fixed (%d stations)',nPerMember),'fixed at C and D only (fixed-fixed beam)'));
[stationXY,stationS,tangent,memberID,sNode]=discretizeFrame(A,C,D,B,nPerMember);
Ltotal=stationS(end);
FxA=H_dir_A*H; FyA=P; MzA=M_dir_A*M_app;
FxB=H_dir_B*H; FyB=P; MzB=M_dir_B*M_app;
idx1=(memberID==1);
XY1=stationXY(idx1,:); S1=stationS(idx1); T1=tangent(idx1,:);
loadList1=struct('pos',{A},'F',{[FxA,FyA]},'M',{MzA},'s',{0});
[M1,N1,V1]=internalForces(XY1,S1,T1,loadList1);
RCx_leg=-FxA;
RCy_leg=-FyA;
RCM_leg=-( MzA + (A(1)-C(1))*FyA - (A(2)-C(2))*FxA );
idx2=(memberID==2);
XY2=stationXY(idx2,:); S2=stationS(idx2); T2=tangent(idx2,:);
xi2=S2 - sNode(2);
if beamFullyFixed
    M2=zeros(size(xi2)); V2=zeros(size(xi2)); N2=zeros(size(xi2));
    RCy_beam=w_udl*topWidth/2; RDy_beam=w_udl*topWidth/2;
    RCM_beam=0; RDM_beam=0;
else
    M2=(w_udl/12)*(6*topWidth*xi2 - 6*xi2.^2 - topWidth^2);
    V2=w_udl*(topWidth/2 - xi2);
    N2=zeros(size(xi2));
    RCy_beam=w_udl*topWidth/2; RDy_beam=w_udl*topWidth/2;
    RCM_beam=-w_udl*topWidth^2/12; RDM_beam=-w_udl*topWidth^2/12;
end
idx3=(memberID==3);
XY3=stationXY(idx3,:); S3=stationS(idx3); T3=tangent(idx3,:);
local3=sNode(4) - S3;
Tcut3=-T3;
loadList3=struct('pos',{B},'F',{[FxB,FyB]},'M',{MzB},'s',{0});
[M3,N3raw,V3raw]=internalForces(XY3,local3,Tcut3,loadList3);
N3=-N3raw; V3=-V3raw;
RDx_leg=-FxB;
RDy_leg=-FyB;
RDM_leg=-( MzB + (B(1)-D(1))*FyB - (B(2)-D(2))*FxB );
RCx=RCx_leg; RCy=RCy_leg + RCy_beam; RCM=RCM_leg + RCM_beam;
RDx=RDx_leg; RDy=RDy_leg + RDy_beam; RDM=RDM_leg + RDM_beam;
fprintf('\n Fixed-support reactions \n');
fprintf('At C: Rx=%.4f N, Ry=%.4f N, M=%.4f N.mm  (leg: %.4f,%.4f,%.4f | beam: 0,%.4f,%.4f)\n',...
    RCx,RCy,RCM,RCx_leg,RCy_leg,RCM_leg,RCy_beam,RCM_beam);
fprintf('At D: Rx=%.4f N, Ry=%.4f N, M=%.4f N.mm  (leg: %.4f,%.4f,%.4f | beam: 0,%.4f,%.4f)\n',...
    RDx,RDy,RDM,RDx_leg,RDy_leg,RDM_leg,RDy_beam,RDM_beam);
fprintf('Leg A-C check: sumFx=%.3e, sumFy=%.3e\n',RCx_leg+FxA,RCy_leg+FyA);
fprintf('Leg D-B check: sumFx=%.3e, sumFy=%.3e\n',RDx_leg+FxB,RDy_leg+FyB);
fprintf('Beam C-D check: sumFy=%.3e (should be ~0)\n',RCy_beam+RDy_beam-w_udl*topWidth);
Mfinal=zeros(size(stationS)); Nfinal=Mfinal; Vfinal=Mfinal;
Mfinal(idx1)=M1; Nfinal(idx1)=N1; Vfinal(idx1)=V1;
Mfinal(idx2)=M2; Nfinal(idx2)=N2; Vfinal(idx2)=V2;
Mfinal(idx3)=M3; Nfinal(idx3)=N3; Vfinal(idx3)=V3;
[Mmax,idxM]=max(abs(Mfinal));
fprintf('\nMax |M| along frame = %.3f N.mm at s = %.2f mm (member %d)\n',...
         Mmax,stationS(idxM),memberID(idxM));
[Asel,Isel,J_sel]=sectionProps(b_sel,h_sel,t);
EI=E*Isel; EA=E*Asel;
T_nom=P*e_P + H*e_H;
Am_sel=(b_sel-t)*(h_sel-t);
tau_torsion_nom=T_nom/(2*Am_sel*t);
phi_nom=T_nom*legLen/(G*J_sel);
tau_station=zeros(size(stationS));
tau_station(idx1)=tau_torsion_nom;
tau_station(idx3)=tau_torsion_nom;
fprintf('\n Torsion (nominal load, selected section) \n');
fprintf('Torque T = P*e_P + H*e_H = %.3f N.mm\n',T_nom);
fprintf('Twist over one leg length = %.6f rad = %.5f deg\n',phi_nom,rad2deg(phi_nom));
fprintf('Torsional shear stress tau  = %.4f MPa\n',tau_torsion_nom);
[MvAx,NvAx]=cantileverVirtual(XY1,S1,T1,A,[1,0,0]);
[MvAy,NvAy]=cantileverVirtual(XY1,S1,T1,A,[0,1,0]);
[MvAt,NvAt]=cantileverVirtual(XY1,S1,T1,A,[0,0,1]);
uxA=trapz(S1,M1.*MvAx)/EI + trapz(S1,N1.*NvAx)/EA;
uyA=trapz(S1,M1.*MvAy)/EI + trapz(S1,N1.*NvAy)/EA;
thetaA=trapz(S1,M1.*MvAt)/EI + trapz(S1,N1.*NvAt)/EA;
[MvBx,NvBx]=cantileverVirtual(XY3,local3,Tcut3,B,[1,0,0]);
[MvBy,NvBy]=cantileverVirtual(XY3,local3,Tcut3,B,[0,1,0]);
[MvBt,NvBt]=cantileverVirtual(XY3,local3,Tcut3,B,[0,0,1]);
[sortS3,ord3]=sort(local3);
uxB=trapz(sortS3,M3(ord3).*MvBx(ord3))/EI + trapz(sortS3,N3raw(ord3).*NvBx(ord3))/EA;
uyB=trapz(sortS3,M3(ord3).*MvBy(ord3))/EI + trapz(sortS3,N3raw(ord3).*NvBy(ord3))/EA;
thetaB=trapz(sortS3,M3(ord3).*MvBt(ord3))/EI + trapz(sortS3,N3raw(ord3).*NvBt(ord3))/EA;
dzA=-uyA; dyA=-uxA;
dzB=-uyB; dyB=+uxB;
shapeIdx1=round(linspace(1,numel(S1),nShapePts));
ux1=zeros(size(shapeIdx1)); uy1=ux1;
for k=1:numel(shapeIdx1)
    ii=shapeIdx1(k); qP=XY1(ii,:); qS=S1(ii);
    [Mvx,Nvx]=cantileverVirtual(XY1,S1,T1,qP,[1,0,0],qS);
    [Mvy,Nvy]=cantileverVirtual(XY1,S1,T1,qP,[0,1,0],qS);
    ux1(k)=trapz(S1,M1.*Mvx)/EI + trapz(S1,N1.*Nvx)/EA;
    uy1(k)=trapz(S1,M1.*Mvy)/EI + trapz(S1,N1.*Nvy)/EA;
end
shapeIdx3=round(linspace(1,numel(local3),nShapePts));
ux3=zeros(size(shapeIdx3)); uy3=ux3;
for k=1:numel(shapeIdx3)
    ii=shapeIdx3(k); qP=XY3(ii,:); qS=local3(ii);
    [Mvx,Nvx]=cantileverVirtual(XY3,local3,Tcut3,qP,[1,0,0],qS);
    [Mvy,Nvy]=cantileverVirtual(XY3,local3,Tcut3,qP,[0,1,0],qS);
    ux3k=trapz(sortS3,M3(ord3).*Mvx(ord3))/EI + trapz(sortS3,N3raw(ord3).*Nvx(ord3))/EA;
    uy3k=trapz(sortS3,M3(ord3).*Mvy(ord3))/EI + trapz(sortS3,N3raw(ord3).*Nvy(ord3))/EA;
    ux3(k)=ux3k; uy3(k)=uy3k;
end
xi2s=linspace(0,topWidth,nShapePts);
v2s=(w_udl.*xi2s.^2.*(topWidth-xi2s).^2)/(24*EI);
if beamFullyFixed,v2s=zeros(size(xi2s)); end
shapeX=[XY1(shapeIdx1,1); dx+xi2s'; XY3(shapeIdx3,1)];
shapeY_undef=[XY1(shapeIdx1,2); height*ones(numel(xi2s),1); XY3(shapeIdx3,2)];
ux_all=[ux1'; zeros(numel(xi2s),1); ux3'];
uy_all=[uy1'; -v2s'; uy3'];
magFactor=0.3*max(baseWidth,height) / max(1e-9,max(hypot(ux_all,uy_all)));
magList=[1,magFactor];
fig1Name={'Frame Shape (true scale)','Frame Shape (scaled up)'};
fig1File={'fig1a_deformed_shape_true_scale_overhang.png','fig1b_deformed_shape_scaled_overhang.png'};
fig1Head={'True-Scale Shape (x1)','Scaled-Up Shape'};
for km=1:2
    mf=magList(km);
    fig1=newFig(fig1Name{km},1300,850);
    plot([A(1) C(1) D(1) B(1)],[A(2) C(2) D(2) B(2)],'b-o','LineWidth',1.8,...
         'MarkerSize',9,'MarkerFaceColor','b','DisplayName','Undeformed'); hold on;
    plot([XY1(shapeIdx1,1); XY3(shapeIdx3,1)],[XY1(shapeIdx1,2); XY3(shapeIdx3,2)],...
         'bo','MarkerSize',7,'MarkerFaceColor','b','HandleVisibility','off');
    plot(shapeX+mf*ux_all,shapeY_undef+mf*uy_all,'r-s','LineWidth',1.6,...
         'MarkerSize',6,'DisplayName',sprintf('Deformed (x%.1f)',mf));
    if beamFullyFixed
        plot(XY2(:,1),XY2(:,2),'k^','MarkerFaceColor','k','MarkerSize',6,...
            'DisplayName','Fixed nodes (every point of C-D)');
    end
    legDir=(A-C)/norm(A-C);
    legEnd=A+legDir*(L_leg_total-legLen);
    plot([A(1) legEnd(1)],[A(2) legEnd(2)],'b--','LineWidth',1.8,'DisplayName','Leg below the axle (no load)');
    plot([B(1) B(1)-legEnd(1)+A(1)],[B(2) legEnd(2)],'b--','LineWidth',1.8,'HandleVisibility','off');
    plot(C(1),C(2),'k^','MarkerFaceColor','k','MarkerSize',10,'HandleVisibility','off');
    plot(D(1),D(2),'k^','MarkerFaceColor','k','MarkerSize',10,'HandleVisibility','off');
    text(C(1),C(2)+18,'FIXED (C)','HorizontalAlignment','right');
    text(D(1),D(2)+18,'FIXED (D)','HorizontalAlignment','left');
    axis equal; grid on; xlabel('x (mm)'); ylabel('y (mm)');
    xl=xlim; yl=ylim;
    padx=max(0.1*diff(xl),0.25*diff(yl));
    xlim([xl(1)-padx,xl(2)+padx]);
    ylim([yl(1),yl(2)+0.1*diff(yl)]);
    ttl={['Fixed Fuselage Attachment: ' fig1Head{km}],...
           sprintf('(b = %g mm, h = %g mm, t = %g mm)',b_sel,h_sel,t)};
    if km == 1
        ttl{end+1}=sprintf('Tip A and B: \\delta_z = %.4f mm (down +),  \\delta_y = %.4f mm (outward +)',dzA,dyA);
    end
    title(ttl);
    legend('Location','best');
    savefig_safe(fig1,figDir,fig1File{km},FS);
end
fig2=newFig('Internal Force Diagrams (M, N, V)',1400,850);
yyaxis left
hM=plot(stationS,Mfinal,'b-','LineWidth',2.4,...
          'DisplayName','M(s): bending moment [N.mm]  (left axis)');
hold on; grid on;
ylabel('Bending moment  M  [N.mm]');
yyaxis right
hN=plot(stationS,Nfinal,'r-','LineWidth',2.4,...
          'DisplayName','N(s): axial force [N]  (right axis)');
hV=plot(stationS,Vfinal,'m--','LineWidth',2.4,...
          'DisplayName','V(s): shear force [N]  (right axis)');
ylabel('Axial force  N,  Shear force  V  [N]');
ax2=gca;
ax2.YAxis(1).Color=[0 0 1];
ax2.YAxis(2).Color=[0 0 0];
yyaxis left
vline(sNode(2)); vline(sNode(3));
ylL=ylim;
nodeLbl={'A','C','D','B'}; nodeAlign={'left','center','center','right'};
for kn=1:4
    text(sNode(kn),ylL(2),nodeLbl{kn},'HorizontalAlignment',nodeAlign{kn},...
         'VerticalAlignment','top','FontWeight','bold');
end
xlabel('Arc length s (mm), A -> C -> D -> B');
title('Internal Force Diagrams: M, N and V');
legend([hM hN hV],'Location','best');
savefig_safe(fig2,figDir,'fig2_internal_forces_overhang.png',FS);
nb=numel(bh_vec); nh=numel(bh_vec);
SigMax=nan(nh,nb);
for ib=1:nb
    for ih=1:nh
        bb=bh_vec(ib); hh=bh_vec(ih);
        if bb <= 2*t || hh <= 2*t,continue; end
        [Ag,Ig,Jg]=sectionProps(bb,hh,t);
        Amg=(bb-t)*(hh-t);
        sigma_s=abs(Mfinal)*(hh/2)/Ig + abs(Nfinal)/Ag;
        tau_torsion_s=zeros(size(stationS));
        tau_torsion_s(idx1)=T_nom/(2*Amg*t);
        tau_torsion_s(idx3)=T_nom/(2*Amg*t);
        tau_V_s=abs(Vfinal)/(2*t*hh);
        tau_s=tau_torsion_s + tau_V_s;
        sigma_vm=sqrt(sigma_s.^2 + 3*tau_s.^2);
        SigMax(ih,ib)=max(sigma_vm);
    end
end
sigma_sel_station=abs(Mfinal)*(h_sel/2)/Isel + abs(Nfinal)/Asel;
tau_V_sel_station=abs(Vfinal)/(2*t*h_sel);
tau_sel_station=tau_station + tau_V_sel_station;
sigma_vm_sel=sqrt(sigma_sel_station.^2 + 3*tau_sel_station.^2);
sigSel=max(sigma_vm_sel);
fprintf('\n Selected Section (b=%g, h=%g, t=%g mm) \n',b_sel,h_sel,t);
fprintf('Area = %.3f mm^2, Iz = %.3f mm^4, J = %.3f mm^4\n',Asel,Isel,J_sel);
fprintf('Max VON MISES combined stress (bending+axial+torsion+shear) = %.4f MPa   (allowable = %.3f MPa)  to %s\n',...
    sigSel,sigma_allow,ternary(sigSel<=sigma_allow,'OK','FAILS'));
nFine=200;
bh_fine=linspace(2*t+0.5,max(bh_vec),nFine);
[Bf,Hf]=meshgrid(bh_fine,bh_fine);
SigMaxFine=nan(nFine,nFine);
SigBendAxialFine=nan(nFine,nFine);
for ib=1:nFine
    for ih=1:nFine
        bb=bh_fine(ib); hh=bh_fine(ih);
        [Ag,Ig,~]=sectionProps(bb,hh,t);
        Amg=(bb-t)*(hh-t);
        sigma_s=abs(Mfinal)*(hh/2)/Ig + abs(Nfinal)/Ag;
        tau_torsion_s=zeros(size(stationS));
        tau_torsion_s(idx1)=T_nom/(2*Amg*t);
        tau_torsion_s(idx3)=T_nom/(2*Amg*t);
        tau_V_s=abs(Vfinal)/(2*t*hh);
        tau_s=tau_torsion_s + tau_V_s;
        SigMaxFine(ih,ib)=max(sqrt(sigma_s.^2+3*tau_s.^2));
        SigBendAxialFine(ih,ib)=max(sigma_s);
    end
end
lvl=sigma_allow*[1/16 1/8 1/4 1/2 1 2 4 8 16];
Zlog=log10(SigMaxFine);
Zlog=min(max(Zlog,log10(lvl(1))+1e-6),log10(lvl(end))-1e-6);
fig4=newFig('b vs h Stress Contour (von Mises)',1300,850);
[~,hF]=contourf(Bf,Hf,Zlog,log10(lvl),'LineColor',[0.25 0.25 0.25],'LineWidth',0.8); hold on;
hF.Annotation.LegendInformation.IconDisplayStyle='off';
try,cmapV=turbo(256); catch,cmapV=jet(256); end
colormap(gca,cmapV);
caxis([log10(lvl(1)) log10(lvl(end))]);
cb=colorbar;
cb.Ticks=log10(lvl);
cb.TickLabels=arrayfun(@(v) sprintf('%.3g',v),lvl,'UniformOutput',false);
cb.Label.String='\sigma_{vM,max}  (MPa)';
[~,hC]=contour(Bf,Hf,SigMaxFine,[sigma_allow sigma_allow],'r-','LineWidth',3.5);
hC.DisplayName=sprintf('\\sigma_{vM} = \\sigma_{allow} = %.2f MPa',sigma_allow);
hSel=plot(b_sel,h_sel,'ko','MarkerFaceColor',ternary(sigSel<=sigma_allow,[1 1 0],[1 0 0]),...
            'MarkerSize',12,'LineWidth',1.5);
hSel.DisplayName=sprintf('Selected (b=%g, h=%g)',b_sel,h_sel);
set(gca,'YDir','normal','Layer','top'); grid on;
xlabel('b (mm)'); ylabel('h (mm)');
xlim([min(bh_fine) max(bh_fine)]); ylim([min(bh_fine) max(bh_fine)]);
title({'Max von Mises Stress \sigma_{vM,max}(b,h)',...
       sprintf('(bending + axial + torsion, t = %g mm)',t)});
legend('Location','northeast');
savefig_safe(fig4,figDir,'fig4_design_boundary_overhang.png',FS);
P_curve=loadScale * P;
dz_curve=loadScale * dzA;
dy_curve=loadScale * dyA;
fig5=newFig('Load vs Tip Deflection',1300,850);
plot(P_curve,dz_curve,'b-o','LineWidth',2,'MarkerSize',5,...
     'DisplayName','Vertical \delta_z (downward +)'); hold on;
plot(P_curve,dy_curve,'r-s','LineWidth',2,'MarkerSize',5,...
     'DisplayName','Lateral \delta_y (outward +)');
plot(P,dzA,'kp','MarkerFaceColor','y','MarkerSize',15,'HandleVisibility','off');
plot(P,dyA,'kp','MarkerFaceColor','y','MarkerSize',15,'HandleVisibility','off');
grid on; xlabel('Wheel vertical load  P  (N)'); ylabel('Wheel tip deflection (mm)');
title({'Load vs Wheel Tip Deflection','(cantilever overhang)'});
legend('Location','best');
savefig_safe(fig5,figDir,'fig5_load_deflection_overhang.png',FS);
T_curve=loadScale*T_nom;
phi_curve=loadScale*phi_nom;
fig6=newFig('Wheel Spanwise Rotation (Torsion)',1300,850);
plot(P_curve,rad2deg(phi_curve),'g-^','LineWidth',2,'MarkerSize',5); hold on;
plot(P,rad2deg(phi_nom),'kp','MarkerFaceColor','y','MarkerSize',15);
grid on; xlabel('Wheel vertical load  P  (N)'); ylabel('Wheel spanwise rotation \phi (deg)');
title({'Wheel Torsional Rotation vs Load',...
       sprintf('(T = P e_P + H e_H, b = %g mm, h = %g mm)',b_sel,h_sel)});
savefig_safe(fig6,figDir,'fig6_torsion_rotation_overhang.png',FS);
phi_station=zeros(size(stationS));
phi_station(idx1)=phi_nom*(sNode(2)-stationS(idx1))/legLen;
phi_station(idx3)=phi_nom*(stationS(idx3)-sNode(3))/legLen;
phi_deg_station=rad2deg(phi_station);
perim_sel=2*((b_sel-t)+(h_sel-t));
q_sel=T_nom/(2*Am_sel);
tau_allow=sigma_allow/sqrt(3);
MS_tors=tau_allow/tau_torsion_nom - 1;
sigNoTors=max(sigma_sel_station);
fig7=newFig('Torsion Along the Frame',1400,850);
yyaxis left
hPhi=plot(stationS,phi_deg_station,'b-','LineWidth',2.4,...
            'DisplayName','\phi(s): twist angle [deg]  (left axis)');
hold on; grid on;
ylabel('Twist angle  \phi  [deg]');
yyaxis right
hTau=plot(stationS,tau_station,'r--','LineWidth',2.4,...
            'DisplayName','\tau(s): torsional shear stress [MPa]  (right axis)');
ylabel('Torsional shear stress  \tau  [MPa]');
ax7=gca;
ax7.YAxis(1).Color=[0 0 1];
ax7.YAxis(2).Color=[1 0 0];
yyaxis right; ylim([0,max(1.25*max(tau_station),1e-6)]);
yyaxis left; ylim([0,max(1.25*max(phi_deg_station),1e-6)]);
plot([sNode(1) sNode(4)],max(phi_deg_station)*[1 1],'kp',...
     'MarkerFaceColor','y','MarkerSize',15,'HandleVisibility','off');
vline(sNode(2)); vline(sNode(3));
ylL=ylim;
nodeLbl={'A','C','D','B'}; nodeAlign={'left','center','center','right'};
for kn=1:4
    text(sNode(kn),ylL(2),nodeLbl{kn},'HorizontalAlignment',nodeAlign{kn},...
         'VerticalAlignment','top','FontWeight','bold');
end
text(sNode(1),max(phi_deg_station),sprintf('  \\phi_{max} = %.3f deg',max(phi_deg_station)),...
     'HorizontalAlignment','left','VerticalAlignment','bottom');
text(sNode(4),max(phi_deg_station),sprintf('\\phi_{max} = %.3f deg  ',max(phi_deg_station)),...
     'HorizontalAlignment','right','VerticalAlignment','bottom');
xlabel('Arc length s (mm), A -> C -> D -> B');
title({'Torsion along the Frame',...
       sprintf('(T = %.1f N.mm per wheel, b = %g mm, h = %g mm)',T_nom,b_sel,h_sel)});
legend([hPhi hTau],'Location','northoutside');
savefig_safe(fig7,figDir,'fig7_torsion_along_frame_overhang.png',FS);
Iy_sel=(h_sel*b_sel^3 - (h_sel-2*t)*(b_sel-2*t)^3)/12;
I_min=min(Isel,Iy_sel);
K_eff=2;
Nleg_max=max([abs(N1); abs(N3raw)]);
Pcr_leg=pi^2*E*I_min/(K_eff*legLen)^2;
MS_buck_euler=Pcr_leg/max(Nleg_max,eps) - 1;
fprintf('\n Euler (column) buckling of the leg \n');
fprintf('I_min (weak axis) = %.4f mm^4, K = %.1f, L = %.3f mm\n',I_min,K_eff,legLen);
fprintf('N_leg,max = %.4f N,  P_cr = %.2f N  -> MS = %.4f (%s)\n',...
    Nleg_max,Pcr_leg,MS_buck_euler,ternary(MS_buck_euler>=0,'OK','FAILS'));
w_b_sel=b_sel - 2*t;
w_h_sel=h_sel - 2*t;
w_gov=max(w_b_sel,w_h_sel);
sigma_cr_local=k_plate*pi^2*E/(12*(1-nu^2)) * (t/w_gov)^2;
sigma_compress_max=max(sigma_sel_station);
MS_buck_local=sigma_cr_local/sigma_compress_max - 1;
fprintf('\n Local (flat-panel) buckling of the tube walls \n');
fprintf('Governing flat width w = %.4f mm (k = %g, simply-supported panel)\n',w_gov,k_plate);
fprintf('sigma_cr,local = %.4f MPa,  applied (bending+axial) = %.4f MPa  -> MS = %.4f (%s)\n',...
    sigma_cr_local,sigma_compress_max,MS_buck_local,ternary(MS_buck_local>=0,'OK','FAILS'));
w_gov_fine=max(Bf-2*t,Hf-2*t);
sigma_cr_fine=k_plate*pi^2*E/(12*(1-nu^2)) * (t./w_gov_fine).^2;
MSbuck_fine=sigma_cr_fine./max(SigBendAxialFine,eps) - 1;
fig8=newFig('Local Buckling Margin of Safety (b vs h)',1300,850);
feasMask=double(MSbuck_fine >= 0);
imagesc(bh_fine,bh_fine,feasMask); hold on;
set(gca,'YDir','normal');
colormap(gca,[1 0.85 0.85; 0.75 1 0.75]);
caxis([0 1]);
[~,hC8]=contour(Bf,Hf,MSbuck_fine,[0 0],'r-','LineWidth',3.5);
hC8.DisplayName='MS_{buckling} = 0 boundary';
hSel8=plot(b_sel,h_sel,'ko','MarkerFaceColor',ternary(MS_buck_local>=0,[1 1 0],[1 0 0]),...
            'MarkerSize',12,'LineWidth',1.5);
hSel8.DisplayName=sprintf('Selected (b=%g, h=%g) -> MS=%.3f',b_sel,h_sel,MS_buck_local);
set(gca,'Layer','top'); grid on; axis xy;
xlabel('b (mm)'); ylabel('h (mm)');
xlim([min(bh_fine) max(bh_fine)]); ylim([min(bh_fine) max(bh_fine)]);
title({'Local (Flat-Panel) Buckling: Feasible Region',...
       sprintf('(t = %g mm, k = %g; green = MS_{buckling} \\geq 0)',t,k_plate)});
legend([hC8 hSel8],'Location','best');
savefig_safe(fig8,figDir,'fig8_local_buckling_overhang.png',FS);
fprintf('\n Margin of Safety \n');
MS_vm=sigma_allow/sigSel - 1;
fprintf('Combined (von Mises) MS = %.4f  (%s)\n',MS_vm,ternary(MS_vm>=0,'POSITIVE - OK','NEGATIVE - FAILS'));
fprintf('\n\n');
fprintf('\n');
fprintf(' RESULTS SUMMARY\n');
fprintf('\n');
fprintf('\n[1] GEOMETRY (fixed at C, D; free/overhanging at A, B)\n');
fprintf('Node A (mm) = (%.4f, %.4f)  [FREE, wheel tip]\n',A(1),A(2));
fprintf('Node C (mm) = (%.4f, %.4f)  [FIXED]\n',C(1),C(2));
fprintf('Node D (mm) = (%.4f, %.4f)  [FIXED]\n',D(1),D(2));
fprintf('Node B (mm) = (%.4f, %.4f)  [FREE, wheel tip]\n',B(1),B(2));
fprintf('Leg length (overhang, A-C = D-B) = %.4f mm\n',legLen);
fprintf('Top beam (fixed-fixed) length = %.4f mm\n',topWidth);
fprintf('\n[2] APPLIED LOADS\n');
fprintf('UDL on top beam   w  = %.5f N/mm  (total = %.4f N)\n',w_udl,w_udl*topWidth);
fprintf('Wheel vertical load  P = %.4f N (upward) at BOTH A and B\n',P);
fprintf('H = eps*P = %.4f N,  M_app = %.4f N.mm\n',H,M_app);
fprintf('\n[3] MATERIAL\n');
fprintf('E=%.2f MPa, nu=%.3f, G=%.2f MPa, sigma_allow=%.4f MPa\n',E,nu,G,sigma_allow);
fprintf('\n[4] STATICS -- LEG A-C (determinate cantilever, fixed at C)\n');
fprintf('Tip load at A: Fx=%.4f N, Fy=%.4f N, M=%.4f N.mm\n',FxA,FyA,MzA);
fprintf('Reaction at C (leg only): Rx=%.4f N, Ry=%.4f N, M=%.4f N.mm\n',RCx_leg,RCy_leg,RCM_leg);
fprintf('\n[5] STATICS, TOP BEAM C-D (fixed-fixed, closed form)\n');
fprintf('M(x) = (w/12)*(6*L*x - 6*x^2 - L^2),  V(x) = w*(L/2 - x)\n');
fprintf('M(0)=M(L) = %.4f N.mm (hogging),  M(L/2) = %.4f N.mm (sagging)\n',...
    -w_udl*topWidth^2/12,w_udl*topWidth^2/24);
fprintf('Reactions: Ry = %.4f N each end, End moment = %.4f N.mm each end\n',RCy_beam,RCM_beam);
fprintf('\n[6] STATICS, LEG D-B (mirror of leg A-C)\n');
fprintf('Tip load at B: Fx=%.4f N, Fy=%.4f N, M=%.4f N.mm\n',FxB,FyB,MzB);
fprintf('Reaction at D (leg only): Rx=%.4f N, Ry=%.4f N, M=%.4f N.mm\n',RDx_leg,RDy_leg,RDM_leg);
fprintf('\n[7] TOTAL FUSELAGE ATTACHMENT REACTIONS\n');
fprintf('At C: Rx=%.4f N, Ry=%.4f N, M=%.4f N.mm\n',RCx,RCy,RCM);
fprintf('At D: Rx=%.4f N, Ry=%.4f N, M=%.4f N.mm\n',RDx,RDy,RDM);
fprintf('\n[8] GOVERNING INTERNAL FORCES\n');
fprintf('Max |M| = %.4f N.mm at s=%.3f mm (member %d)\n',Mmax,stationS(idxM),memberID(idxM));
fprintf('M at A (tip) = %.4f N.mm   M at B (tip) = %.4f N.mm\n',M1(1),M3(end));
fprintf('\n[9] SELECTED SECTION (b=%g, h=%g, t=%g mm)\n',b_sel,h_sel,t);
fprintf('Area=%.4f mm^2, Iz=%.4f mm^4, J=%.4f mm^4\n',Asel,Isel,J_sel);
fprintf('\n[10] COMBINED STRESS CHECK (von Mises: bending+axial+torsion+shear)\n');
fprintf('sigma_vm = sqrt( (|M|c/I + |N|/A)^2 + 3*(tau_torsion+tau_V)^2 )\n');
fprintf('tau_torsion (legs only) = T/(2*Am*t) = %.4f MPa;  tau_V,max (approx, everywhere) = %.4f MPa\n',...
    tau_torsion_nom,max(tau_V_sel_station));
fprintf('NOTE: tau_V = |V|/(2*t*h) treats the two side walls as the shear-carrying webs of the\n');
fprintf('box section, an engineering approximation, not an exact closed-section shear-flow result.\n');
fprintf('Torsion and shear are added directly (worst case, same wall) before the von Mises combination.\n');
fprintf('sigma_vm_max = %.4f MPa,  sigma_allow = %.4f MPa\n',sigSel,sigma_allow);
fprintf('Margin of Safety = %.4f  (%s)\n',MS_vm,ternary(MS_vm>=0,'POSITIVE - OK','NEGATIVE - FAILS'));
fprintf('Euler (column) buckling of leg:  N_leg,max = %.4f N,  P_cr = %.2f N  -> MS = %.4f (%s)\n',...
    Nleg_max,Pcr_leg,MS_buck_euler,ternary(MS_buck_euler>=0,'OK','FAILS'));
fprintf('Local (flat-panel) buckling:  sigma_cr,local = %.4f MPa (w = %.3f mm, k = %g)  -> MS = %.4f (%s)\n',...
    sigma_cr_local,w_gov,k_plate,MS_buck_local,ternary(MS_buck_local>=0,'OK','FAILS'));
fprintf('NOTE: the local-buckling applied stress uses the bending+axial MAGNITUDE as a conservative\n');
fprintf('proxy for the compressive stress; it does not track which side of the section is in compression.\n');
fprintf('\n[11] WHEEL TIP DEFLECTIONS (unit-load method, nominal load)\n');
printTipTable(dzA,dyA,thetaA,dzB,dyB,thetaB);
fprintf('\n[12] TORSION\n');
fprintf('T = %.4f N.mm,  phi = %.6e rad = %.5f deg,  tau = %.4f MPa\n',T_nom,phi_nom,rad2deg(phi_nom),tau_torsion_nom);
fprintf('Torque build-up: P*e_P = %.4f N.mm (e_P = %.3f mm),  H*e_H = %.4f N.mm (e_H = %.3f mm),  T = %.4f N.mm\n',...
    P*e_P,e_P,H*e_H,e_H,T_nom);
fprintf('Section (thin-walled closed): Am = %.4f mm^2, mid-line perimeter = %.4f mm, J = %.4f mm^4\n',...
    Am_sel,perim_sel,J_sel);
fprintf('Torsional rigidity G*J = %.6e N.mm^2,  stiffness G*J/L = %.4f N.mm/rad,  twist rate T/(G*J) = %.6e rad/mm\n',...
    G*J_sel,G*J_sel/legLen,T_nom/(G*J_sel));
fprintf('Shear flow q = T/(2*Am) = %.4f N/mm,  tau = q/t = %.4f MPa\n',q_sel,tau_torsion_nom);
fprintf('tau_allow = sigma_allow/sqrt(3) = %.4f MPa,  pure-torsion Margin of Safety = %.4f\n',tau_allow,MS_tors);
fprintf('von Mises max WITH torsion = %.4f MPa,  WITHOUT torsion (bending+axial only) = %.4f MPa\n',sigSel,sigNoTors);
fprintf('Torsion raises the von Mises stress by %.4f MPa (%.3f %%)\n',sigSel-sigNoTors,100*(sigSel-sigNoTors)/sigNoTors);
fprintf('Torsion distribution along the frame (T constant on legs A-C and D-B, zero on top beam C-D; phi = 0 at fixed ends C, D):\n');
printTorsionTable(sNode,T_nom,tau_torsion_nom,phi_nom);
fprintf('\n[13] b-h SWEEP (von Mises, incl. shear)\n');
for ib=1:numel(bh_vec)
    for ih=1:numel(bh_vec)
        fprintf('  b=%-5g h=%-5g -> sigma_vm_max = %8.4f MPa  (%s)\n',...
            bh_vec(ib),bh_vec(ih),SigMax(ih,ib),ternary(SigMax(ih,ib)<=sigma_allow,'OK','FAILS'));
    end
end
fprintf('\n\n');
fprintf(' END OF RESULTS SUMMARY\n');
fprintf('\n');
function [stationXY,stationS,tangent,memberID,sNode]=discretizeFrame(A,C,D,B,nPerMember)
    segs={A,C; C,D; D,B};
    stationXY=[]; stationS=[]; tangent=[]; memberID=[];
    sNode=zeros(1,4);
    sCum=0;
    for m=1:3
        P1=segs{m,1}; P2=segs{m,2};
        Lm=norm(P2-P1);
        et=(P2-P1)/Lm;
        xi=linspace(0,Lm,nPerMember)';
        xy=P1 + xi*et;
        s=sCum + xi;
        stationXY=[stationXY; xy];
        stationS=[stationS ; s];
        tangent=[tangent ; repmat(et,nPerMember,1)];
        memberID=[memberID ; m*ones(nPerMember,1)];
        sCum=sCum + Lm;
        sNode(m+1)=sCum;
    end
end
function [Ms,Ns,Vs]=internalForces(stationXY,stationS,tangent,loadList)
    n=numel(stationS);
    Ms=zeros(n,1); Ns=zeros(n,1); Vs=zeros(n,1);
    Lpos=[loadList.s];
    Fx=arrayfun(@(q) q.F(1),loadList);
    Fy=arrayfun(@(q) q.F(2),loadList);
    Mz=[loadList.M];
    Lx=arrayfun(@(q) q.pos(1),loadList);
    Ly=arrayfun(@(q) q.pos(2),loadList);
    for i=1:n
        xc=stationXY(i,1); yc=stationXY(i,2); sc=stationS(i);
        mask=Lpos <= sc + 1e-6;
        Rx=sum(Fx(mask)); Ry=sum(Fy(mask));
        Mc=sum(Mz(mask)) + sum((Lx(mask)-xc).*Fy(mask) - (Ly(mask)-yc).*Fx(mask));
        et=tangent(i,:); en=[-et(2),et(1)];
        Ns(i)=Rx*et(1) + Ry*et(2);
        Vs(i)=Rx*en(1) + Ry*en(2);
        Ms(i)=-Mc;
    end
end
function [Area,Iz,J]=sectionProps(b,h,t)
    Area=b*h - (b-2*t)*(h-2*t);
    Iz=(b*h^3 - (b-2*t)*(h-2*t)^3)/12;
    bm=b-t; hm=h-t;
    Am=bm*hm;
    perim_mid=2*(bm+hm);
    J=4*Am^2*t/perim_mid;
end
function [Mv,Nv]=cantileverVirtual(stationXY,stationS,tangent,qPos,Fvec,qS)
    if nargin < 6,qS=0; end
    loadListV=struct('pos',{qPos},'F',{[Fvec(1),Fvec(2)]},'M',{Fvec(3)},'s',{qS});
    [Mv,Nv,~]=internalForces(stationXY,stationS,tangent,loadListV);
end
function out=ternary(cond,a,b)
    if cond,out=a; else,out=b; end
end
function vline(xval)
    yl=ylim; hold on;
    plot([xval xval],yl,'k--','HandleVisibility','off');
    ylim(yl);
end
function printTipTable(dzA,dyA,thA,dzB,dyB,thB)
    fprintf('Axes: z positive DOWNWARD, y positive OUTWARD (spanwise, away from centre-line)\n');
    fprintf('(dz = -u_y ; dy = -u_x at node A, +u_x at node B; u = frame-plane unit-load result)\n');
    fprintf('theta = tip-section rotation in the frame plane (counter-clockwise positive)\n');
    hdr=[' ',repmat(' ',1,4),'\n'];
    fprintf(hdr);
    fprintf('| %-4s | %16s | %16s | %16s | %16s |\n','Node','dz [mm]','dy [mm]','|d| [mm]','theta [deg]');
    fprintf(hdr);
    fprintf('| %-4s | %16.5f | %16.5f | %16.5f | %16.5f |\n','A',dzA,dyA,hypot(dzA,dyA),rad2deg(thA));
    fprintf('| %-4s | %16.5f | %16.5f | %16.5f | %16.5f |\n','B',dzB,dyB,hypot(dzB,dyB),rad2deg(thB));
    fprintf(hdr);
end
function printTorsionTable(sN,T,tauv,phiN)
    hdr=[' ',repmat(' ',1,4),'\n'];
    names={'A','C','D','B'};
    phis=rad2deg([phiN 0 0 phiN]);
    fprintf(hdr);
    fprintf('| %-4s | %16s | %16s | %16s | %16s |\n','Node','s [mm]','T [N.mm]','tau [MPa]','phi [deg]');
    fprintf(hdr);
    for k=1:4
        fprintf('| %-4s | %16.4f | %16.4f | %16.5f | %16.5f |\n',names{k},sN(k),T,tauv,phis(k));
    end
    fprintf(hdr);
end
function fig=newFig(nameStr,w,h)
    fig=figure('Name',nameStr,'Color','w','Units','pixels',...
            'Position',[60 60 w h],'PaperPositionMode','auto');
end
function applyFontSize(figHandle,fs)
    axs=findall(figHandle,'Type','axes');
    for k=1:numel(axs)
        try
            axs(k).TitleFontSizeMultiplier=1;
            axs(k).LabelFontSizeMultiplier=1;
        catch
        end
    end
    set(findall(figHandle,'-property','FontSize'),'FontSize',fs);
    cbs=findall(figHandle,'Type','colorbar');
    for k=1:numel(cbs)
        try
            cbs(k).FontSize=fs;
            cbs(k).Label.FontSize=fs;
        catch
        end
    end
end
function savefig_safe(figHandle,figDir,filename,fs)
    applyFontSize(figHandle,fs);
    drawnow;
    outPath=fullfile(figDir,filename);
    try
        print(figHandle,outPath,'-dpng','-r300');
    catch ME
        warning('Could not save figure "%s": %s\nContinuing without this file.',outPath,ME.message);
    end
end
