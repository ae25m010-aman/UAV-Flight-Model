function FM1_GeometryFigures(g,in)
parts=FM1_GeometryBuild(g);
x_ht_le=g.Xt-0.25*g.ct; x_ht_te=g.Xt+0.75*g.ct; x_vt_le=g.Xt-0.25*g.cv; x_vt_te=g.Xt+0.75*g.cv;
yw=g.W_f/2 + g.y_leg + 12; z_low=g.z_axle - g.rw; top=g.z_boom + g.hv;
f=newfig(1400,900); drawparts(parts); iso3(); hold on;
d=-420; dd=z_low - 40;
dim_line([0 d dd],[g.L_total d dd],sprintf('Length %.2f',g.L_total));
dim_line([g.L_total+60 -g.b/2 dd],[g.L_total+60 g.b/2 dd],sprintf('Wing span %.0f',g.b),[0 300 0]);
dim_line([g.x_le g.b/2+40 g.H+40],[g.x_le+g.c g.b/2+40 g.H+40],sprintf('Chord %.0f',g.c),[0 0 45]);
dim_line([x_ht_le -g.bt/2-50 g.z_boom],[x_ht_te -g.bt/2-50 g.z_boom],sprintf('HT chord %.1f',g.ct),[0 -90 0]);
dim_line([x_ht_te+40 -g.bt/2 g.z_boom],[x_ht_te+40 g.bt/2 g.z_boom],sprintf('HT span %.1f',g.bt),[60 -170 0]);
dim_line([x_vt_te+15 0 g.z_boom],[x_vt_te+15 0 top],sprintf('VT height %.0f',g.hv),[0 170 0]);
dim_line([x_vt_le 0 top+30],[x_vt_te 0 top+30],sprintf('VT chord %.0f',g.cv),[0 0 40]);
dim_line([-70 0 g.H/2-g.prop_D/2],[-70 0 g.H/2+g.prop_D/2],sprintf('Prop dia %.1f',g.prop_D),[-40 -160 90]);
dim_line([g.x_main -yw+12 z_low-15],[g.x_main yw-12 z_low-15],sprintf('Track %.1f',2*(yw-12)),[0 0 -35]);
axis tight; axis off; set(gca,'Position',[0 0 1 0.94]); camzoom(1.25);
ax_t=axes('Position',[0 0.94 1 0.06]); axis(ax_t,'off');
text('Parent',ax_t,'Position',[0.5 0.5],'String','G01  UAV isometric view, dimensions in mm','HorizontalAlignment','center','FontWeight','bold','FontSize',12);
FM1_SaveFig(f,'G01_Isometric','UAV 3D isometric view with main dimensions',in);
f=newfig(1500,800); drawparts(parts); view([0 -1 0]); axis equal; hold on; grid on;
xlabel('x (mm)'); zlabel('z (mm)'); yf=-g.b/2-30;
st=sort([0 g.Nr g.Nr+40 g.x_main-20 g.x_main g.x_main+20 g.x_le g.x_spar g.x_spar+40 g.L_f-g.Rr g.x_le+g.c g.L_f-15 g.L_f x_ht_le x_vt_le g.Xt x_vt_te x_ht_te g.L_total]);
for k=1:numel(st)
    lev=z_low - 25 - 62*mod(k,2);
    plot3(st(k)*[1 1],yf*[1 1],[lev+18 z_low-8],'-','Color',[.5 .5 .5]);
    text(st(k),yf,lev+15,sprintf('%.2f',st(k)),'Rotation',90,'HorizontalAlignment','right','FontSize',8);
end
dim_line([-100 yf g.nose_low],[-100 yf g.H-g.nose_low],sprintf('%.0f',g.H-2*g.nose_low));
dim_line([g.Nr-22 yf 0],[g.Nr-22 yf g.H],sprintf('%.0f',g.H),[-22 0 0]);
dim_line([g.L_f+28 yf g.H-g.rear_h],[g.L_f+28 yf g.H],sprintf('%.0f',g.rear_h),[0 0 -55]);
dim_line([680 yf g.H-g.boom_h],[680 yf g.H],sprintf('%.0f',g.boom_h),[0 0 26]);
dim_line([x_vt_le-30 yf g.z_boom],[x_vt_le-30 yf top],sprintf('%.0f',g.hv));
dim_line([g.x_main+78 yf g.z_axle],[g.x_main+78 yf -g.block],sprintf('Leg drop %.1f',g.leg_drop),[50 0 0]);
dim_line([g.x_main-72 yf -g.block],[g.x_main-72 yf 0],sprintf('Block %.0f',g.block),[-42 0 0]);
dim_line([g.x_tw+50 yf g.z_boom-g.L_strut],[g.x_tw+50 yf g.z_boom],sprintf('%.0f',g.L_strut));
dim_line([g.x_main-g.rw yf z_low-5],[g.x_main+g.rw yf z_low-5],sprintf('Wheel dia %.0f',2*g.rw),[110 0 8]);
dim_line([g.x_tw-g.rt yf g.z_boom-g.L_strut-g.rt-6],[g.x_tw+g.rt yf g.z_boom-g.L_strut-g.rt-6],sprintf('Dia %.0f',2*g.rt),[0 0 -22]);
dim_line([-40 yf g.H/2+g.prop_D/2+10],[-40 yf g.H/2-g.prop_D/2-10],sprintf('Prop dia %.1f',g.prop_D),[0 0 195]);
xlim([-230 g.L_total+90]); zlim([z_low-150 top+40]); ylim([yf-50 g.b/2+50]);
title('G02  Side view (x-z), x stations and heights in mm');
FM1_SaveFig(f,'G02_Side_View','Side view with x stations and heights',in);
f=newfig(1200,1000); drawparts(parts); view(2); axis equal; hold on; grid on;
xlabel('x (mm)'); ylabel('y (mm)');
z=900;
dim_line([0 -g.b/2-70 z],[g.L_total -g.b/2-70 z],sprintf('Length %.2f',g.L_total));
dim_line([g.x_le-90 -g.b/2 z],[g.x_le-90 g.b/2 z],sprintf('Span %.0f',g.b),[0 350 0]);
dim_line([g.x_le g.b/2+50 z],[g.x_le+g.c g.b/2+50 z],sprintf('Chord %.0f  (%.0f to %.0f)',g.c,g.x_le,g.x_le+g.c));
dim_line([g.L_f+25 -g.W_f/2 z],[g.L_f+25 g.W_f/2 z],sprintf('%.1f',g.W_f),[0 -80 0]);
dim_line([x_ht_le-30 -g.bt/2 z],[x_ht_le-30 g.bt/2 z],sprintf('HT span %.1f',g.bt),[0 150 0]);
dim_line([x_ht_le g.bt/2+40 z],[x_ht_te g.bt/2+40 z],sprintf('HT chord %.1f',g.ct));
dim_line([640 -g.boom_w/2 z],[640 g.boom_w/2 z],sprintf('Boom %.1f',g.boom_w),[0 55 0]);
xlim([-200 g.L_total+120]); ylim([-g.b/2-160 g.b/2+130]);
title('G03  Top view (x-y), dimensions in mm');
FM1_SaveFig(f,'G03_Top_View','Top view with wing, tail and fuselage dimensions',in);
f=newfig(1300,800); drawparts(parts); view([-1 0 0]); axis equal; hold on; grid on;
ylabel('y (mm)'); zlabel('z (mm)');
x=-300;
dim_line([x -g.b/2 top+50],[x g.b/2 top+50],sprintf('Span %.0f',g.b));
dim_line([x -yw+12 z_low-30],[x yw-12 z_low-30],sprintf('Track %.1f',2*(yw-12)));
dim_line([x -g.W_f/2 -g.block-40],[x g.W_f/2 -g.block-40],sprintf('%.1f',g.W_f));
dim_line([x yw+25 z_low],[x yw+25 0],sprintf('%.1f',-z_low),[0 -60 0]);
dim_line([x -g.bt/2 z_low-100],[x g.bt/2 z_low-100],sprintf('HT span %.1f',g.bt));
dim_line([x -420 g.z_boom],[x -420 top],sprintf('VT height %.0f',g.hv));
dim_line([x 420 g.H/2-g.prop_D/2],[x 420 g.H/2+g.prop_D/2],sprintf('Prop dia %.1f',g.prop_D));
ylim([-g.b/2-60 g.b/2+60]); zlim([z_low-160 top+110]);
title('G04  Front view (y-z), dimensions in mm');
FM1_SaveFig(f,'G04_Front_View','Front view with span, track and heights',in);
f=newfig(1300,850); hold on;
for k=1:numel(parts)
    a=0.12; patch('Vertices',parts(k).V,'Faces',parts(k).F,'FaceColor',parts(k).col,'FaceAlpha',a,'EdgeColor','none');
end
red=[0.8 0.2 0.15]; grn=[0.15 0.55 0.3]; pur=[0.45 0.25 0.65];
for xb=[g.Nr g.x_main-20 g.x_spar]
    ring(xb,xb+40,g.W_f,g.H,15,red);
end
ring(g.L_f-15,g.L_f,g.W_f,g.rear_h,10,red,g.H-g.rear_h);
tp=[0 g.H-g.nose_low; g.Nr g.H; g.L_f g.H];
bt=[0 g.nose_low; g.Nr 0; g.L_f-g.Rr 0; g.L_f g.H-g.rear_h];
for s=[-1 1]
    plot3(tp(:,1),s*g.W_f/2*ones(3,1),tp(:,2),'-','Color',grn,'LineWidth',3);
    plot3(bt(:,1),s*g.W_f/2*ones(4,1),bt(:,2),'-','Color',grn,'LineWidth',3);
end
af=FM1_ClarkY(); z0=g.H + g.z_chord*g.c;
for yr=(-3:3)*280
    plot3(g.x_le+af(:,1)*g.c,yr*ones(size(af,1),1),z0+af(:,2)*g.c,'-','Color',pur,'LineWidth',1.5);
end
for k=1:numel(g.str_x)
    plot3(g.str_x(k)*[1 1],[-g.b/2 g.b/2],g.str_z(k)*[1 1],'-','Color',[0.1 0.4 0.75],'LineWidth',1.5);
end
fill3(g.x_spar*[1 1 1 1],[-280 280 280 -280],[g.H g.H g.H+0.117*g.c g.H+0.117*g.c],[0.2 0.2 0.2],'FaceAlpha',0.55);
iso3();
h=[];
h(1)=plot3(NaN,NaN,NaN,'-','Color',red,'LineWidth',3); h(2)=plot3(NaN,NaN,NaN,'-','Color',grn,'LineWidth',3);
h(3)=plot3(NaN,NaN,NaN,'-','Color',pur,'LineWidth',2); h(4)=plot3(NaN,NaN,NaN,'-','Color',[0.1 0.4 0.75],'LineWidth',2);
h(5)=plot3(NaN,NaN,NaN,'-','Color',[0.2 0.2 0.2],'LineWidth',3);
legend(h,{'Bulkheads (3 + rear)','Longerons (4 runs)','Ribs (7)','Stringers (5)','Spar (560 long)'},'Location','northeast');
title('G05  Internal structure');
FM1_SaveFig(f,'G05_Internal_Structure','Bulkheads, longerons, ribs, stringers and spar',in);
f=newfig(1200,500);
subplot(1,3,1); hold on; rectangle('Position',[-g.W_f/2 0 g.W_f g.H]);
rectangle('Position',[-g.W_f/2+15 15 g.W_f-30 g.H-30]); axis equal; grid on;
xlabel('y'); ylabel('z'); title('Front (y-z)');
text(0,g.H+7,sprintf('%.1f',g.W_f),'HorizontalAlignment','center');
text(0,g.H/2,sprintf('Opening\n%.1f x %.0f',g.W_f-30,g.H-30),'HorizontalAlignment','center');
text(g.W_f/2+4,g.H/2,sprintf('%.0f',g.H)); text(0,7,'15','HorizontalAlignment','center');
xlim([-70 80]); ylim([-10 g.H+18]);
subplot(1,3,2); hold on; rectangle('Position',[0 0 40 g.H]);
plot([0 40],[15 15],'k--'); plot([0 40],[g.H-15 g.H-15],'k--'); axis equal; grid on;
xlabel('x'); ylabel('z'); title('Side (x-z)'); text(20,g.H+7,'40','HorizontalAlignment','center');
xlim([-20 60]); ylim([-10 g.H+18]);
subplot(1,3,3); hold on; rectangle('Position',[-g.W_f/2 0 g.W_f 40]);
plot([-g.W_f/2+15 -g.W_f/2+15],[0 40],'k--'); plot([g.W_f/2-15 g.W_f/2-15],[0 40],'k--'); axis equal; grid on;
xlabel('y'); ylabel('x'); title('Top (x-y)'); text(0,47,sprintf('%.1f',g.W_f),'HorizontalAlignment','center');
xlim([-70 70]); ylim([-10 60]);
FM1_SaveFig(f,'G06_Bulkhead','Fuselage bulkhead in three views',in);
f=newfig(1200,500); hold on; af=FM1_ClarkY();
plot(g.x_le+af(:,1)*g.c,z0+af(:,2)*g.c,'k','LineWidth',1.5);
xs=g.x_spar; hs=0.117*g.c; zs=g.H;
plot([xs xs],[zs zs+hs],'Color',[0.8 0.2 0.15],'LineWidth',2.5);
plot([xs xs+20],[zs+hs zs+hs],'Color',[0.8 0.2 0.15],'LineWidth',2.5);
plot([xs xs+40],[zs zs],'Color',[0.8 0.2 0.15],'LineWidth',2.5);
plot(g.str_x,g.str_z,'s','Color',[0.1 0.4 0.75],'MarkerFaceColor',[0.1 0.4 0.75]);
plot(g.x_le+0.25*g.c,z0+0.02*g.c,'k+','MarkerSize',12,'LineWidth',2);
text(g.x_le+0.25*g.c,z0+0.02*g.c-8,'c/4 (235)','HorizontalAlignment','center');
plot([g.x_le-20 g.x_le+g.c+20],[g.H g.H],'-','Color',[0.5 0.5 0.5]);
text(g.x_le+g.c+5,g.H-6,'Fuselage top','FontSize',9);
text(xs+44,zs+6,sprintf('Spar at %.0f, depth %.1f',xs,hs));
text(xs+24,zs+hs-4,'Upper flange 20'); text(xs+44,zs-6,'Lower flange 40');
axis equal; grid on; xlabel('x (mm)'); ylabel('z (mm)');
title(sprintf('G07  Wing section, Clark Y, chord %.0f, from %.0f to %.0f',g.c,g.x_le,g.x_le+g.c));
FM1_SaveFig(f,'G07_Wing_Section','Clark Y section with spar and stringers',in);
f=newfig(1500,700); hold on;
for k=1:numel(parts)
    patch('Vertices',parts(k).V,'Faces',parts(k).F,'FaceColor',parts(k).col,'FaceAlpha',0.15,'EdgeColor','none');
end
keep=g.W > 0;
h1=scatter3(g.X(keep),(-g.b/2-30)*ones(1,sum(keep)),g.Z(keep),8+g.W(keep)*0.35,[0.1 0.35 0.7],'filled');
h2=plot3(g.Xcg,-g.b/2-40,g.Zcg,'rp','MarkerSize',18,'MarkerFaceColor','r');
h3=plot3(g.Xn,-g.b/2-40,g.Zcg,'gd','MarkerSize',12,'MarkerFaceColor','g');
view([0 -1 0]); axis equal; grid on; xlabel('x (mm)'); zlabel('z (mm)');
xlim([-120 g.L_total+60]); zlim([z_low-40 top+40]); ylim([-g.b/2-60 g.b/2+60]);
legend([h1 h2 h3],{'Point masses (size = mass)','CG','Neutral point'},'Location','northeast');
title(sprintf('G08  Mass %.0f g, Xcg %.1f, Zcg %.1f, neutral point %.1f mm',sum(g.W),g.Xcg,g.Zcg,g.Xn));
FM1_SaveFig(f,'G08_Mass_Points_CG','Point masses, CG and neutral point',in);
f=newfig(1500,700); hold on;
th=atan2(g.z_tw_contact-g.z_main_contact,g.x_tw-g.x_main);
c=cos(th); s=sin(th);
for k=1:numel(parts)
    V=parts(k).V;
    xr=g.x_main + (V(:,1)-g.x_main)*c + (V(:,3)-g.z_main_contact)*s;
    zr=-(V(:,1)-g.x_main)*s + (V(:,3)-g.z_main_contact)*c;
    patch('Vertices',[xr V(:,2) zr],'Faces',parts(k).F,'FaceColor',parts(k).col,'EdgeColor','none','FaceAlpha',parts(k).alpha);
end
plot3([-400 1200],[0 0],[0 0],'k','LineWidth',2);
xc=g.x_main + (g.Xcg-g.x_main)*c + (g.Zcg-g.z_main_contact)*s;
zc=-(g.Xcg-g.x_main)*s + (g.Zcg-g.z_main_contact)*c;
plot3(xc,-g.b/2-30,zc,'rp','MarkerSize',16,'MarkerFaceColor','r');
text(xc+25,-g.b/2-30,zc,'CG','Color','r','FontWeight','bold');
view([0 -1 0]); axis equal; xlim([-250 1150]); zlim([-40 480]); ylim([-g.b/2-60 g.b/2+60]); grid on; xlabel('x (mm)'); zlabel('Height above ground (mm)');
title(sprintf('G09  Main leg drop %.1f, tail strut %.1f: sits %.2f deg nose-up (stall at %.2f deg). Tail wheel carries %.1f%% of the weight',...
    g.leg_drop,g.L_strut,th*180/pi,g.alpha_stall,(g.Xcg-g.x_main)/(g.x_tw-g.x_main)*100));
FM1_SaveFig(f,'G09_Ground_Attitude','Aircraft on its wheels at the ground attitude',in);
end
function f=newfig(w,h)
f=figure('Color','w','Position',[60 60 w h]);
end
function drawparts(parts)
hold on;
for k=1:numel(parts)
    patch('Vertices',parts(k).V,'Faces',parts(k).F,'FaceColor',parts(k).col,...
        'FaceAlpha',parts(k).alpha,'EdgeColor',[0.25 0.25 0.25],'EdgeAlpha',0.25,'LineWidth',0.3);
end
end
function iso3()
axis equal; view([-1 -1 0.75]); grid on; xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
end
function dim_line(p1,p2,txt,off)
if nargin < 4,off=[0 0 0]; end
c=[0.7 0.05 0.05];
plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'-','Color',c,'LineWidth',1);
plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'.','Color',c,'MarkerSize',10);
m=(p1+p2)/2 + off;
text(m(1),m(2),m(3),txt,'Color',c,'FontSize',9,'FontWeight','bold','HorizontalAlignment','center',...
    'VerticalAlignment','middle','BackgroundColor','w','Margin',1);
end
function ring(x1,x2,w,h,t,col,z0)
if nargin < 7,z0=0; end
y=[-w/2 w/2 w/2 -w/2 -w/2]; z=z0 + [0 0 h h 0];
yi=[-w/2+t w/2-t w/2-t -w/2+t -w/2+t]; zi=z0 + [t t h-t h-t t];
for x=[x1 x2]
    plot3(x*ones(1,5),y,z,'-','Color',col,'LineWidth',1.5);
    plot3(x*ones(1,5),yi,zi,'-','Color',col,'LineWidth',1);
end
for k=1:4
    plot3([x1 x2],y(k)*[1 1],z(k)*[1 1],'-','Color',col,'LineWidth',1.5);
end
end
