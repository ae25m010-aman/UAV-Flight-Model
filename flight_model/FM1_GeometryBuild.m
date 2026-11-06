function parts=FM1_GeometryBuild(g,n_foil)
if nargin < 2,n_foil=1; end
parts=struct('name',{},'V',{},'F',{},'col',{},'alpha',{});
blue=[0.72 0.8 0.9]; grey=[0.55 0.57 0.6]; wing=[0.88 0.88 0.82];
tail=[0.9 0.7 0.45]; dark=[0.15 0.15 0.17]; gear=[0.45 0.5 0.58]; wood=[0.72 0.55 0.35];
o=[0 g.H-g.nose_low; 0 g.nose_low; g.Nr 0; g.L_f-g.Rr 0; g.L_f g.H-g.rear_h; g.L_f g.H; g.Nr g.H];
parts=add_part(parts,'Fuselage',extrude_y(o,-g.W_f/2,g.W_f/2),blue,1);
parts=add_part(parts,'Boom',mesh_box(g.Xb,g.L_total,-g.boom_w/2,g.boom_w/2,g.H-g.boom_h,g.H),grey,1);
af=FM1_ClarkY(); af=af(1:n_foil:end,:); if any(af(end,:)~=af(1,:)),af=[af; af(1,:)]; end
z0=g.H + g.z_chord*g.c;
parts=add_part(parts,'Wing',loft(g.x_le + af(:,1)*g.c,z0 + af(:,2)*g.c,-g.b/2,g.b/2),wing,1);
ns=(0:0.1:1)'; yt=5*0.09*(0.2969*sqrt(ns)-0.126*ns-0.3516*ns.^2+0.2843*ns.^3-0.1036*ns.^4);
s=[flipud(ns) flipud(yt); ns(2:end) -yt(2:end)];
x_h=g.Xt - 0.25*g.ct; x_v=g.Xt - 0.25*g.cv;
parts=add_part(parts,'Horizontal tail',loft(x_h + s(:,1)*g.ct,g.z_boom + s(:,2)*g.ct,-g.bt/2,g.bt/2),tail,1);
m=loft(x_v + s(:,1)*g.cv,s(:,2)*g.cv,0,g.hv);
Vv=[m.V(:,1) m.V(:,3) g.z_boom + m.V(:,2)];
parts=add_part(parts,'Vertical tail',struct('V',Vv,'F',m.F),tail,1);
xa=g.x_main - g.lg_x/2; xb=g.x_main + g.lg_x/2;
parts=add_part(parts,'Wooden block',mesh_box(xa,xb,-g.W_f/2,g.W_f/2,-g.block,0),wood,1);
parts=add_part(parts,'LG middle',mesh_box(xa,xb,-g.W_f/2,g.W_f/2,-g.block-g.lg_t,-g.block),gear,1);
yw=g.W_f/2 + g.y_leg;
for sgn=[-1 1]
    v=[xa sgn*g.W_f/2 -g.block; xb sgn*g.W_f/2 -g.block; xb sgn*yw g.z_axle+g.lg_t; xa sgn*yw g.z_axle+g.lg_t; ...
         xa sgn*g.W_f/2 -g.block-g.lg_t; xb sgn*g.W_f/2 -g.block-g.lg_t; xb sgn*yw g.z_axle; xa sgn*yw g.z_axle];
    parts=add_part(parts,'LG leg',struct('V',v,'F',boxfaces()),gear,1);
    parts=add_part(parts,'Main wheel',cyl_y(g.x_main,sgn*(yw+12),g.z_axle,g.rw,25),dark,1);
end
parts=add_part(parts,'Tail strut',mesh_box(g.x_tw-3,g.x_tw+3,-3,3,g.z_boom-g.L_strut,g.H-g.boom_h),grey,1);
parts=add_part(parts,'Tail wheel',cyl_y(g.x_tw,0,g.z_boom-g.L_strut,g.rt,15),dark,1);
parts=add_part(parts,'Motor',cyl_x(-35,0,g.H/2,27),grey,1);
parts=add_part(parts,'Prop hub',cyl_x(-48,-35,g.H/2,10),dark,1);
th=linspace(0,2*pi,41)'; r=g.prop_D/2;
Vd=[(-40)*ones(41,1) r*cos(th) g.H/2 + r*sin(th); -40 0 g.H/2];
Fd=[(1:40)' (2:41)' 42*ones(40,1) 42*ones(40,1)];
parts=add_part(parts,'Prop disc',struct('V',Vd,'F',Fd),[0.5 0.5 0.5],0.18);
end
function p=add_part(p,name,m,col,al)
i=numel(p)+1; p(i).name=name; p(i).V=m.V; p(i).F=m.F; p(i).col=col; p(i).alpha=al;
end
function m=extrude_y(o,y1,y2)
n=size(o,1);
V=[o(:,1) y1*ones(n,1) o(:,2); o(:,1) y2*ones(n,1) o(:,2)];
F=[(1:n)' [2:n 1]' [n+2:2*n n+1]' (n+1:2*n)'];
caps=[ones(n-2,1) (2:n-1)' (3:n)' (3:n)'];
F=[F; caps; caps+n];
m=struct('V',V,'F',F);
end
function m=loft(x,z,y1,y2)
x=x(:); z=z(:); n=numel(x);
V=[x y1*ones(n,1) z; x y2*ones(n,1) z];
F=[(1:n-1)' (2:n)' (n+2:2*n)' (n+1:2*n-1)'];
caps=[ones(n-3,1) (2:n-2)' (3:n-1)' (3:n-1)'];
F=[F; caps; caps+n];
m=struct('V',V,'F',F);
end
function F=boxfaces()
F=[1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end
function m=mesh_box(x1,x2,y1,y2,z1,z2)
V=[x1 y1 z1; x2 y1 z1; x2 y2 z1; x1 y2 z1; x1 y1 z2; x2 y1 z2; x2 y2 z2; x1 y2 z2];
m=struct('V',V,'F',boxfaces());
end
function m=cyl_y(xc,yc,zc,r,w)
t=linspace(0,2*pi,25)'; t(end)=[]; n=numel(t);
V=[xc+r*cos(t) (yc-w/2)*ones(n,1) zc+r*sin(t); xc+r*cos(t) (yc+w/2)*ones(n,1) zc+r*sin(t)];
F=[(1:n)' [2:n 1]' [n+2:2*n n+1]' (n+1:2*n)'];
caps=[ones(n-2,1) (2:n-1)' (3:n)' (3:n)'];
m=struct('V',V,'F',[F; caps; caps+n]);
end
function m=cyl_x(x1,x2,zc,r)
t=linspace(0,2*pi,25)'; t(end)=[]; n=numel(t);
V=[x1*ones(n,1) r*cos(t) zc+r*sin(t); x2*ones(n,1) r*cos(t) zc+r*sin(t)];
F=[(1:n)' [2:n 1]' [n+2:2*n n+1]' (n+1:2*n)'];
caps=[ones(n-2,1) (2:n-1)' (3:n)' (3:n)'];
m=struct('V',V,'F',[F; caps; caps+n]);
end
