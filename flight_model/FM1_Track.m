function tr=FM1_Track(ms,in)
L=ms.log; n=numel(L.t);
ds=[0 diff(L.x)];
X=zeros(1,n); Y=X; psi=X; orbit=false(1,n);
xc=0; yc=0; p=0;
R_l=ms.V_loiter*in.t_loiter/(2*pi*round(ms.V_loiter*in.t_loiter/(2*pi*ms.loiter_radius)-0.5)+pi);
iD=L.phase==7; iE=L.phase>=8;
S_fin=sum(ds(iE)); S_des=sum(ds(iD));
S_b=sum(ds(L.phase==6)); S_o=sum(ds(L.phase<=4));
X_home=S_o - S_b;
X_stop=60;
straight=X_home - (X_stop + S_fin);
excess=S_des - straight;
Nd=0; R_d=1;
if excess > 0
    Nd=max(1,round(excess/(2*pi*150)));
    R_d=excess/(2*pi*Nd);
end
sD=0; S5=0;
for k=1:n
    ph=L.phase(k); d=ds(k);
    if ph <= 4
        p=0; xc=xc + d;
    elseif ph == 5
        p=p + d/R_l; xc=xc + d*cos(p); yc=yc + d*sin(p);
    elseif ph == 6
        S5=S5 + d; fr=S5/S_b; p=pi;
        xc=xc - d; yc=2*R_l*(1 - (3*fr^2 - 2*fr^3));
    elseif ph == 7
        sD=sD + d;
        if sD <= excess && Nd > 0,p=pi - sD/R_d; orbit(k)=true; else,p=pi; end
        xc=xc + d*cos(p); yc=yc + d*sin(p);
    else
        p=pi; xc=xc - d;
    end
    X(k)=xc; Y(k)=yc; psi(k)=p;
end
if Nd > 0,Y(iE)=0; end
dpsi=[0 diff(unwrap(psi))]; dt=max([1e-3 diff(L.t)],1e-3);
rate=dpsi./dt;
phi=-atan(rate.*L.V/9.81);
phi(L.h < 2)=0;
phi=max(min(phi,35*pi/180),-35*pi/180);
tr.orbit=orbit; tr.X=X; tr.Y=Y; tr.psi=psi; tr.phi=phi; tr.R_d=R_d; tr.Nd=Nd; tr.X_home=X_home;
end
