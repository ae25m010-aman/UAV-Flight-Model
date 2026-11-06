function [sd,a]=FM3_Deriv(s,u,pp,m,ac,c,in)
g=9.81;
ub=s(4); vb=s(5); wb=s(6); p=s(7); q=s(8); r=s(9);
ph=s(10); th=s(11); ps=s(12);
sf=sin(ph); cf=cos(ph); st=sin(th); ct=cos(th); sp=sin(ps); cp=cos(ps);
C=[ct*cp,sf*st*cp-cf*sp,cf*st*cp+sf*sp; ct*sp,sf*st*sp+cf*cp,cf*st*sp-sf*cp; -st,sf*ct,cf*ct];
h=-s(3); hh=max(h,0);
V=sqrt(ub^2 + vb^2 + wb^2);
rho=1.225*((288 - 0.0065*hh)/288)^4.2559;
qb=0.5*rho*V^2;
if V > 0.5
    alpha=atan2(wb,ub); beta=asin(max(min(vb/V,1),-1));
else
    alpha=0; beta=0;
end
if V > 3
    ph_=p*ac.b/(2*V); rh=r*ac.b/(2*V); qh=q*ac.c/(2*V);
else
    ph_=0; rh=0; qh=0;
end
W=ac.wheels;
Nw=[0 0 0];
hw=h - (C(3,1)*W(:,1) + C(3,2)*W(:,2) + C(3,3)*W(:,3));
hmin=min(hw);
CLlin=ac.CL0 + ac.CLa*alpha + ac.CLq*qh + ac.CLde*u.de;
CL=CLlin; extra=0;
if CLlin > ac.CL_max
    extra=CLlin - ac.CL_max; CL=ac.CL_max - c.k_post*extra;
elseif CLlin < ac.CL_neg
    extra=ac.CL_neg - CLlin; CL=ac.CL_neg + c.k_post*extra;
end
CL=max(min(CL,ac.CL_max),ac.CL_neg);
Kg=ac.K;
if in.use_ground_effect
    x16=16*(max(hmin,0) + ac.h_wing_cg)/ac.b;
    Kg=ac.K*x16^2/(1 + x16^2);
end
qr=qb + pp.fv*(pp.qs - qb);
qe=qb + pp.fh*(pp.qs - qb);
CY0=ac.CYb*beta + ac.CYp*ph_ + ac.CYr*rh;
Yr=qr*ac.S*ac.CYdr*u.dr;
CY_all=CY0; if qb > 1,CY_all=CY0 + Yr/(qb*ac.S); end
CD=ac.CD0 + Kg*CL^2 + c.k_cd_stall*4*(extra/ac.CLa)^2 + min(CY_all^2/ac.kCY,0.3);
Lf=qb*ac.S*CL; Df=qb*ac.S*CD;
Fx=-Df*cos(alpha) + Lf*sin(alpha);
Fz=-Df*sin(alpha) - Lf*cos(alpha);
Fy=qb*ac.S*CY0 + Yr;
Cl=ac.Clb*beta + ac.Clp*ph_ + (ac.Clr_VT + CL/3)*rh;
Cn=ac.Cnb*beta + (ac.Cnp_VT - CL/6)*ph_ + ac.Cnr*rh;
Lm=qb*ac.S*ac.b*(Cl + ac.Clda*u.da) + qr*ac.S*ac.b*ac.Cldr*u.dr;
Nm=qb*ac.S*ac.b*(Cn + ac.Cnda*u.da) + qr*ac.S*ac.b*ac.Cndr*u.dr;
Cm=ac.Cm0 + ac.Cma*alpha + ac.Cmq*qh;
Mm=qb*ac.S*ac.c*Cm + qe*ac.S*ac.c*ac.Cmde*u.de;
T=pp.T; sh=sin(u.hub); ch=cos(u.hub);
Tx=T*ch; Ty=T*sh;
Fx=Fx + Tx; Fy=Fy + Ty;
Lm=Lm - ac.arm_T*Ty - pp.Q;
Mm=Mm + ac.arm_T*Tx;
Nm=Nm + ac.x_prop*Ty;
e=c.k_pfactor*ac.prop_R;
Nm=Nm - T*e*alpha;
Mm=Mm - T*e*beta;
Fsw=pp.qs*ac.Sv*pp.fv*ac.CLa_VT*sin(pp.eps);
Fy=Fy + Fsw;
Lm=Lm + pp.zv*Fsw;
Nm=Nm - ac.l_v*Fsw;
H=c.prop_inertia*2*pi*pp.n;
Mm=Mm - r*H;
Nm=Nm + q*H;
Fx=Fx + m*g*C(3,1); Fy=Fy + m*g*C(3,2); Fz=Fz + m*g*C(3,3);
if hmin < 0
    kw=[c.k_main/2 c.k_main/2 c.k_tail]; cw=[c.c_main/2 c.c_main/2 c.c_tail];
    mu=max(in.mu_roll,in.mu_landing);
    for i=1:3
        if hw(i) < 0
            rx=W(i,1); ry=W(i,2); rz=W(i,3);
            vw=[ub + q*rz - r*ry; vb + r*rx - p*rz; wb + p*ry - q*rx];
            ve=C*vw;
            N=max(0,-kw(i)*hw(i) + cw(i)*ve(3));
            vl=ve(1)*cp + ve(2)*sp; vs=-ve(1)*sp + ve(2)*cp;
            if i == 3,vs=vs - vl*u.dt; end
            fl=-mu*N*tanh(vl/0.2); fs=-c.mu_side*N*tanh(vs/0.2);
            Fe=[fl*cp - fs*sp; fl*sp + fs*cp; -N];
            Fb=C'*Fe;
            Fx=Fx + Fb(1); Fy=Fy + Fb(2); Fz=Fz + Fb(3);
            Lm=Lm + ry*Fb(3) - rz*Fb(2);
            Mm=Mm + rz*Fb(1) - rx*Fb(3);
            Nm=Nm + rx*Fb(2) - ry*Fb(1);
            Nw(i)=N;
        end
    end
end
Ix=ac.Ixx; Iy=ac.Iyy; Iz=ac.Izz; Ixz=ac.Ixz; G=Ix*Iz - Ixz^2;
pd=(Iz*Lm + Ixz*Nm + Ixz*(Ix-Iy+Iz)*p*q - (Iz*(Iz-Iy)+Ixz^2)*q*r)/G;
qd=(Mm + (Iz-Ix)*p*r - Ixz*(p^2-r^2))/Iy;
rd=(Ixz*Lm + Ix*Nm + (Ix*(Ix-Iy)+Ixz^2)*p*q - Ixz*(Ix-Iy+Iz)*q*r)/G;
ve=C*[ub; vb; wb];
sd=[ve; r*vb-q*wb+Fx/m; p*wb-r*ub+Fy/m; q*ub-p*vb+Fz/m; pd; qd; rd; ...
      p + st/ct*(q*sf+r*cf); q*cf-r*sf; (q*sf+r*cf)/ct];
a=struct('V',V,'alpha',alpha,'beta',beta,'CL',CL,'CD',CD,'CY',CY_all,'L',Lf,'D',Df,'qb',qb,'rho',rho,...
    'hw',hmin,'Nw',Nw,'stalled',extra>0,'hdot',-ve(3),'vx',ve(1),'vy',ve(2),'Cm',Cm);
end
