function [sd,a]=FM2_Deriv(s,de,T,phi,m,ac,c,in)
g=9.81;
h=s(2); vx=s(3); vz=s(4); th=s(5); q=s(6);
V=sqrt(vx^2 + vz^2);
if V > 0.3,ex=vx/V; ez=vz/V; else,ex=1; ez=0; end
gam=atan2(ez,ex);
alpha=th - gam;
if h < 0,h=0; end
rho=1.225*((288 - 0.0065*h)/288)^4.2559;
qb=0.5*rho*V^2;
qh=0;
if V > 3,qh=q*ac.c/(2*V); end
cth=cos(th); sth=sin(th);
rmx=ac.xb_main*cth - ac.zb_main*sth; rmz=ac.xb_main*sth + ac.zb_main*cth;
rtx=ac.xb_tail*cth - ac.zb_tail*sth; rtz=ac.xb_tail*sth + ac.zb_tail*cth;
hm=s(2) + rmz; ht=s(2) + rtz;
hw=min(hm,ht);
CLlin=ac.CL0 + ac.CLa*alpha + ac.CLq*qh + ac.CLde*de;
CL=CLlin; extra=0;
if CLlin > ac.CL_max
    extra=CLlin - ac.CL_max; CL=ac.CL_max - c.k_post*extra;
elseif CLlin < ac.CL_neg
    extra=ac.CL_neg - CLlin; CL=ac.CL_neg + c.k_post*extra;
end
CL=max(min(CL,ac.CL_max),ac.CL_neg);
Kg=ac.K;
if in.use_ground_effect
    x16=16*(max(hw,0) + ac.h_wing_cg)/ac.b;
    Kg=ac.K*x16^2/(1 + x16^2);
end
CD=ac.CD0 + Kg*CL^2 + c.k_cd_stall*4*(extra/ac.CLa)^2;
Cm=ac.Cm0 + ac.Cma*alpha + ac.Cmq*qh + ac.Cmde*de;
Lf=qb*ac.S*CL; Df=qb*ac.S*CD;
Fx=T*cth - Df*ex - Lf*cos(phi)*ez;
Fz=T*sth - Df*ez + Lf*cos(phi)*ex - m*g;
M=qb*ac.S*ac.c*Cm + T*ac.arm_T;
mu=max(in.mu_roll,in.mu_landing);
Nm=0; Nt=0;
if hm < 0
    Nm=max(0,-c.k_main*hm - c.c_main*(vz + q*rmx));
    fr=-mu*Nm*tanh((vx - q*rmz)/0.2);
    Fx=Fx + fr; Fz=Fz + Nm; M=M + rmx*Nm - rmz*fr;
end
if ht < 0
    Nt=max(0,-c.k_tail*ht - c.c_tail*(vz + q*rtx));
    fr=-mu*Nt*tanh((vx - q*rtz)/0.2);
    Fx=Fx + fr; Fz=Fz + Nt; M=M + rtx*Nt - rtz*fr;
end
sd=[vx; vz; Fx/m; Fz/m; q; M/ac.Iyy];
a=struct('V',V,'gam',gam,'alpha',alpha,'CL',CL,'CD',CD,'Cm',Cm,'L',Lf,'D',Df,'qb',qb,'rho',rho,...
    'hw',hw,'Nm',Nm,'Nt',Nt,'stalled',extra>0);
end
