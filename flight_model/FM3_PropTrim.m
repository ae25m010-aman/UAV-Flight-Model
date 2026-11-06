function tb=FM3_PropTrim(V,gamma_deg,bank_deg,h,ac,c,in,fix_r,fix_h)
g=9.81;
t=FM2_Trim(V,h,gamma_deg,bank_deg,ac.m,ac);
rho=FM1_Atmosphere(h); qb=0.5*rho*V^2;
pr=FM1_Propulsion(t.T,V,rho,0.7,FM1_Aux('cruise',in),in);
n=pr.RPM/60;
p=FM3_Slip(t.T,pr.P_shaft,n,V,rho,ac,c);
T=t.T; alpha=t.alpha*pi/180;
e=c.k_pfactor*ac.prop_R;
Fsw=p.qs*ac.Sv*p.fv*ac.CLa_VT*sin(p.eps);
N_pf=-T*e*alpha; N_sw=-ac.l_v*Fsw;
N0=N_pf + N_sw; L0=-p.Q + p.zv*Fsw; Y0=Fsw;
K=qb*ac.S*ac.b; qr=(qb + p.fv*(p.qs-qb))/qb;
W=ac.m*g;
Aa=[K*qr*ac.Cndr K*ac.Cnda 0; K*qr*ac.Cldr K*ac.Clda 0; qb*ac.S*qr*ac.CYdr 0 W];
xa=-Aa\[N0; L0; Y0];
Ab=[ac.x_prop*T K*ac.Cnda 0; -ac.arm_T*T K*ac.Clda 0; T 0 W];
xb=-Ab\[N0; L0; Y0];
Ac=[K*ac.Cnb K*ac.Cnda 0; K*ac.Clb K*ac.Clda 0; qb*ac.S*ac.CYb 0 W];
xc=-Ac\[N0; L0; Y0];
tb=struct('V',V,'T',T,'alpha',t.alpha,'Q',p.Q,'RPM',pr.RPM,'eps',p.eps*180/pi,'fv',p.fv,...
    'N_pf',N_pf,'N_sw',N_sw,'L_tq',-p.Q,'dr',xa(1)*180/pi,'da_a',xa(2)*180/pi,'bank_a',asind(xa(3)),...
    'hub',xb(1)*180/pi,'da_b',xb(2)*180/pi,'bank_b',asind(xb(3)),'beta_none',xc(1)*180/pi,'da_none',xc(2)*180/pi);
if nargin >= 9
    Nh=ac.x_prop*T*fix_h*pi/180;
    tb.dr_left_hub=-(N0 + Nh)/(K*qr*ac.Cndr)*180/pi;
    tb.dr_left_rudder=tb.dr - fix_r;
end
end
