function p=FM3_Slip(T,Ps,n,V,rho,ac,c)
Vs=max(V,0.5); R=ac.prop_R; A=pi*R^2;
Traw=max(T,0);
vi=0.5*(-Vs + sqrt(Vs^2 + 2*Traw/(rho*A)));
Vss=Vs + 2*vi;
Q=0; if n > 0,Q=Ps/(2*pi*n); end
mdot=rho*A*(Vs + vi);
v_th=Q/(mdot*0.7*R);
p.T=T; p.Q=Q; p.n=n;
p.eps=c.k_swirl*atan(v_th/Vss);
qb=0.5*rho*V^2;
p.qs=qb + c.k_slip_q*(0.5*rho*Vss^2 - qb);
rs=R*sqrt((Vs + vi)/Vss);
dz=ac.dz_tail_hub;
top=min(rs,dz + ac.hv);
hin=max(top - dz,0);
p.fv=hin/ac.hv;
p.zv=ac.dz_hub_cg_up + dz + hin/2;
w=0; if rs > dz,w=2*sqrt(rs^2 - dz^2); end
p.fh=min(w/ac.bt,1);
end
