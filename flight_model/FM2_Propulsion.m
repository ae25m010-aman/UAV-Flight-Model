function r=FM2_Propulsion(d,V,rho,SoC,P_aux,in,n0)
D=in.prop.D;
OCV=in.batt.N_cells*interp1(in.batt.SoC_tab,in.batt.OCV_tab,max(min(SoC,1),0));
Rs=in.batt.R_pack + in.esc.R_wire;
n_lo=max(V/(in.prop.J(end)*D),5);
n_hi=in.prop.RPM_max/60;
r=struct('T',0,'n',n_lo,'P_shaft',0,'I_m',0,'P_elec',P_aux,'V_b',0,'I_b',0,'J',0,'eta_p',0);
if d < 0.01 || over(n_lo,d,V,rho,OCV,Rs,P_aux,in) > 0
    r.V_b=(OCV + sqrt(max(OCV^2 - 4*Rs*P_aux,0)))/2;
    r.I_b=P_aux/r.V_b;
    return
end
a=n_lo; b=n_hi;
if n0 > n_lo*1.2 && n0 < n_hi*0.8
    if over(n0*0.85,d,V,rho,OCV,Rs,P_aux,in) < 0,a=n0*0.85; end
    if over(n0*1.15,d,V,rho,OCV,Rs,P_aux,in) > 0,b=n0*1.15; end
end
if over(b,d,V,rho,OCV,Rs,P_aux,in) <= 0
    n=b;
else
    for i=1:16
        c=(a+b)/2;
        if over(c,d,V,rho,OCV,Rs,P_aux,in) > 0,b=c; else,a=c; end
    end
    n=a;
end
[T,Ps,Im,Vm]=point(n,V,rho,in);
P=Vm*Im/in.esc.eta + P_aux;
r.T=T; r.n=n; r.P_shaft=Ps; r.I_m=Im; r.P_elec=P;
r.V_b=(OCV + sqrt(max(OCV^2 - 4*Rs*P,0)))/2;
r.I_b=P/r.V_b;
r.J=V/(n*D);
if Ps > 0,r.eta_p=T*V/Ps; end
end
function g=over(n,d,V,rho,OCV,Rs,P_aux,in)
[~,~,Im,Vm]=point(n,V,rho,in);
P=Vm*Im/in.esc.eta + P_aux;
Vb=(OCV + sqrt(max(OCV^2 - 4*Rs*P,0)))/2;
g=max([Vm/(min(d,in.esc.d_max)*Vb),Im/in.mot.I_max,Im/in.esc.I_max,Vm*Im/in.mot.P_max]) - 1;
end
function [T,Ps,Im,Vm]=point(n,V,rho,in)
p=in.prop; D=p.D;
J=min(V/(n*D),p.J(end));
rpm=min(max(n*60,p.RPM(1)),p.RPM(end));
ir=min(find(p.RPM <= rpm,1,'last'),numel(p.RPM)-1);
fr=(rpm - p.RPM(ir))/(p.RPM(ir+1) - p.RPM(ir));
jj=J/(p.J(2)-p.J(1));
ij=min(floor(jj)+1,numel(p.J)-1);
fj=jj - (ij-1);
CT=(1-fr)*((1-fj)*p.CT(ir,ij)+fj*p.CT(ir,ij+1)) + fr*((1-fj)*p.CT(ir+1,ij)+fj*p.CT(ir+1,ij+1));
CP=(1-fr)*((1-fj)*p.CP(ir,ij)+fj*p.CP(ir,ij+1)) + fr*((1-fj)*p.CP(ir+1,ij)+fj*p.CP(ir+1,ij+1));
T=p.k_T*CT*rho*n^2*D^4;
Ps=p.k_P*CP*rho*n^3*D^5;
Im=Ps/(2*pi*n)/(60/(2*pi*in.mot.KV)) + in.mot.I0;
Vm=n*60/in.mot.KV + Im*in.mot.R_m;
end
