function pf=FM1_Performance(ac,in)
g=9.81; W=ac.m*g;
[rho,~,~]=FM1_Atmosphere(in.h_cruise);
rho0=FM1_Atmosphere(0);
P_aux=FM1_Aux('cruise',in);
pf.CD=ac.CD0 + ac.K*((W./(0.5*rho*(10:0.25:45).^2*ac.S)).^2);
pf.rho=rho;
pf.V_stall=sqrt(2*W/(rho *ac.S*ac.CL_max));
pf.V_stall_0=sqrt(2*W/(rho0*ac.S*ac.CL_max));
pf.CL_md=sqrt(ac.CD0/ac.K);
pf.CL_mp=sqrt(3)*pf.CL_md;
pf.LD_max=1/(2*sqrt(ac.CD0*ac.K));
pf.V_md=sqrt(2*W/(rho*ac.S*pf.CL_md));
pf.V_mp=sqrt(2*W/(rho*ac.S*pf.CL_mp));
pf.gamma_glide=atand(1/pf.LD_max);
pf.sink_min=pf.V_mp*(ac.CD0 + ac.K*pf.CL_mp^2)/pf.CL_mp;
V=10:0.25:45;
n=numel(V);
pf.V=V;
[pf.T_req,pf.P_req,pf.T_av,pf.P_av,pf.ROC,pf.P_elec,pf.eta_p,pf.CL]=deal(nan(1,n));
for i=1:n
    q=0.5*rho*V(i)^2;
    CL=W/(q*ac.S);
    D=q*ac.S*(ac.CD0 + ac.K*CL^2);
    pf.CL(i)=CL;
    pf.T_req(i)=D;
    pf.P_req(i)=D*V(i);
    fp=FM1_Propulsion(Inf,V(i),rho,0.8,P_aux,in);
    pf.T_av(i)=fp.T;
    pf.P_av(i)=fp.T*V(i);
    pf.ROC(i)=(fp.T - D)*V(i)/W;
    op=FM1_Propulsion(D,V(i),rho,0.8,P_aux,in);
    if ~op.limited
        pf.P_elec(i)=op.P_elec;
        pf.eta_p(i)=op.eta_p;
    end
end
fly=V >= pf.V_stall;
[pf.ROC_max,k]=max(pf.ROC.*fly); pf.V_y=V(k);
angle=asind(min(max(pf.ROC./V,-1),1)).*fly;
[pf.gamma_max,k]=max(angle); pf.V_x=V(k);
ok=pf.T_av >= pf.T_req & fly;
pf.V_max=NaN; pf.V_min_thrust=NaN;
if any(ok)
    pf.V_max=V(find(ok,1,'last'));
    pf.V_min_thrust=V(find(ok,1,'first'));
end
idx=find(fly & ~isnan(pf.P_elec));
[pf.P_elec_min,k]=min(pf.P_elec(idx)); pf.V_best_end=V(idx(k));
[~,k]=max(V(idx)./pf.P_elec(idx)); pf.V_best_range=V(idx(k));
Q=in.batt.Q_Ah; N=in.batt.N_cells;
soc=linspace(in.batt.SoC_min,1,50);
ocv=N*interp1(in.batt.SoC_tab,in.batt.OCV_tab,soc);
pf.E_usable_Wh=trapz(soc,ocv)*Q;
pf.E_total_Wh=N*trapz(in.batt.SoC_tab,in.batt.OCV_tab)*Q;
pf.Endurance_min=pf.E_usable_Wh/pf.P_elec_min*60;
Pr=interp1(V,pf.P_elec,pf.V_best_range);
pf.Range_km=pf.E_usable_Wh/Pr*3600*pf.V_best_range/1000;
hs=0:250:6000; pf.ceiling=NaN; pf.hs=hs; pf.ROC_h=nan(size(hs));
for j=1:numel(hs)
    rj=FM1_Atmosphere(hs(j)); best=-Inf;
    Vs=sqrt(2*W/(rj*ac.S*ac.CL_max));
    for v=Vs:1:45
        qq=0.5*rj*v^2; CLj=W/(qq*ac.S);
        Dj=qq*ac.S*(ac.CD0+ac.K*CLj^2);
        fj=FM1_Propulsion(Inf,v,rj,0.8,P_aux,in);
        best=max(best,(fj.T-Dj)*v/W);
    end
    pf.ROC_h(j)=best;
    if best >= 0.5,pf.ceiling=hs(j); end
end
s=FM1_Propulsion(Inf,0.01,rho0,1,P_aux,in);
pf.T_static=s.T; pf.I_static=s.I_m; pf.P_static=s.P_elec; pf.RPM_static=s.RPM;
pf.TW_static=s.T/W;
end
