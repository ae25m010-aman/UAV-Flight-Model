function r=FM1_Propulsion(T_req,V,rho,SoC,P_aux,in)
D=in.prop.D;
OCV=in.batt.N_cells * interp1(in.batt.SoC_tab,in.batt.OCV_tab,max(min(SoC,1),0));
Rs=in.batt.R_pack + in.esc.R_wire;
if T_req <= 0
    r=make_result(0,0,0,0,0,0,0,P_aux,OCV,Rs,V,false);
    return
end
n_hi=in.prop.RPM_max/60;
n_lo=max(V/(in.prop.J(end)*D),5);
if rpm_margin(n_hi) > 0
    a=n_lo; b=n_hi;
    for i=1:30
        c=(a+b)/2;
        if rpm_margin(c) > 0,b=c; else,a=c; end
    end
    n_hi=a;
end
limited=false;
if isinf(T_req) || thrust_at(n_hi) <= T_req
    n=n_hi;
    limited=~isinf(T_req);
else
    a=n_lo; b=n_hi;
    for i=1:30
        c=(a+b)/2;
        if thrust_at(c) < T_req,a=c; else,b=c; end
    end
    n=(a+b)/2;
end
[T,Ps,Im,Vm]=motor_point(n);
J=V/(n*D);
eta_p=0;
if Ps > 0,eta_p=T*V/Ps; end
r=make_result(T,n*60,J,eta_p,Ps,Im,Vm,Vm*Im/in.esc.eta + P_aux,OCV,Rs,V,limited);
    function t=thrust_at(nn)
        t=motor_point(nn);
    end
    function [T,Ps,Im,Vm]=motor_point(nn)
        J_=min(V/(nn*D),in.prop.J(end));
        [CT,CP]=prop_coeffs(J_,nn*60,in.prop);
        T=in.prop.k_T * CT * rho * nn^2 * D^4;
        Ps=in.prop.k_P * CP * rho * nn^3 * D^5;
        Kt=60/(2*pi*in.mot.KV);
        Im=Ps/(2*pi*nn)/Kt + in.mot.I0;
        Vm=nn*60/in.mot.KV + Im*in.mot.R_m;
    end
    function g=rpm_margin(nn)
        [~,~,Im,Vm]=motor_point(nn);
        P=Vm*Im/in.esc.eta + P_aux;
        Vb=(OCV + sqrt(max(OCV^2 - 4*Rs*P,0)))/2;
        g=max([Vm/(in.esc.d_max*Vb),Im/in.mot.I_max,Im/in.esc.I_max,Vm*Im/in.mot.P_max]) - 1;
    end
end
function [CT,CP]=prop_coeffs(J,rpm,p)
rpm=min(max(rpm,p.RPM(1)),p.RPM(end));
ir=min(find(p.RPM <= rpm,1,'last'),numel(p.RPM)-1);
fr=(rpm - p.RPM(ir))/(p.RPM(ir+1) - p.RPM(ir));
jj=J/(p.J(2)-p.J(1));
ij=min(floor(jj)+1,numel(p.J)-1);
fj=jj - (ij-1);
CT=(1-fr)*((1-fj)*p.CT(ir,ij)+fj*p.CT(ir,ij+1)) + fr*((1-fj)*p.CT(ir+1,ij)+fj*p.CT(ir+1,ij+1));
CP=(1-fr)*((1-fj)*p.CP(ir,ij)+fj*p.CP(ir,ij+1)) + fr*((1-fj)*p.CP(ir+1,ij)+fj*p.CP(ir+1,ij+1));
end
function r=make_result(T,RPM,J,eta_p,Ps,Im,Vm,P,OCV,Rs,V,limited)
r.T=T; r.RPM=RPM; r.J=J; r.eta_p=eta_p; r.P_shaft=Ps;
r.I_m=Im; r.V_m=Vm; r.P_elec=P; r.limited=limited;
r.V_b=(OCV + sqrt(max(OCV^2 - 4*Rs*P,0)))/2;
r.I_b=P/r.V_b;
r.d=Vm/r.V_b;
r.eta_total=0;
if P > 0,r.eta_total=T*V/P; end
end
