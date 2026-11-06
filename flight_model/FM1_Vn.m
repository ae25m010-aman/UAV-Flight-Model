function vn=FM1_Vn(ac,in,pf,ms)
g=9.81; W=ac.m*g;
rho=pf.rho; rho0=FM1_Atmosphere(0);
n_pos=in.vn.n_pos;
if isempty(n_pos)
    n_pos=ac.CL_max*0.5*rho0*in.V_cruise^2*ac.S/W;
end
vn.rho=rho; vn.n_pos=n_pos; vn.n_neg=in.vn.n_neg;
vn.CL_max=ac.CL_max; vn.CL_neg=in.vn.CL_max_neg;
vn.V_S=sqrt(2*W/(rho*ac.S*ac.CL_max));
vn.V_corner=vn.V_S*sqrt(vn.n_pos);
vn.V_C=in.V_cruise;
vn.V_L=ms.V_loiter;
vn.V_A=vn.V_S*sqrt(vn.n_pos);
vn.V_D=in.vn.k_dive*in.V_cruise;
vn.V_Sneg=sqrt(2*W/(rho*ac.S*abs(vn.CL_neg)));
vn.V_G=vn.V_Sneg*sqrt(abs(vn.n_neg));
V_a=min(vn.V_corner,vn.V_D); V_g=min(vn.V_G,vn.V_D);
v1=linspace(0,V_a,60); n1=0.5*rho*v1.^2*ac.S*ac.CL_max/W; n1=min(n1,vn.n_pos);
v2=linspace(V_g,0,60); n2=0.5*rho*v2.^2*ac.S*vn.CL_neg/W; n2=max(n2,vn.n_neg);
V_e=min(max(vn.V_C,V_g),vn.V_D);
vn.env_V=[v1 vn.V_D vn.V_D V_e v2];
vn.env_n=[n1 vn.n_pos 0 vn.n_neg n2];
wl=W/ac.S; a=ac.CLa;
mu=2*wl/(rho*ac.c*a*g);
vn.Kg=0.88*mu/(5.3+mu);
dn=@(V,U) vn.Kg*rho0*U*(V*sqrt(rho/rho0))*a/(2*wl);
vn.n_gC=1 + dn(vn.V_C,in.vn.U_gust_C);
vn.n_gD=1 + dn(vn.V_D,in.vn.U_gust_D);
vn.n_gCm=1 - dn(vn.V_C,in.vn.U_gust_C);
vn.n_gDm=1 - dn(vn.V_D,in.vn.U_gust_D);
vn.show_gust=in.vn.show_gust;
L=ms.log;
r=FM1_Atmosphere(max(L.h,0));
vn.n_log=0.5*r.*L.V.^2*ac.S.*L.CL./(L.m*g);
vn.outside=L.CL > ac.CL_max + 1e-6 | vn.n_log > vn.n_pos | L.V > vn.V_D;
end
