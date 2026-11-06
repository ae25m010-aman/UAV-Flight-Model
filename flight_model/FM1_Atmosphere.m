function [rho,T,a]=FM1_Atmosphere(h)
T=288 - 0.0065*h;
rho=1.225*(T/288).^4.2559;
a=sqrt(1.4*287*T);
end
