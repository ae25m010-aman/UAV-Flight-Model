function t=FM2_Trim(V,h,gamma_deg,phi_deg,m,ac)
g=9.81; rho=FM1_Atmosphere(h); qb=0.5*rho*V^2;
gam=gamma_deg*pi/180; phi=phi_deg*pi/180;
alpha=0.05; de=0; T=5;
for i=1:40
    CL=(m*g*cos(gam) - T*sin(alpha))/(qb*ac.S*cos(phi));
    de=-(ac.Cm0 + ac.Cma*alpha + T*ac.arm_T/(qb*ac.S*ac.c))/ac.Cmde;
    alpha=(CL - ac.CL0 - ac.CLde*de)/ac.CLa;
    D=qb*ac.S*(ac.CD0 + ac.K*CL^2);
    T=(D + m*g*sin(gam))/cos(alpha);
end
t=struct('alpha',alpha*180/pi,'de',de*180/pi,'T',T,'CL',CL,'CD',ac.CD0+ac.K*CL^2,'theta',(alpha+gam)*180/pi,'V',V);
end
