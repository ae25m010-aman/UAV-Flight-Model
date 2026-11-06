function md=FM2_Modes(V,h,m,ac,c,in)
t=FM2_Trim(V,h,0,0,m,ac);
th=t.alpha*pi/180;
x0=[V; 0; th; 0];
f=@(x) sub4(x,h,t.de*pi/180,t.T,m,ac,c,in);
A=zeros(4); e=[1e-3 1e-3 1e-4 1e-4];
for k=1:4
    d=zeros(4,1); d(k)=e(k);
    A(:,k)=(f(x0+d) - f(x0-d))/(2*e(k));
end
ev=eig(A);
[~,i]=sort(abs(ev),'descend'); ev=ev(i);
md.A=A; md.eig=ev;
md.wn_sp=abs(ev(1)); md.z_sp=-real(ev(1))/abs(ev(1));
md.wn_ph=abs(ev(3)); md.z_ph=-real(ev(3))/abs(ev(3));
md.trim=t;
end
function y=sub4(x,h,de,T,m,ac,c,in)
s=[0; h+1000; x(1); x(2); x(3); x(4)];
s(2)=h;
[sd,~]=FM2_Deriv(s,de,T,0,m,ac,c,in);
y=sd(3:6);
end
