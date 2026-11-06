function md=FM3_Modes(V,h,ac,c,in)
t=FM2_Trim(V,h,0,0,ac.m,ac);
al=t.alpha*pi/180;
pp=struct('T',t.T,'Q',0,'n',0,'eps',0,'qs',0.5*FM1_Atmosphere(h)*V^2,'fv',0,'zv',0,'fh',0);
s0=[0;0;-(h+50);V*cos(al);0;V*sin(al);0;0;0;0;al;0];
u=struct('de',t.de*pi/180,'da',0,'dr',0,'hub',0,'dt',0);
idx=[4 5 6 7 8 9 10 11];
n=numel(idx); A=zeros(n);
for k=1:n
    d=zeros(12,1); d(idx(k))=1e-4;
    f1=FM3_Deriv(s0+d,u,pp,ac.m,ac,c,in); f2=FM3_Deriv(s0-d,u,pp,ac.m,ac,c,in);
    A(:,k)=(f1(idx)-f2(idx))/2e-4;
end
lg=[1 3 5 8]; lt=[2 4 6 7];
el=eig(A(lg,lg)); [~,i]=sort(abs(el),'descend'); el=el(i);
md.sp=el(1); md.ph=el(3);
et=eig(A(lt,lt));
ic=find(abs(imag(et)) > 1e-6);
ir=find(abs(imag(et)) <= 1e-6);
md.dr=et(ic(1));
[~,j]=sort(abs(et(ir)),'descend');
md.roll=et(ir(j(1))); md.spiral=et(ir(j(2)));
end
