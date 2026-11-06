function FM2_Video(ms,ac,in,g,vn,cc)
L=ms.log; tr=FM2_Track(ms,in);
alpha=L.alpha; soc=L.SoC*100; th_air=L.theta;
edges=[0 cumsum(ms.ph_t)]; tf=[];
vsec=in.out.video_sec;
for k=1:9
    nf=round(vsec(k)*in.out.video_fps);
    tf=[tf linspace(edges(k),edges(k+1),nf)];
end
[tu,iu]=unique(L.t);
fi=@(y) interp1(tu,y(iu),tf,'linear','extrap');
Fr.h=max(fi(L.h),0); Fr.V=fi(L.V); Fr.T=fi(L.T); Fr.D=fi(L.D); Fr.P=fi(L.P); Fr.I=fi(L.I_b);
Fr.soc=fi(soc); Fr.CL=fi(L.CL); Fr.d=fi(L.d*100); Fr.RPM=fi(L.RPM); Fr.X=fi(tr.X); Fr.Y=fi(tr.Y);
Fr.psi=atan2(fi(sin(tr.psi)),fi(cos(tr.psi))); Fr.phi=fi(tr.phi); Fr.th=fi(th_air); Fr.Wh=fi(L.Wh_);
Fr.x=fi(L.x); Fr.a=fi(alpha); Fr.de=fi(L.de); Fr.n=fi(vn.n_log); Fr.out=fi(double(vn.outside)) > 0.5;
Fr.ph=zeros(size(tf));
for k=1:9,Fr.ph(tf>=edges(k))=k; end
Pw=[Fr.X' Fr.Y' Fr.h'];
if isempty(in.out.cam_xyz)
    cam_pos=[0.5*(min(Fr.X)+max(Fr.X)),min(Fr.Y)-0.5*(max(Fr.X)-min(Fr.X))-300,700];
else
    cam_pos=in.out.cam_xyz;
end
Wf=min(max(40 + 1.3*Fr.h,40),520);
Wf(Fr.ph==5)=max(Wf(Fr.ph==5),3.6*ms.loiter_radius);
in_orbit=fi(double(tr.orbit)) > 0.5;
Wf(in_orbit)=max(Wf(in_orbit),2.8*tr.R_d);
Wf=exp(move_mean(log(Wf),41));
wc=2*round(3*in.out.video_fps/2)+1;
Tc=[move_mean(Pw(:,1)',wc)' move_mean(Pw(:,2)',wc)' move_mean(Pw(:,3)',wc)'];
circ=move_mean(double(Fr.ph==5 | in_orbit),41)';
Tgt=repmat(1-circ,1,3).*Pw + repmat(circ,1,3).*Tc;
Dist=sqrt(sum((Tgt - repmat(cam_pos,size(Tgt,1),1)).^2,2))';
Sc=max(1,in.out.uav_frame_share*Wf/(g.b/1000));
parts=FM1_GeometryBuild(g,3);
Vb=[]; Fb=[]; Cb=[];
for k=1:numel(parts)
    if strcmp(parts(k).name,'Prop disc'),continue; end
    Fb=[Fb; parts(k).F + size(Vb,1)];
    Vb=[Vb; parts(k).V];
    Cb=[Cb; repmat(parts(k).col,size(parts(k).F,1),1)];
end
Vb=(Vb - repmat([g.Xcg 0 g.Zcg],size(Vb,1),1))/1000;
hub_b=[(-40-g.Xcg)/1000 0 (g.H/2-g.Zcg)/1000];
z_wheel=(g.z_main_contact - g.Zcg)/1000;
fig=figure('Color',[0.97 0.97 0.97],'Position',[20 40 1280 720],'MenuBar','none','Resize','off');
axG=axes('Position',[0.012 0.335 0.495 0.645],'Color',[0.78 0.86 0.68],'XTick',[],'YTick',[],'Box','on');
hold(axG,'on'); aspG=(0.645*720)/(0.495*1280);
patch('Parent',axG,'XData',[-1 1 1 -1],'YData',[-1 -1 1 1],'FaceColor',[0.78 0.86 0.68],'EdgeColor','none');
hSky=patch('Parent',axG,'XData',[0 0 0 0],'YData',[0 0 0 0],'FaceColor',[0.75 0.87 0.97],'EdgeColor','none');
hGrid=plot(axG,NaN,NaN,'-','Color',[0.62 0.72 0.54],'LineWidth',0.5);
hRwy=patch('Parent',axG,'XData',[0 0 0 0],'YData',[0 0 0 0],'FaceColor',[0.35 0.35 0.37],'EdgeColor','none');
hRwyC=plot(axG,NaN,NaN,'w--','LineWidth',1);
hFut=plot(axG,NaN,NaN,'-','Color',[0.55 0.55 0.55],'LineWidth',0.6);
c8=FM1_PhaseColors(9)*0.7; hPast=[];
for k=1:9,hPast(k)=plot(axG,NaN,NaN,'-','Color',c8(k,:),'LineWidth',1.8); end
hSh=plot(axG,NaN,NaN,'-','Color',[0.2 0.2 0.2],'LineWidth',1);
hUAV=patch('Parent',axG,'Vertices',[0 0],'Faces',1,'FaceVertexCData',[0 0 0],'FaceColor','flat','EdgeColor','none');
hPr=plot(axG,NaN,NaN,'-','Color',[0.1 0.1 0.1],'LineWidth',2);
hLr=text('Parent',axG,'Position',[0 0],'String','Runway','FontSize',9,'FontWeight','bold','Color',[0.15 0.15 0.15]);
hLd=text('Parent',axG,'Position',[0 0],'String','Delivery area','FontSize',9,'FontWeight','bold','Color',[0.15 0.15 0.15]);
axGt=axes('Position',[0.012 0.335 0.495 0.645],'Color','none'); axis(axGt,'off'); set(axGt,'Color','none'); xlim(axGt,[0 1]); ylim(axGt,[0 1]);
hTitle=text('Parent',axGt,'Position',[0.015 0.95],'String','','FontWeight','bold','FontSize',13,'Interpreter','none');
hCam=text('Parent',axGt,'Position',[0.015 0.04],'String','','FontSize',9,'Interpreter','none');
ax1=axes('Position',[0 0.015 0.165 0.31]); hold(ax1,'on');
hC=patch('Parent',ax1,'Vertices',Vb,'Faces',Fb,'FaceVertexCData',Cb,'FaceColor','flat','EdgeColor','none');
hGround=plot3(ax1,NaN,NaN,NaN,'-','Color',[0.35 0.55 0.35]);
axis(ax1,'equal'); xlim(ax1,[-1.15 1.15]); ylim(ax1,[-1.15 1.15]); zlim(ax1,[-0.7 0.7]);
view(ax1,[-125 16]); axis(ax1,'off');
hAtt=text('Parent',ax1,'Units','normalized','Position',[0.02 0.04],'String','','FontSize',8,'Interpreter','none');
ax2=axes('Position',[0.185 0.075 0.125 0.225]); hold(ax2,'on');
for k=1:9,i=L.phase==k; plot3(ax2,tr.X(i)/1000,tr.Y(i)/1000,L.h(i),'Color',min(c8(k,:)+0.25,1),'LineWidth',1); end
hTrail=plot3(ax2,NaN,NaN,NaN,'k','LineWidth',1.2);
hNow=plot3(ax2,NaN,NaN,NaN,'o','MarkerSize',5,'MarkerFaceColor','r','MarkerEdgeColor','k');
view(ax2,[-30 30]); grid(ax2,'on'); box(ax2,'on');
xlabel(ax2,'X (km)'); ylabel(ax2,'Y (km)'); zlabel(ax2,'h (m)'); set(ax2,'FontSize',7);
axN=axes('Position',[0.35 0.07 0.175 0.25]); hold(axN,'on');
FM1_VnDraw(axN,vn,false);
hNt=plot(axN,NaN,NaN,'-','Color',[0.05 0.25 0.6],'LineWidth',1);
hNp=plot(axN,NaN,NaN,'o','MarkerSize',6,'MarkerFaceColor',[0.1 0.7 0.2],'MarkerEdgeColor','k');
title(axN,'V-n diagram','FontSize',8);
vars={Fr.h,'Height (m)'; Fr.V,'Airspeed (m/s)'; Fr.th,'Pitch attitude (deg)'; Fr.a,'Angle of attack (deg)'; ...
        Fr.de,'Elevator (deg), + = nose down'; Fr.T,'Thrust (N)'; Fr.P,'Battery power (W)'; Fr.soc,'State of charge (%)'};
nv=size(vars,1); axv=[]; hv=[]; hc=[];
for k=1:nv
    col=mod(k-1,2); row=floor((k-1)/2);
    axv(k)=axes('Position',[0.555+col*0.225 0.725-row*0.178 0.19 0.118]); hold(axv(k),'on');
    y=vars{k,1}; yl=[min(y) max(y)]; pad=max(0.08*(yl(2)-yl(1)),1e-3); yl=yl + [-pad pad];
    axes(axv(k)); FM1_PhaseShade(ms,yl,[0 tf(end)],false);
    plot(axv(k),tf,y,'-','Color',[0.75 0.75 0.75]);
    hv(k)=plot(axv(k),NaN,NaN,'-','Color',[0.05 0.25 0.6],'LineWidth',1.6);
    hc(k)=plot(axv(k),[0 0],yl,'r-');
    xlim(axv(k),[0 tf(end)]); ylim(axv(k),yl); set(axv(k),'FontSize',7,'Layer','top'); grid(axv(k),'on'); box(axv(k),'on');
    title(axv(k),vars{k,2},'FontSize',9);
    if row < 3,set(axv(k),'XTickLabel',{}); end
end
axInfo=axes('Position',[0.53 0 0.46 0.135]); axis(axInfo,'off'); xlim(axInfo,[0 1]); ylim(axInfo,[0 1]); hold(axInfo,'on');
hInfo=text('Parent',axInfo,'Position',[0 0.62],'String','','FontSize',9,'FontName','FixedWidth','Interpreter','none');
c0=FM1_PhaseColors(9); short={'Roll','Air','Climb','Cruise','Loiter','Back','Descent','Approach','Land'};
hBox=[];
for k=1:9
    x0=(k-1)/9;
    hBox(k)=patch('Parent',axInfo,'XData',x0+[0 .105 .105 0],'YData',[0.02 0.02 .27 .27],'FaceColor',c0(k,:),'EdgeColor',[.6 .6 .6]);
    text('Parent',axInfo,'Position',[x0+.0525 .145],'String',sprintf('%d %s',k,short{k}),'HorizontalAlignment','center','FontSize',7);
end
h3=axes('Position',[0.53 0.885 0.46 0.09]); axis(h3,'off');
hHead=text('Parent',h3,'Position',[0 0.5],'String','','FontSize',14,'FontWeight','bold','Interpreter','none');
vw=[]; frames_dir='';
base=fullfile(in.out.folder,'FM2_Mission_Simulation');
try
    vw=VideoWriter(base,'MPEG-4');
catch
    try
        vw=VideoWriter(base,'Motion JPEG AVI');
        vw.Quality=in.out.video_quality;
    catch
        vw=[];
    end
end
if ~isempty(vw)
    vw.FrameRate=in.out.video_fps;
    open(vw);
else
    frames_dir=fullfile(in.out.folder,'FM2_Video_Frames');
    [~,~,~]=mkdir(frames_dir);
    fprintf('VideoWriter is not available here: saving video frames as PNG in %s\n',frames_dir);
end
Lr=800; rwy=[-100 -15 0; Lr -15 0; Lr 15 0; -100 15 0];
rwyC=[-100 0 0; Lr 0 0];
i4=find(L.phase==5); deliv=[mean(tr.X(i4)) mean(tr.Y(i4)) 0];
nF=numel(tf); ang=0; lit=[0.4 -0.55 0.73];
for f=1:nF
    th=Fr.th(f)*pi/180; ph=Fr.phi(f); ps=Fr.psi(f);
    F0=[cos(th)*cos(ps) cos(th)*sin(ps) sin(th)];
    R0=[sin(ps) -cos(ps) 0]; U0=cross(R0,F0);
    Rt=R0*cos(ph) - U0*sin(ph); Ut=U0*cos(ph) + R0*sin(ph);
    cam=aim_camera(cam_pos,Tgt(f,:));
    half=Wf(f)/(2*Dist(f));
    xlim(axG,[-half half]); ylim(axG,[-half half]*aspG);
    [~,vh]=cam_project(cam_pos + 1e7*[cam.f(1) cam.f(2) 0]/max(norm(cam.f(1:2)),1e-6),cam);
    set(hSky,'XData',[-1 1 1 -1],'YData',[vh vh 1 1]);
    sp=nice_step(Wf(f)/5);
    gx0=round(Tgt(f,1)/sp)*sp; gy0=round(Tgt(f,2)/sp)*sp; nl=22;
    span=min(sp*nl,1500);
    xs=gx0 + (-nl:nl)*sp; ys=gy0 + (-nl:nl)*sp;
    G=nan(3*numel(xs)+3*numel(ys),3); n=0;
    for a=xs,G(n+1,:)=[a gy0-span 0]; G(n+2,:)=[a gy0+span 0]; n=n+3; end
    for a=ys,G(n+1,:)=[gx0-span a 0]; G(n+2,:)=[gx0+span a 0]; n=n+3; end
    [u,v]=cam_project(G,cam); set(hGrid,'XData',u,'YData',v);
    [u,v]=cam_project(rwy,cam); set(hRwy,'XData',u,'YData',v);
    [u,v]=cam_project(rwyC,cam); set(hRwyC,'XData',u,'YData',v);
    [u,v]=cam_project([Lr*0.5 -40 0],cam); place_label(hLr,u,v,half,aspG);
    [u,v]=cam_project(deliv,cam); place_label(hLd,u,v,half,aspG);
    [u,v]=cam_project(Pw,cam);
    set(hFut,'XData',u,'YData',v);
    for k=1:9
        i=Fr.ph==k & (1:nF)<=f;
        set(hPast(k),'XData',u(i),'YData',v(i));
    end
    s=Sc(f);
    W_b=-Vb(:,1)*F0 + Vb(:,2)*Rt + Vb(:,3)*Ut;
    lift=-min(W_b(:,3));
    Pos=Pw(f,:) + [0 0 s*lift];
    Pts=s*W_b + repmat(Pos,size(W_b,1),1);
    [u,v,w]=cam_project(Pts,cam);
    d1=W_b(Fb(:,3),:) - W_b(Fb(:,1),:); d2=W_b(Fb(:,4),:) - W_b(Fb(:,2),:);
    nrm=cross(d1,d2,2); nn=sqrt(sum(nrm.^2,2)); nn(nn<1e-9)=1;
    shade=0.55 + 0.45*abs((nrm(:,1)*lit(1)+nrm(:,2)*lit(2)+nrm(:,3)*lit(3))./nn);
    depth=(w(Fb(:,1))+w(Fb(:,2))+w(Fb(:,3))+w(Fb(:,4)))/4;
    [~,ord]=sort(depth,'descend');
    C=Cb.*repmat(shade,1,3);
    set(hUAV,'Vertices',[u v],'Faces',Fb(ord,:),'FaceVertexCData',C(ord,:));
    [u,v]=cam_project([Pos; Pos(1:2) 0],cam); set(hSh,'XData',u,'YData',v);
    if Fr.RPM(f) > 1,ang=ang + 0.9; end
    hub=-hub_b(1)*F0 + hub_b(3)*Ut;
    bl=(g.prop_D/2000)*(cos(ang)*Rt + sin(ang)*Ut);
    [u,v]=cam_project([Pos+s*(hub-bl); Pos+s*(hub+bl)],cam); set(hPr,'XData',u,'YData',v);
    set(hTitle,'String',sprintf('3-DOF outside view: %s',ms.phase{Fr.ph(f)}));
    set(hCam,'String',sprintf('Camera to UAV %.0f m  |  Frame width %.0f m  |  UAV drawn x%.1f',norm(Pw(f,:)-cam_pos),Wf(f),s));
    set(hC,'Vertices',W_b);
    zg=-lift - Fr.h(f);
    if zg > -0.7
        off=mod(Fr.x(f),0.5); gx=[]; gy=[]; gz=[];
        for a=(-1.5:0.5:1.5) - off,gx=[gx a a NaN]; gy=[gy -1.5 1.5 NaN]; gz=[gz zg zg NaN]; end
        for a=-1.5:0.5:1.5,gx=[gx -1.5 1.5 NaN]; gy=[gy a a NaN]; gz=[gz zg zg NaN]; end
        set(hGround,'XData',gx,'YData',gy,'ZData',gz);
    else
        set(hGround,'XData',NaN,'YData',NaN,'ZData',NaN);
    end
    set(hAtt,'String',sprintf('Pitch %.1f  Bank %.1f  Alpha %.1f  Elev %.1f deg',Fr.th(f),ph*180/pi,Fr.a(f),Fr.de(f)));
    set(hTrail,'XData',Fr.X(1:f)/1000,'YData',Fr.Y(1:f)/1000,'ZData',Fr.h(1:f));
    set(hNow,'XData',Fr.X(f)/1000,'YData',Fr.Y(f)/1000,'ZData',Fr.h(f));
    set(hNt,'XData',Fr.V(1:f),'YData',Fr.n(1:f));
    if Fr.out(f),cn=[0.9 0.1 0.1]; else,cn=[0.1 0.7 0.2]; end
    set(hNp,'XData',Fr.V(f),'YData',Fr.n(f),'MarkerFaceColor',cn);
    for k=1:nv
        y=vars{k,1};
        set(hv(k),'XData',tf(1:f),'YData',y(1:f));
        set(hc(k),'XData',tf(f)*[1 1]);
    end
    set(hHead,'String',sprintf('Phase %d of 9: %s    t = %.1f s',Fr.ph(f),ms.phase{Fr.ph(f)},tf(f)));
    warn=''; if Fr.soc(f) < in.batt.SoC_min*100,warn='  BELOW RESERVE'; end
    set(hInfo,'String',sprintf(['h %6.1f m    V %5.2f m/s   T %5.2f N   D %5.2f N\n' ...
        'P %6.1f W    I %5.2f A     RPM %5.0f    Thr %3.0f %%\n' ...
        'SoC %5.1f %%   Used %5.1f Wh   CL %4.2f   n %4.2f%s'],Fr.h(f),Fr.V(f),Fr.T(f),Fr.D(f),Fr.P(f),Fr.I(f),Fr.RPM(f),Fr.d(f),Fr.soc(f),Fr.Wh(f),Fr.CL(f),Fr.n(f),warn));
    for k=1:9
        if k == Fr.ph(f),set(hBox(k),'EdgeColor',[0.8 0 0],'LineWidth',2); else,set(hBox(k),'EdgeColor',[.6 .6 .6],'LineWidth',0.5); end
    end
    drawnow;
    if ~isempty(vw)
        writeVideo(vw,getframe(fig));
    else
        print(fig,'-dpng','-r96',fullfile(frames_dir,sprintf('frame_%05d.png',f)));
    end
    if mod(f,100)==0,fprintf('Video frame %d of %d\n',f,nF); end
end
if ~isempty(vw),close(vw); end
fid=fopen(fullfile(in.out.folder,in.out.list_name),'a');
fprintf(fid,'%-28s %s\n','FM2_Mission_Simulation','3-DOF mission video: outside camera view and UAV close-up on the left, live parameters on the right');
fclose(fid);
if ~in.out.keep_open,close(fig); end
end
function cam=aim_camera(pos,tgt)
fw=tgt - pos; fw=fw/norm(fw);
rt=cross(fw,[0 0 1]); rt=rt/norm(rt);
cam.pos=pos; cam.f=fw; cam.r=rt; cam.up=cross(rt,fw);
end
function [u,v,w]=cam_project(P,cam)
d=P - repmat(cam.pos,size(P,1),1);
w=d*cam.f';
u=(d*cam.r')./w;
v=(d*cam.up')./w;
behind=w < 1;
u(behind)=NaN; v(behind)=NaN;
end
function place_label(h,u,v,half,asp)
if isfinite(u) && abs(u) < 0.8*half && abs(v) < 0.9*half*asp
    set(h,'Position',[u v],'Visible','on');
else
    set(h,'Visible','off');
end
end
function y=move_mean(x,n)
m=numel(x); h=floor(n/2); y=zeros(size(x));
cs=[0 cumsum(x(:)')];
for i=1:m
    a=max(i-h,1); b=min(i+h,m);
    y(i)=(cs(b+1)-cs(a))/(b-a+1);
end
end
function s=nice_step(x)
p=10^floor(log10(x)); r=x/p;
if r < 1.5,s=p; elseif r < 3.5,s=2*p; elseif r < 7.5,s=5*p; else,s=10*p; end
end
