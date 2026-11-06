clc
clear
close all
plot_results=true;
save_output=true;
output_file='wing_design_output.txt';
save_figs=true;
fig_folder='wing_figures';
fig_dpi=300;
save_fig_files=true;
if save_output
    diary off
    if exist(output_file,'file'),delete(output_file); end
    diary(output_file)
end
c_full=280;
c_box=240;
t=0.5;
rib_t=0.5;
N0=500;
E=69000;
nu=0.33;
x_AC=0.25*c_full;
spar1_x=80;
spar1_flange_L=[20 40];
spar1_rib_range=[1 2];
spar_t=1;
clip_rib_range=[3 4];
sigma_allow_skin=95/6.75;
sigma_allow_spar=276/6.75;
sigma_allow_str=95/6.75;
A_lump=10;
A_str=A_lump;
stringers_initial=[
    0.1714286 1
    0.44642857143 1
    0.7285714 1
    0.6642857 -1
    0.1285714 -1
];
semi_span=0.84;
rib_y=[0;0.28;0.56;0.84];
num_ribs=length(rib_y);
num_bays=num_ribs-1;
data_y=0:0.02:0.84;
Lift_Span=[17.489;22.315;24.272;25.746;26.965;28.017;28.948;29.787;30.55;31.251;31.899;32.499;33.059;33.582;34.072;34.531;34.961;35.366;35.747;36.104;36.441;36.756;37.052;37.33;37.59;37.833;38.059;38.27;38.465;38.644;38.809;38.96;39.097;39.22;39.33;39.426;39.509;39.579;39.636;39.68;39.712;39.731;39.737];
SF=[0.35;0.796;1.282;1.796;2.336;2.896;3.475;4.071;4.682;5.307;5.945;6.595;7.256;7.928;8.609;9.3;9.999;10.706;11.421;12.143;12.872;13.607;14.348;15.095;15.847;16.603;17.364;18.13;18.899;19.672;20.448;21.227;22.009;22.794;23.58;24.369;25.159;25.951;26.743;27.537;28.331;29.126;29.922];
BM=[0.002;0.003497;0.023881;0.064075;0.12575;0.210215;0.318566;0.451753;0.610614;0.7959;1.00829;1.2484;1.516793;1.813984;2.140446;2.496612;2.882882;3.299619;3.74716;4.225809;4.735846;5.277522;5.851065;6.45668;7.094548;7.764827;8.467655;9.203148;9.971403;10.77249;11.60648;12.47339;13.37326;14.30607;15.2718;16.27043;17.30188;18.36608;19.46294;20.59235;21.75416;22.94823;24.17438];
Lift_Span=flip(Lift_Span); SF=flip(SF); BM=flip(BM);
W_total=6.87991*9.81;
load_scale=(W_total/2)/SF(1);
Lift_Span=Lift_Span*load_scale; SF=SF*load_scale; BM=BM*load_scale;
M_from_shear=false;
if M_from_shear
    BM_int=cumtrapz(data_y(:),SF(:));
    BM=BM_int(end)-BM_int;
end
V_rib=interp1(data_y,SF,rib_y,'pchip');
M_rib=interp1(data_y,BM,rib_y,'pchip');
M_rib_Nmm=M_rib*1000;
rib_type=cell(1,num_ribs);
for r=1:num_ribs
    has1=r >= spar1_rib_range(1) && r <= spar1_rib_range(2);
    inclip=r >= clip_rib_range(1) && r <= clip_rib_range(2);
    if has1,rib_type{r}='front';
    elseif inclip && r == clip_rib_range(2),rib_type{r}='clip';
    else,rib_type{r}='none';
    end
end
bay_type=cell(1,num_bays);
for k=1:num_bays
    has1=k >= spar1_rib_range(1) && (k+1) <= spar1_rib_range(2);
    inclip=k >= clip_rib_range(1) && (k+1) <= clip_rib_range(2);
    if has1,bay_type{k}='front';
    elseif inclip,bay_type{k}='clip';
    else,bay_type{k}='none';
    end
end
af_full=[
0 0
0.0005 0.002339
0.001 0.0037271
0.002 0.0058025
0.004 0.0089238
0.008 0.013735
0.012 0.0178581
0.02 0.0253735
0.03 0.0330215
0.04 0.0391283
0.05 0.0442753
0.06 0.0487571
0.08 0.0564308
0.1 0.0629981
0.12 0.0686204
0.14 0.073436
0.16 0.0775707
0.18 0.0810687
0.2 0.0839202
0.22 0.0861433
0.24 0.0878308
0.26 0.089084
0.28 0.0900016
0.3 0.0906804
0.32 0.0911857
0.34 0.0915079
0.36 0.0916266
0.38 0.0915212
0.4 0.0911712
0.42 0.0905657
0.44 0.0897175
0.46 0.0886427
0.48 0.0873572
0.5 0.0858772
0.52 0.0842145
0.54 0.0823712
0.56 0.080348
0.58 0.0781451
0.6 0.0757633
0.62 0.0732055
0.64 0.0704822
0.66 0.0676046
0.68 0.0645843
0.7 0.0614329
0.72 0.0581599
0.74 0.0547675
0.76 0.0512565
0.78 0.0476281
0.8 0.0438836
0.82 0.0400245
0.84 0.0360536
0.86 0.031974
0.88 0.0277891
0.9 0.0235025
0.92 0.0191156
0.94 0.0146239
0.96 0.0100232
0.97 0.0076868
0.98 0.0053335
0.99 0.002969
1 0
0.99 -0.0009666
0.98 -0.0013339
0.97 -0.0017011
0.96 -0.0020683
0.94 -0.0028028
0.92 -0.0035373
0.9 -0.0042718
0.88 -0.0050063
0.86 -0.0057408
0.84 -0.0064753
0.82 -0.0072098
0.8 -0.0079443
0.78 -0.0086788
0.76 -0.0094133
0.74 -0.0101478
0.72 -0.0108823
0.7 -0.0116169
0.68 -0.0123515
0.66 -0.0130862
0.64 -0.0138207
0.62 -0.0145551
0.6 -0.0152893
0.58 -0.0160232
0.56 -0.0167572
0.54 -0.0174914
0.52 -0.0182262
0.5 -0.0189619
0.48 -0.0196986
0.46 -0.0204353
0.44 -0.0211708
0.42 -0.0219042
0.4 -0.0226341
0.38 -0.0233606
0.36 -0.024087
0.34 -0.0248176
0.32 -0.0255565
0.3 -0.0263079
0.28 -0.0270696
0.26 -0.0278164
0.24 -0.0285181
0.22 -0.0291445
0.2 -0.0296656
0.18 -0.030049
0.16 -0.0302546
0.14 -0.0302404
0.12 -0.0299633
0.1 -0.0293786
0.08 -0.0284595
0.06 -0.0271277
0.05 -0.0260452
0.04 -0.0245211
0.03 -0.0226056
0.02 -0.0202723
0.012 -0.0169733
0.008 -0.0142862
0.004 -0.0105126
0.002 -0.0078113
0.001 -0.0059418
0.0005 -0.00467
0 0
];
xn_full=af_full(:,1);
yn_full=af_full(:,2);
P=struct('c_full',c_full,'c_box',c_box,'t',t,'N0',N0,'E',E,'nu',nu,'x_AC',x_AC,...
    'spar1_x',spar1_x,'fl1_L',spar1_flange_L,'spar_t',spar_t,...
    'xn',xn_full,'yn',yn_full,'V_rib',V_rib,'rib_y',rib_y,...
    'bay_type',{bay_type});
fprintf('ALLOWABLES: skin/stringer = %.3f MPa (95/6.75), spar web/flange = %.3f MPa (276/6.75)\n',sigma_allow_skin,sigma_allow_spar);
fprintf('Stringers idealised as lumped booms, A_lump = %.3f mm^2 each\n',A_lump);
fprintf('Spar 1 at x=%.1f mm, present rib %d to rib %d\n',spar1_x,spar1_rib_range(1),spar1_rib_range(2));
fprintf('Between rib %d and rib %d the box is clipped to c_box = %.0f mm (aileron cut away); the closing fold is plain skin, not a spar\n',clip_rib_range(1),clip_rib_range(2),c_box);
fprintf('rib type   : %s\n',strjoin(rib_type,', '));
fprintf('bay type   : %s\n',strjoin(bay_type,', '));
sec_front_ns=build_section('front',P,[],0);
sec_none_ns=build_section('none',P,[],0);
sec_clip_ns=build_section('clip',P,[],0);
fprintf('front section (spar 1 only), no stringers: xc=%.3f yc=%.3f Ixx=%.4e Iyy=%.4e Ixy=%.4e\n',sec_front_ns.xc,sec_front_ns.yc,sec_front_ns.Ixx,sec_front_ns.Iyy,sec_front_ns.Ixy);
fprintf('none  section (no spar, full chord),  no stringers: xc=%.3f yc=%.3f Ixx=%.4e Iyy=%.4e Ixy=%.4e\n',sec_none_ns.xc,sec_none_ns.yc,sec_none_ns.Ixx,sec_none_ns.Iyy,sec_none_ns.Ixy);
fprintf('clip  section (no spar, clipped to c_box), no stringers: xc=%.3f yc=%.3f Ixx=%.4e Iyy=%.4e Ixy=%.4e\n',sec_clip_ns.xc,sec_clip_ns.yc,sec_clip_ns.Ixx,sec_clip_ns.Iyy,sec_clip_ns.Ixy);
stringers=stringers_initial;
max_stringer_iterations=50;
for opt_iter=1:max_stringer_iterations
    an=analyze_wing(P,stringers,A_str);
    nst=size(stringers,1);
    [failed,worst_ratio,worst_bay,worst_panel]=check_buckling(an,P,stringers,false);
    if ~failed
        fprintf('\nstringer layout converged after %d iterations, %d stringers, worst tau/Pcr = %.4f\n',opt_iter,nst,worst_ratio);
        break
    end
    if worst_panel < 0
        fprintf('iteration %2d: spar web governs in bay %d (tau/Pcr=%.3f) - stringers cannot fix it\n',opt_iter,worst_bay,worst_ratio);
        break
    end
    sec=an.(['sec_' bay_type{worst_bay}]);
    [~,st_s]=map_all_stringers(stringers,sec.x_node,c_full,sec.idx_upper,sec.idx_lower,sec.s_nodes);
    k=worst_panel; k_next=mod(k,nst)+1;
    if k < nst
        s_new=0.5*(st_s(k)+st_s(k_next));
    else
        s_new=0.5*(st_s(k)+(st_s(1)+sec.perim));
        if s_new >= sec.perim,s_new=s_new-sec.perim; end
    end
    [~,new_node]=min(abs(sec.s_nodes-s_new));
    if ismember(new_node,sec.idx_upper),surf_new=1; else,surf_new=-1; end
    x_new=sec.x_node(new_node)/c_full;
    dup=false;
    for j=1:size(stringers,1)
        if abs(stringers(j,1)-x_new) < 1e-3 && stringers(j,2) == surf_new,dup=true; break; end
    end
    if dup,x_new=x_new-0.01; end
    stringers=[stringers; x_new surf_new];
    fprintf('iteration %2d: added stringer at x/c=%.4f surf=%+d, bay %d (%s) governed (tau/Pcr=%.3f)\n',opt_iter,x_new,surf_new,worst_bay,bay_type{worst_bay},worst_ratio);
end
an=analyze_wing(P,stringers,A_str);
nst=size(stringers,1);
rib_sec=cell(1,num_ribs);
rib_q=cell(1,num_ribs);
rib_qw=cell(1,num_ribs);
rib_xSC=zeros(1,num_ribs);
for r=1:num_ribs
    ty=rib_type{r};
    rib_sec{r}=an.(['sec_' ty]);
    rib_q{r}=an.(['q_' ty])(:,r);
    rib_qw{r}=an.(['qw_' ty])(:,r);
    rib_xSC(r)=an.(['x_SC_' ty]);
end
sec_front=an.sec_front; sec_none=an.sec_none; sec_clip=an.sec_clip;
fprintf('\nfinal stringer locations (x/c, surface)\n');
for k=1:nst
    if stringers(k,2) == 1,lbl='upper'; else,lbl='lower'; end
    fprintf('%8d %10.4f %8s\n',k,stringers(k,1),lbl);
end
fprintf('\nGEOMETRY CHECK (nodes found by coordinates)\n');
types_all={'front','none','clip'}; names_all={'front (spar 1 only)','none (no spar, full chord)','clip (no spar, clipped to c_box)'};
for m=1:3
    s=an.(['sec_' types_all{m}]);
    fprintf('%s: perimeter = %.2f mm, cells = %d\n',names_all{m},s.perim,numel(s.Lcell_t));
    if s.has1
        fprintf('  spar1 web top(%.3f,%.3f) bottom(%.3f,%.3f) tilt=%.4f deg, length=%.3f mm, flange area [up,lo]=%.2f %.2f mm^2 over %d %d nodes\n',...
            s.x_node(s.idxS1_upper),s.y_node(s.idxS1_upper),s.x_node(s.idxS1_lower),s.y_node(s.idxS1_lower),...
            atan2d(s.x_node(s.idxS1_upper)-s.x_node(s.idxS1_lower),s.y_node(s.idxS1_upper)-s.y_node(s.idxS1_lower)),s.web_len1,...
            s.fl_area_tot(1),s.fl_area_tot(2),numel(s.fl_nodes{1}),numel(s.fl_nodes{2}));
    end
    if strcmp(s.stype,'clip')
        fprintf('  closing fold (plain skin, no spar) top(%.3f,%.3f) bottom(%.3f,%.3f) length=%.3f mm\n',...
            s.x_node(s.idx_fold_upper),s.y_node(s.idx_fold_upper),s.x_node(s.idx_fold_lower),s.y_node(s.idx_fold_lower),s.fold_len);
    end
end
fprintf('\nfront section (final): xc=%.3f yc=%.3f Ixx=%.4e Iyy=%.4e Ixy=%.4e, shear centre = %.3f mm\n',sec_front.xc,sec_front.yc,sec_front.Ixx,sec_front.Iyy,sec_front.Ixy,an.x_SC_front);
fprintf('none  section (final): xc=%.3f yc=%.3f Ixx=%.4e Iyy=%.4e Ixy=%.4e, shear centre = %.3f mm\n',sec_none.xc,sec_none.yc,sec_none.Ixx,sec_none.Iyy,sec_none.Ixy,an.x_SC_none);
fprintf('clip  section (final): xc=%.3f yc=%.3f Ixx=%.4e Iyy=%.4e Ixy=%.4e, shear centre = %.3f mm\n',sec_clip.xc,sec_clip.yc,sec_clip.Ixx,sec_clip.Iyy,sec_clip.Ixy,an.x_SC_clip);
fprintf('\nFINAL PANEL BUCKLING TABLE\n');
fprintf('%5s %8s %10s %8s %8s %10s %12s %12s %10s %8s\n','Bay','Type','Member','a(mm)','b(mm)','q(N/mm)','tau(MPa)','Pcr(MPa)','MS','Status');
[~,~,~,~,brows]=check_buckling(an,P,stringers,true);
vm_all=cell(num_ribs,1);
vm_tab=zeros(num_ribs,6);
vm_tab(:,5)=NaN;
ratio_max=-inf; ratio_where='';
vm_max=-inf; vm_where='';
for r=1:num_ribs
    sec=rib_sec{r}; q_col=rib_q{r}; qw=rib_qw{r};
    x_bar=sec.x_node-sec.xc; y_bar=sec.y_node-sec.yc;
    denom=sec.Ixx*sec.Iyy-sec.Ixy^2;
    sig=M_rib_Nmm(r)*(sec.Iyy*y_bar-sec.Ixy*x_bar)/denom;
    tau=q_col./sec.t_seg;
    vm=sqrt(sig.^2+3*tau.^2);
    vm_all{r}=vm;
    allow=sigma_allow_skin*ones(sec.N,1);
    allow(sec.spar_mask)=sigma_allow_spar;
    ratio=vm./allow;
    [rm,in]=max(ratio);
    rib_ratio=rm;
    vm_tab(r,1)=max(abs(sig));
    vm_tab(r,2)=max(vm(~sec.spar_mask));
    vm_tab(r,3)=ternary(any(sec.spar_mask),max(vm(sec.spar_mask)),NaN);
    if rm > ratio_max
        ratio_max=rm;
        if sec.spar_mask(in),tag='spar flange/end face'; else,tag='skin/stringer'; end
        ratio_where=sprintf('rib %d (%s), node %d (%s): vm=%.3f MPa, allowable=%.3f MPa',r,rib_type{r},in,tag,vm(in),allow(in));
    end
    [vmx,in2]=max(vm);
    if vmx > vm_max
        vm_max=vmx;
        vm_where=sprintf('rib %d (y=%.3f m, %s), node %d',r,rib_y(r),rib_type{r},in2);
    end
    for w=1:size(sec.web_nodes,1)
        iu=sec.web_nodes(w,1); il=sec.web_nodes(w,2);
        len=hypot(sec.x_node(iu)-sec.x_node(il),sec.y_node(iu)-sec.y_node(il));
        npt=max(ceil(len/mean(sec.b_seg)),3)+1;
        tt=linspace(0,1,npt)';
        xw=sec.x_node(iu)+tt*(sec.x_node(il)-sec.x_node(iu));
        yw=sec.y_node(iu)+tt*(sec.y_node(il)-sec.y_node(iu));
        sw=M_rib_Nmm(r)*(sec.Iyy*(yw-sec.yc)-sec.Ixy*(xw-sec.xc))/denom;
        tw=qw(w)/spar_t;
        vmw=sqrt(sw.^2+3*tw^2);
        [vwm,iw]=max(vmw);
        vm_tab(r,4)=vwm;
        rib_ratio=max(rib_ratio,vwm/sigma_allow_spar);
        if vwm/sigma_allow_spar > ratio_max
            ratio_max=vwm/sigma_allow_spar;
            ratio_where=sprintf('rib %d (%s), spar 1 web, point %d: vm=%.3f MPa, allowable=%.3f MPa',r,rib_type{r},iw,vwm,sigma_allow_spar);
        end
        if vwm > vm_max
            vm_max=vwm; vm_where=sprintf('rib %d (y=%.3f m, %s), spar 1 web',r,rib_y(r),rib_type{r});
        end
    end
    vm_tab(r,6)=rib_ratio;
end
fprintf('\nVON MISES STRESS CHECK\n');
fprintf('allowables: skin/stringer = %.3f MPa, spar web/flange = %.3f MPa\n',sigma_allow_skin,sigma_allow_spar);
fprintf('largest von Mises stress anywhere = %.3f MPa at %s\n',vm_max,vm_where);
fprintf('governing stress ratio vm/allowable = %.4f to %s\n',ratio_max,ratio_where);
fprintf('margin of safety = %.3f to %s\n',1/ratio_max-1,ternary(ratio_max<1,'no failure predicted','exceeds allowable, redesign needed'));
if plot_results
    col_s1=[0 0.3 0.9];
    for r=1:num_ribs
        figure; hold on; grid on; axis equal
        [H,L]=draw_section(rib_sec{r},rib_xSC(r),stringers,P);
        legend(H,L,'Location','best');
        title(sprintf('Rib %d section (y = %.2f m, %s)',r,rib_y(r),rib_type{r}));
        xlabel('x (mm)'); ylabel('y (mm)'); hold off
    end
    figure; hold on; grid on
    y1=rib_y(spar1_rib_range(1):spar1_rib_range(2));
    q1=an.qw_front(1,spar1_rib_range(1):spar1_rib_range(2));
    plot(y1,q1,'r--o','LineWidth',2,'MarkerSize',9,'DisplayName',sprintf('Spar 1 web (rib %d-%d)',spar1_rib_range(1),spar1_rib_range(2)))
    xlabel('span y (m)'); ylabel('spar 1 web shear flow (N/mm)')
    legend('Location','best'); title('Spar 1 web shear flow along the span'); hold off
    for r=1:num_ribs
        sec=rib_sec{r}; q_col=rib_q{r}; qw=rib_qw{r}; x_SC=rib_xSC(r);
        figure; hold on; grid on; axis equal
        [H,L]=draw_section(sec,x_SC,stringers,P);
        scale=25/max([abs(q_col); abs(qw(:))]);
        [xa,ya,ua,va,xwv,ywv,uwv,vwv]=arrow_data(sec,q_col,qw,scale);
        h1=quiver(xa,ya,ua,va,0,'Color',[0.8 0 0],'LineWidth',1,'MaxHeadSize',1.5);
        H=[H h1]; L=[L,{'Shear flow, skin panels'}];
        if ~isempty(xwv)
            h2=quiver(xwv,ywv,uwv,vwv,0,'Color',[0 0.6 0.6],'LineWidth',1.5,'MaxHeadSize',1.5);
            H=[H h2]; L=[L,{'Shear flow, spar 1 web (positive = upper to lower)'}];
        end
        legend(H,L,'Location','best');
        title(sprintf('Rib %d shear flow (y = %.2f m, %s)',r,rib_y(r),rib_type{r}));
        xlabel('x (mm)'); ylabel('y (mm)'); hold off
        [xu,qu,xl,ql]=extract_surface_q(sec,q_col);
        figure; hold on; grid on
        plot(xu/c_full,qu,'b-o','LineWidth',2,'MarkerSize',5,'DisplayName','Upper surface')
        plot(xl/c_full,ql,'r-s','LineWidth',2,'MarkerSize',5,'DisplayName','Lower surface')
        xlabel('x/c'); ylabel('Shear flow q (N/mm)')
        title(sprintf('Shear flow vs x/c, rib %d (y = %.2f m)',r,rib_y(r)))
        legend('Location','best'); hold off
    end
    bay_sec={sec_front,sec_none,sec_clip};
    bay_q_lo={an.q_front(:,1),an.q_none(:,2),an.q_clip(:,3)};
    bay_q_hi={an.q_front(:,2),an.q_none(:,3),an.q_clip(:,4)};
    vm_front1=compute_vm_vec(sec_front,an.q_front(:,1),M_rib_Nmm(1));
    vm_front2=compute_vm_vec(sec_front,an.q_front(:,2),M_rib_Nmm(2));
    vm_none2=compute_vm_vec(sec_none,an.q_none(:,2),M_rib_Nmm(2));
    vm_none3=compute_vm_vec(sec_none,an.q_none(:,3),M_rib_Nmm(3));
    vm_clip3=compute_vm_vec(sec_clip,an.q_clip(:,3),M_rib_Nmm(3));
    vm_clip4=compute_vm_vec(sec_clip,an.q_clip(:,4),M_rib_Nmm(4));
    bay_vm_lo={vm_front1,vm_none2,vm_clip3};
    bay_vm_hi={vm_front2,vm_none3,vm_clip4};
    [Xgrid,Ygrid,Qup,Qlo]=build_surface_grids_bays(rib_y,bay_sec,bay_q_lo,bay_q_hi,c_full);
    [~,~,VMup,VMlo]=build_surface_grids_bays(rib_y,bay_sec,bay_vm_lo,bay_vm_hi,c_full);
    plot_contour_pair(Xgrid,Ygrid,Qup,Qlo,semi_span,'Upper surface shear flow','Lower surface shear flow','q (N/mm)');
    plot_contour_pair(Xgrid,Ygrid,VMup,VMlo,semi_span,'Upper surface von Mises stress','Lower surface von Mises stress','von Mises (MPa)');
    figure; hold on; grid on; axis equal
    view(40,25)
    xlabel('x (mm)'); ylabel('y (mm)'); zlabel('span (mm)')
    title('3-D wing model')
    Hs=gobjects(0); Ls={};
    skin_col=[0.65 0.8 1]; fl_col1=[0.4 0.65 1];
    nChord=80;
    bay_geo={sec_front,sec_none,sec_clip};
    for r=1:num_bays
        zA=rib_y(r)*1000; zB=rib_y(r+1)*1000;
        geo=bay_geo{r};
        x_lim=max(geo.x_node);
        xs=linspace(0,x_lim,nChord);
        for sgn=[1 -1]
            yq=get_surface_y(geo,xs,sgn);
            n=numel(xs);
            hh=surface([xs;xs],[yq;yq],[zA*ones(1,n); zB*ones(1,n)],'FaceColor',skin_col,'FaceAlpha',0.18,'EdgeColor','none');
            if r == 1 && sgn == 1,Hs(end+1)=hh; Ls{end+1}='Skin (transparent)'; end
        end
    end
    for r=1:num_ribs
        sec=rib_sec{r};
        hh=plot3([sec.x_node; sec.x_node(1)],[sec.y_node; sec.y_node(1)],rib_y(r)*1000*ones(sec.N+1,1),'k-','LineWidth',1.3);
        if r == 1,Hs(end+1)=hh; Ls{end+1}='Ribs'; end
    end
    zA=rib_y(spar1_rib_range(1))*1000; zB=rib_y(spar1_rib_range(2))*1000;
    hh1=surface([sec_front.x_node(sec_front.idxS1_upper) sec_front.x_node(sec_front.idxS1_upper); sec_front.x_node(sec_front.idxS1_upper) sec_front.x_node(sec_front.idxS1_upper)],...
        [sec_front.y_node(sec_front.idxS1_upper) sec_front.y_node(sec_front.idxS1_lower); sec_front.y_node(sec_front.idxS1_upper) sec_front.y_node(sec_front.idxS1_lower)],...
        [zA zA; zB zB],'FaceColor',col_s1,'FaceAlpha',0.35,'EdgeColor','none');
    Hs(end+1)=hh1; Ls{end+1}=sprintf('Spar 1 web (rib %d to rib %d)',spar1_rib_range(1),spar1_rib_range(2));
    for r=spar1_rib_range(1):spar1_rib_range(2)
        z=rib_y(r)*1000;
        for f=1:2
            nd=sec_front.fl_nodes{f};
            hh=plot3(sec_front.x_node(nd),sec_front.y_node(nd),z*ones(numel(nd),1),'-','Color',fl_col1,'LineWidth',4);
            if r == spar1_rib_range(1) && f == 1,Hs(end+1)=hh; Ls{end+1}='Spar 1 flanges (20 and 40 mm, toward TE)'; end
        end
    end
    for f=1:2
        nd=sec_front.fl_nodes{f};
        xt=sec_front.x_node(nd(end))*ones(2,1); yt=sec_front.y_node(nd(end))*ones(2,1);
        xr=sec_front.x_node(nd(1))*ones(2,1); yr=sec_front.y_node(nd(1))*ones(2,1);
        plot3(xt,yt,[zA;zB],'-','Color',fl_col1,'LineWidth',2);
        plot3(xr,yr,[zA;zB],'-','Color',fl_col1,'LineWidth',2);
    end
    for k=1:nst
        xs_=zeros(num_ribs,1); ys_=zeros(num_ribs,1);
        for r=1:num_ribs
            sec=rib_sec{r};
            [stn,~,~]=map_all_stringers(stringers(k,:),sec.x_node,c_full,sec.idx_upper,sec.idx_lower,sec.s_nodes);
            xs_(r)=sec.x_node(stn); ys_(r)=sec.y_node(stn);
        end
        if stringers(k,2) == 1,cc=[0 0.7 0]; else,cc=[0.85 0 0.85]; end
        hh=plot3(xs_,ys_,rib_y*1000,'-','Color',cc,'LineWidth',2.5);
        if k == find(stringers(:,2) == 1,1),Hs(end+1)=hh; Ls{end+1}='Upper stringers'; end
        if k == find(stringers(:,2) == -1,1),Hs(end+1)=hh; Ls{end+1}='Lower stringers'; end
    end
    legend(Hs,Ls,'Location','best');
    hold off
end
t_st=0.5;
b_st=10;
[A_L,Imin_L]=equal_leg_angle_props(b_st,t_st);
b_equiv=(A_lump/t_st+t_st)/2;
fprintf('\nLUMPED to L-SECTION\n');
fprintf('lumped area used in analysis = %.3f mm^2 (equivalent equal-leg L: b = %.2f mm at t = %.2f mm)\n',A_lump,b_equiv,t_st);
fprintf('chosen L %.1f x %.1f x %.2f mm: A = %.3f mm^2 (%.1f%% difference from lumped)\n',b_st,b_st,t_st,A_L,100*(A_L/A_lump-1));
Kc_free=0.425;
c_col=1;
L_col=(rib_y(2)-rib_y(1))*1000;
Mroot=M_rib_Nmm(1);
sec_root=rib_sec{1};
[st_node_root,~,~]=map_all_stringers(stringers,sec_root.x_node,c_full,sec_root.idx_upper,sec_root.idx_lower,sec_root.s_nodes);
xb_r=sec_root.x_node-sec_root.xc; yb_r=sec_root.y_node-sec_root.yc;
den_r=sec_root.Ixx*sec_root.Iyy-sec_root.Ixy^2;
sig_root=Mroot*(sec_root.Iyy*yb_r-sec_root.Ixy*xb_r)/den_r;
sig_applied_root=max(abs(sig_root(st_node_root)));
s_g=gerard_crippling(b_st,t_st,E,95);
s_kc=simple_kc_crippling(b_st,t_st,E,nu,Kc_free);
s_cl=column_buckling_stress(b_st,t_st,E,L_col,c_col);
fprintf('\nSTRINGER SIZING (equal-leg angle, %.1f x %.1f x %.2f mm)\n',b_st,b_st,t_st);
fprintf('A_L = %.3f mm^2, Imin = %.3f mm^4, column length = %.0f mm\n',A_L,Imin_L,L_col);
fprintf('allowable used for stringers = %.3f MPa (95/6.75)\n',sigma_allow_str);
fprintf('actual root bending stress at worst stringer = %.3f MPa\n',sig_applied_root);
fprintf('Gerard crippling stress = %.2f MPa, MS = %.2f\n',s_g,s_g/sigma_allow_str-1);
fprintf('simple Kc crippling stress = %.2f MPa, MS = %.2f\n',s_kc,s_kc/sigma_allow_str-1);
fprintf('Euler column buckling stress = %.2f MPa, MS = %.2f\n',s_cl,s_cl/sigma_allow_str-1);
b_sweep=linspace(3,30,300);
sigma_cc_gerard=arrayfun(@(b) gerard_crippling(b,t_st,E,95),b_sweep);
sigma_cc_kc=arrayfun(@(b) simple_kc_crippling(b,t_st,E,nu,Kc_free),b_sweep);
sigma_col_curve=arrayfun(@(b) column_buckling_stress(b,t_st,E,L_col,c_col),b_sweep);
feasible_gerard=(sigma_cc_gerard >= sigma_allow_str) & (sigma_col_curve >= sigma_allow_str);
feasible_kc=(sigma_cc_kc >= sigma_allow_str) & (sigma_col_curve >= sigma_allow_str);
if any(feasible_gerard)
    fprintf('feasible b range (Gerard)    = %.2f to %.2f mm\n',min(b_sweep(feasible_gerard)),max(b_sweep(feasible_gerard)));
else
    fprintf('feasible b range (Gerard)    = none at t_st = %.2f mm\n',t_st);
end
if any(feasible_kc)
    fprintf('feasible b range (simple Kc) = %.2f to %.2f mm\n',min(b_sweep(feasible_kc)),max(b_sweep(feasible_kc)));
else
    fprintf('feasible b range (simple Kc) = none at t_st = %.2f mm\n',t_st);
end
fprintf('chosen b = %.1f mm is %s\n',b_st,ternary(s_g>=sigma_allow_str && s_kc>=sigma_allow_str && s_cl>=sigma_allow_str,'ADEQUATE','NOT adequate'));
if plot_results
    for m=1:2
        figure; hold on; grid on
        if m == 1
            hc=plot(b_sweep,sigma_cc_gerard,'b-','LineWidth',2); lab1='Gerard crippling allowable'; feas=feasible_gerard;
            ttl=sprintf('Stringer sizing, Gerard method (t = %.2f mm)',t_st);
        else
            hc=plot(b_sweep,sigma_cc_kc,'b-','LineWidth',2); lab1='Simple Kc crippling allowable'; feas=feasible_kc;
            ttl=sprintf('Stringer sizing, simple Kc method (t = %.2f mm)',t_st);
        end
        hk=plot(b_sweep,sigma_col_curve,'r-','LineWidth',2);
        ha=yline(sigma_allow_str,'k--','LineWidth',1.5);
        yl=ylim; ylim(yl);
        hx=plot([b_st b_st],yl,'m-.','LineWidth',2);
        Hh=[hc hk ha hx]; Ll={lab1,'Column (Euler) buckling allowable','Design allowable (95/6.75 MPa)',sprintf('Chosen b = %.0f mm',b_st)};
        if any(feas)
            hp=patch([min(b_sweep(feas)) max(b_sweep(feas)) max(b_sweep(feas)) min(b_sweep(feas))],[yl(1) yl(1) yl(2) yl(2)],[0.6 0.9 0.6],'FaceAlpha',0.25,'EdgeColor','none');
            Hh(end+1)=hp; Ll{end+1}='Feasible b';
        end
        xlabel('leg length b (mm)'); ylabel('stress (MPa)')
        legend(Hh,Ll,'Location','best'); title(ttl); hold off
    end
end
rho=2710;
area_front=sum(sec_front.b_seg.*sec_front.t_seg) + spar_t*sec_front.web_len1 + sum(sec_front.fl_area_tot) + nst*A_lump;
area_none=sum(sec_none.b_seg.*sec_none.t_seg) + nst*A_lump;
area_clip=sum(sec_clip.b_seg.*sec_clip.t_seg) + nst*A_lump;
len_front=rib_y(spar1_rib_range(2))-rib_y(spar1_rib_range(1));
len_none=rib_y(clip_rib_range(1))-rib_y(spar1_rib_range(2));
len_clip=rib_y(clip_rib_range(2))-rib_y(clip_rib_range(1));
mass_front=area_front*1e-6*len_front*rho;
mass_none=area_none*1e-6*len_none*rho;
mass_clip=area_clip*1e-6*len_clip*rho;
rib_factor=2;
riv_frac=1.3;
rib_area=zeros(num_ribs,1); rib_mass=zeros(num_ribs,1);
for r=1:num_ribs
    rib_area(r)=polyarea(rib_sec{r}.x_node,rib_sec{r}.y_node);
    rib_mass(r)=rib_factor*rib_area(r)*rib_t*1e-9*rho;
end
mass_ribs=sum(rib_mass);
mass_total=mass_front+mass_none+mass_clip+mass_ribs;
fprintf('\nMASS ESTIMATE (rho = %.0f kg/m^3 assumed; fasteners, adhesive, aileron excluded)\n',rho);
fprintf('bay 1, front section (spar 1)        area = %.2f mm^2, length = %.2f m, mass = %.4f kg\n',area_front,len_front,mass_front);
fprintf('bay 2, none section (no spar)        area = %.2f mm^2, length = %.2f m, mass = %.4f kg\n',area_none,len_none,mass_none);
fprintf('bay 3, clip section (no spar, clipped) area = %.2f mm^2, length = %.2f m, mass = %.4f kg\n',area_clip,len_clip,mass_clip);
fprintf('ribs (%.2f mm thick plates, area enclosed by each rib''s own outline):\n',rib_t);
for r=1:num_ribs
    fprintf('  rib %d (%s): area = %.2f mm^2, mass = %.4f kg\n',r,rib_type{r},rib_area(r),rib_mass(r));
end
fprintf('total rib mass = %.4f kg\n',mass_ribs);
fprintf('estimated structural mass of one semi-span = %.4f kg (%.1f g)\n',mass_total,1000*mass_total);
fprintf('both semi-spans with rivet fraction %.2f = %.1f g (2526 g in the stability code)\n',riv_frac,2000*mass_total*riv_frac);
sep=repmat('-',1,0);
fprintf('\n%s\nREPORT DATA SUMMARY\n%s\n',sep,sep);
fprintf('\n[1] INPUT DATA, LOADS AND STRUCTURAL LAYOUT\n');
fprintf('full chord = %.1f mm, semi-span = %.3f m\n',c_full,semi_span);
fprintf('skin thickness = %.2f mm, spar web and flange thickness = %.2f mm\n',t,spar_t);
fprintf('E = %.0f MPa, nu = %.2f\n',E,nu);
fprintf('spar 1: x = %.2f mm (x/c = %.4f), C-flanges upper %.0f and lower %.0f mm toward TE, present rib %d to rib %d\n',spar1_x,spar1_x/c_full,spar1_flange_L(1),spar1_flange_L(2),spar1_rib_range(1),spar1_rib_range(2));
fprintf('aileron cut-out: box clipped to c_box for aileron = %.1f mm, present rib %d to rib %d (no spar there)\n',c_box,clip_rib_range(1),clip_rib_range(2));
fprintf('aerodynamic centre x_AC = %.2f mm (0.25 c_full)\n',x_AC);
fprintf('allowables: skin/stringer = %.3f MPa (95/6.75), spar web/flange = %.3f MPa (276/6.75)\n',sigma_allow_skin,sigma_allow_spar);
fprintf('stringers: %d lumped booms, A_lump = %.2f mm^2 each\n',nst,A_lump);
fprintf('\n%5s %8s %10s %14s %10s\n','Rib','y(m)','V(N)','M(N.mm)','Type');
for r=1:num_ribs
    fprintf('%5d %8.3f %10.3f %14.1f %10s\n',r,rib_y(r),V_rib(r),M_rib_Nmm(r),rib_type{r});
end
fprintf('\n[2] SECTION PROPERTIES (booms: skin + present web + spread flanges + lumped stringers)\n');
fprintf('%-38s %14s %14s %14s\n','Quantity','front(rib1-2)','none(rib2-3)','clip(rib3-4)');
prf=@(nm,a,b,c) fprintf('%-38s %14.4f %14.4f %14.4f\n',nm,a,b,c);
pre=@(nm,a,b,c) fprintf('%-38s %14.4e %14.4e %14.4e\n',nm,a,b,c);
prf('centroid xc (mm)',sec_front.xc,sec_none.xc,sec_clip.xc);
prf('centroid yc (mm)',sec_front.yc,sec_none.yc,sec_clip.yc);
pre('Ixx (mm^4)',sec_front.Ixx,sec_none.Ixx,sec_clip.Ixx);
pre('Iyy (mm^4)',sec_front.Iyy,sec_none.Iyy,sec_clip.Iyy);
pre('Ixy (mm^4)',sec_front.Ixy,sec_none.Ixy,sec_clip.Ixy);
prf('total boom area (mm^2)',sum(sec_front.B),sum(sec_none.B),sum(sec_clip.B));
prf('contour perimeter (mm)',sec_front.perim,sec_none.perim,sec_clip.perim);
prf('shear centre x_SC (mm)',an.x_SC_front,an.x_SC_none,an.x_SC_clip);
prf('torque arm x_AC - x_SC (mm)',x_AC-an.x_SC_front,x_AC-an.x_SC_none,x_AC-an.x_SC_clip);
fprintf('enclosed cell areas, front (mm^2): %s\n',mat2str(sec_front.Acell,6));
fprintf('enclosed cell areas, none  (mm^2): %s\n',mat2str(sec_none.Acell,6));
fprintf('enclosed cell areas, clip  (mm^2): %s\n',mat2str(sec_clip.Acell,6));
fprintf('\n[3] STRINGER LAYOUT (positions read off the front-type section)\n');
[stn,sst_s,sst]=map_all_stringers(stringers,sec_front.x_node,c_full,sec_front.idx_upper,sec_front.idx_lower,sec_front.s_nodes);
pan_w=[diff(sst_s); sec_front.perim-sst_s(end)+sst_s(1)];
fprintf('%4s %8s %8s %9s %9s %10s %18s\n','No','Surface','x/c','x(mm)','y(mm)','A(mm^2)','panel to next(mm)');
for k=1:numel(stn)
    if sst(k) == 1,sl='upper'; else,sl='lower'; end
    fprintf('%4d %8s %8.4f %9.2f %9.2f %10.2f %18.2f\n',k,sl,sec_front.x_node(stn(k))/c_full,sec_front.x_node(stn(k)),sec_front.y_node(stn(k)),A_lump,pan_w(k));
end
fprintf('\n[4] SHEAR FLOW PER RIB (N/mm; spar web flow positive = upper to lower). "-" = no spar at that rib\n');
fprintf('%4s %8s %10s %12s %12s %10s %8s %12s\n','Rib','y(m)','V(N)','Mz(N.mm)','max|q| skin','at x(mm)','surf','q spar1');
for r=1:num_ribs
    sec=rib_sec{r}; q_col=rib_q{r}; qw=rib_qw{r}; xsc=rib_xSC(r);
    [qm,im]=max(abs(q_col));
    if any(im == sec.idx_upper),sl='upper'; else,sl='lower'; end
    q1s='-';
    if sec.has1,q1s=sprintf('%.4f',qw(1)); end
    fprintf('%4d %8.3f %10.3f %12.1f %12.4f %10.2f %8s %12s\n',r,rib_y(r),V_rib(r),V_rib(r)*(x_AC-xsc),qm,sec.x_mid(im),sl,q1s);
end
fprintf('\n[5] BENDING STRESS PER RIB, sigma = M (Iyy y - Ixy x)/(Ixx Iyy - Ixy^2)\n');
fprintf('%4s %8s %14s %14s %10s\n','Rib','y(m)','M(N.mm)','max|sigma|','type');
for r=1:num_ribs
    fprintf('%4d %8.3f %14.1f %14.4f %10s\n',r,rib_y(r),M_rib_Nmm(r),vm_tab(r,1),rib_type{r});
end
fprintf('\n[6] VON MISES STRESS PER RIB (MPa). "-" = no spar at that rib\n');
fprintf('allowables: skin/stringer nodes = %.3f, spar flange and web = %.3f\n',sigma_allow_skin,sigma_allow_spar);
fprintf('%4s %10s %14s %14s %12s %12s %10s\n','Rib','type','skin/stringer','flange/face','spar1 web','gov. ratio','MS');
for r=1:num_ribs
    fl=ternary(isnan(vm_tab(r,3)),'-',sprintf('%.4f',vm_tab(r,3)));
    w1=ternary(vm_tab(r,4)==0,'-',sprintf('%.4f',vm_tab(r,4)));
    fprintf('%4d %10s %14.4f %14s %12s %12.4f %10.3f\n',r,rib_type{r},vm_tab(r,2),fl,w1,vm_tab(r,6),1/vm_tab(r,6)-1);
end
fprintf('\n[7] SHEAR BUCKLING SUMMARY PER BAY (worst member of each type, MS = Pcr/tau - 1). "-" = no spar web in that bay\n');
names_all=brows(:,3);
bays_all=cell2mat(brows(:,1));
info_all=cell2mat(brows(:,9));
tau_all=cell2mat(brows(:,7));
pcr_all=cell2mat(brows(:,8));
MS_all=pcr_all./tau_all-1;
isskin=strncmp(names_all,'panel',5) & ~info_all;
isweb=strcmp(names_all,'spar1web');
fprintf('%4s %10s %13s %8s %12s %10s %10s %10s\n','Bay','Type','y range (m)','a(mm)','worst panel','b(mm)','MS skin','MS web');
for bay=1:num_bays
    ks=find(bays_all == bay & isskin);
    [msk,ii]=min(MS_all(ks)); kk=ks(ii);
    kw=find(bays_all == bay & isweb);
    if isempty(kw),msw_txt='-'; else,msw_txt=sprintf('%.3f',MS_all(kw(1))); end
    fprintf('%4d %10s %6.2f-%-6.2f %8.1f %12s %10.2f %10.3f %10s\n',bay,bay_type{bay},rib_y(bay),rib_y(bay+1),brows{kk,4},brows{kk,3},brows{kk,5},msk,msw_txt);
end
fprintf('(the full panel-by-panel table is printed above)\n');
fprintf('\n[8] STRINGERS: ROOT-RIB LOADS AND L-SECTION CHECK\n');
fprintf('%4s %8s %9s %16s %12s\n','No','Surface','x(mm)','sigma root(MPa)','load (N)');
for k=1:numel(st_node_root)
    if sst(k) == 1,sl='upper'; else,sl='lower'; end
    fprintf('%4d %8s %9.2f %16.4f %12.3f\n',k,sl,sec_root.x_node(st_node_root(k)),sig_root(st_node_root(k)),sig_root(st_node_root(k))*A_lump);
end
fprintf('lumped area used in analysis = %.3f mm^2; L %.1f x %.1f x %.2f mm has A = %.3f mm^2, Imin = %.3f mm^4\n',A_lump,b_st,b_st,t_st,A_L,Imin_L);
fprintf('capacities: Gerard crippling = %.2f MPa, simple Kc = %.2f MPa, Euler column (L = %.0f mm) = %.2f MPa\n',s_g,s_kc,L_col,s_cl);
fprintf('\n[9] MARGIN OF SAFETY SUMMARY (MS = allowable/actual - 1, PASS if MS >= 0)\n');
ms_rows={};
[msk1,k1]=min(MS_all(isskin));
idxs_skin=find(isskin);
ms_rows(end+1,:)={sprintf('skin panels, shear buckling, worst overall [bay %d]',brows{idxs_skin(k1),1}),brows{idxs_skin(k1),7},brows{idxs_skin(k1),8}};
kw1=find(strcmp(names_all,'spar1web'));
[~,jj]=min(MS_all(kw1)); k=kw1(jj);
ms_rows(end+1,:)={sprintf('spar 1 web, shear buckling [bay %d]',brows{k,1}),brows{k,7},brows{k,8}};
[act,r_]=max(vm_tab(:,2)); ms_rows(end+1,:)={sprintf('von Mises, skin/stringer nodes [rib %d]',r_),act,sigma_allow_skin};
[act,r_]=max(vm_tab(:,3)); ms_rows(end+1,:)={sprintf('von Mises, spar flange [rib %d]',r_),act,sigma_allow_spar};
[act,r_]=max(vm_tab(:,4)); ms_rows(end+1,:)={sprintf('von Mises, spar 1 web [rib %d]',r_),act,sigma_allow_spar};
ms_rows(end+1,:)={'stringer bending stress, root rib',sig_applied_root,sigma_allow_str};
ms_rows(end+1,:)={'L stringer crippling (Gerard) vs design stress',sigma_allow_str,s_g};
ms_rows(end+1,:)={'L stringer crippling (simple Kc) vs design stress',sigma_allow_str,s_kc};
ms_rows(end+1,:)={'L stringer column buckling vs design stress',sigma_allow_str,s_cl};
fprintf('%-52s %10s %10s %10s %8s\n','Check','Actual','Allowable','MS','Status');
all_pass=true; ms_min=inf; ms_min_name='';
for i=1:size(ms_rows,1)
    ms=ms_rows{i,3}/ms_rows{i,2}-1;
    if ms < 0,st_txt='FAIL'; all_pass=false; else,st_txt='PASS'; end
    if ms < ms_min,ms_min=ms; ms_min_name=ms_rows{i,1}; end
    fprintf('%-52s %10.3f %10.3f %10.3f %8s\n',ms_rows{i,1},ms_rows{i,2},ms_rows{i,3},ms,st_txt);
end
fprintf('\noverall: %s; lowest margin of safety = %.3f (%s)\n',ternary(all_pass,'ALL CHECKS PASS','AT LEAST ONE CHECK FAILS'),ms_min,ms_min_name);

if plot_results
    figs=findall(0,'Type','figure');
    for k=1:numel(figs)
        style_fig(figs(k));
    end
    fprintf('\n%d figures created, all text set to 20 pt\n',numel(figs));
    if save_figs
        if ~exist(fig_folder,'dir'),mkdir(fig_folder); end
        nums=arrayfun(@(f) f.Number,figs);
        [~,ord]=sort(nums);
        figs=figs(ord);
        drawnow
        for k=1:numel(figs)
            f=figs(k);
            axs=findall(f,'Type','axes');
            ttl='figure';
            if ~isempty(axs)
            tt=axs(end).Title.String;
            if iscell(tt),tt=tt{1}; end
            if ~isempty(tt),ttl=tt; end
            end
            ttl=regexprep(ttl,'[^A-Za-z0-9]+','_');
            ttl=ttl(1:min(numel(ttl),60));
            base=fullfile(fig_folder,sprintf('Fig%02d_%s',k,ttl));
            exportgraphics(f,[base '.png'],'Resolution',fig_dpi);
            if save_fig_files,savefig(f,[base '.fig']); end
        end
        fprintf('%d figures saved in %s\n',numel(figs),fullfile(pwd,fig_folder));
    end
end
if save_output
    diary off
    fprintf('command-window output saved to %s\n',fullfile(pwd,output_file));
end
function out=ternary(cond,a,b)
    if cond,out=a; else,out=b; end
end
function style_fig(fig)
    set(fig,'Units','normalized','OuterPosition',[0 0.03 1 0.97]);
    fontsize(fig,20,'points');
    axs=findall(fig,'Type','axes');
    for k=1:numel(axs)
        ax=axs(k);
        ax.FontSize=20; ax.TitleFontSizeMultiplier=1; ax.LabelFontSizeMultiplier=1;
        ax.Title.FontSize=20; ax.XLabel.FontSize=20; ax.YLabel.FontSize=20; ax.ZLabel.FontSize=20;
    end
    lgs=findall(fig,'Type','legend');
    for k=1:numel(lgs),lgs(k).FontSize=20; end
    cbs=findall(fig,'Type','colorbar');
    for k=1:numel(cbs),cbs(k).FontSize=20; cbs(k).Label.FontSize=20; end
    txs=findall(fig,'Type','text');
    for k=1:numel(txs),txs(k).FontSize=20; end
end
function plot_contour_pair(Xgrid,Ygrid,ValUp,ValLo,semi_span,ttlU,ttlL,cblabel)
    lo=min([ValUp(:); ValLo(:)],[],'omitnan');
    hi=max([ValUp(:); ValLo(:)],[],'omitnan');
    figure
    for m=1:2
        subplot(1,2,m)
        if m == 1,V=ValUp; ttl=ttlU; else,V=ValLo; ttl=ttlL; end
        contourf(Xgrid,Ygrid,V,20,'LineStyle','none')
        clim([lo hi])
        cb=colorbar; cb.Label.String=cblabel; colormap(gca,parula)
        xlabel('x/c'); ylabel('span y (m)')
        title(ttl)
        axis([0 1 0 semi_span])
    end
end
function vm=compute_vm_vec(sec,q_col,M)
    x_bar=sec.x_node-sec.xc; y_bar=sec.y_node-sec.yc;
    denom=sec.Ixx*sec.Iyy-sec.Ixy^2;
    sig=M*(sec.Iyy*y_bar-sec.Ixy*x_bar)/denom;
    tau=q_col./sec.t_seg;
    vm=sqrt(sig.^2+3*tau.^2);
end
function [x_node,y_node]=resample_contour(xn,yn,chord,Nnodes)
    x_raw=xn*chord;
    y_raw=yn*chord;
    if hypot(x_raw(end)-x_raw(1),y_raw(end)-y_raw(1)) > 1e-6
        x_raw=[x_raw; x_raw(1)];
        y_raw=[y_raw; y_raw(1)];
    end
    d_arc=hypot(diff(x_raw),diff(y_raw));
    s_raw=[0; cumsum(d_arc)];
    perim=s_raw(end);
    s_nodes=linspace(0,perim,Nnodes+1)';
    s_nodes(end)=[];
    x_node=interp1(s_raw,x_raw,s_nodes,'pchip');
    y_node=interp1(s_raw,y_raw,s_nodes,'pchip');
end
function [x_node,y_node]=insert_web(x_node,y_node,x_target)
    N=length(x_node);
    insert_list=[];
    for i=1:N
        i_next=mod(i,N)+1;
        x1=x_node(i); x2=x_node(i_next);
        y1=y_node(i); y2=y_node(i_next);
        if (x1-x_target)*(x2-x_target) < 0
            frac=(x_target-x1)/(x2-x1);
            insert_list=[insert_list; i x_target y1+frac*(y2-y1)];
        end
    end
    if size(insert_list,1) ~= 2
        error('web at x=%.2f must cross the contour exactly twice, found %d',x_target,size(insert_list,1));
    end
    insert_list=sortrows(insert_list,1,'descend');
    for r=1:2
        i_edge=insert_list(r,1);
        x_node=[x_node(1:i_edge); insert_list(r,2); x_node(i_edge+1:end)];
        y_node=[y_node(1:i_edge); insert_list(r,3); y_node(i_edge+1:end)];
    end
end
function [x_node,y_node]=clip_to_blunt(x_node,y_node,x_clip)
    N=length(x_node);
    cross_edge=[]; cross_pt=[];
    for i=1:N
        i_next=mod(i,N)+1;
        x1=x_node(i); x2=x_node(i_next);
        y1=y_node(i); y2=y_node(i_next);
        if (x1-x_clip)*(x2-x_clip) < 0
            frac=(x_clip-x1)/(x2-x1);
            cross_edge=[cross_edge; i];
            cross_pt=[cross_pt; x_clip y1+frac*(y2-y1)];
        end
    end
    if length(cross_edge) ~= 2
        error('clip at x=%.2f must cross the contour exactly twice, found %d',x_clip,length(cross_edge));
    end
    i_A=cross_edge(1); i_B=cross_edge(2);
    x_node=[x_node(1:i_A); cross_pt(1,1); cross_pt(2,1); x_node(i_B+1:end)];
    y_node=[y_node(1:i_A); cross_pt(1,2); cross_pt(2,2); y_node(i_B+1:end)];
end
function [iu,il]=find_pair(x_node,y_node,xt)
    idx=find(abs(x_node-xt) < 1e-6);
    if numel(idx) ~= 2
        error('expected 2 nodes at x=%.3f, found %d',xt,numel(idx));
    end
    if y_node(idx(1)) > y_node(idx(2)),iu=idx(1); il=idx(2); else,iu=idx(2); il=idx(1); end
end
function [b_seg,x_mid,y_mid,perim,s_nodes]=seg_props(x_node,y_node)
    N=length(x_node);
    j=mod((1:N)',N)+1;
    b_seg=hypot(x_node(j)-x_node,y_node(j)-y_node);
    x_mid=0.5*(x_node+x_node(j));
    y_mid=0.5*(y_node+y_node(j));
    s_nodes=[0; cumsum(b_seg(1:end-1))];
    perim=sum(b_seg);
end
function idx_range=circular_range(i_start,i_end,N)
    if i_end >= i_start
        idx_range=i_start:i_end;
    else
        idx_range=[i_start:N,1:i_end];
    end
end
function [dA,nodes]=flange_area(b_seg,N,i0,dir,L,thk)
    dA=zeros(N,1); nodes=i0; cur=i0; rem_len=L;
    while rem_len > 1e-9
        if dir > 0
            seg=cur; nxt=mod(cur,N)+1;
        else
            seg=mod(cur-2,N)+1; nxt=seg;
        end
        len=min(b_seg(seg),rem_len);
        dA(cur)=dA(cur)+thk*len/2;
        dA(nxt)=dA(nxt)+thk*len/2;
        rem_len=rem_len-len;
        cur=nxt;
        nodes=[nodes; cur];
    end
end
function [B,xc,yc,Ixx,Iyy,Ixy]=lump_booms(x_node,y_node,b_seg,t_seg,webs,extra)
    N=length(x_node);
    pi_=[(1:N)'; webs(:,1)];
    pj_=[mod((1:N)',N)+1; webs(:,2)];
    plen=[b_seg; webs(:,3)];
    pth=[t_seg; webs(:,4)];
    B=accumarray(pi_,plen.*pth/2,[N 1]) + accumarray(pj_,plen.*pth/2,[N 1]) + extra;
    for iter=1:100
        xc=sum(x_node.*B)/sum(B); yc=sum(y_node.*B)/sum(B);
        xb=x_node-xc; yb=y_node-yc;
        Ixx=sum(B.*yb.^2); Iyy=sum(B.*xb.^2); Ixy=sum(B.*xb.*yb);
        D=Ixx*Iyy-Ixy^2;
        sig=(Iyy*yb-Ixy*xb)/D;
        si=sig(pi_); sj=sig(pj_);
        ri=ones(size(si)); rj=ones(size(si));
        m=abs(si) > 1e-12; ri(m)=sj(m)./si(m);
        m=abs(sj) > 1e-12; rj(m)=si(m)./sj(m);
        ri=min(max(ri,-1),3); rj=min(max(rj,-1),3);
        Bn=accumarray(pi_,plen.*pth/6.*(2+ri),[N 1]) + accumarray(pj_,plen.*pth/6.*(2+rj),[N 1]) + extra;
        conv=max(abs(Bn-B)) < 1e-9;
        B=Bn;
        if conv,break; end
    end
    xc=sum(x_node.*B)/sum(B); yc=sum(y_node.*B)/sum(B);
    xb=x_node-xc; yb=y_node-yc;
    Ixx=sum(B.*yb.^2); Iyy=sum(B.*xb.^2); Ixy=sum(B.*xb.*yb);
end
function [st_node,st_s,st_surf]=map_all_stringers(stringers,x_node,c_full,idx_upper,idx_lower,s_nodes)
    nst=size(stringers,1);
    st_node=zeros(nst,1); st_s=zeros(nst,1); st_surf=stringers(:,2);
    x_frac=x_node/c_full;
    for k=1:nst
        if stringers(k,2) == 1,idxs=idx_upper; else,idxs=idx_lower; end
        [~,local]=min(abs(x_frac(idxs)-stringers(k,1)));
        st_node(k)=idxs(local);
        st_s(k)=s_nodes(st_node(k));
    end
    [st_s,order]=sort(st_s);
    st_node=st_node(order);
    st_surf=st_surf(order);
end
function sec=build_section(stype,P,stringers,A_str)
    [x_node,y_node]=resample_contour(P.xn,P.yn,P.c_full,P.N0);
    has1=strcmp(stype,'front');
    clipped=strcmp(stype,'clip');
    if has1,[x_node,y_node]=insert_web(x_node,y_node,P.spar1_x); end
    if clipped,[x_node,y_node]=clip_to_blunt(x_node,y_node,P.c_box); end
    N=length(x_node);
    if has1,[idxS1_upper,idxS1_lower]=find_pair(x_node,y_node,P.spar1_x); else,idxS1_upper=[]; idxS1_lower=[]; end
    if clipped,[idx_fold_upper,idx_fold_lower]=find_pair(x_node,y_node,P.c_box); else,idx_fold_upper=[]; idx_fold_lower=[]; end
    [b_seg,x_mid,y_mid,perim,s_nodes]=seg_props(x_node,y_node);
    t_seg=P.t*ones(N,1);
    ds_t=b_seg./t_seg;
    [~,idx_LE]=min(x_node);
    if clipped
        idx_TE=idx_fold_upper;
    else
        [~,idx_TE]=max(x_node);
    end
    idx_upper=circular_range(idx_LE,idx_TE,N);
    idx_lower=circular_range(idx_TE,idx_LE,N);
    web_len1=0;
    if has1,web_len1=hypot(x_node(idxS1_upper)-x_node(idxS1_lower),y_node(idxS1_upper)-y_node(idxS1_lower)); end
    fold_len=0;
    if clipped,fold_len=hypot(x_node(idx_fold_upper)-x_node(idx_fold_lower),y_node(idx_fold_upper)-y_node(idx_fold_lower)); end
    switch stype
        case 'front'
            cell1_nodes=circular_range(idxS1_lower,idxS1_upper,N);
            cell2_nodes=circular_range(idxS1_upper,idxS1_lower,N);
            cell1_panels=cell1_nodes(1:end-1); cell2_panels=cell2_nodes(1:end-1);
            Lcell_t=[sum(ds_t(cell1_panels)) sum(ds_t(cell2_panels))];
            Acell=[polyarea(x_node(cell1_nodes),y_node(cell1_nodes)),polyarea(x_node(cell2_nodes),y_node(cell2_nodes))];
            Lweb_t=web_len1/P.spar_t;
            cell_panels={cell1_panels,cell2_panels};
            webs_for_boom=[idxS1_upper idxS1_lower web_len1 P.spar_t];
            web_nodes=[idxS1_upper idxS1_lower];
        case {'none','clip'}
            cell1_panels=1:N;
            Lcell_t=sum(ds_t(cell1_panels));
            Acell=polyarea(x_node,y_node);
            Lweb_t=[];
            cell_panels={cell1_panels};
            webs_for_boom=zeros(0,4);
            web_nodes=zeros(0,2);
    end
    fl_nodes={[],[],[],[]};
    fl_area_tot=zeros(1,4);
    extra=zeros(N,1);
    spar_mask=false(N,1);
    if has1
        [d1,n1]=flange_area(b_seg,N,idxS1_upper,+1,P.fl1_L(1),P.spar_t);
        [d2,n2]=flange_area(b_seg,N,idxS1_lower,-1,P.fl1_L(2),P.spar_t);
        extra=extra+d1+d2;
        fl_nodes{1}=n1; fl_nodes{2}=n2;
        fl_area_tot(1)=sum(d1); fl_area_tot(2)=sum(d2);
        spar_mask([n1;n2])=true;
    end
    if ~isempty(stringers)
        [st_node_local,~,~]=map_all_stringers(stringers,x_node,P.c_full,idx_upper,idx_lower,s_nodes);
        for k=1:numel(st_node_local)
            extra(st_node_local(k))=extra(st_node_local(k))+A_str;
        end
    end
    [B,xc,yc,Ixx,Iyy,Ixy]=lump_booms(x_node,y_node,b_seg,t_seg,webs_for_boom,extra);
    sec.stype=stype; sec.has1=has1; sec.has2=false;
    sec.x_node=x_node; sec.y_node=y_node; sec.N=N;
    sec.b_seg=b_seg; sec.t_seg=t_seg; sec.x_mid=x_mid; sec.y_mid=y_mid;
    sec.s_nodes=s_nodes; sec.perim=perim;
    sec.xc=xc; sec.yc=yc; sec.Ixx=Ixx; sec.Iyy=Iyy; sec.Ixy=Ixy; sec.B=B;
    sec.idx_LE=idx_LE; sec.idx_TE=idx_TE;
    sec.idx_upper=idx_upper; sec.idx_lower=idx_lower;
    sec.idxS1_upper=idxS1_upper; sec.idxS1_lower=idxS1_lower; sec.web_len1=web_len1;
    sec.idx_fold_upper=idx_fold_upper; sec.idx_fold_lower=idx_fold_lower; sec.fold_len=fold_len;
    sec.cell_panels=cell_panels;
    sec.Lcell_t=Lcell_t; sec.Lweb_t=Lweb_t; sec.Acell=Acell;
    sec.web_nodes=web_nodes;
    sec.fl_nodes=fl_nodes; sec.fl_area_tot=fl_area_tot;
    sec.spar_mask=spar_mask;
    sec.aft_cell_nodes=[];
end
function q=basic_shear_flow(Vy,B,x_node,y_node,xc,yc,Ixx,Iyy,Ixy)
    N=length(x_node);
    x_bar=x_node-xc; y_bar=y_node-yc;
    D=Ixx*Iyy-Ixy^2;
    cx=-Vy*Ixy/D;
    cy=Vy*Iyy/D;
    q=zeros(N,1);
    for i=2:N
        q(i)=q(i-1)-(cx*B(i)*x_bar(i)+cy*B(i)*y_bar(i));
    end
end
function q0=solve_redundant_chain(Qb,Lcell_t,Lweb_t)
    n=length(Lcell_t);
    M=zeros(n,n); rhs=zeros(n,1);
    for i=1:n
        Lw_left=0; Lw_right=0;
        if i > 1,Lw_left=Lweb_t(i-1); end
        if i < n,Lw_right=Lweb_t(i); end
        M(i,i)=Lcell_t(i)+Lw_left+Lw_right;
        if i > 1,M(i,i-1)=-Lw_left; end
        if i < n,M(i,i+1)=-Lw_right; end
        rhs(i)=-Qb(i);
    end
    q0=M\rhs;
end
function qT=solve_torsion_chain(Mz,Acell,Lcell_t,Lweb_t)
    n=length(Lcell_t);
    D=zeros(n,n);
    for i=1:n
        D(i,i)=D(i,i)+Lcell_t(i);
        if i > 1
            D(i,i)=D(i,i)+Lweb_t(i-1);
            D(i,i-1)=D(i,i-1)-Lweb_t(i-1);
        end
        if i < n
            D(i,i)=D(i,i)+Lweb_t(i);
            D(i,i+1)=D(i,i+1)-Lweb_t(i);
        end
    end
    M=zeros(n,n); rhs=zeros(n,1);
    for i=1:n-1
        M(i,:)=D(i,:)/Acell(i)-D(i+1,:)/Acell(i+1);
    end
    M(n,:)=2*Acell(:)';
    rhs(n)=Mz;
    qT=M\rhs;
end
function [q_total,q_web]=assemble_flow_chain(q_basic,q0,qT,cell_panels,N)
    n=length(cell_panels);
    q_total=zeros(N,1);
    for i=1:n
        q_total(cell_panels{i})=q_basic(cell_panels{i})+q0(i)+qT(i);
    end
    q_web=zeros(n-1,1);
    for i=1:n-1
        q_web(i)=(q0(i)-q0(i+1))+(qT(i)-qT(i+1));
    end
end
function T_q=section_torque_chain(x_node,y_node,xc,yc,q_total,web_nodes,q_web)
    N=length(x_node);
    x_bar=x_node-xc; y_bar=y_node-yc;
    T_q=0;
    for i=1:N
        i_next=mod(i,N)+1;
        dx=x_node(i_next)-x_node(i); dy=y_node(i_next)-y_node(i);
        xm=0.5*(x_bar(i)+x_bar(i_next)); ym=0.5*(y_bar(i)+y_bar(i_next));
        T_q=T_q+q_total(i)*(xm*dy-ym*dx);
    end
    for w=1:size(web_nodes,1)
        iu=web_nodes(w,1); il=web_nodes(w,2);
        dxw=x_node(il)-x_node(iu); dyw=y_node(il)-y_node(iu);
        xmw=0.5*((x_node(iu)-xc)+(x_node(il)-xc));
        ymw=0.5*((y_node(iu)-yc)+(y_node(il)-yc));
        T_q=T_q+q_web(w)*(xmw*dyw-ymw*dxw);
    end
end
function [x_SC,q_basic,q0]=shear_center_calc(sec)
    q_basic=basic_shear_flow(1,sec.B,sec.x_node,sec.y_node,sec.xc,sec.yc,sec.Ixx,sec.Iyy,sec.Ixy);
    ncell=length(sec.Lcell_t);
    Qb=zeros(ncell,1);
    for i=1:ncell
        p=sec.cell_panels{i};
        Qb(i)=sum(q_basic(p).*sec.b_seg(p)./sec.t_seg(p));
    end
    q0=solve_redundant_chain(Qb,sec.Lcell_t,sec.Lweb_t);
    [q_total_unit,q_web_unit]=assemble_flow_chain(q_basic,q0,zeros(ncell,1),sec.cell_panels,sec.N);
    T_q=section_torque_chain(sec.x_node,sec.y_node,sec.xc,sec.yc,q_total_unit,sec.web_nodes,q_web_unit);
    x_SC=sec.xc+T_q;
end
function an=analyze_wing(P,stringers,A_str)
    types={'front','none','clip'};
    R=numel(P.rib_y);
    for i=1:numel(types)
        ty=types{i};
        sec=build_section(ty,P,stringers,A_str);
        [xSC,qb,q0]=shear_center_calc(sec);
        ncell=length(sec.Lcell_t);
        qmat=zeros(sec.N,R);
        qwmat=zeros(max(ncell-1,0),R);
        for r=1:R
            Vy=P.V_rib(r);
            Mz=Vy*(P.x_AC-xSC);
            qT=solve_torsion_chain(Mz,sec.Acell,sec.Lcell_t,sec.Lweb_t);
            [qt,qw]=assemble_flow_chain(qb*Vy,q0*Vy,qT,sec.cell_panels,sec.N);
            qmat(:,r)=qt;
            if ncell > 1,qwmat(:,r)=qw; end
        end
        an.(['sec_' ty])=sec;
        an.(['x_SC_' ty])=xSC;
        an.(['q_' ty])=qmat;
        an.(['qw_' ty])=qwmat;
    end
end
function Pcr=plate_pcr(a,l,t,E,nu)
    short=min(a,l); long=max(a,l);
    K=5.34+4*(short/long)^2;
    Pcr=K*(pi^2*E/(12*(1-nu^2)))*(t/short)^2;
end
function print_row(bay,bt,name,a,b,q,tau,Pcr)
    MS=Pcr/tau-1;
    status='PASS'; if tau > Pcr,status='FAIL'; end
    fprintf('%5d %8s %10s %8.2f %8.2f %10.4f %12.4f %12.4f %10.4f %8s\n',bay,bt,name,a,b,q,tau,Pcr,MS,status);
end
function [failed,worst_ratio,worst_bay,worst_panel,rows]=check_buckling(an,P,stringers,verbose)
    nst=size(stringers,1);
    failed=false; worst_ratio=-inf; worst_bay=0; worst_panel=0;
    rows=cell(0,9);
    num_bays=numel(P.rib_y)-1;
    for bay=1:num_bays
        a=(P.rib_y(bay+1)-P.rib_y(bay))*1000;
        bt=P.bay_type{bay};
        sec=an.(['sec_' bt]);
        q_mat=an.(['q_' bt]);
        qw_mat=an.(['qw_' bt]);
        [~,st_s]=map_all_stringers(stringers,sec.x_node,P.c_full,sec.idx_upper,sec.idx_lower,sec.s_nodes);
        q_bay=max(abs(q_mat(:,bay)),abs(q_mat(:,bay+1)));
        for k=1:nst
            k_next=mod(k,nst)+1;
            if k < nst
            l_panel=st_s(k_next)-st_s(k);
            panel_nodes=find(sec.s_nodes >= st_s(k) & sec.s_nodes <= st_s(k_next));
            else
            l_panel=sec.perim-st_s(k)+st_s(1);
            panel_nodes=[find(sec.s_nodes >= st_s(k)); find(sec.s_nodes <= st_s(1))];
            end
            if isempty(panel_nodes)
            [~,panel_nodes]=min(abs(sec.s_nodes-st_s(k)));
            end
            q_local=max(q_bay(panel_nodes));
            tau=max(q_bay(panel_nodes)./sec.t_seg(panel_nodes));
            Pcr=plate_pcr(a,l_panel,P.t,P.E,P.nu);
            if tau/Pcr > worst_ratio,worst_ratio=tau/Pcr; worst_bay=bay; worst_panel=k; end
            if tau > Pcr,failed=true; end
            rows(end+1,:)={bay,bt,sprintf('panel%d',k),a,l_panel,q_local,tau,Pcr,false};
        end
        if strcmp(bt,'front')
            qwv=max(abs(qw_mat(1,bay)),abs(qw_mat(1,bay+1)));
            tau_w=qwv/P.spar_t;
            Pcr_w=plate_pcr(a,sec.web_len1,P.spar_t,P.E,P.nu);
            if tau_w/Pcr_w > worst_ratio,worst_ratio=tau_w/Pcr_w; worst_bay=bay; worst_panel=-1; end
            if tau_w > Pcr_w,failed=true; end
            rows(end+1,:)={bay,bt,'spar1web',a,sec.web_len1,qwv,tau_w,Pcr_w,false};
        end
    end
    if verbose
        for i=1:size(rows,1)
            print_row(rows{i,1},rows{i,2},rows{i,3},rows{i,4},rows{i,5},rows{i,6},rows{i,7},rows{i,8});
        end
    end
end
function [H,L]=draw_section(sec,x_SC,stringers,P)
    col_s1=[0 0.3 0.9];
    fl1=[0.4 0.65 1];
    H=gobjects(0); L={};
    for f=1:2
        nd=sec.fl_nodes{f};
        if isempty(nd),continue; end
        hh=plot(sec.x_node(nd),sec.y_node(nd),'-','Color',fl1,'LineWidth',9);
        if f == 1,H(end+1)=hh; L{end+1}='Spar 1 flanges (20 and 40 mm x 1 mm, toward TE)'; end
    end
    hh=plot([sec.x_node; sec.x_node(1)],[sec.y_node; sec.y_node(1)],'-','Color',[0.2 0.2 0.2],'LineWidth',1.5);
    H(end+1)=hh; L{end+1}=ternary(strcmp(sec.stype,'clip'),'Skin contour, clipped at c_{box} for aileron','Skin contour (0.5 mm)');
    if sec.has1
        hh=plot(sec.x_node([sec.idxS1_upper sec.idxS1_lower]),sec.y_node([sec.idxS1_upper sec.idxS1_lower]),'-','Color',col_s1,'LineWidth',3);
        H(end+1)=hh; L{end+1}='Spar 1 web (1 mm)';
    end
    [st_node,~,st_surf]=map_all_stringers(stringers,sec.x_node,P.c_full,sec.idx_upper,sec.idx_lower,sec.s_nodes);
    up=st_surf == 1; lo=st_surf == -1;
    if any(up)
        hh=plot(sec.x_node(st_node(up)),sec.y_node(st_node(up)),'o','MarkerFaceColor',[0 0.7 0],'MarkerEdgeColor','k','MarkerSize',12);
        H(end+1)=hh; L{end+1}='Upper stringers (lumped booms)';
    end
    if any(lo)
        hh=plot(sec.x_node(st_node(lo)),sec.y_node(st_node(lo)),'o','MarkerFaceColor',[0.85 0 0.85],'MarkerEdgeColor','k','MarkerSize',12);
        H(end+1)=hh; L{end+1}='Lower stringers (lumped booms)';
    end
    hh=plot(sec.xc,sec.yc,'ks','MarkerFaceColor','y','MarkerSize',14);
    H(end+1)=hh; L{end+1}='Centroid';
    hh=plot(x_SC,sec.yc,'kp','MarkerFaceColor','c','MarkerSize',20);
    H(end+1)=hh; L{end+1}='Shear centre';
end
function [xa,ya,ua,va,xw,yw,uw,vw]=arrow_data(sec,q_col,qw,scale)
    N=sec.N; i=(1:N)'; j=mod(i,N)+1;
    dx=sec.x_node(j)-sec.x_node(i); dy=sec.y_node(j)-sec.y_node(i); ds=hypot(dx,dy);
    xa=sec.x_mid; ya=sec.y_mid;
    ua=dx./ds.*q_col*scale;
    va=dy./ds.*q_col*scale;
    xw=[]; yw=[]; uw=[]; vw=[];
    for w=1:size(sec.web_nodes,1)
        iu=sec.web_nodes(w,1); il=sec.web_nodes(w,2);
        pu=[sec.x_node(iu) sec.y_node(iu)]; pl=[sec.x_node(il) sec.y_node(il)];
        len=norm(pl-pu);
        n=max(ceil(len/mean(sec.b_seg)),3);
        tt=((1:n)'-0.5)/n;
        pts=pu+tt*(pl-pu);
        dir=(pl-pu)/len;
        xw=[xw; pts(:,1)]; yw=[yw; pts(:,2)];
        uw=[uw; qw(w)*scale*dir(1)*ones(n,1)];
        vw=[vw; qw(w)*scale*dir(2)*ones(n,1)];
    end
end
function [xu,qu,xl,ql]=extract_surface_q(sec,q_col)
    idxU=sec.idx_upper; xu=sec.x_node(idxU); qu=q_col(idxU);
    idxL=sec.idx_lower; xl=sec.x_node(idxL); ql=q_col(idxL);
    [xu,ord]=sort(xu); qu=qu(ord);
    [xu,ia]=unique(xu,'stable'); qu=qu(ia);
    [xl,ord]=sort(xl); ql=ql(ord);
    [xl,ia]=unique(xl,'stable'); ql=ql(ia);
end
function yq=get_surface_y(sec,x_query,surface)
    if surface > 0,idx=sec.idx_upper; else,idx=sec.idx_lower; end
    xs=sec.x_node(idx); ys=sec.y_node(idx);
    [xs,ord]=sort(xs); ys=ys(ord);
    [xs,ia]=unique(xs,'stable'); ys=ys(ia);
    x_query=min(x_query,max(xs));
    yq=interp1(xs,ys,x_query,'linear','extrap');
end
function [vu,vl]=surf_row_data(sec,col,xq,c_full)
    [xu,vu_raw,xl,vl_raw]=extract_surface_q(sec,col);
    xu=xu/c_full; xl=xl/c_full;
    x_max=max(sec.x_node)/c_full;
    maskU=xu <= x_max+1e-8; maskL=xl <= x_max+1e-8;
    xu=xu(maskU); vu_raw=vu_raw(maskU); xl=xl(maskL); vl_raw=vl_raw(maskL);
    if numel(xu) >= 2,vu=interp1(xu,vu_raw,xq,'linear',NaN); else,vu=nan(size(xq)); end
    if numel(xl) >= 2,vl=interp1(xl,vl_raw,xq,'linear',NaN); else,vl=nan(size(xq)); end
    vu(xq > x_max+1e-8)=NaN;
    vl(xq > x_max+1e-8)=NaN;
end
function [Xgrid,Ygrid,ValUp,ValLo]=build_surface_grids_bays(rib_y,bay_sec,bay_col_lo,bay_col_hi,c_full)
    nBay=numel(bay_sec);
    xq=linspace(0,1,120);
    eps_y=1e-4*max(rib_y);
    Yrows=[]; Uprows=[]; Lorows=[];
    for k=1:nBay
        [vu_lo,vl_lo]=surf_row_data(bay_sec{k},bay_col_lo{k},xq,c_full);
        [vu_hi,vl_hi]=surf_row_data(bay_sec{k},bay_col_hi{k},xq,c_full);
        y_lo=rib_y(k) + (k>1)*eps_y;
        y_hi=rib_y(k+1) - (k<nBay)*eps_y;
        Yrows=[Yrows; y_lo; y_hi];
        Uprows=[Uprows; vu_lo; vu_hi];
        Lorows=[Lorows; vl_lo; vl_hi];
    end
    Ygrid=repmat(Yrows,1,length(xq));
    Xgrid=repmat(xq,size(Yrows,1),1);
    ValUp=Uprows; ValLo=Lorows;
end
function [A_str,Imin]=equal_leg_angle_props(b,t_st)
    A_str=t_st*(2*b-t_st);
    Imin=t_st*b^3/12;
end
function sigma_cc=gerard_crippling(b,t_st,E,sigma_cy)
    sigma_cc=0.56*sigma_cy*((t_st/b)*sqrt(E/sigma_cy))^0.75;
end
function sigma_cc=simple_kc_crippling(b,t_st,E,nu,Kc)
    sigma_cc=Kc*(pi^2*E/(12*(1-nu^2)))*(t_st/b)^2;
end
function sigma_col=column_buckling_stress(b,t_st,E,L,c_col)
    [A_str,Imin]=equal_leg_angle_props(b,t_st);
    sigma_col=c_col*pi^2*E*Imin/(A_str*L^2);
end
