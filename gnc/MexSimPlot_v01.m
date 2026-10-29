% clear all;clc; 
% close all;    
function [] = MexSimPlot_v01(h)
% h=load('plsfile.DAT');
% load('plsfile.mat');h = R2000';
        
ts =0.;   tf = 10000.;
% ts =600.;   tf = 750.;
stg =6;


h=h(h(:,1)<tf&h(:,1)>ts,:);
r2d=180./pi;
d2r = 1/r2d;

sim_fig = figure;   
    
h_tabgp = uitabgroup(sim_fig);

tab1  = uitab(h_tabgp,'Title','Tetam');
tab2  = uitab(h_tabgp,'Title','phim');
tab3  = uitab(h_tabgp,'Title','ayc');
tab4  = uitab(h_tabgp,'Title','apc');
tab5  = uitab(h_tabgp,'Title','gama');
tab6  = uitab(h_tabgp,'Title','psim');
tab7  = uitab(h_tabgp,'Title','Wx');
tab8  = uitab(h_tabgp,'Title','Wy');
tab9  = uitab(h_tabgp,'Title','Wz');
tab10 = uitab(h_tabgp,'Title','Ax');
tab11 = uitab(h_tabgp,'Title','Ay');
tab12 = uitab(h_tabgp,'Title','Az');
tab13 = uitab(h_tabgp,'Title','Vm');
tab14 = uitab(h_tabgp,'Title','slant_range');
tab15 = uitab(h_tabgp,'Title','Xcp');
% tab16 = uitab(h_tabgp,'Title','Vm');
tab17 = uitab(h_tabgp,'Title','XYZ');
tab18 = uitab(h_tabgp,'Title','X&Hm');
% tab19 = uitab(h_tabgp,'Title','Z');
% tab20 = uitab(h_tabgp,'Title','H');


tab21 = uitab(h_tabgp,'Title','q');
tab22 = uitab(h_tabgp,'Title','stage');
tab23 = uitab(h_tabgp,'Title','alfa');
tab24 = uitab(h_tabgp,'Title','dela');
tab25 = uitab(h_tabgp,'Title','delr');
tab26 = uitab(h_tabgp,'Title','dele');
tab27 = uitab(h_tabgp,'Title','dac1');
tab28 = uitab(h_tabgp,'Title','dac2.');
tab29 = uitab(h_tabgp,'Title','duPitch.');
tab30 = uitab(h_tabgp,'Title','duRoll');
tab31 = uitab(h_tabgp,'Title','duYaw');
tab32 = uitab(h_tabgp,'Title','wxf');
tab33 = uitab(h_tabgp,'Title','wyf');
tab34 = uitab(h_tabgp,'Title','wzf');
tab35 = uitab(h_tabgp,'Title','Err');
tab36 = uitab(h_tabgp,'Title','mass');
tab37 = uitab(h_tabgp,'Title','thrust');
tab38 = uitab(h_tabgp,'Title','gains');

tab46 = uitab(h_tabgp,'Title','Pitch');
tab47 = uitab(h_tabgp,'Title','Yaw');
tab48 = uitab(h_tabgp,'Title','Roll');

axes('parent',tab1);plot(h(:,1),h(:,16),h(:,1),h(:,17),'LineWidth',1.);grid on;title('tetam(deg)');legend('FD','CMD');
axes('parent',tab2);plot(h(:,1),h(:,14),'LineWidth',1.);grid on;title('phim (deg)');%legend('eros','stc');
axes('parent',tab3);plot(h(:,1),h(:,19),h(:,1),h(:,18)/1,'LineWidth',1.);grid on;title('ayc (g)');legend('CMD','FD');
axes('parent',tab4);plot(h(:,1),h(:,21),h(:,1),h(:,20)/1,'LineWidth',1.);grid on;title('apc (g)');legend('CMD','FD');
% axes('parent',tab5);
% subplot(2,2,1);plot(h(:,1),h(:,39),h(:,1),h(:,38),'LineWidth',1.);grid on;title('gama(deg)');legend('CMD','FD');
% subplot(2,2,2);plot(h(:,1),h(:,56)*r2d,'LineWidth',1.);grid on;title('du gama(deg) ');legend('Wx filter','Wx filter1');
% subplot(2,2,3);plot(h(:,1),h(:,57)*r2d,'LineWidth',1.);grid on;title('gama corec(deg) ');legend('Wx filter','Wx filter1');
axes('parent',tab5);plot(h(:,1),h(:,65),'LineWidth',1.);grid on;title('gama(deg)');legend('CMD','FD');

axes('parent',tab6);plot(h(:,1),h(:,15),'LineWidth',1.);grid on;title('psi(deg) ');%legend('Fusion','INS');
axes('parent',tab7);plot(h(:,1),h(:,11)*r2d,'LineWidth',1.);grid on;title('wx(deg) ');%legend('Fusion','INS');
axes('parent',tab8);plot(h(:,1),(h(:,12)*r2d),h(:,1),h(:,59)*r2d,'LineWidth',1.);grid on;title('wy(deg) ');legend('wy','wyf');
axes('parent',tab9);plot(h(:,1),h(:,13)*r2d,h(:,1),h(:,60)*r2d,'LineWidth',1.);grid on;title('wz(deg) ');legend('wz','wzf');
axes('parent',tab10);plot(h(:,1),h(:,8)/9.81,'LineWidth',1.);grid on;title('ax(g) ');%legend('Fusion','INS');
axes('parent',tab11);plot(h(:,1),h(:,9)/9.81,'LineWidth',1.);grid on;title('ay(g) ');%legend('Fusion','INS');
axes('parent',tab12);plot(h(:,1),h(:,10)/9.81,'LineWidth',1.);grid on;title('az(g) ');%legend('Fusion','INS');
axes('parent',tab13);
subplot(2,2,1);plot(h(:,1),h(:,5),'LineWidth',1.);grid on;ylabel('Vx(m/s)','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('vx-(vf0)','vx-(vl0)');
subplot(2,2,2);plot(h(:,1),h(:,6),'LineWidth',1.);grid on;ylabel('Vy(m/s)','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('vy-(vf1)','vy-(vl1)');
subplot(2,2,3);plot(h(:,1),h(:,7),'LineWidth',1.);grid on;ylabel('Vz(m/s)','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('vz-(vf2)','vz-(vl2)');
subplot(2,2,4);plot(h(:,1),h(:,34),h(:,1),h(:,26)/340,'LineWidth',1.);grid on;ylabel('Vm(m/s)','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('vm-(vf)','vm-(vl)');

axes('parent',tab14);plot(h(:,1),h(:,61),'LineWidth',1.);grid on;ylabel('slant_range(m)','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('ins','gps');
axes('parent',tab15);
subplot(2,2,1);plot(h(:,1),h(:,72),'LineWidth',1.);grid on;ylabel('static margin','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('ins','gps');
subplot(2,2,2);plot(h(:,34),h(:,72),'LineWidth',1.);grid on;ylabel('static margin','fontsize',15);xlabel('mach','fontsize',15);%legend('ins','gps');
subplot(2,2,3);plot(h(:,1),h(:,70),'LineWidth',1.);grid on;ylabel('xcg(m)','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('ins','gps');

axes('parent',tab17);
subplot(2,2,1);plot(h(:,1),h(:,2),h(:,1),h(:,67),'LineWidth',1.);grid on;title('X');%legend('X (r-l0)','r-l-ins0','X-rf0');
subplot(2,2,2);plot(h(:,1),h(:,3),h(:,1),h(:,68),'LineWidth',1.);grid on;title('Y');%legend('Y_(r_l0)','r_l_ins1','Y_rf1');
subplot(2,2,3);plot(h(:,1),h(:,4),h(:,1),h(:,69),'LineWidth',1.);grid on;title('Z');%legend('Y_(r_l1)','r_l_ins2','Z_rf2');
subplot(2,2,4);plot(h(:,1),h(:,25),'LineWidth',1.);grid on;ylabel('h(m)','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('ins','gps');

% axes('parent',tab18);plot3(h(:,2),h(:,4),h(:,25),'LineWidth',1.);grid on;ylabel('x vs h(m)','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('ins','gps');
axes('parent',tab18);plot(h(:,2),h(:,25),'LineWidth',1.);grid on;ylabel('x vs h(m)','fontsize',15);xlabel('time(sec)','fontsize',15);%legend('ins','gps');

axes('parent',tab21);plot(h(:,1),h(:,36),h(:,1),h(:,35)/10000.,'LineWidth',1.);grid on;title('q (N/cm^2)');%legend('Fusion','INS');
axes('parent',tab22);plot(h(:,1),h(:,37),'LineWidth',1.);grid on;title('stage');%legend('Fusion','INS');
axes('parent',tab23);
subplot(3,1,1);plot(h(:,1),h(:,27),'LineWidth',1.);grid on;title('alfa deg)');%legend('MD_e','MD_g');
subplot(3,1,2);plot(h(:,1),h(:,28),'LineWidth',1.);grid on;title('beta deg)');%legend('MD_e','MD_g');
subplot(3,1,3);plot(h(:,1),h(:,63),'LineWidth',1.);grid on;title('alfab deg)');%legend('MD_e','MD_g');


axes('parent',tab24);plot(h(:,1),h(:,87),'LineWidth',1.);grid on;title('dela (deg)');legend('dela','delac');
axes('parent',tab25);plot(h(:,1),h(:,88),h(:,1),h(:,90),'LineWidth',1.);grid on;title('delrc (deg)');legend('delrc0','delrc');
axes('parent',tab26);plot(h(:,1),h(:,89),h(:,1),h(:,91),'LineWidth',1.);grid on;title('delec (deg)');legend('delec0','delec');
axes('parent',tab27);plot(h(:,1),h(:,43),h(:,1),h(:,45),'LineWidth',1.);grid on;title('dac1&3(deg)');legend('dac1','dac3');
axes('parent',tab28);plot(h(:,1),h(:,44),h(:,1),h(:,46),'LineWidth',1.);grid on;title('dac2&4(deg) ');legend('dac2','dac4');
axes('parent',tab29);plot(h(:,1),h(:,54),'LineWidth',1.);grid on;title('dac3(deg)');%legend('Fusion','INS');
axes('parent',tab30);plot(h(:,1),h(:,53),'LineWidth',1.);grid on;title('dac4(deg) ');%legend('Fusion','INS');
axes('parent',tab31);plot(h(:,1),h(:,55),'LineWidth',1.);grid on;title('psi(deg) ');%legend('Fusion','INS');
axes('parent',tab32);plot(h(:,1),h(:,58)*r2d,h(:,1),h(:,67)*r2d,'LineWidth',1.);grid on;title('Wx filter(deg) ');legend('Wx filter','Wx filter1');
axes('parent',tab33);plot(h(:,1),h(:,59)*r2d,h(:,1),h(:,68)*r2d,'LineWidth',1.);grid on;title('Wy filter(deg) ');legend('Wy filter','Wy filter1');
axes('parent',tab34);plot(h(:,1),h(:,60)*r2d,h(:,1),h(:,69)*r2d,'LineWidth',1.);grid on;title('Wz filter(deg) ');legend('Wz filter','Wz filter1');
axes('parent',tab35);
 subplot(2,2,1);plot(h(:,1),h(:,73),'LineWidth',1.);grid on;title('x err0 (m) ');%legend('Fusion','INS');
 subplot(2,2,2);plot(h(:,1),h(:,74),'LineWidth',1.);grid on;title('z err1 (m) ');%legend('Fusion','INS');
 subplot(2,2,3);plot(h(:,1),h(:,75),'LineWidth',1.);grid on;title('y err2 (m) ');%legend('Fusion','INS');
%  subplot(2,2,4);plot(h(:,1),h(:,32),'LineWidth',1.);grid on;title('Iyy ');%legend('Fusion','INS');
axes('parent',tab36);
 subplot(2,2,1);plot(h(:,1),h(:,29),'LineWidth',1.);grid on;title('mass(kg) ');%legend('Fusion','INS');
 subplot(2,2,2);plot(h(:,1),h(:,30),'LineWidth',1.);grid on;title('xbc ');%legend('Fusion','INS');
 subplot(2,2,3);plot(h(:,1),h(:,31),'LineWidth',1.);grid on;title('Ixx ');%legend('Fusion','INS');
 subplot(2,2,4);plot(h(:,1),h(:,32),'LineWidth',1.);grid on;title('Iyy ');%legend('Fusion','INS');


axes('parent',tab37);plot(h(:,1),h(:,33)/9.81,h(:,1),h(:,62)/9.81,'LineWidth',1.);grid on;title('thrust(kg) ');legend('booster','stg2');
% 
% 
 axes('parent',tab38);
 subplot(3,3,1);plot(h(:,1),h(:,76),'LineWidth',1.);grid on;title('Kwp');%legend('Fusion','INS');
 subplot(3,3,2);plot(h(:,1),h(:,77),'LineWidth',1.);grid on;title('kpp ');%legend('Fusion','INS');
 subplot(3,3,3);plot(h(:,1),h(:,78),'LineWidth',1.);grid on;title('kip ');%legend('Fusion','INS');
 subplot(3,3,4);plot(h(:,1),h(:,79),'LineWidth',1.);grid on;title('kwac ');%legend('Fusion','INS');
 subplot(3,3,5);plot(h(:,1),h(:,80),'LineWidth',1.);grid on;title('kpac ');%legend('Fusion','INS');
 subplot(3,3,6);plot(h(:,1),h(:,81),'LineWidth',1.);grid on;title('kiac ');%legend('Fusion','INS');
 subplot(3,3,7);plot(h(:,1),h(:,82),'LineWidth',1.);grid on;title('kwr ');%legend('Fusion','INS');
 subplot(3,3,8);plot(h(:,1),h(:,83),'LineWidth',1.);grid on;title('kpr ');%legend('Fusion','INS');
 subplot(3,3,9);plot(h(:,1),h(:,84),'LineWidth',1.);grid on;title('kir ');%legend('Fusion','INS');

  axes('parent',tab46);
 subplot(2,2,1);plot(h(:,1),h(:,16),h(:,1),h(:,17),'LineWidth',1.);grid on;title('tetam(deg)');legend('FD','CMD');
 subplot(2,2,2);plot(h(:,1),h(:,54)*r2d,'LineWidth',1.);grid on;title('du pitch(deg)');%legend('Fusion','INS');
 subplot(2,2,3);plot(h(:,1),h(:,24),h(:,1),h(:,91)*r2d,'LineWidth',1.);grid on;title('delec (deg)');legend('delec0','delec');
%  subplot(2,2,4);plot(h(:,1),h(:,79),'LineWidth',1.);grid on;title('kwac ');%legend('Fusion','INS');

  axes('parent',tab47);
 subplot(2,2,1);plot(h(:,1),h(:,19),h(:,1),h(:,62)/9.81,'LineWidth',1.);grid on;title('ayc (g)');legend('CMD','FD');
 subplot(2,2,2);plot(h(:,1),h(:,55),'LineWidth',1.);grid on;title('du yaw(deg) ');%legend('Fusion','INS');
 subplot(2,2,3);plot(h(:,1),h(:,23),h(:,1),h(:,90)*r2d,'LineWidth',1.);grid on;title('delrc (deg)');legend('delrc0','delrc');
%  subplot(2,2,4);plot(h(:,1),h(:,79),'LineWidth',1.);grid on;title('kwac ');%legend('Fusion','INS');

  axes('parent',tab48);
 subplot(2,2,1);plot(h(:,1),h(:,14),'LineWidth',1.);grid on;title('phim (deg)');%legend('eros','stc');
 subplot(2,2,2);plot(h(:,1),h(:,53)*r2d,'LineWidth',1.);grid on;title('du roll(deg) ');%legend('Fusion','INS');
 subplot(2,2,3);plot(h(:,1),h(:,22),h(:,1),h(:,87)*r2d,'LineWidth',1.);grid on;title('dela (deg)');legend('dela','delac');
%  subplot(2,2,4);plot(h(:,1),h(:,79),'LineWidth',1.);grid on;title('kwac ');%legend('Fusion','INS');

% axes('parent',tab39);plot(h(:,1),h(:,93),'LineWidth',1.);grid on;title('psi(deg) ');%legend('Fusion','INS');
% axes('parent',tab40);plot(h(:,1),h(:,95),'LineWidth',1.);grid on;title('psi(deg) ');%legend('Fusion','INS');
% axes('parent',tab41);plot(h(:,1),h(:,94),'LineWidth',1.);grid on;title('psi(deg) ');%legend('Fusion','INS');
% axes('parent',tab42);plot(h(:,1),h(:,96),'LineWidth',1.);grid on;title('psi(deg) ');%legend('Fusion','INS');
% axes('parent',tab43);plot(h(:,1),h(:,97),'LineWidth',1.);grid on;title('psi(deg) ');%legend('Fusion','INS');
% axes('parent',tab44);plot(h(:,1),h(:,98),'LineWidth',1.);grid on;title('psi(deg) ');%legend('Fusion','INS');
% axes('parent',tab45);plot(h(:,1),h(:,99),'LineWidth',1.);grid on;title('psi(deg) ');%legend('Fusion','INS');

Xer=h(end,2)-h(end,67);
Yer=h(end,3)-h(end,68);
Zer=h(end,4)-h(end,69);
XYZ1_m2t = [Xer,Yer,Zer]
XYZ2 = [h(end,73),h(end,75),h(end,74)]

%% gains
% h=load('PLSFILE.DAT');r2d=180./pi;
% 
% h_fig_gains = figure;h_tabgp_gains = uitabgroup(h_fig_gains);
% 
% tab0_g  = uitab(h_tabgp_gains,'Title','dynpress');
% tab1_g  = uitab(h_tabgp_gains,'Title','Kwp');   tab2_g = uitab(h_tabgp_gains,'Title','kpp');
% tab3_g  = uitab(h_tabgp_gains,'Title','kip');   tab4_g = uitab(h_tabgp_gains,'Title','kwac');
% tab5_g  = uitab(h_tabgp_gains,'Title','kpac');  tab6_g = uitab(h_tabgp_gains,'Title','kiac');
% tab7_g  = uitab(h_tabgp_gains,'Title','kwr');   tab8_g = uitab(h_tabgp_gains,'Title','kpr');
% tab9_g  = uitab(h_tabgp_gains,'Title','kir');   tab10_g = uitab(h_tabgp_gains,'Title','kpg');
% tab11_g = uitab(h_tabgp_gains,'Title','kig');
% 
% axes('parent',tab0_g);plot(h(:,1),h(:,36),h(:,1),h(:,35)/10000.,'LineWidth',1.);grid on;title('q (N/cm^2)');%legend('Fusion','INS');
% axes('parent',tab1_g ); plot(h(:,1),h(:,77),'LineWidth',1.);grid on;title('Kwp');
% axes('parent',tab2_g ); plot(h(:,1),h(:,78),'LineWidth',1.);grid on;title('Kpp');
% axes('parent',tab3_g ); plot(h(:,1),h(:,79),'LineWidth',1.);grid on;title('Kip');
% axes('parent',tab4_g ); plot(h(:,1),h(:,80),'LineWidth',1.);grid on;title('Kwac');
% axes('parent',tab5_g ); plot(h(:,1),h(:,81),'LineWidth',1.);grid on;title('Kpac');
% axes('parent',tab6_g ); plot(h(:,1),h(:,82),'LineWidth',1.);grid on;title('Kiac');
% axes('parent',tab7_g ); plot(h(:,1),h(:,83),'LineWidth',1.);grid on;title('Kwr');
% axes('parent',tab8_g ); plot(h(:,1),h(:,84),'LineWidth',1.);grid on;title('Kpr');
% axes('parent',tab9_g ); plot(h(:,1),h(:,85),'LineWidth',1.);grid on;title('Kir');
% axes('parent',tab10_g );plot(h(:,1),h(:,86),'LineWidth',1.);grid on;title('Kpg');
% axes('parent',tab11_g );plot(h(:,1),h(:,87),'LineWidth',1.);grid on;title('Kig');
% cftool(h(14615:end,36),h(14615:end,81))
% cftool(h(14615:100:end,36),h(14615:100:end,80))

     
%      K_p_integ_acc = (1.304e+04) / (q*q*q + 9201*q*q + -1.442e+04*q + 1.048e+04);
% 
%           K_p_integ_acc = (1.436) / (q*q -1.55*q + 1.129);

end