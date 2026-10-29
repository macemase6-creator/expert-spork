% function [gains] = get_pid2(chnl)

 clear all;
    % clc; 
    % close all

r2d=180./pi;    d2r = pi/180.;


mex  mex_sim.CPP;
%%

tic
clear pls du_err pls


sim.Ts = 0;
sim.Tf = 1000;%1000;
sim.CtrlFlg =1;
sim.flg = 3;    
% sim.AeroNoise = 1;     
sim.HGV =0;     
sim.thr_type=2;
sim.TipOff = 0;     
sim.NoiseChoice = 1;
sim.PIDTuneFlg = 0;   
sim.Ctrl_Mode=1;
sim.Stage = 1;  
sim.PrintFlg =1;  
sim.FinWear=0;
sim.FinCorr=1;
% sim.TpSet =-1;     
sim.Channel = 2;% 1->roll     2-> ayc     3-> p_pp    4-> xyz
sim.MotorFlg = 0;
% sim.ChnlName = 'roll';%     2-> ayc     3-> p_pp    4-> xyz
sim.RangeChoice = 0; %  1->up%down  2-> k_test 0.5 & 1.5  
% sim.range_typ = [1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sim.MultiTrgt = 0;%[1,2,3,4]
sim.Stg6Guid = 0;% 0 => LOS 	1 => PN
sim.ReentryIgn =0;% 0 => No ignition in stg6	1=> stg6 ignition
sim.TalFlag=1;

% Coordinate selection:
%   ManualMode = 0 -> use the six coordinates below (manual mode)
%   ManualMode = 1 -> use TrgtCnt0/launp0 and the catalogue files
sim.ManualMode = 1;

% Manual coordinates [latitude deg, longitude deg, altitude m]
sim.Latl  = 15.333333;
sim.Longl = 44.183333;
sim.Hl    = 2000;
sim.Latt  = 16.003601;
sim.Longt = 40.872898;
sim.Ht    = 40;

launp0=[2,2,2,2,2,2,2,2,2,2,2,2,2,2,2];
% TrgtCnt0=[13,14,15,16,17,18,19,20,21,22,23,24,25,26];
TrgtCnt0=[1,2,3,4,5,6,7,8,9,10,11,12,13,14];
%1900,1800,1700,1600,1500,1400,1300,1200,1100,1000,900,800,700,1958

% TrgtCnt0=[1,2,4,6,7,8,9,10,11,12,13];
%1900,1800,1600,1500,1300,1200,1100,1000,900,800,700

 % TrgtCnt0=[14,15,17,18,20,21,23,24,25,26];
%1900,1800,1600,1500,1300,1200,1000,900,800,700

% TrgtCnt0=[4,5,6,7,8,9,10,11,12];
%1600,1500,1400,1300,1200,1100,1000,900,800
% 
% % TrgtCnt0=[9,10,11,12];
% % 
    TrgtCnt0=[1,2,4,5,8,9,10,12];
 %1900,1800,1600,1500,1200,1100,1000,800
   % TrgtCnt0=[9,10,11,12];
   % TrgtCnt0=[1,2,3,4,5,6,7,8];
  %    TrgtCnt0=[7,8,9,10,11,12];
  % TrgtCnt0=[1,3,6,8,10,12];
 %1100,1000,900,800
        % TrgtCnt0=[9,10,11,12];%800
            % TrgtCnt0=[5];%800
         TrgtCnt0=[13,14,15,16];%800
  %      TrgtCnt0=[7,8,9,10];%800
    % TrgtCnt0=[1,2,3,4,14];%800
                     TrgtCnt0=[15];
 NoiseChoice=[0];
  NoiseChoice=[0,1,2,3,4,5];
% NoiseChoice=[0];
% NoiseChoice=[1,2,3,4,5];

            % NoiseChoice=[1,2,5];
% 
% launp0=[1];
gains = 0;
cntj=0; 
cnt=0;
nchoice=3;  
% nchoice=4;  

    nchoice=5;

for k=1:length(TrgtCnt0)%TrgtCnt%*2
        k
    %         con =  mod(k,2);
        cnt=cnt+1
        sim.TrgtCnt0=TrgtCnt0(cnt);
        sim.launp0=launp0(cnt);
        
        cntj=0;
        mex  mex_sim.CPP;
        sim.nchoice = nchoice;
        if nchoice==5 %&& con==0
            for j=1:length(NoiseChoice)

                cntj=cntj+1;
                sim.NoiseChoice = NoiseChoice(j);%j;%i;
                [~,pls{cnt,cntj}] = mex_sim(sim,gains');

            end
            XYZ1_m2t=MexSimPlot_all_v01((pls(cnt,:))',sim);
            XYZ_1{cnt}=XYZ1_m2t;
%             data(cnt,1:7) = pls2{cnt}(1,1:7);
        elseif nchoice ==3 %|| nchoice ==4 %&& con==1
            for n=1:2
                % n
                if n==2
                    mex  mex_sim.CPP;
                end
                sim.nchoice = n+2;
                sim.Tf = 1000;
                sim.PrintFlg =0;  
                sim.NoiseChoice = 0;
    
                [du_err0,pls{cnt,1}] = mex_sim(sim,gains');
            end
            % pls=pls0;
        end
    end
% end
toc
disp('################ Simulation ends ################')
%%
if(nchoice==5)
   % [tele]= R2000_OcVsTele(pls,sim,15);return;
end
if(nchoice==5)
     % [tele]= R2000_OcVsTele(pls,sim);
    splot.print=0;
    plot_flg=0;
    answer = questdlg('Would you like to plot results?','Select choice ','Yes','No','No');
    % Handle response
    switch answer
        case 'Yes'  ,plot_flg = 1;
        case 'No'   ,plot_flg = 0;
    end
 % plot_flg = 1;
    if( plot_flg)
        splot.NoiseChoice = NoiseChoice;
        for j=1:length(TrgtCnt0)%TrgtCnt
            trgt=j;
            splot.Rng=trgt;
            splot.TrgtCnt0=TrgtCnt0(j);
            splot.launp0 = launp0(j);
              XYZ1_m=MexSimPlot6_v01((pls(trgt,:))',sim,splot);
           %  [XYZ1_m2t] = MexSimPlot6_sum_v01((pls(trgt,:))',sim,splot);
        end
    end


  % h=cell2mat(pls);
  % h = h(1:nnz(h(:,1)),:);
% idx=h(:,1)>=0 & h(:,1)<=100;%34.57+5;
% idx3=h(:,1)>=0 & h(:,1)<=100;%34.57+5;
% 
  % idx=h(:,1)>=0 & h(:,1)<=37.8499;
  % total=trapz(h(idx,1),h(idx,33))
% thr_3=[h(idx3,1),h(idx3,64)];
% thr_2=[h(idx,1),h(idx,52)];
% % % sthr_2=[h(idx,1),h(idx,62)];
% 
% total_2=trapz(thr_2(thr_2(:,2)>0,1),thr_2(thr_2(:,2)>0,2));
% 
% total_3=trapz(thr_3(thr_3(:,2)>0,1),thr_3(thr_3(:,2)>0,2));
end

%%
% j=1;
% trgt=j;
% splot.Rng=trgt;
% splot.TrgtCnt0=TrgtCnt0(j);
% splot.launp0 = launp0(j);
% XYZ1_m=MexSimPlot6_v01((pls(trgt,:))',sim,splot);
