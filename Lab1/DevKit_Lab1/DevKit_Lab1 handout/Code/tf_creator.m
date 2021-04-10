% process data and determine the transfer function for each case

files={'sim_step_0_5_1_0.mat', 'height_step_0_5_1_0.mat';
    'sim_step_1_0_0_5.mat', 'height_step_1_0_0_5.mat';
    'sim_step_2_4_0_25.mat', 'height_step_2_4_0_25.mat';
    'sim_step_3_2_0_2.mat', 'height_step_3_2_0_2.mat'};

height_par={ 0.5,   1,      'sim_step_0_5_1_0.mat';
             1,     0.5,    'sim_step_1_0_0_5.mat';
             2.4,   0.25,   'sim_step_2_4_0_25.mat';
             3.2,   0.2,    'sim_step_3_2_0_2.mat'};

data={'0.5_1.0';
      '1.0_0.5';
      '2.4_0.25';
      '3.2_0.3'};
  
color1=[0 0.4470 0.7410];
color2=[0.6350 0.0780 0.1840];
color3=[0.4660 0.6740 0.1880];
color4=[0.8500 0.3250 0.0980];
lw=1;

end_time=[30,26,26,30];
n_poles=2;
n_zeros=0;
Ts=0.065; %sampling time
for i=1:length(files)
    load(files{i,1}) % data from sim
    load(files{i,2}, 'states')
    real_time=states.time; % time
    real_height=states.signals.values(:,9); 
    tfdata=iddata(real_height(round(14.8/Ts):round(end_time(i)/Ts)), ...
        h_ref.Data(round(14.8/Ts):round(end_time(i)/Ts)), Ts);
    sys=tfest(tfdata,n_poles,n_zeros);
    data{i,2}=sys.Numerator;
    data{i,3}=sys.Denominator;
    data{i,4}=sys.Report.Fit.FitPercent;
end

for i=1:length(data)
   opt = stepDataOptions;
   opt.InputOffset=1;
   opt.StepAmplitude=height_par{i,2};
   sys=tf(data{i,2}, data{i,3});
   data{i,5}=sys;
   
   % plot estimation with the real one and the simulated
   initial_time=14; % in seconds
   sys.IODelay=15;
   y=step(sys,(initial_time:Ts:end_time(i)),opt); % create the step
   figure('Name',join(['tf_' data{i,1}])) 
   plot(initial_time:Ts:end_time(i),y,'Color',color1,'LineWidth',1.5) 
   load(files{i,1}) % data from sim
   load(files{i,2}, 'states')
   real_time=states.time; % time
   real_height=states.signals.values(:,9); 
   hold on
   plot(real_time(round(initial_time/Ts):round(end_time(i)/Ts)), ...
       real_height(round(initial_time/Ts):round(end_time(i)/Ts)),...
       'Color',color2,'LineWidth',1.5);
   plot(h_ref.Time(round(initial_time/Ts):round(end_time(i)/Ts)),...
       h_ref.Data(round(initial_time/Ts):round(end_time(i)/Ts)),...
       'Color',color3,'LineWidth',lw);
   xlabel('Tempo [s]');
   ylabel('Altura [m]');
   legend('h(t) estimado','h(t) real','h(t) de referência',...
        'Location','southeast')
end


figure('Name','pole_zero_map')
for i=1:length(data)-1
    [wn,zeta,p]=damp(data{i,5});
    hold on
    real_part=real(p);
    imag_part=imag(p);
    scatter(real_part,imag_part,'filled')
end
legend('k_p=0.5','k_p=1.0','k_p=2.4','k_p=3.2','Location','northwest')
