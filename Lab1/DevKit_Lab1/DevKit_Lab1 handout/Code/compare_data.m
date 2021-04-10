% scrip to compare data from simulation and real results

color1=[0 0.4470 0.7410];
color2=[0.6350 0.0780 0.1840];
color3=[0.4660 0.6740 0.1880];
color4=[0.8500 0.3250 0.0980];
lw=2;

files={'sim_step_0_5_1_0.mat', 'height_step_0_5_1_0.mat','comp_0_5_1_0';
    'sim_step_1_0_0_5.mat', 'height_step_1_0_0_5.mat','comp_1_0_0_5';
    'sim_step_2_4_0_25.mat', 'height_step_2_4_0_25.mat', 'comp_2_4_0_25';
    'sim_step_3_2_0_2.mat', 'height_step_3_2_0_2.mat', 'comp_3_2_0_2'};

end_time=[30,26,26,30];
Ts=0.065;
Ts_data=0.005;
for i=1:length(files)
    load(files{i,1}) % data from sim
    load(files{i,2}, 'states')
    real_time=states.time;
    real_height=states.signals.values(:,9);
    figure('Name', files{i,3})
    plot(h_sim.Time(1:round(end_time(i)/Ts_data)),...
        h_sim.Data(1:end_time(i)/Ts_data),'Color',color1,'LineWidth',lw)
    hold on 
    plot(real_time(1:round(end_time(i)/Ts)), ...
        real_height(1:round(end_time(i)/Ts)),'Color',color2,'LineWidth',lw)
    plot(h_ref.Time(1:round(end_time(i)/Ts)),...
        h_ref.Data(1:round(end_time(i)/Ts)),'LineWidth',1)
    xlabel('Tempo [s]');
    ylabel('Altura [m]');
    legend('h(t) simulado','h(t) real','h(t) de referência',...
        'Location','southeast')
end