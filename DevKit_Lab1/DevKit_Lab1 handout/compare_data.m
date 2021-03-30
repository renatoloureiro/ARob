% scrip to compare data from simulation and real results

files={'sim_step_0_5_1_0.mat', 'height_step_0_5_1_0.mat';
    'sim_step_1_0_0_5.mat', 'height_step_1_0_0_5.mat';
    'sim_step_2_4_0_25.mat', 'height_step_2_4_0_25.mat';
    'sim_step_3_2_0_2.mat', 'height_step_3_2_0_2.mat'};

for i=1:length(files)
    load(files{i,1}) % data from sim
    load(files{i,2}, 'states')
    real_time=states.time;
    real_height=states.signals.values(:,9);
    figure()
    plot(h_sim.Time,h_sim.Data)
    hold on 
    plot(real_time, real_height)
end