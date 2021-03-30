% scrip for generate data from simulation

height_par={ 0.5,   1,      'sim_step_0_5_1_0.mat';
             1,     0.5,    'sim_step_1_0_0_5.mat';
             2.4,   0.25,   'sim_step_2_4_0_25.mat';
             3.2,   0.2,    'sim_step_3_2_0_2.mat'};
for i=1:length(height_par)
    Kp=height_par{i,1};
    h_input=height_par{i,2};
    sim('ARDroneHoverSim') %output are h_ref and h_sim
    save (height_par{i,3}, 'h_ref', 'h_sim')
    clear h_ref h_sim
end