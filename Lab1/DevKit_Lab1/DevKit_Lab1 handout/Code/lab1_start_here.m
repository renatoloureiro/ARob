run(join([pwd '/ARDroneSimulinkDevKit_V1.1/start_here.m']));

if ismember(1, strcmp('ARDroneHoverSim', find_system('SearchDepth',0)))==1
    rewrite_sim_data=1; % if I want to rewrite -> 1
    if rewrite_sim_data==1
        run('generate_data')
    end
    run('compare_data') %plots the real and simulated gain/reference input
    
    save_plots=1; % if I want to save the plots as pdf files -> 1
    if save_plots==1
        if ~exist('Img', 'dir')
            mkdir('Img')
        end
        save_plots_pdf()
    end  
    
    run('tf_creator') %creates tf for each case and plots a comparison
    if save_plots==1
        if ~exist('Img', 'dir')
            mkdir('Img')
        end
        save_plots_pdf()
    end  
    
    
else
   disp('Simulink model not open');
   exit(0);
end


% -------------------------------------------------------------------

function save_plots_pdf()
    figHandles = get(groot, 'Children');
    for i=1:length(figHandles)
        h=figure(figHandles(i).Number);
        set(h,'Units','Inches');
        pos = get(h,'Position');
        set(h,'PaperPositionMode','Auto','PaperUnits','Inches',...
                    'PaperSize',[pos(3), pos(4)]);
        print(h,[pwd join(['/Img/',h.Name])],'-dpdf','-r0');
        close(h);
    end
end