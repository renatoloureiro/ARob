files_sim={'sim_step_0_5_1_0.mat';
    'sim_step_1_0_0_5.mat';
    'sim_step_2_4_0_25.mat';
    'sim_step_3_2_0_2.mat'};

for q=4
    load(files_sim{q});
    tfdata=iddata(h_sim.Data(1:13:round(26/0.005)),...
        h_ref.Data(1:round(26/0.065)),0.065);
    n_poles=3;
    n_zeros=2;
    sys3=tfest(tfdata,n_poles,n_zeros);

    [wn,z]=damp(sys3);
    covariance_m=getcov(sys3);
    w_max=sqrt(sys3.Denominator(3) + sqrt(covariance_m(3,3)));
    w_min=sqrt(sys3.Denominator(3) - sqrt(covariance_m(3,3)));
    e_max=(sys3.Denominator(2) + sqrt(covariance_m(2,2)))/(2*w_min);
    e_min=(sys3.Denominator(2) - sqrt(covariance_m(2,2)))/(2*w_max);
    
    only_poles=1;
    if only_poles==1
        scatter(real(pole(sys3)),imag(pole(sys3)),'filled')
        hold on
        if q==4
            legend('k_p=0.5','k_p=1.0','k_p=2.4','k_p=3.2','Location',...
                'northwest')
        end
    else
        for j=1:2
            t=0:(w_max-w_min)/100:(w_max-w_min);
            if j==1
                theta=asin(e_max);
            else
                theta=asin(e_min);
            end
            for i=1:length(t)
                x(i)=-(w_min + t(i))*sin(theta);
                y(i)=(w_min + t(i))*cos(theta);
            end
             plot(x,y)
             hold on
        end

        clear t
        for j=1:2
            t=0:(e_max-e_min)/100:(e_max-e_min);
            if j==1
            for i=1:length(t)
                    x(i)=-(w_min)*sin(asin(t(i) + e_min));
                    y(i)=(w_min)*cos(asin(t(i) + e_min));
            end
            else
               for i=1:length(t)
                    x(i)=-(w_max)*sin(asin(t(i) + e_min));
                    y(i)=(w_max)*cos(asin(t(i) + e_min));
            end 
            end
            plot(x,y)
        end

        if q==1
            plot(pole(sys3), [0,0],'+')
        else
           plot(pole(sys3),'+'); 
        end
    end
end