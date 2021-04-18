disp('Select one of the following options :');
disp('    (1) get mean and covariance for each exp. {A, B, C}');
disp('    (2) run a simulation to find estimates ');
option = input(' ');

if option==1
    filename={'EXP_A.mat', 'EXP_B.mat','EXP_C.mat'};
    aux={'A','B','C'};
    g=9.8;
    for  i=1:length(filename)
        [~,~,phys_accs,phys_gyros,altitude_vision,altitude_vz,navdata]=...
            trata_dados(filename{i});
        stat.(aux{i}).phys_accs.mean=mean(phys_accs*g/1000); % m/s^2
        stat.(aux{i}).phys_accs.cov=cov(phys_accs*g/1000);

        stat.(aux{i}).phys_gyros.mean=mean(phys_gyros*pi/180); % rad
        stat.(aux{i}).phys_gyros.cov=cov(phys_gyros*pi/180);

        stat.(aux{i}).altitude_vision.mean=mean(altitude_vision*10^42);
        stat.(aux{i}).altitude_vision.cov=cov(altitude_vision*10^42);

        stat.(aux{i}).altitude_vz.mean=mean(altitude_vz); % I need to understand the units
        stat.(aux{i}).altitude_vz.cov=cov(altitude_vz);
    end
    disp('end of calculation successful');
elseif option==2 % calculate kalman gain and test it in the D experiment
    disp('Select what it is to estimate :');
    disp('    (1) pitch w/o bias');
    disp('    (2) roll w/o bias');
    disp('    (3) altitude w/o bias');
    option = input(' ');
    [raw, euler, phys_accs,phys_gyros,altitude_vision,...
                altitude_vz,navdata]=trata_dados('EXP_D.mat');
    switch option
        case 1 % pitch estimation
            sys=ss(0, [1,1], 1, 0);
            Q=1.788*10^-3;  R=2*Q;
            [K_est,L]=kalman(sys, Q,R);
            name='theta_stuff';
            out=estimate_stuff(navdata.time, raw.theta,...
                phys_gyros(:,2).*pi./180,L,K_est, name, euler.theta*pi/180);
            legend('estimated theta','real theta')
        case 2 % roll estimation
            sys=ss(0, [1,1], 1, 0);
            Q=0.0021;  R=5*Q;
            [K_est,L]=kalman(sys, Q,R);
            name='phi_stuff';
            out=estimate_stuff(navdata.time, raw.phi,...
                phys_gyros(:,1).*pi./180,L,K_est, name, euler.phi*pi/180);
            legend('estimated phi','real phi')
            
        case 3 % altitude estimation
            A=[0 1; 0 0];   B=[0; 1]; C=[1 0]; D=0;
            G=eye(2);
            H=[0 0];
            sys=ss(A, [B G], C, [D H]);
            Q=0.005*[0 0; 0 1];
            R=0.005/150;
            
        otherwise
            disp('Incorrect input option');
    end
else
   disp('wrong input option'); 
end


function out=estimate_stuff(time, y, u ,L, K_est, name, real)
    aux1=[time,u];
    aux2=[time,y];
    in = Simulink.SimulationInput('estimate_1');
    in = in.setVariable('aux1', aux1);
    in = in.setVariable('aux2', aux2);
    in = in.setBlockParameter('estimate_1/L','Gain',int2str(L));
    in = in.setBlockParameter('estimate_1/A_kalman','Gain',int2str(K_est.A));
    in = in.setBlockParameter('estimate_1/To Workspace','VariableName',name);
    out=sim(in);
    figure()
    plot(out.tout,out.(name));
    hold on 
    plot(out.tout,real);
end

