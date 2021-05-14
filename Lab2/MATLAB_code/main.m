disp('Select one of the following options :');
disp('    (1) get mean and covariance for each exp. {A, B, C}');
disp('    (2) run a simulation to find estimates ');
disp('    (3) run a simulation with article implementation ');
option = input(' ');

if option==1
    filename={'EXP_A.mat', 'EXP_B.mat','EXP_C.mat'};
    aux={'A','B','C'};
    g=9.8;
    for  i=1:length(filename)
        [~,~,~,phys_accs,phys_gyros,altitude_vision,altitude_vz,navdata]=...
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
    [altitude_real, raw, euler, phys_accs,phys_gyros,altitude_vision,...
                altitude_vz,navdata]=trata_dados('EXP_D.mat');
    switch option
        case 1 % pitch estimation
            sys=ss(0, [1,1], 1, 0);
            %Q=1.788*10^-3;  
            Q=0.0081;
            R=5*Q;
            [K_est,L]=kalman(sys, Q,R);
            name='theta_stuff';
            out=estimate_stuff(navdata.time, raw.theta,...
                phys_gyros(:,2).*pi./180,L,K_est.A, name, euler.theta*pi/180);
            legend('\theta estimado','\theta real')
        case 2 % roll estimation
            sys=ss(0, [1,1], 1, 0);
            %Q=0.0021;  
            Q=0.0098;
            R=0.6*Q;
            [K_est,L]=kalman(sys, Q,R);
            name='phi_stuff';
            out=estimate_stuff(navdata.time, raw.phi,...
                phys_gyros(:,1).*pi./180,L,K_est.A, name, euler.phi*pi/180);
            legend('\phi estimado','\phi real')
            
        case 3 % altitude estimation
            % need to calculate hpp=sin(theta)*ax -sin(phi)cos(theta)*ay -
            % cos(phi)cos(theta)*az + g
            
            g=9.8;
            for i=1:6000
                hpp(i)=sin(euler.theta(i)*pi/180)*phys_accs(i,1)*g/1000 - ...
                    sin(euler.phi(i)*pi/180)*cos(euler.theta(i)*pi/180)*phys_accs(i,2)*g/1000 -...
                    cos(euler.phi(i)*pi/180)*cos(euler.theta(i)*pi/180)*(phys_accs(i,3) + 1000)*g/1000 ;%+g
            end
           
            A=[0 1; 0 0];   B=[0; 1]; C=[1 0]; D=0;
            G=eye(2);
            H=[0 0];
            sys=ss(A, [B G], C, [D H]);
            Q=0.005*[0 0; 0 1];
            R=0.001/150;
            [K_est,L]=kalman(sys, Q,R);
            aux=K_est.A;
            %L=[5.0886;12.9471;-1.7321]
            %aux=[-5.0886    1.0000         0;
            %        -12.9471         0   -1.0000;
            %        1.7321         0         0];
            
            name='altitude_stuff';
            hpp(1:12)=0;
            raw.altitude(1:12)=0.334*10^-42;
            out=estimate_stuff(navdata.time, double(transpose((raw.altitude.*(10^42) - 0.334)*1.40)),...
                double(transpose(hpp)),L,aux, name, altitude_real.est_z);
        case 4 % pitch estimation with bias
        case 5 % roll estimation with bias
        case 6 % altitude estimation with bias    
        otherwise
            disp('Incorrect input option');
    end
elseif option==3
    [altitude_real, raw, euler, phys_accs,phys_gyros,altitude_vision,...
                altitude_vz,navdata]=trata_dados('EXP_D.mat');
     %phys_accs(:,3)=phys_accs(:,3) + 1000;
     phys_accs(:,2)=phys_accs(:,2) + 0.3*1000/9.8;
     phys_accs(:,1)=phys_accs(:,1) - 0.1*1000/9.8;
     phys_accs(1:12,3)=-1000;
     Q_w=[1 0 0; 0 1 0; 0 0 1]*(pi/180)^2;
     Q_b=1/8*Q_w;%(1/8)*Q_w;
     Q=[Q_w, zeros(3,3);zeros(3,3), Q_b];
     R=eye(3)*0.005;
     C=[eye(3), zeros(3,3)];
     in = Simulink.SimulationInput('article_implementation');
     in = in.setVariable('Q', Q);
     in = in.setVariable('R', R);
     in = in.setVariable('C', C);
     out=sim(in)
else
   disp('wrong input option'); 
end


function out=estimate_stuff(time, y, u ,L, K_est, name, real)
    aux1=[time,u];
    aux2=[time,y];
    in = Simulink.SimulationInput('estimate_1');
    in = in.setVariable('aux1', aux1);
    in = in.setVariable('aux2', aux2);
    in = in.setVariable('A_kalman', K_est);
    in = in.setVariable('L', L);
    %in = in.setBlockParameter('estimate_1/L','Gain',int2str(L));
    %in = in.setBlockParameter('estimate_1/A_kalman','Gain',int2str(K_est.A));
    in = in.setBlockParameter('estimate_1/To Workspace','VariableName',name);
    out=sim(in);
    figure()
    plot(out.tout,out.(name));
    hold on 
    plot(out.tout,real);
end

