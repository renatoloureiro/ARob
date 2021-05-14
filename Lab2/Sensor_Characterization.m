% Experiments
fileNames = ["EXP_A.mat", "EXP_B.mat", "EXP_C.mat"];
exp = {'A', 'B', 'C'};

for i = 1:length(fileNames)
    % Load data from respective experience
    load(fileNames(i));

    % Initializing Variables
    g = 9.80665;
    numberSamples = length(timeRT);
    
    accsData = zeros(3, numberSamples);
    gyroData = zeros(3, numberSamples);
    altitude.vision = zeros(1, numberSamples);
    altitude.vz = zeros(1, numberSamples);
    altitude.raw = zeros(1, numberSamples);

    % Simulate the experiment, storing sensor information
    for j = 1:numberSamples
        [~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~,   ...
         accsData(:, j),                    ...
         gyroData(:, j),                         ...
         altitude.vision(j),                   ...
         altitude.vz(j),                       ...
         altitude.raw(j),                      ...
         ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~    ...
         ] ...
            = trata_dados_large(navdata.signals.values(j,:));
    end
    
    figure(i);
    
    subplot(3, 3, 1);
    stat.(exp{i}).accs.mean(1) = mean(accsData(1, :)) * g / 1000;
    stat.(exp{i}).accs.var(1) = var(accsData(1, :)) * (g / 1000)^2;
    plot(timeRT, accsData(1, :) * g / 1000);
    title('Accelerometer - x direction');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    
    subplot(3, 3, 2);
    stat.(exp{i}).accs.mean(2) = mean(accsData(2, :)) * g / 1000;
    stat.(exp{i}).accs.var(2) = var(accsData(2, :)) * (g / 1000)^2;
    plot(timeRT, accsData(2, :) * g / 1000);
    title('Accelerometer - y direction');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    
    subplot(3, 3, 3);
    stat.(exp{i}).accs.mean(3) = mean(accsData(3, :)) * g / 1000 + g;
    stat.(exp{i}).accs.var(3) = var(accsData(3, :)) * (g / 1000)^2;
    plot(timeRT, accsData(3, :) * g / 1000);
    title('Accelerometer - z direction');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    
    subplot(3, 3, 4);
    stat.(exp{i}).gyro.mean(1) = mean(gyroData(1, :)) * pi / 180;
    stat.(exp{i}).gyro.var(1) = var(gyroData(1, :)) * (pi / 180)^2;
    plot(timeRT, gyroData(1, :));
    title('Gyro Rate - p');
    xlabel('Time (s)');
    ylabel('Angular Speed (º/s)');
    
    subplot(3, 3, 5);
    stat.(exp{i}).gyro.mean(2) = mean(gyroData(2, :)) * pi / 180;
    stat.(exp{i}).gyro.var(2) = var(gyroData(2, :)) * (pi / 180)^2;
    plot(timeRT, gyroData(2, :));
    title('Gyro Rate - q');
    xlabel('Time (s)');
    ylabel('Angular Speed (º/s)');
    
    subplot(3, 3, 6);
    stat.(exp{i}).gyro.mean(3) = mean(gyroData(3, :)) * pi / 180;
    stat.(exp{i}).gyro.var(3) = var(gyroData(3, :)) * (pi / 180)^2;
    plot(timeRT, gyroData(3, :));
    title('Gyro Rate - r');
    xlabel('Time (s)');
    ylabel('Angular Speed (º/s)');
    
    subplot(3, 3, 7);
    plot(timeRT, altitude.vision);
    title('Altitude Vision');
    xlabel('Time (s)');
    
    subplot(3, 3, 8);
    plot(timeRT, altitude.vz);
    title('Altitude vz');
    xlabel('Time (s)');
    
    subplot(3, 3, 9);
    plot(timeRT, altitude.raw(j));
    title('Altitude Raw');
    xlabel('Time (s)');
end

clear accsData altitude exp fileNames gyroData i j navdata;
clear numberSamples sampleTime states timeRT;

%% Functions
function [ardrone_state,            ...
          vbat_flying_percentage,   ...
          theta, phi, psi, ...
          altitude, ...
          vx, vy, vz, ...
          cksumError, ...
          body_v,...
          phys_accs, ...
          phys_gyros, ...
          altitude_vision, altitude_vz, altitude_raw, ...
          heading_unwrapped, heading_gyro_unwrapped, heading_fusion_unwrapped, ...
          magneto_radius, ...
          est_z, est_zdot, ...
          theta_a, phi_a, ...
          mx, my, mz] ...
          = trata_dados_large(data)

%initialize arrays
vbat_flying_percentage = 0;
phi = 0;
theta = 0;
psi = 0;
altitude = 0;
vx = 0;
vy = 0;
vz = 0;
phys_accs=zeros(3,1);
phys_gyros=zeros(3,1);
body_v=zeros(3,1);
altitude_vision = 0; 
altitude_vz = 0;
altitude_raw = 0;
heading_unwrapped=0;
heading_gyro_unwrapped = 0;
heading_fusion_unwrapped = 0;
magneto_radius = 0;
est_z = 0;
est_zdot = 0;
theta_a = 0;
phi_a = 0;
mx = 0;
my = 0; 
mz = 0;

% Checksum array. cksum(1) contains the computed checksum, cksum(2)
% contains the checksum sent by the ARDrone. 
cksum = [0 0]; 

% Computing checksum
cksum(1) =  sum(double(data(1:end-8)))+442; 
% Checksum given by the drone
cksum(2) = double(data(end-3)) + double(data(end-2))*256 +  double(data(end-1))*256^2 + double(data(end))*256^3 ; 
% Checksum error
cksumError = abs(cksum(1)-cksum(2));

% Converting bytes to drone information

% header data
ardrone_state = getDroneState(data(1:4));

% demo data
vbat_flying_percentage(1) = decode_32bit(data(21:24),1); % unit: percentage 0 -100
theta(1) = decode_32bit(data(25:28),0)/1000; % unit: deg
phi(1) = decode_32bit(data(29:32),0)/1000; % unit: deg
psi(1) = decode_32bit(data(33:36),0)/1000; % unit: deg
altitude(1) = decode_32bit(data(37:40),1)/1000; % unit: m
vx(1) = decode_32bit(data(41:44),0)/1000; % unit: m/s
vy(1) = decode_32bit(data(45:48),0)/1000; % unit: m/s
vz(1) = decode_32bit(data(49:52),0)/1000; % unit: m/s
%detection_camera_type(1) = decode_32bit(data(109:112),1);

phys_accs(1) = decode_32bit(data(231:234),0);
phys_accs(2) = decode_32bit(data(235:238),0);
phys_accs(3) = decode_32bit(data(239:242),0);
phys_gyros(1) = decode_32bit(data(243:246),0);
phys_gyros(2) = decode_32bit(data(247:250),0);
phys_gyros(3) = decode_32bit(data(251:254),0);
if (phys_gyros(3)>100)
    phys_gyros(3)=100;
elseif (phys_gyros(3)<-100)
    phys_gyros(3)=-100;
end

% euler angles
theta_a(1) = decode_32bit(data(287:290),0);
phi_a(1) = decode_32bit(data(291:294),0);

altitude_vision(1) = decode_32bit(data(519:522),0);    %%%% typecast(data(519:522), 'int32');
altitude_vz(1) = decode_32bit(data(523:526),0);
altitude_raw(1) = decode_32bit(data(531:534),0);       %%%% typecast(data(531:534), 'int32');
 
heading_unwrapped(1) = decode_32bit(data(1712:1715),0);
heading_gyro_unwrapped(1) = decode_32bit(data(1716:1719),0);
heading_fusion_unwrapped(1) = decode_32bit(data(1720:1723),0);

magneto_radius(1) = decode_32bit(data(1737:1740),0);
% magneto
mx(1) = decode_16bit(data(1670:1671),1);
my(1) = decode_16bit(data(1672:1673),1);
mz(1) = decode_16bit(data(1674:1675),1);

est_z(1) = decode_32bit(data(1821:1824),0);
est_zdot(1) = decode_32bit(data(1825:1828),0);

%cks
%cks = decode_32bit(data(end-3:end),1); 
    
end


function res = decode_32bit(input, mark)
% if mark = 1; then output a int
% else output the float value which is defined by SDK description.
% the input is an array (1*4)
% e.g. : NDS(Battery_OS:Battery_OS+data_length-1)

% hex_value = dec2hex(input);
% hex_value = [hex_value(4,:),hex_value(3,:),hex_value(2,:),hex_value(1,:)];
if mark ==1
    res = typecast(input,'uint32');%hex2dec(hex_value);
elseif mark ==2
    res = hex2dec(hex_value)-2^31;
else
    res = typecast(input,'single');%typecast(uint32(hex2dec(hex_value)),'single');
end
end

function res = decode_16bit(input,mark)
%hex_value = dec2hex(input);
%hex_value = [hex_value(2,:),hex_value(1,:)];
%res = hex2dec(hex_value);
res = typecast(input,'uint16')-2^15;
if mark==1
    res=res-2^15;
end
end

function bin = getDroneState(input)
bin = zeros(32,1);
hex = dec2hex(input);
% put the hex_value in the order [31, 30......0]
hex = [hex(4,:),hex(3,:),hex(2,:),hex(1,:)];
for i=1:length(hex)
    if hex(i)=='F'
        bin((i*4)-3:i*4)=[1 1 1 1];
    elseif hex(i)=='E'
        bin((i*4)-3:i*4)=[1 1 1 0];
    elseif hex(i)=='D'
        bin((i*4)-3:i*4)=[1 1 0 1];
    elseif hex(i)=='C'
        bin((i*4)-3:i*4)=[1 1 0 0];
    elseif hex(i)=='B'
        bin((i*4)-3:i*4)=[1 0 1 1];
    elseif hex(i)=='A'
        bin((i*4)-3:i*4)=[1 0 1 0];
    elseif hex(i)=='9'
        bin((i*4)-3:i*4)=[1 0 0 1];
    elseif hex(i)=='8'
        bin((i*4)-3:i*4)=[1 0 0 0];
    elseif hex(i)=='7'
        bin((i*4)-3:i*4)=[0 1 1 1];
    elseif hex(i)=='6'
        bin((i*4)-3:i*4)=[0 1 1 0];
    elseif hex(i)=='5'
        bin((i*4)-3:i*4)=[0 1 0 1];
    elseif hex(i)=='4'
        bin((i*4)-3:i*4)=[0 1 0 0];
    elseif hex(i)=='3'
        bin((i*4)-3:i*4)=[0 0 1 1];
    elseif hex(i)=='2'
        bin((i*4)-3:i*4)=[0 0 1 0];
    elseif hex(i)=='1'
        bin((i*4)-3:i*4)=[0 0 0 1];
    elseif hex(i)=='0'
        bin((i*4)-3:i*4)=[0 0 0 0];
    end
end

end