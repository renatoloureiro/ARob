% Experiment D data where the Kalman Filter will be tested
load("EXP_D.mat");

% Initializing Variables
g = 9.80665;
numberSamples = length(timeRT);

real.theta = zeros(1, numberSamples);
real.phi = zeros(1, numberSamples);
real.height = zeros(1, numberSamples);
accsData = zeros(3, numberSamples);
gyroData = zeros(3, numberSamples);

% Simulate the navigation data
for j = 1:numberSamples
    [~, ~,                              ...
     real.theta(j), real.phi(j), ~,     ...
     real.height(j),                    ...
     ~, ~, ~, ~, ~,                     ...
     accsData(:, j),                    ...
     gyroData(:, j),                    ...
     ~, ~, ~,                ...
     ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~]   ...
    = trata_dados_large(navdata.signals.values(j,:));
end

% Calculate raw Euler angles
raw.phi = atan2(-accsData(2, :), -accsData(3,:));
raw.theta = atan2(accsData(1, :), sqrt(accsData(2,:).^2 + accsData(3,:).^2));

clear j navdata numberSamples sampleTime states waypoints;

%% Pitch without Bias
B = 1;
sys = ss(0, B, 1, 0);                       
Q = stat.C.gyro.var(2);
R = 5 * Q;
[K_est, L] = kalman(sys, Q, R);
A_Kalman = K_est.A;
Test_Platform_Angles;

u = [timeRT gyroData(2,:)'.*pi./180];
y = [timeRT raw.theta'];
out = sim('Comparison');
estimate.theta = out.estimate;
time.theta = out.tout;
plot(out.tout, estimate.theta);
hold on;
plot(timeRT, real.theta*pi/180);
hold off;
legend('\theta_{estimate}', '\theta_{real}');
xlabel('Time [s]');
ylabel('Angle [rad]');

%% Pitch with Bias
A = [0 -1; 0 0];
B = [1; 0];
C = [1 0];
D = 0;
G = eye(2);
H = [0 0];
sys = ss(A, [B G], C, [D H]);
Q = stat.C.gyro.var(2) * [1 0; 0 20];
R = 200 * stat.C.gyro.var(2);
[K_est, L] = kalman(sys, Q, R);
A_Kalman = K_est.A;
Test_Platform_Angles;

% u = [timeRT gyroData(2,:)'.*pi./180];
% y = [timeRT raw.theta'];
% out = sim('Comparison');
% estimate.theta_bias = out.estimate(:,1);
% plot(out.tout, estimate.theta_bias);
% hold on;
% plot(time.theta, estimate.theta);
% plot(timeRT, real.theta * pi / 180);
% hold off;
% legend('\theta_{estimate\_bias}', '\theta_{estimate}', '\theta_{real}');
% xlabel('Time [s]');
% ylabel('Angle [rad]');

%% Roll without Bias
B = 1;
sys = ss(0, B, 1, 0);                       
Q = stat.C.gyro.var(1);
R = 5 * Q;
[K_est, L] = kalman(sys, Q, R);
A_Kalman = K_est.A;
Test_Platform_Angles;

u = [timeRT gyroData(1,:)'.*pi./180];
y = [timeRT raw.phi'];
out = sim('Comparison');
estimate.phi = out.estimate;
time.phi = out.tout;
plot(out.tout, estimate.phi);
hold on;
plot(timeRT, real.phi*pi/180);
hold off;
legend('\phi_{estimate}', '\phi_{real}');
xlabel('Time [s]');
ylabel('Angle [rad]');

%% Roll with Bias
A = [0 -1; 0 0];
B = [1; 0];
C = [1 0];
D = 0;
G = eye(2);
H = [0 0];
sys = ss(A, [B G], C, [D H]);
Q = stat.C.gyro.var(1) * [1 0; 0 1/10];
R = 1/5 * stat.C.gyro.var(1);
[K_est, L] = kalman(sys, Q, R);
A_Kalman = K_est.A;
Test_Platform_Angles;

% u = [timeRT gyroData(1,:)'.*pi./180];
% y = [timeRT raw.phi'];
% out = sim('Comparison');
% estimate.phi_bias = out.estimate(:,1);
% plot(out.tout, estimate.phi_bias);
% hold on;
% plot(time.phi, estimate.phi);
% plot(timeRT, real.phi * pi / 180);
% hold off;
% legend('\phi_{estimate\_bias}', '\phi_{estimate}', '\phi_{real}');
% xlabel('Time [s]');
% ylabel('Angle [rad]');

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
theta_a = 0;
phi_a = 0;
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