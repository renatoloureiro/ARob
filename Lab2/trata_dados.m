function [altitude_real,raw,euler, phys_accs,phys_gyros,altitude_vision,...
    altitude_vz,navdata] = trata_dados(filename)

load(filename);
time = navdata.time;
N = length(time);
phys_accs = zeros(N,3);
phys_gyros = zeros(N,3);
altitude_vision = zeros(N,1);
altitude_vz = zeros(N,1);

for i = 1:N
    [ardrone_state, vbat_flying_percentage,...
    theta(i), phi(i), psi(i), altitude(i), vx, vy, vz , cksumError, body_v,...
    phys_accs(i,:),phys_gyros(i,:),altitude_vision(i),altitude_vz(i),altitude_raw(i),heading_unwrapped,heading_gyro_unwrapped,...
    heading_fusion_unwrapped,magneto_radius,est_z(i),est_zdot(i),theta_a,phi_a,mx, my, mz]= trata_dados_large(navdata.signals.values(i,:));
end
    euler.theta=theta;
    euler.phi=phi;
    euler.psi=psi;
    euler.altitude=altitude;
    
    raw.theta=-atan2(phys_accs(:,1) ,sqrt(phys_accs(:,2).^2 + phys_accs(:,3).^2));
    raw.phi=atan2(phys_accs(:,2),phys_accs(:,3));
    altitude_real.est_z=est_z;
    altitude_real.est_zdot=est_zdot;
end

%%    
function [ardrone_state, vbat_flying_percentage,...
    theta, phi, psi, altitude, vx, vy, vz , cksumError, body_v,...
    phys_accs,phys_gyros,altitude_vision,altitude_vz,altitude_raw,heading_unwrapped,heading_gyro_unwrapped,...
    heading_fusion_unwrapped,magneto_radius,est_z,est_zdot,theta_a,phi_a,mx, my, mz]= trata_dados_large(data)

%initialize arrays
phys_accs=zeros(3,1);
phys_gyros=zeros(3,1);
body_v=zeros(3,1);

% Checksum array. cksum(1) contains the computed checksum, cksum(2)
% contains the checksum sent by the ARDrone. 
cksum = [0 0 ]; 

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
vbat_flying_percentage = decode_32bit(data(21:24),1); % unit: percentage 0 -100
theta = decode_32bit(data(25:28),0)/1000;       % unit: deg
phi = decode_32bit(data(29:32),0)/1000;         % unit: deg
psi = decode_32bit(data(33:36),0)/1000;         % unit: deg
altitude = decode_32bit(data(37:40),1)/1000;    % unit: m
vx = decode_32bit(data(41:44),0)/1000;          % unit: m/s
vy = decode_32bit(data(45:48),0)/1000;          % unit: m/s
vz = decode_32bit(data(49:52),0)/1000;          % unit: m/s
detection_camera_type = decode_32bit(data(109:112),1);

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
theta_a = decode_32bit(data(287:290),0);
phi_a = decode_32bit(data(291:294),0);

altitude_vision = decode_32bit(data(519:522),0);    %%%% typecast(data(519:522), 'int32');
altitude_vz = decode_32bit(data(523:526),0);
altitude_raw = decode_32bit(data(531:534),0);       %%%% typecast(data(531:534), 'int32');
 
heading_unwrapped = decode_32bit(data(1712:1715),0);
heading_gyro_unwrapped = decode_32bit(data(1716:1719),0);
heading_fusion_unwrapped = decode_32bit(data(1720:1723),0);

magneto_radius = decode_32bit(data(1737:1740),0);
% magneto
mx = decode_16bit(data(1670:1671),1);
my = decode_16bit(data(1672:1673),1);
mz = decode_16bit(data(1674:1675),1);

est_z = decode_32bit(data(1821:1824),0);
est_zdot = decode_32bit(data(1825:1828),0);

%cks
cks = decode_32bit(data(end-3:end),1); 
    
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



