[altitude_real, raw, euler, phys_accs,phys_gyros,altitude_vision,...
                altitude_vz,navdata]=trata_dados('EXP_D.mat');
g=9.8;
for i=1:6000
   hpp(i)=sin(euler.theta(i)*pi/180)*phys_accs(i,1)*g/1000 - ...
    sin(euler.phi(i)*pi/180)*cos(euler.theta(i)*pi/180)*phys_accs(i,2)*g/1000 -...
    cos(euler.phi(i)*pi/180)*cos(euler.theta(i)*pi/180)*(phys_accs(i,3) + 1000)*g/1000 ;%+g
end

altitude_vision=altitude_vision.*10^42;
A=[0 1 0 ; 0 0 -1 ; 0 0 0 ];
B=[0;1;0];
C=[1 0 0 ;0 1 0 ];
D=[0;0];
G=eye(3)
H=[0 0 0 ; 0 0 0 ]
sys=ss(A, [B G], C, [D H])
Q=diag([0 0.005 1*10^-4 ])
R=diag([0.0005/150 0.05/150])
[K_est, L]=kalman(sys,Q,R)
aux=K_est.A;