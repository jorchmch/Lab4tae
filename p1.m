clc,clear

% Angulos de Euler - Cuerpo A
yaw = 45;
pitch = -5;
roll = -1;

% yaw = 30;
% pitch = -45;
% roll = 60;


yaw = yaw * pi/180;
pitch = pitch * pi/180;
roll = roll * pi/180;


% Convencion 3-2-1
c1 = cos(yaw);  % theta1
s1 = sin(yaw);  % theta1
c2 = cos(pitch);  % theta2
s2 = sin(pitch);  % theta2
c3 = cos(roll);  % theta3
s3 = sin(roll);  % theta3

% Matriz de rotacion

A = [c2*c1, c2*s1, -s2;
    s3*s2*c1 - c3*s1, s3*s2*s1 + c3*c1, s3*c2;
    c3*s2*c1 + s3*s1, c3*s2*s1 - s3*c1, c3*c2]
disp('Yaw Pitch Roll Calculado')
yawCalc = atan2(A(1,2),A(1,1)) * 180 / pi
pitchCalc = tan(A(1,3)) * 180 / pi
rollCalc = atan2(A(2,3),A(3,3)) * 180 / pi 


% Angulos de Euler - Cuerpo B
yaw = -88;
pitch = 1;
roll = -3;
% yaw = 10;
% pitch = 25;
% roll = -15;

yaw = yaw * pi/180;
pitch = pitch * pi/180;
roll = roll * pi/180;


% Convencion 3-2-1
c1 = cos(yaw);  % theta1
s1 = sin(yaw);  % theta1
c2 = cos(pitch);  % theta2
s2 = sin(pitch);  % theta2
c3 = cos(roll);  % theta3
s3 = sin(roll);  % theta3

% Matriz de rotacion

B = [c2*c1, c2*s1, -s2;
    s3*s2*c1 - c3*s1, s3*s2*s1 + c3*c1, s3*c2;
    c3*s2*c1 + s3*s1, c3*s2*s1 - s3*c1, c3*c2]

disp('Yaw Pitch Roll Calculado')

yawCalc = atan2(B(1,2),B(1,1)) * 180 / pi
pitchCalc = tan(B(1,3)) * 180 / pi
rollCalc = atan2(B(2,3),B(3,3)) * 180 / pi 

% DCM
BA = B*A'
disp('Yaw Pitch Roll Calculado')
yawCalc = atan2(BA(1,2),BA(1,1)) * 180 / pi
pitchCalc = tan(BA(1,3)) * 180 / pi
rollCalc = atan2(BA(2,3),BA(3,3)) * 180 / pi 


AB = A*B'
disp('Yaw Pitch Roll Calculado')
yawCalc = atan2(AB(1,2),AB(1,1)) * 180 / pi
pitchCalc = tan(AB(1,3)) * 180 / pi
rollCalc = atan2(AB(2,3),AB(3,3)) * 180 / pi 