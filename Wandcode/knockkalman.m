%% zxh 2017
close all;
clc;
clear all;
connector off;
dbstop if error;
warning off;

connector on % Enable Connector

m                       = mobiledev;% Connect to device
cd ..
path(path,genpath(pwd));
N                       = 100;%The initial number of iterations
% minX                    = 0;
% maxX                    = N;
% minY                    = -0.1;
% maxY                    =  0.1;
% data.vis.bound          = [minX, maxX, minY, maxY];
% imu                     = demoUseGsensor.iniFig(data.vis.bound);
% data.vis.acc            = [0,N,0,10];
% accfig                  = demoUseGsensor.Fig(data.vis.acc);


hold on;
%% start the acc data 
m.AccelerationSensorEnabled=1;
m.AngularVelocitySensorEnabled=1;
m.logging=1;

%% init

test.acc    = 0;
test.omega  = 0;
A           = 1;
C           = 1;
R           = 0.0023;
Q           = 4e-4;

thre        = 0.01;
win         = 10; 

pause(2)
[acc,tacc]      = accellog(m);%Data Prefetch
disp('Acquiring Data')

for k = 2:N
    % Acquire 1 second of data
    pause(1)
    [acc,tacc]      = accellog(m);
    [ang,tang]      = angvellog(m);

    if isempty(acc)
        display('DONT HAVE INPUT')
        continue;
    else
        data.xacc   = acc(:,1);
        data.yacc   = acc(:,2);
        data.zacc   = acc(:,3);
        z           = data.zacc';
        t           = k;
        %% kalman
        zhat        = kalman(z, A, C, R, Q, t);
        
        zdata       = 1:size(z');
        g           = figure(2);
        plot(zdata,z','r',zdata,zhat,'k');
        hold on;
        legend('z','zhat');

        diffz       = diff(zhat,1);
        size(diffz)
        meandiff    = abs(mean(diffz(end-win:end)));
       
        if (meandiff>thre)
            test.acc = 1;
        end
        if (test.acc)
            disp('Knock');
            display(meandiff);
        else
            disp('Idle');
            display(meandiff);
        end
        meansave(k-1)    = meandiff;
        figure(1);
        plot(k,meandiff,'*k');
        hold on;
        test.acc = 0;
    end
    
end
discardlogs(m);
% saveas(h,'acc.jpg');
% saveas(g,'angle.jpg')
clear m

connector off