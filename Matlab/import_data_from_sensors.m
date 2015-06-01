function [tu,tempu,au,mu,gu,tf,tempf,af,mf,gf] = import_data_from_sensors()
addpath('data');
%filenamef='Serial_1432141657.57_bi_forearm.txt';
%filenameu='Serial_1432141657.57_kj_upperarm.txt';
%filenamef='Serial_1432292573.22_bi_forearm.txt';
%filenameu='Serial_1432292573.22_kj_upperarm.txt';
filenamef='Serial_1432310697.2_bi_forearm.txt';
filenameu='Serial_1432310697.2_kj_upperarm.txt';
[tempf,af(:,1),af(:,2),af(:,3),mf(:,1),mf(:,2),mf(:,3),...
    gf(:,1),gf(:,2),gf(:,3)] = import_sensor(filenamef);
[tempu,au(:,1),au(:,2),au(:,3),mu(:,1),mu(:,2),mu(:,3),...
    gu(:,1),gu(:,2),gu(:,3)] = import_sensor(filenameu);
dtRef=0.01;
tu=dtRef*(0:1:(length(tempu)-1));
tf=length(tempu)/length(tempf)*dtRef*(0:1:(length(tempf)-1));