function [...
    gbiasx,gbiasy,gbiasz,...
    gsensx,gsensy,gsensz,...
    gtimevarx,gtimevary,gtimevarz,...
    mbiasx,mbiasy,mbiasz,...
    msensx,msensy,msensz,...
    mtimevarx,mtimevary,mtimevarz,...
    atimevarx,atimevary,atimevarz...
    ] = sensor_static_calibration(filename,workingtemperature,dt,startrow,endrow)
%% Raw sensor data import
[temp,ax,ay,az,mx,my,mz,gx,gy,gz] =...
    temperature_text_import(...
    filename,startrow,endrow);
    %'Temperature_bi_1431942285.53.txt',400,41500);
    %'Temperature_kj_1431899686.36.txt',700,45000);
t=dt*(0:1:length(temp)-1)';
%workingtemperature = 27;
mit= min(temp);
mat= max(temp);

%% Gyro calculations
[gpx,gSx] = polyfit(temp,gx,1);
[gpy,gSy] = polyfit(temp,gy,1);
[gpz,gSz] = polyfit(temp,gz,1);

%delta is an estimate of the standard deviation of the error in predicting
% a future observation at x by p(x).
%If the coefficients in p are least squares estimates computed by polyfit,
% and the errors in the data input to polyfit are independent, normal,
% and have constant variance, then y±delta contains at least 50% of the
% predictions of future observations at x.
[gbiasx,gdeltax] = polyval(gpx,workingtemperature,gSx);
[gbiasy,gdeltay] = polyval(gpy,workingtemperature,gSy);
[gbiasz,gdeltaz] = polyval(gpz,workingtemperature,gSz);
gsensx = gpx(1);
gsensy = gpy(1);
gsensz = gpz(1);
gtimevarx=var(gx-(gpx(1)*temp+gpx(2)));
gtimevary=var(gy-(gpy(1)*temp+gpy(2)));
gtimevarz=var(gz-(gpz(1)*temp+gpz(2)));

%% Magnetometer calculations
[mpx,mSx] = polyfit(temp,mx,1);
[mpy,mSy] = polyfit(temp,my,1);
[mpz,mSz] = polyfit(temp,mz,1);
[mbiasx,mdeltax] = polyval(mpx,workingtemperature,mSx);
[mbiasy,mdeltay] = polyval(mpy,workingtemperature,mSy);
[mbiasz,mdeltaz] = polyval(mpz,workingtemperature,mSz);
msensx = mpx(1);
msensy = mpy(1);
msensz = mpz(1);
mtimevarx=var(mx-(mpx(1)*temp+mpx(2)));
mtimevary=var(my-(mpy(1)*temp+mpy(2)));
mtimevarz=var(mz-(mpz(1)*temp+mpz(2)));


%% Accelerometer calculations
atimevarx=var(ax);
atimevary=var(ay);
atimevarz=var(az);

%%
fig=figure();
axes1 = axes('Parent',fig,'YGrid','on','XGrid','on');
box(axes1,'on');
hold(axes1,'all');
gg = 1/14.375;
hold all;
plot1 = plot(temp,gg*gy,'rx',...
    temp,gg*gz,'gx',...
    temp,gg*gx,'b.');
plot2=plot([mit mat],gg*(gpy(1)*[mit mat]+gpy(2)),'b-',...
    [mit mat],gg*(gpz(1)*[mit mat]+gpz(2)),'black-',...
    [mit mat],gg*(gpx(1)*[mit mat]+gpx(2)),'r-');
set(plot1(1),'Color',[1 0 0],'DisplayName','raw y');
set(plot1(2),'Color',[0 1 0],'DisplayName','raw z');
set(plot1(3),'Marker','.','Color',[0 0 1],'DisplayName','raw x');
set(plot2(1),'Color',[0 0 1],'DisplayName','y fit');
set(plot2(2),'DisplayName','z fit','Color',[0 0 0]);
set(plot2(3),'Color',[1 0 0],'DisplayName','x fit');
xlabel('Temperature [°C]');
ylabel('Angular velocity [°/s]');
title('Gyro measurements vs. temperature');
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.759515186543874 0.556233680919357 0.108265027322404 0.232570806100218]);

set(fig,'Units','normalized');
set(fig,'Position',[0.1 0.1 0.6 0.6]);
hold off;

%%
fig2=figure();
axes2 = axes('Parent',fig2,'YGrid','on','XGrid','on');
box(axes2,'on');
hold(axes2,'all');
mg = 1/1090;
hold all;
plot3 = plot(temp,mg*my,'rx',...
    temp,mg*mz,'gx',...
    temp,mg*mx,'b.');
plot4=plot([mit mat],mg*(mpy(1)*[mit mat]+mpy(2)),'b-',...
    [mit mat],mg*(mpz(1)*[mit mat]+mpz(2)),'black-',...
    [mit mat],mg*(mpx(1)*[mit mat]+mpx(2)),'r-');
set(plot3(1),'Color',[1 0 0],'DisplayName','raw y');
set(plot3(2),'Color',[0 1 0],'DisplayName','raw z');
set(plot3(3),'Marker','.','Color',[0 0 1],'DisplayName','raw x');
set(plot4(1),'Color',[0 0 1],'DisplayName','y fit');
set(plot4(2),'DisplayName','z fit','Color',[0 0 0]);
set(plot4(3),'Color',[1 0 0],'DisplayName','x fit');
xlabel('Temperature [°C]');
ylabel('Magnetic flux [Gauss]');
title('Magnetometer measurements vs. temperature');
legend2 = legend(axes2,'show');
set(legend2,...
    'Position',[0.759515186543874 0.556233680919357 0.108265027322404 0.232570806100218]);
set(fig2,'Units','normalized');
set(fig2,'Position',[0.1 0.1 0.6 0.6]);
hold off;

%%
%save(strcat(filename(1:end-4),'.mat'),'wbiasx','wbiasy','wbiasz','deltax','deltay','deltaz','sensx','sensy','sensz','workingtemperature');