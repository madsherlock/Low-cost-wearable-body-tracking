%% callmain
[...
    plotrawboth,...
    plotmagnetometer,...
    plotYPRboth1,...
    animation1,...
    canimation1,...
    calanimation1,...
    covu1,covf1...
    ]=...
    main(true,'Madgwick');
%% callmain
[...
    plotrawboth,...
    plotmagnetometer,...
    plotYPRboth2,...
    animation2,...
    canimation2,...
    calanimation2,...
    covu2,covf2...
    ]=...
    main(false,'Madgwick');

%%
clear; filter_type='Madgwick'; use_mag=false;