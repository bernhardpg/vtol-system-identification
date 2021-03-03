load(fullfile(matlabroot,'toolbox','ident','iddemos','data','twotankdata'));
z = iddata(y,u,0.2,'Name','Two tanks');

FileName = 'twotanks_c';

Order = [1 1 2];

Parameters = {0.5;0.0035;0.019; ...
    9.81;0.25;0.016};

InitialStates = [0;0.1];

Ts = 0;

nlgr = idnlgrey(FileName,Order,Parameters,InitialStates,Ts, ...
    'Name','Two tanks');
nlgr.Parameters(1).Fixed = true;
nlgr.Parameters(4).Fixed = true;
nlgr.Parameters(5).Fixed = true;

nlgr = nlgreyest(z,nlgr);