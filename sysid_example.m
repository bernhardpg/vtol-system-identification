Order         = [1 1 2];               % Model orders [ny nu nx].
Parameters    = [0.5; 0.003; 0.019; ...
                 9.81; 0.25; 0.016];   % Initial parameter vector.
InitialStates = [0; 0.1];              % Initial values of initial states.


model = idnlgrey('twotanks_c', Order, Parameters, InitialStates, 0)

% Compute model response
y = sim(model,data)