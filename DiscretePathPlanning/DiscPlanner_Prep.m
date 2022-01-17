% Case Study - Sim time = 15.3850

%% Discrete trajectories other vehicles
occupiedCells_otherVehicle1 = [
   43.0000    2.0000   15.3850   15.5050;
   44.0000    2.0000   15.5050   16.0750;
   45.0000    2.0000   16.0750   16.8050;
   46.0000    2.0000   16.8050   18.1350;
   47.0000    2.0000   18.1350   20.3850];

occupiedCells_otherVehicle2 = [
   49.0000    4.0000   15.3850   15.7950;
   50.0000    4.0000   15.7950   16.2450;
   51.0000    4.0000   16.2450   16.7650];
%    52.0000    4.0000   16.7650   17.3850; % Comment out for unreachable states
%    53.0000    4.0000   17.3850   18.2650; 
%    54.0000    4.0000   18.2650   20.3850];

%% Discrete Trajectories ego vehicle (decisions)
% Decision 1: 
egoVehicle_occCells_Traj1 = [
   39.0000    2.0000   15.3850   15.8850;
   40.0000    2.0000   15.8850   15.9250;
   40.0000    3.0000   15.9250   16.0550;
   40.0000    4.0000   16.0550   16.3750;
   41.0000    4.0000   16.3750   16.7750;
   42.0000    4.0000   16.7750   17.1550;
   43.0000    4.0000   17.1550   17.5150;
   44.0000    4.0000   17.5150   17.8550;
   45.0000    4.0000   17.8550   18.1850;
   46.0000    4.0000   18.1850   18.5050;
   47.0000    4.0000   18.5050   18.8050;
   48.0000    4.0000   18.8050   19.0950;
   49.0000    4.0000   19.0950   19.3750;
   50.0000    4.0000   19.3750   19.6550;
   51.0000    4.0000   19.6550   19.9150;
   52.0000    4.0000   19.9150   20.1750;
   53.0000    4.0000   20.1750   20.3850];

% Decision 2:
egoVehicle_occCells_Traj2 = [
   39.0000    2.0000   15.3850   15.8650;
   40.0000    2.0000   15.8650   16.3050;
   41.0000    2.0000   16.3050   16.7150;
   42.0000    2.0000   16.7150   17.1050;
   43.0000    2.0000   17.1050   17.4550;
   44.0000    2.0000   17.4550   17.4650;
   45.0000    2.0000   17.4650   17.8150;
   46.0000    2.0000   17.8150   17.9250;
   47.0000    2.0000   17.9250   18.1450;
   48.0000    2.0000   18.1450   18.4650;
   49.0000    2.0000   18.4650   18.7650;
   50.0000    2.0000   18.7650   19.0650;
   51.0000    2.0000   19.0650   19.3450;
   52.0000    2.0000   19.3450   19.6250;
   53.0000    2.0000   19.6250   19.8850;
   54.0000    2.0000   19.8850   20.1450;
   55.0000    2.0000   20.1450   20.3850];

% Decision 3:
egoVehicle_occCells_Traj3 = [
   39.0000    2.0000   15.3850   15.8650;
   40.0000    2.0000   17.4650   17.8150;
   41.0000    2.0000   19.0650   19.3450;
   42.0000    2.0000   20.1450   20.3850];

% Decision 4:
egoVehicle_occCells_Traj4 = [
   39.0000    2.0000   15.3850   15.8650;
   40.0000    2.0000   17.4650   17.8150;
   40.0000    1.0000   20.1450   20.3850];

%% Structure 
decisions = cell(1, 3);
decisions{1} = egoVehicle_occCells_Traj1;
decisions{2} = egoVehicle_occCells_Traj2;
decisions{3} = egoVehicle_occCells_Traj3;
decisions{4} = egoVehicle_occCells_Traj4;

TS_Others = cell(1, 2);
TS_Others{1} = CellChecker.createTSfromCells(occupiedCells_otherVehicle1);
TS_Others{2} = CellChecker.createTSfromCells(occupiedCells_otherVehicle2);

%% Compute safe decisions
dG_safeDecisions = []; % Digraph of all safe decisions
X_max = -inf; % Furthest reachable longitudinal cell position
bestDecision = []; % Store best decision according to furthest reachable longitudinal cell position
bestTransitions = []; % Transitions to furthest destination

% Check trajectrory from each decision against all trajectories from other vehicles
for id_decision = 1:length(decisions)
    occupied_cells_Ego = decisions{id_decision};
    
    TS_Ego = CellChecker.createTSfromCells(occupied_cells_Ego);
    
    for id_other = 1:length(TS_Others)
        TS_Other = TS_Others{id_other};
        isSafeTS = CellChecker.isSafeTransitions(TS_Ego, TS_Other);
        if ~isSafeTS
            break
        end
    end
    
    if isSafeTS
        dG_decision = CellChecker.createDigraph(TS_Ego.states);
        if isempty(dG_safeDecisions)
            dG_safeDecisions = dG_decision;
        else
            dG_safeDecisions = CellChecker.mergeDigraphs(dG_safeDecisions, dG_decision);
        end
        
        % Get safe decision which goes furthest
        X_reachable = occupied_cells_Ego(end, 1);
        if X_reachable > X_max
            X_max = X_reachable;
            bestDecision = id_decision;
            bestTransitions = dG_decision.Edges.EndNodes;
        end
    end
end

%% Show results
plot_digraph = plot(dG_safeDecisions);
disp(['Best decision: ', num2str(bestDecision)]);
highlight(plot_digraph, bestTransitions(:, 1), bestTransitions(:, 2));