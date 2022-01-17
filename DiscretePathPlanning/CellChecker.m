classdef CellChecker
% Check if discrete trajectory (consisting of discrete discreteCells) is safe
% against other discrete trajectories by comparing the transition systems
    
    methods
        function obj = CellChecker()
            %CELLCHECKER Construct an instance of this class
            %   Detailed explanation goes here
        end
    end
    
    methods (Static)
        function TS = createTSfromCells(discreteCells)
        % Create a transition system containing the states (transition from state to next state in 
        % the list) and the entrance and exit time to each state
            
            X_cell = discreteCells(:, 1);
            Y_cell = discreteCells(:, 2);
            t_enter = discreteCells(:, 3);
            t_exit = discreteCells(:, 4);
            
            states = ["X" + num2str(X_cell), "Y" + num2str(Y_cell)];
            uStates = states(:, 1) + states(:, 2);
            
            TS.states = uStates;
            TS.entranceTime = t_enter;
            TS.exitTime = t_exit;
        end
        
        function dG = createDigraph(states)
        % Create a digraph from states
           
            % TODO: Multiple cells are occupied at the same time
            idx = length(states);
            dG = digraph(states(1:(idx-1)), states(2:idx));
        end
        
        function dG = mergeDigraphs(dG, dG2Addd)
        % Merge two digraphs
            
            newNodes = setdiff(dG2Addd.Nodes, dG.Nodes);
            [~, id_newEdges] = setdiff(cell2mat(dG2Addd.Edges.EndNodes), cell2mat(dG.Edges.EndNodes), 'rows');
            newEdges = dG2Addd.Edges.EndNodes(id_newEdges, :);
            
            dG = addnode(dG, newNodes);
            dG = addedge(dG, newEdges(:, 1), newEdges(:, 2));
        end
        
        function isSafe = isSafeTransitions(TS1, TS2)
        % Check whether two transition systems are safe against each other
            
            isSafe = true;
            
            [overlappingStates, id_overlapping_TS1, id_overlapping_TS2] = intersect(TS1.states, TS2.states);
            
            if ~isempty(overlappingStates)
                isSafe_temporal = CellChecker.isSafeTemporalDiff(TS1, TS2, id_overlapping_TS1, id_overlapping_TS2);
                
                if ~isSafe_temporal
                    isSafe = false;
                end
            end
        end

        function isSafe = isSafeTemporalDiff(TS1, TS2, id_TS1, id_TS2)
        % Check whether the temporal difference between relevant equal states of  
        % two transition systems are safe against each other
            
            isSafe = all((TS1.entranceTime(id_TS1) > TS2.exitTime(id_TS2))) || ...% Enter after other has left ...OR... 
                     all((TS1.exitTime(id_TS1) < TS2.exitTime(id_TS2))); % Exit before other has entered
        end
    end
end

