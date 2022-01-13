classdef CellChecker
    %CELLCHECKER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = CellChecker()
            %CELLCHECKER Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
    
    methods (Static)
        
        function uStates = createTSfromCells(CellRows)

            states = ["X"+num2str(CellRows(:,1)) "Y"+num2str(CellRows(:,2))];
            uStates= states(:,1)+states(:,2);
        end
        
        function dG = createDigraph(states)
            idx = length(states);
            dG = digraph(states(1:(idx-1)), states(2:idx));
        end
        
        function clean_dG = removeTheSameStates(dG2, states1,states2)

            states2Remove= [];
            for i=1:length(states1)
                for j=1:length(states2)
                    
                    if strcmp(states1(i),states2(j))
                        
                        %if checkTemporalDiff(states1(i),states2(j))
                        states2Remove = [states2Remove; states1(i)];
                    else
                        
                    end
                    
                end
            end
            
            clean_dG = dG2.rmnode(states2Remove);
        end
        
        function bool = checkTemporalDiff(state1Row,state2Row)
            bool = true;
            if state1Row(3)< state2Row(4)
                if state1Row(4)>stateRow(3)
                    bool = false;
                end
            end
            
        end
        
        
    end
end

