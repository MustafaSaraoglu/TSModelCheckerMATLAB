classdef VehicleTS < handle
    
    properties
        states                  (1,:) string    % states S
        actions                 (1,:) string    % actions Act
        transitions             (:,3) string    % transitions Tr
        initialStates           (1,:) string    % initial states I
        atomicProps             (1,:) string    % atomic propositions AP
        labels                  (:,2) string    % labels ( state | label ) L
    end
    
    methods
        function obj = VehicleTS(states, actions, transitions, initialStates, atomicProps, labels)
            obj.states = states;
            obj.actions = actions;
            obj.transitions = transitions;
            obj.initialStates = initialStates;
            obj.atomicProps = atomicProps;
            obj.labels = labels;
        end
        
        function plotDigraph(obj)
            G = digraph(obj.transitions(:,1)',obj.transitions(:,2)');
            G.Nodes.Name = "s"+obj.states';
            plot(G);
        end
        
        function transitions = createDigraphFromRoad(obj,RoadGrids)
            transitions = [];
            
            for x = 1:size(RoadGrids,2)-1
                                
                for y = 1:size(RoadGrids,1)
                    %Longitudinal
                    transitions = [transitions; RoadGrids(y,x) RoadGrids(y,x+1)];
                    %Lateral
                    if y-1>0
                        transitions = [transitions; RoadGrids(y,x) RoadGrids(y-1,x+1)];
                    end
                    if y+1<=size(RoadGrids,1)
                        transitions = [transitions; RoadGrids(y,x) RoadGrids(y+1,x+1)];
                    end
                end
            end
        end
        
        function visualize_search(obj,G,t)
            % G is a graph or digraph object, and t is a table resulting from a call to
            % BFSEARCH or DFSEARCH on that graph.
            %
            % Example inputs: G = digraph([1 2 3 3 3 3 4 5 6 7 8 9 9 9 10], ...
            %     [7 6 1 5 6 8 2 4 4 3 7 1 6 8 2]);
            % t = dfsearch(G, 1, 'allevents', 'Restart', true);
            
            % Copyright 1984-2019 The MathWorks, Inc.
            
            isundirected = isa(G, 'graph');
            if isundirected
                % Replace graph with corresponding digraph, because we need separate
                % edges for both directions
                [src, tgt] = findedge(G);
                G = digraph([src; tgt], [tgt; src], [1:numedges(G), 1:numedges(G)]);
            end
            
            h = plot(G,'NodeColor',[0.5 0.5 0.5],'EdgeColor',[0.5 0.5 0.5], ...
                'EdgeLabelMode', 'auto');
            
            for ii=1:size(t,1)
                switch t.Event(ii)
                    case 'startnode'
                        highlight(h,t.Node(ii),'MarkerSize',min(h.MarkerSize)*2);
                    case 'discovernode'
                        highlight(h,t.Node(ii),'NodeColor','r');
                    case 'finishnode'
                        highlight(h,t.Node(ii),'NodeColor','k');
                    otherwise
                        if isundirected
                            a = G.Edges.Weight;
                            b = t.EdgeIndex(ii);
                            edgeind = intersect(find(a == b),...
                                findedge(G,t.Edge(ii,1),t.Edge(ii,2)));
                        else
                            edgeind = t.EdgeIndex(ii);
                        end
                        switch t.Event(ii)
                            case 'edgetonew'
                                highlight(h,'Edges',edgeind,'EdgeColor','b');
                            case 'edgetodiscovered'
                                highlight(h,'Edges',edgeind,'EdgeColor',[0.8 0 0.8]);
                            case 'edgetofinished'
                                highlight(h,'Edges',edgeind,'EdgeColor',[0 0.8 0]);
                        end
                end
                
                nodeStr = t.Node;
                if isnumeric(nodeStr)
                    nodeStr = num2cell(nodeStr);
                    nodeStr = cellfun(@num2str, nodeStr, 'UniformOutput', false);
                end
                
                edgeStr = t.Edge;
                if isnumeric(edgeStr)
                    edgeStr = num2cell(edgeStr);
                    edgeStr = cellfun(@num2str, edgeStr, 'UniformOutput', false);
                end
                
                if iscell(t.Node(ii))
                    title([char(t{ii, 1}) ' on Node ' nodeStr{ii}]);
                else
                    title([char(t{ii, 1}) ' on Edge (' edgeStr{ii, 1} ', '...
                        edgeStr{ii, 2},') with edge index ' sprintf('%d ', t{ii, 4})]);
                end
                
            end
            disp('Done.')
        end 
        end
    
end

