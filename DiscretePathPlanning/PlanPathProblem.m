%% Inputs - Will depend on the current positions during Runtime
states = 1:10;
actions = ["fwd" "left" "right"];
transitions = [[1 2 "left"];[1 3 "fwd"];[2 3 "right"];[3 4 "fwd"];[4 5 "left"];[4 6 "right"];[5 8 "fwd"];[6 7 "fwd"];[7 9 "left"];[7 10 "fwd"];[8 9 "fwd"]];
initialStates = states(1);
atomicProps = false;

labels= split("VehicleAt " + states);
labels = [labels(:,:,1); labels(:,:,2)]';

%% Transition System Generation
Ego = VehicleTS(states, actions, transitions, initialStates, atomicProps, labels);
Ego.plotDigraph();


%% Advanced
RoadGridsX = "x" + [1:15];
RoadGridsY = "y" + [1:6];

RoadGrids = RoadGridsX + RoadGridsY';

DrivingModes = ["FreeDrive" "Follow" "EmergencyBrake"];
LaneModes = ["LaneKeeping" "LaneChanging"];
%ConstantActionResolution = 1 second (rate of changing modes and decisions)

%V1ReachableDiscreteStates = calculateDiscreteReachableSet(RoadGrids, Transitions, currentGrid, setOfPossibleActions, TimeHorizon);

V2ReachableX = "x" + [6:15];
V2ReachableY = "y" + [5:5];
V2Reachable = V2ReachableX + V2ReachableY';
%% Example
RoadTransitions = Ego.createDigraphFromRoad(RoadGrids);
G = digraph(RoadTransitions(:,1)',RoadTransitions(:,2)');
%plot(G)
R1 = dfsearch(G,'x1y2','allevents','Restart', true); % Depth First Search - https://www.mathworks.com/help/matlab/ref/graph.dfsearch.html
R2 = dfsearch(G,'x1y3','allevents','Restart', true);
R3 = dfsearch(G,'x5y5','allevents','Restart', true);
%Ego.visualize_search(G,t)
