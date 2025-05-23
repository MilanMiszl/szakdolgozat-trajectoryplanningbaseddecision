% Jármű pozíciók globális koordinátákban
Ego_Vhcl_Pos = [0 -1.75 0.6467 0];
%ego_sp = [0, 0, 0, 0, 0]; %s_dot, d_dot (sebességek), s_ddot, d_ddot (gyorsulások), omega 
Target_Vhcl_Pos = [50 -1.75 0.6467 0;
                     80 -1.75 0.6467 0];
ego_CartS = [0, 12, 0]; % kappa (curve 1/m), speed (m/s), Acc (m/s^2)

% Lokális koordinátarendszer beállítása az Ego járműhöz képest
egoX = 10; % Felskálázott koordináták
egoY = Ego_Vhcl_Pos(2);
laneWidth = 3.5; % Sáv szélessége felskálázva
numLanes = 2; % Sávok száma
laneStartY = -3.5; % Az úttest kezdő pozíciója Y tengelyen
laneEndY = 3.5; % Az úttest vége Y tengelyen

% Target Vehicle pozíciójának kiszámítása lokális koordinátákban
trgtX = egoX + (Target_Vhcl_Pos(:,1) - Ego_Vhcl_Pos(1));
trgtY = egoY + (Target_Vhcl_Pos(:,2) - Ego_Vhcl_Pos(2));

% Jármű által foglalt terület
trgtFR = [];
for i=1:numel(trgtY)
    if trgtY(i) > 3.5
        trgtFR = [trgtFR; trgtX(i) + 3, trgtY(i) - 8.5];
    elseif trgtY(i) > 0
        trgtFR = [trgtFR; trgtX(i) + 3, trgtY(i) - 5];
    else 
        trgtFR = [trgtFR; trgtX(i) + 3, trgtY(i) - 1.5];
    end
end
trgtFL = [trgtX + 3, trgtY + 1.5];
trgtRR = [trgtX - 3, trgtY - 1.5];

% Generáljuk a négy sarokpont közötti összes egész szám koordinátát
trgt_Dim = [];
for i=1:numel(trgtRR(:,1))
    [xGrid, yGrid] = meshgrid(floor(trgtRR(i,1)):ceil(trgtFL(i,1)), ...
                          floor(trgtFR(i,2)):ceil(trgtFL(i,2)));
    trgt_Dim = [trgt_Dim; xGrid(:), yGrid(:) + 10];
end

% Felskálázott occupancy map létrehozása
ss = stateSpaceSE2([0 180; 0 20; -pi pi]);
map = binaryOccupancyMap(180, 20, 1);
map.GridLocationInWorld = [0, -10]; % A középpont Y tengelyen 0
[X, Y] = meshgrid(1:20, 1:180); % Térkép koordinátái
coords = [X(:), Y(:)]; % Mátrixba rendezés
setOccupancy(map, coords, 0, "grid"); % Minden cella szabadra állítása
setOccupancy(map, trgt_Dim, 1, "local"); % Előzendő jármű területének foglaltra állítása

% Állapot validátor
stateValidator = validatorOccupancyMap(ss, "Map", map);
stateValidator.Map = map;
stateValidator.StateSpace.StateBounds(1:2,:) = [map.XWorldLimits; map.YWorldLimits];

refPath = [0, -1.75; 30, -1.75; 60, -1.75; 90 -1.75; 120, -1.75; 150, -1.75; 180, -1.75];

show(map);
hold on;
plot(egoX, egoY, 'r.', 'MarkerSize', 15);
plot(refPath(:, 1), refPath(:, 2), 'g-', 'LineWidth', 2);
hold off;

%% Útvonal tervezés
planned = false;    %persistent
if isempty(planned)
    planned = false;
end
executed = true;  %persistent
if isempty(executed)
    executed = true;
end
prevroutend = []; %persistent
distances = [];
distances2 = [];
lateral_error = []; %persistent
yaw_error = []; %persistent
if isempty(lateral_error)
    lateral_error = 0;
end
if isempty(yaw_error)
    yaw_error = 0;
end

if planned == false && executed == true
    planner = trajectoryOptimalFrenet(refPath,stateValidator);
    planner.TerminalStates.Longitudinal = 100;
    % Távolság az előzendő járműtől
    distanceToTarget = Ego_Vhcl_Pos(1) - Target_Vhcl_Pos(1);
    % Ha az ego jármű legalább 8 méterrel megelőzte a célt, akkor engedélyezd a refPath-hez való visszatérést
    if (abs(distanceToTarget) < 8)
        planner.TerminalStates.Lateral = 3.5;  % Csak az előzési sávot engedélyezd
    else
        planner.TerminalStates.Lateral = 0:3.5:3.5;  % Normál sávok 
    end

    planner.FeasibilityParameters.MaxAcceleration = 4;
    planner.Weights.LongitudinalSmoothness = 100;
    planner.NumSegments = 4;
    
    initCartState = [egoX, Ego_Vhcl_Pos(2)+lateral_error(end), Ego_Vhcl_Pos(4), ego_CartS(1), ego_CartS(2), ego_CartS(3)];
    initFrenetState = cart2frenet(planner, initCartState);

    globpath_points = [];
    lineEquations = [];
    distances = [];
    planpos = [Ego_Vhcl_Pos(1) - egoX, Ego_Vhcl_Pos(2) - egoY]; % Ego_Vhcl_Pos(2) - egoY = 0, mert egoY = Ego_Vhcl_Pos(2)

    planned = true;
    executed = false;
    % Tervezett trajektória
    route = plan(planner,initFrenetState);
    % Trajektória globális koordináta rendszerben 
    globpath_points = [route(:,1) + planpos(1), route(:,2) + planpos(2), route(:,3), route(:,5), route(:,4), route(:,6)];   % [x,y,d_dot,s_dot, d_ddot, s_ddot]
else
    globpath_points = [];
    distances = [];
end

%% Tervezés ábrázolása sávhatárokkal
show(map)
hold on
show(planner,'Trajectory','all')
hold on

refPathEnd = cart2frenet(planner,[planner.Waypoints(end,:) 0 0 0 0]);
laneOffsets = unique([planner.TerminalStates.Lateral+1.75 planner.TerminalStates.Lateral-1.75]);
numLaneOffsets = numel(laneOffsets);
xRefPathEnd = ceil(refPathEnd(1));
laneXY = zeros((numLaneOffsets*xRefPathEnd)+numLaneOffsets,2);
xIndex = 0;

for laneID = 1:numLaneOffsets
    for x = 1:xRefPathEnd
        laneCart = frenet2cart(planner,[x 0 0 laneOffsets(laneID) 0 0]);
        xIndex = xIndex + 1;
        laneXY(xIndex,:) = laneCart(1:2);
    end
    xIndex = xIndex + 1;
    laneXY(xIndex,:) = NaN(1,2);
end

plot(laneXY(:,1),laneXY(:,2),'LineWidth',0.5,'Color',[0.5 0.5 0.5],'DisplayName','Sáv határok','LineStyle','--')
legend('Referencia pontok', 'Referencia útvonal', 'Potenciális trajektória', 'Optimális trajektória' ,'Sáv határok');

%% Hibák számolása

if ~isempty(globpath_points) && planned == true && executed == false
    for i = 1:(size(globpath_points,1)-1)
        x1 = globpath_points(i,1);
        y1 = globpath_points(i,2);
        x2 = globpath_points(i+1,1);
        y2 = globpath_points(i+1,2);
            
        if i+2 <= size(globpath_points)
            x3 = globpath_points(i+2, 1); 
            y3 = globpath_points(i+2, 2); 
        else
            x3 = x2;
            y3 = y2;
        end
        R = ((x2 - x1)^2 + (y2 - y1)^2) / (2 * abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)));
        % Az útkanyarulat előjelének meghatározása 
        if (y3 - y1) > 0 
            curvature = 1/R; % Pozitív kanyarulat 
        elseif (y3 - y1) < 0 
            curvature = -1/R; % Negatív kanyarulat 
        else 
            curvature = 0; % Egyenes út 
        end
        
        % Egyenes egyenlete
        A = -(y2-y1);
        B = x2-x1;
        C = (y2-y1)*x1 - (x2-x1)*y1;
        lineEquations = [lineEquations; globpath_points(i+1,1), A, B, C, atan2((y2-y1),(x2-x1)), globpath_points(i+1,2), curvature, globpath_points(i+1,3), globpath_points(i+1,4), globpath_points(i+1,5), globpath_points(i+1,6)];
        planned = false;
        equals = A*globpath_points(i+1,1)+B*globpath_points(i+1,2)+C;
        %disp(equals)
    end
end

if executed == false
    for i = 1:size(lineEquations)
        num = abs(lineEquations(i,2)*Ego_Vhcl_Pos(1)+lineEquations(i,3)*Ego_Vhcl_Pos(2)+lineEquations(i,4));
        denum = sqrt(lineEquations(i,2)^2 + lineEquations(i,3)^2);
        dist = num/denum;
        if abs(dist) < 0.01
            distances = [distances; lineEquations(i,1), 0, lineEquations(i,5), lineEquations(i,6), lineEquations(i,7), lineEquations(i,8), lineEquations(i,9), lineEquations(i,10), lineEquations(i,11)];
        else
            distances = [distances; lineEquations(i,1), dist, lineEquations(i,5), lineEquations(i,6), lineEquations(i,7), lineEquations(i,8), lineEquations(i,9), lineEquations(i,10), lineEquations(i,11)];
            dx = Ego_Vhcl_Pos(1) - lineEquations(i,1);
            dy = Ego_Vhcl_Pos(2) - lineEquations(i,6);
        end
    end

    valid_distances = distances(distances(:,1) >= Ego_Vhcl_Pos(1) & distances(:,1) <= Ego_Vhcl_Pos(1) + 3, :);
    if size(valid_distances) ~= 0
        [~, min_index] = min(valid_distances(:, 2));
        lateral_error = [lateral_error; valid_distances(min_index, 2)];
        yaw_error = [yaw_error; valid_distances(min_index, 3) - Ego_Vhcl_Pos(4)]; 
        prevroutend = [valid_distances(min_index,1)-Ego_Vhcl_Pos(1)+egoX, valid_distances(min_index,7), valid_distances(min_index,9), valid_distances(min_index,4)-refPath(1,2),valid_distances(min_index,6),valid_distances(min_index,8)];
    end

    if ~isempty(lineEquations) && Ego_Vhcl_Pos(1) >= lineEquations(20,1)
        executed = true;
        %planned = false;
    end
end

