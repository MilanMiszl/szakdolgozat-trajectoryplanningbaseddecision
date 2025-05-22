%Jármű pozíciók
Ego_Vhcl_Pos = [155 1.75  0.6467 0];
Target_Vhcl_Pos = [130 -1.75 0.6467];
%Vhcl_Dim = [4.8 1.8 1.29];

% Lokális koordináta rendszerbe transzformálás
egoX = 30;
egoY = 20;

plandist = 25;
laneWidth = 3.5;
numLanes = 2;
laneStartY = -3.5; % Az úttest kezdő pozíciója Y-tengelyen
laneEndY = 3.5; % Az úttest vége Y-tengelyen

% Target Vehicle pos lokális koordináta rendszerben
trgtX = egoX + Target_Vhcl_Pos(1) - Ego_Vhcl_Pos(1);
trgtY = egoY + Target_Vhcl_Pos(2) - Ego_Vhcl_Pos(2); 

% Jármű által foglalt terület
trgtFL = [trgtX + 3, trgtY + 1.2];
trgtFR = [trgtX + 3, trgtY - 1.2];
trgtRR = [trgtX - 3, trgtY - 1.2];
% trgtFL = [trgtX+Vhcl_Dim(1)/2, trgtY+Vhcl_Dim(2)/2];
% trgtFR = [trgtX+Vhcl_Dim(1)/2, trgtY-Vhcl_Dim(2)/2];
% trgtRL = [trgtX-Vhcl_Dim(1)/2, trgtY+Vhcl_Dim(2)/2];
% trgtRR = [trgtX-Vhcl_Dim(1)/2, trgtY-Vhcl_Dim(2)/2];

% Generáljuk a négy sarokpont közötti összes egész szám koordinátát
[xGrid, yGrid] = meshgrid(floor(trgtRR(1)):ceil(trgtFL(1)), floor(trgtFR(2)):ceil(trgtFL(2)));
trgt_Dim = [xGrid(:), yGrid(:)];

% Foglaltsági térkép létrehozása
ss = stateSpaceSE2([0 180; 0 40; -pi pi]);
map = occupancyMap(180, 40, 1);
[X, Y] = meshgrid(1:40, 1:180); % Térkép összes cellájának koordinátái
coords = [X(:), Y(:)]; % Koordináták mátrixba rendezése
setOccupancy(map, coords, 0,"grid"); % Minden cella szabadra állítása
setOccupancy(map, trgt_Dim, 1,"local");

% Állapot validátor létrehozása
validator = validatorOccupancyMap(ss, "Map", map);
validator.ValidationDistance = 0.1;

%show(map)
    
%% Útvonal tervezés

planned = false;    %persistent
if isempty(planned)
    planned = false;
end
executed = true;  %persistent
if isempty(executed)
    executed = true;
end
planner = plannerHybridAStar(validator, 'MinTurningRadius', 6.1, 'MotionPrimitiveLength', 3, 'ReverseCost', 100, 'DirectionSwitchingCost', 100, 'NumMotionPrimitives', 7, 'AnalyticExpansionInterval',15);

if planned == false && executed == true
    globpath_points = [];
    lineEquations = [];
    distances = [];
    distances2 = [];
    planpos = [Ego_Vhcl_Pos(1) - egoX, Ego_Vhcl_Pos(2) - egoY];

    if (egoX + plandist >= trgtRR(1) - 20) && egoX < trgtX -10 
        start = [egoX, egoY, Ego_Vhcl_Pos(4)];
        disp('Balra sávváltás')
        goal = [trgtX - 10, trgtY + laneWidth, 0];
    elseif egoX + plandist > trgtRR(1) - 20 && egoX < trgtRR(1) - 5 && egoY-trgtY <= 1 
        start = [egoX, egoY, Ego_Vhcl_Pos(4)];
        disp('Jármű megközelítés')
        goal = [trgtX - 15, trgtY, 0];
    elseif (egoX >= trgtX-11 && egoX < trgtX +20) && (egoY >= trgtY + laneWidth - 0.75 && egoY <= trgtY + laneWidth + 0.75)
        start = [egoX, egoY, Ego_Vhcl_Pos(4)];
        disp('Előzés')
        goal = [trgtX + 15, trgtY + laneWidth, 0];
    elseif egoX >= trgtX + 19 && (egoY >= trgtY + laneWidth - 0.75 && egoY <= trgtY + laneWidth + 0.75)
        start = [egoX, egoY, Ego_Vhcl_Pos(4)];
        disp('Jobbra sávváltás')
        goal = [egoX + plandist, trgtY, 0];
    else
        % Egyenes út
        planpos = [Ego_Vhcl_Pos(1) - egoX, Ego_Vhcl_Pos(2) - egoY];
        start = [egoX, egoY, Ego_Vhcl_Pos(4)];
        goal = [egoX + plandist, trgtY, 0];

    end
    planned = true;
    executed = false;
    % Referencia trajektória
    refpath = plan(planner, start, goal);
    % Trajektória globális koordináta rendszerben
    globpath_points = [refpath.States(:,1) + planpos(1), refpath.States(:,2) + planpos(2), refpath.States(:,3)];
else
    globpath_points = [];
end

% Megjelenítés 
figure; 
show(planner); 
% Tervezett útvonal megjelenítése 
hold on; 
%quiver(Ego_Vhcl_Pos(1), Ego_Vhcl_Pos(2), cos(Ego_Vhcl_Pos(4)), sin(Ego_Vhcl_Pos(4)), 0.5, 'g'); % Jármű orientációja a kezdő pozícióban 
xlabel('X'); 
ylabel('Y'); 
title(' '); 
localLaneStartY = egoY + laneStartY - Ego_Vhcl_Pos(2);
for i = 1:numLanes+1
    laneY = localLaneStartY + (i-1) * laneWidth; 
    plot([0, 180], [laneY, laneY], '--k', 'LineWidth', 1.5); % Szaggatott fekete vonal
end
legend('Mozgásprimitívek előre', 'Mozgásprimitívek hátra', 'Tervezett útvonal', 'Start pozíció' , 'Cél pozíció' ,'Sáv határok');
hold off;


%% Hibák számolása

distances = [];
distances2 = [];
lateral_error = 0; %persistent
yaw_error = 0; %persistent
if isempty(lateral_error)
    lateral_error = 0;
end
if isempty(yaw_error)
    yaw_error = 0;
end

if ~isempty(globpath_points) && planned == true && executed == false
    for i = 1:size(globpath_points,1)-1
        x1 = globpath_points(i,1);
        y1 = globpath_points(i,2);
        x2 = globpath_points(i+1,1);
        y2 = globpath_points(i+1,2);

        % Egyenes egyenletének meghatározása
        A = -(y2-y1);
        B = x2-x1;
        C = (y2-y1)*x1 - (x2-x1)*y1;
        %lineEquations = [lineEquations; x1 + (x2-x1)/2, A, B, C, refpath.States(i+1,3)];
        lineEquations = [lineEquations; globpath_points(i+1,1), A, B, C, globpath_points(i+1,3), globpath_points(i+1,2)];
        planned = false;
        equals = A*globpath_points(i+1,1)+B*globpath_points(i+1,2)+C;
        disp(equals)
    end
end

if executed == false
    for i = 1:size(lineEquations)
        num = abs(lineEquations(i,2)*Ego_Vhcl_Pos(1)+lineEquations(i,3)*Ego_Vhcl_Pos(2)+lineEquations(i,4));
        denum = sqrt(lineEquations(i,2)^2 + lineEquations(i,3)^2);
        dist = num/denum;
        if abs(dist) < 0.01
            distances = [distances; lineEquations(i,1), 0, lineEquations(i,5)];
            distances2 = [distances2; lineEquations(i,1), 0, 0];
        else
            distances = [distances; lineEquations(i,1), dist, lineEquations(i,5)];
            dx = Ego_Vhcl_Pos(1) - lineEquations(i,1);
            dy = Ego_Vhcl_Pos(2) - lineEquations(i,6);
            distances2 = [distances2; lineEquations(i,1), sqrt(dx^2 + dy^2), dy];
        end
    end

    valid_distances = distances(distances(:,1) >= Ego_Vhcl_Pos(1) & distances(:,1) <= Ego_Vhcl_Pos(1) + 3, :);
    if size(valid_distances) ~= 0
        [~, min_index] = min(valid_distances(:, 2));
        lateral_error = valid_distances(min_index, 2);
        yaw_error = valid_distances(min_index, 3) - Ego_Vhcl_Pos(4); 
    end

    if ~isempty(lineEquations) && Ego_Vhcl_Pos(1) >= lineEquations(end,1)
        executed = true;
        %planned = false;
    end
end
