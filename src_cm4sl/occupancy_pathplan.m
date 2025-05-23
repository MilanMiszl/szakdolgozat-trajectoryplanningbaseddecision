% Jármű pozíciók globális koordinátákban
Ego_Vhcl_Pos = [160 -1.75 0.6467 0];
Target_Vhcl_Pos = [200 -1.75 0.6467 0];
% Skálázási faktor
scaleFactor = 10; % 0.1 méteres felbontás minden grid cellához

% Lokális koordinátarendszer beállítása az Ego járműhöz képest
egoX = 10 * scaleFactor; % Felskálázott koordináták
egoY = 10 * scaleFactor;
laneWidth = 3.5 * scaleFactor; % Sáv szélessége felskálázva
numLanes = 2; % Sávok száma
laneStartY = -3.5 * scaleFactor; % Az úttest kezdő pozíciója Y tengelyen
laneEndY = 3.5 * scaleFactor; % Az úttest vége Y tengelyen

% Target Vehicle pozíciójának kiszámítása lokális koordinátákban
trgtX = egoX + (Target_Vhcl_Pos(1) - Ego_Vhcl_Pos(1)) * scaleFactor;
trgtY = egoY + (Target_Vhcl_Pos(2) - Ego_Vhcl_Pos(2)) * scaleFactor;

% Jármű által foglalt terület
trgtFL = [trgtX + 5 * scaleFactor, trgtY + 3 * scaleFactor];
trgtFR = [trgtX + 5 * scaleFactor, trgtY - 3 * scaleFactor];
trgtRR = [trgtX - 4 * scaleFactor, trgtY - 3 * scaleFactor];

% Generáljuk a négy sarokpont közötti összes egész szám koordinátát
[xGrid, yGrid] = meshgrid(floor(trgtRR(1)):ceil(trgtFL(1)), ...
                          floor(trgtFR(2)):ceil(trgtFL(2)));
trgt_Dim = [xGrid(:), yGrid(:)];

% Legközelebbi sáv kiválasztása az ego járműhöz
localLaneStartY = round((egoY + laneStartY - Ego_Vhcl_Pos(2) * scaleFactor));
validLanes = (localLaneStartY:laneWidth:localLaneStartY + numLanes * laneWidth) > trgtY; % Szűrés feltétele
laneDistances = abs((localLaneStartY:laneWidth:localLaneStartY + numLanes * laneWidth) - trgtY);
laneDistances(~validLanes) = Inf; % Az egoY-nál kisebb sávokat kizárjuk a távolság számításból, mindig a bal oldali kell
[~, closestLaneIdx] = min(laneDistances); % A legközelebbi 

% Úttest sávhatárok foglaltsági mátrixának generálása
laneBorders = [];
for i = 0:numLanes
    yLaneBorder = localLaneStartY + i * laneWidth; % Felskálázott sávhatár Y-koordinátája
    
    % Sáv megszakítása az első bal oldali sávhatárnál az előzendő jármű előtt és után
    if i == closestLaneIdx -1 % Első sávhatár az ego bal oldalán (növekvő Y irányban)
        
        preBreakX = trgtX - 30 * scaleFactor; % Jármű előtti 30 méter
        postBreakX = trgtX + 60 * scaleFactor; % Jármű utáni 40 méter

        % Szakasz 1: sávhatár a jármű előtti részen
        laneBorders = [laneBorders; 1, preBreakX, yLaneBorder];
        % Szakasz 2: sávhatár a jármű utáni részen
        laneBorders = [laneBorders; postBreakX, 140 * scaleFactor, yLaneBorder];
    else
        % Más sávok esetében folyamatos határ
        laneBorders = [laneBorders; 1, 140 * scaleFactor, yLaneBorder];
    end
end

%% Loop
planned = false;    %persistent
if isempty(planned)
    planned = false;
end
executed = true;  %persistent
if isempty(executed)
    executed = true;
end

if planned == false && executed == true

    % Felskálázott occupancy map létrehozása
    ss = stateSpaceSE2([0 140 * scaleFactor; 0 20 * scaleFactor; -pi pi]);
    map = occupancyMap(140 * scaleFactor, 20 * scaleFactor, 1); % Nagyobb felbontás
    [X, Y] = meshgrid(1:(20 * scaleFactor), 1:(140 * scaleFactor)); % Térkép koordinátái
    coords = [X(:), Y(:)]; % Mátrixba rendezés
    setOccupancy(map, coords, 0, "grid"); % Minden cella szabadra állítása
    setOccupancy(map, trgt_Dim, 1, "local"); % Előzendő jármű területének foglaltra állítása

    % Sávhatárok hozzáadása 
    laneThickness = 1; % Sávhatárok vastagsága (grid cellákban)
    for k = 1:size(laneBorders, 1) % Sávhatárok beállítása foglaltként
        xCoords = laneBorders(k, 1):laneBorders(k, 2); % Grid X-koordináták
        for t = -laneThickness:laneThickness
            yCoords = repmat(laneBorders(k, 3) + t, size(xCoords)); % Y-koordináták 
            boundaryCoords = [xCoords', yCoords']; % Összevonás koordinátapárokba
            setOccupancy(map, boundaryCoords, 1, "local"); % Sávhatárok foglaltként megjelölése
        end
    end

    % Állapot validátor létrehozása
    validator = validatorOccupancyMap(ss, "Map", map);
    validator.ValidationDistance = 1; % Validation distance a skálázott térképen
    %figure(1)
    %show(map)

    % Útvonal tervezés
    planner = plannerHybridAStar(validator, 'MinTurningRadius', 6.1*scaleFactor, 'MotionPrimitiveLength', 9*scaleFactor, 'ReverseCost', 100, 'DirectionSwitchingCost', 100, 'NumMotionPrimitives', 13);

    globpath_points = [];
    lineEquations = [];
    distances = [];
    planpos = [Ego_Vhcl_Pos(1) - (egoX/scaleFactor), Ego_Vhcl_Pos(2) - (egoY/scaleFactor)];
    start = [egoX, egoY, Ego_Vhcl_Pos(4)];
    if (egoX+(100*scaleFactor) < trgtRR(1)-(40*scaleFactor)) || (egoX+(100*scaleFactor) >= trgtFR(1)+(50*scaleFactor))
        goal = [egoX + 100*scaleFactor, trgtY, 0];
    else
        goal = [trgtRR(1) - 40*scaleFactor, trgtY, 0];
    end
    
    planned = true;
    executed = false;
    % Referencia trajektória
    refpath = plan(planner, start, goal);
    % Trajektória globális koordináta rendszerben
    globpath_points = [refpath.States(:,1)/scaleFactor + planpos(1), refpath.States(:,2)/scaleFactor + planpos(2), refpath.States(:,3)];
else
    globpath_points = [];
    distances = [];
end

figure(2)
show(planner)
hold on
legend('Mozgásprimitívek előre', 'Mozgásprimitívek hátra', 'Tervezett útvonal', 'Start pozíció' , 'Cél pozíció');
xlabel(sprintf('X koordináta (10^{-1} méter)'));
ylabel(sprintf('Y koordináta (10^{-1} méter)'));
hold off

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
            lineEquations = [lineEquations; globpath_points(i+1,1), A, B, C, globpath_points(i+1,3), globpath_points(i+1,2), curvature];
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
            distances = [distances; lineEquations(i,1), 0, lineEquations(i,5), lineEquations(i,6), lineEquations(i,7)];
        else
            distances = [distances; lineEquations(i,1), dist, lineEquations(i,5), lineEquations(i,6), lineEquations(i,7)];
            dx = Ego_Vhcl_Pos(1) - lineEquations(i,1);
            dy = Ego_Vhcl_Pos(2) - lineEquations(i,6);
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
