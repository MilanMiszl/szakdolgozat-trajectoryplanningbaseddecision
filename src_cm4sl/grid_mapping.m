% Skálázási faktor
scaleFactor = 10; % 0.1 méteres felbontás minden grid cellához

% Jármű pozíciók globális koordinátákban
Ego_Vhcl_Pos = [20 -1.75  0.6467 0];
Target_Vhcl_Pos = [100 -1.75 0.6467 0];

% Lokális koordinátarendszer beállítása az Ego járműhöz képest
egoX = 30 * scaleFactor; % Felskálázott koordináták
egoY = 20 * scaleFactor;

plandist = 15 * scaleFactor; % Felskálázott távolság a tervezéshez
laneWidth = 3.5 * scaleFactor; % Sáv szélessége felskálázva
numLanes = 2; % Sávok száma
laneStartY = -3.5 * scaleFactor; % Az úttest kezdő pozíciója Y tengelyen
laneEndY = 3.5 * scaleFactor; % Az úttest vége Y tengelyen

% Target Vehicle pozíciójának kiszámítása lokális koordinátákban
trgtX = egoX + (Target_Vhcl_Pos(1) - Ego_Vhcl_Pos(1)) * scaleFactor;
trgtY = egoY + (Target_Vhcl_Pos(2) - Ego_Vhcl_Pos(2)) * scaleFactor;

% Jármű által foglalt terület
trgtFL = [trgtX + 3 * scaleFactor, trgtY + 1.75 * scaleFactor];
trgtFR = [trgtX + 3 * scaleFactor, trgtY - 1.75 * scaleFactor];
trgtRR = [trgtX - 3 * scaleFactor, trgtY - 1.75 * scaleFactor];

% Generáljuk a négy sarokpont közötti összes egész szám koordinátát
[xGrid, yGrid] = meshgrid(floor(trgtRR(1)):ceil(trgtFL(1)), ...
                          floor(trgtFR(2)):ceil(trgtFL(2)));
trgt_Dim = [xGrid(:), yGrid(:)];

% Legközelebbi sáv kiválasztása az ego járműhöz
localLaneStartY = round((egoY + laneStartY - Ego_Vhcl_Pos(2) * scaleFactor));
validLanes = (localLaneStartY:laneWidth:localLaneStartY + numLanes * laneWidth) > egoY; % Szűrés feltétele
laneDistances = abs((localLaneStartY:laneWidth:localLaneStartY + numLanes * laneWidth) - egoY);
laneDistances(~validLanes) = Inf; % Az egoY-nál kisebb sávokat kizárjuk a távolság számításból

% Legközelebbi sáv meghatározása a fennmaradó sávok közül
[~, closestLaneIdx] = min(laneDistances); % A legközelebbi

% Úttest sávhatárok foglaltsági mátrixának generálása
laneBorders = [];
for i = 0:numLanes
    yLaneBorder = localLaneStartY + i * laneWidth; % Felskálázott sávhatár Y-koordinátája
    
    % Sáv megszakítása az első bal oldali sávhatárnál az előzendő jármű előtt és után
    if i == closestLaneIdx -1 % Első sávhatár az ego bal oldalán (növekvő Y irányban)
        preBreakX = trgtX - 30 * scaleFactor; % Jármű előtti 30 méter
        postBreakX = trgtX + 40 * scaleFactor; % Jármű utáni 40 méter

        % Szakasz 1: sávhatár a jármű előtti részen
        laneBorders = [laneBorders; 1, preBreakX, yLaneBorder];
        % Szakasz 2: sávhatár a jármű utáni részen
        laneBorders = [laneBorders; postBreakX, 180 * scaleFactor, yLaneBorder];
    else
        % Más sávok esetében folyamatos határ
        laneBorders = [laneBorders; 1, 180 * scaleFactor, yLaneBorder];
    end
end

% Felskálázott occupancy map létrehozása
ss = stateSpaceSE2([0 180 * scaleFactor; 0 40 * scaleFactor; -pi pi]);
map = occupancyMap(180 * scaleFactor, 40 * scaleFactor, 1); % Nagyobb felbontás
[X, Y] = meshgrid(1:(40 * scaleFactor), 1:(180 * scaleFactor)); % Térkép koordinátái
coords = [X(:), Y(:)]; % Mátrixba rendezés
setOccupancy(map, coords, 0, "grid"); % Minden cella szabadra állítása
setOccupancy(map, trgt_Dim, 1, "local"); % Előzendő jármű területének foglaltra állítása

% Sávhatárok hozzáadása szélességgel
laneThickness = 5; % Sávhatárok vastagsága (grid cellákban)
for k = 1:size(laneBorders, 1) % Sávhatárok beállítása foglaltként
    yCoords = laneBorders(k, 1):laneBorders(k, 2); % Grid Y-koordináták
    for t = -laneThickness:laneThickness
        xCoords = repmat(laneBorders(k, 3) + t, size(yCoords)); % X-koordináták vastagsággal
        boundaryCoords = [xCoords', yCoords']; % Összevonás koordinátapárokba
        setOccupancy(map, boundaryCoords, 1, "grid"); % Sávhatárok foglaltként megjelölése
    end
end

% Állapot validátor létrehozása
validator = validatorOccupancyMap(ss, "Map", map);
validator.ValidationDistance = 1;

% show(map)
% hold on;
% xlabel(sprintf('X koordináta (10^{-1} méter)'));
% ylabel(sprintf('Y koordináta (10^{-1} méter)'));
% hold off

%%
% Térkép méretei
mapWidth = 180 * scaleFactor;
mapHeight = 40 * scaleFactor;
% Térkép megjelenítése
figure;
hold on;
xlim([0 mapWidth]);
ylim([0 mapHeight]);
axis equal;
xlabel('X');
ylabel('Y');
title('Ego és Target jármű helyzete, sáv információk foglaltsági térkép adatai alapján');
% Ego jármű pozíciójának és orientációjának megjelenítése
plot(egoX, egoY, 'bo', 'MarkerSize', 10, 'DisplayName', 'Ego Vehicle');
quiver(egoX, egoY, cos(Ego_Vhcl_Pos(4)), sin(Ego_Vhcl_Pos(4)), 50, 'b', 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', 'Orientation');
% Előzendő jármű pozíciójának és orientációjának megjelenítése
plot(trgtX, trgtY, 'ro', 'MarkerSize', 10, 'DisplayName', 'Target Vehicle');
quiver(trgtX, trgtY, cos(Target_Vhcl_Pos(4)), sin(Target_Vhcl_Pos(4)), 50, 'r', 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', 'Orientation');
% Sávhatárok megjelenítése
for k = 1:size(laneBorders, 1)
    xCoords = [laneBorders(k, 1), laneBorders(k, 2)];
    yCoord = laneBorders(k, 3);
    plot(xCoords, [yCoord yCoord], 'k--', 'LineWidth', 1, 'DisplayName', 'Lane Border');
end
hold off;

