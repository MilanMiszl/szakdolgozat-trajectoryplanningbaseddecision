Ego_Vhcl_Pos = [110 -5.24  0.6467 0];
Target_Vhcl_Pos = [130 -5.24 0.6467];
Vhcl_Dim = [4.8 1.8 1.29];

% Lokális koordináta rendszerbe transzformálás
egoX = 30;
egoY = 20;
planstart = 20;
endpos = 20;
trgtX = egoX + Target_Vhcl_Pos(1) - Ego_Vhcl_Pos(1);
trgtY = egoY + Target_Vhcl_Pos(2) - Ego_Vhcl_Pos(2); 
localTargetPos = [trgtX, trgtY];
laneWidth = 3.5;

% Jármű által foglalt terület
trgtFL = [trgtX+Vhcl_Dim(1)/2, trgtY+Vhcl_Dim(2)/2];
trgtFR = [trgtX+Vhcl_Dim(1)/2, trgtY-Vhcl_Dim(2)/2];
trgtRL = [trgtX-Vhcl_Dim(1)/2, trgtY+Vhcl_Dim(2)/2];
trgtRR = [trgtX-Vhcl_Dim(1)/2, trgtY-Vhcl_Dim(2)/2];
trgt_Dim = [trgtFL; trgtFR; trgtRL; trgtRR];

% Foglaltsági térkép létrehozása
ss = stateSpaceSE2([0 180; 0 40; -pi pi]);
map = occupancyMap(180, 40, 1);
[X, Y] = meshgrid(1:40, 1:180); % Térkép összes cellájának koordinátái
coords = [X(:), Y(:)]; % Koordináták mátrixba rendezése
setOccupancy(map, coords, 0,"grid"); % Minden cella szabadra állítása
setOccupancy(map, trgt_Dim, 1,"local")
    
% Állapot validátor létrehozása
validator = validatorOccupancyMap(ss, "Map", map);
validator.ValidationDistance = 0.1;
    
% Útvonal tervezés
globpath_points = [];
plannerExecuted = false;
if ((egoX >= trgtX-planstart-2) || (egoX >= trgtX-2 && egoY >= trgtY+laneWidth-0.5 && egoY <= trgtY+laneWidth+0.5)) && egoX<trgtX+endpos
    planner = plannerHybridAStar(validator, 'MinTurningRadius', 6.1, 'MotionPrimitiveLength', 6, 'ReverseCost', 100, 'DirectionSwitchingCost', 100, 'NumMotionPrimitives', 15);
    disp('if')
    % Útvonal adatai
    stpos = [egoX egoY];
    start = [stpos 0];
    if egoX >= trgtX-planstart && egoX <= trgtX-5 && egoY-trgtY < 1 && plannerExecuted == false
        disp('Sávváltás balra')
        planpos = [Ego_Vhcl_Pos(1) - egoX, Ego_Vhcl_Pos(2)-egoY];
        glpos = [trgtX-5, trgtY+laneWidth];
        goal = [glpos 0];
        % Referencia trajektória
        refpath = plan(planner, start, goal);
        % Trajektória globális koordináta rendszerben
        globpath_points = [refpath.States(:,1) + planpos(1), refpath.States(:,2) + planpos(2), refpath.States(:,3)];
        plannerExecuted = true;
    end
    if egoX >= trgtX-5 && egoX <= trgtX && egoY > trgtY+laneWidth-0.2 && plannerExecuted==true
        disp('Előzés')
        globpath_points = [Ego_Vhcl_Pos(1) + (0:5)', repmat(Target_Vhcl_Pos(2)+laneWidth, 6, 1), zeros(6, 1)];
        plannerExecuted = false;
    end
    if egoX >= trgtX+2 && egoY >= trgtY+laneWidth-0.5 && egoY <= trgtY+laneWidth+0.5 && plannerExecuted==false
        disp('Sávváltás jobbra')
        planpos = [Ego_Vhcl_Pos(1) - egoX, Ego_Vhcl_Pos(2)-egoY];
        glpos = [trgtX+endpos, trgtY];
        goal = [glpos 0];
        % Referencia trajektória
        refpath = plan(planner, start, goal);
        % Trajektória globális koordináta rendszerben
        globpath_points = [refpath.States(:,1) + planpos(1), refpath.States(:,2) + planpos(2), refpath.States(:,3)];
        plannerExecuted = true;
    end
elseif plannerExecuted == false
    disp('else, egyenes trajektoria')

    % Ha az útvonaltervezés nem futott le, töltsük fel a globpath_points változót 0 értékekkel
    globpath_points = [Ego_Vhcl_Pos(1) + (0:5)', repmat(Target_Vhcl_Pos(2), 6, 1), zeros(6, 1)];
end
if ~isempty(globpath_points)
    show(planner)
    hold on;
    % Visszatranszformálás lokális koordinátarendszerbe
    local_globpath_points = [globpath_points(:, 1) - Ego_Vhcl_Pos(1) + egoX, globpath_points(:, 2) - Ego_Vhcl_Pos(2) + egoY];
    plot(local_globpath_points(:, 1), local_globpath_points(:, 2), '-o');
    plot(egoX, egoY, '-o', 'MarkerSize', 2, 'LineWidth', 5); % egoX, egoY pozíció
    hold off;
    title('Trajektória');
    xlabel('X koordináta');
    ylabel('Y koordináta');
    grid on;
end

%%
lineEquations = [];
distances = [];
% Referencia útvonal pontjait összekötő egyenesek számítása
if size(globpath_points) ~= 0
    for i = 1:size(globpath_points,1)-1
    x1 = globpath_points(i,1);
    y1 = globpath_points(i,2);
    x2 = globpath_points(i+1,1);
    y2 = globpath_points(i+1,2);
    % Egyenes egyenletének meghatározása
    A = y2-y1;
    B = x1-x2;
    C = (x2-x1)*y1 - (y2-y1)*x1;
    lineEquations = [lineEquations; A, B, C];
    % distances = abs(A*x+B*y+C)/sqrt(A^2+B^2)
    distances = [distances; x1 + (x2-x1)/2 , abs(lineEquations(i,1)*Ego_Vhcl_Pos(1)+lineEquations(i,2)*abs(Ego_Vhcl_Pos(2))+lineEquations(i,3))/sqrt(lineEquations(i,1)^2+lineEquations(i,2)^2), globpath_points(i+1,3)];
    end
    valid_distances = distances(distances(:,1) >= Ego_Vhcl_Pos(1) & distances(:,1) <= Ego_Vhcl_Pos(1) + 3, :);
    [~, min_index] = min(valid_distances(:,2));
    lateral_error = valid_distances(min_index, 2);
    yaw_error = valid_distances(min_index, 3) - Ego_Vhcl_Pos(4);
end
