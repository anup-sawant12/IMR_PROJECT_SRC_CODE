%% SmartDustbinRobot.m
% Multi-bin Smart Dustbin Monitoring Robot - MATLAB R2025
% Uses built-in MATLAB graphics and UI only; no extra toolbox required.
% The robot visits four stationary bins and reports a simulated fill level
% for each. Sensor readings are simulated, not read from physical hardware.

clear; clc; close all;

%% Scene dimensions (metres)
chassisL = 0.90; chassisW = 0.62; chassisH = 0.08;
wheelR = 0.13; wheelW = 0.08;
binL = 0.34; binW = 0.32; binH = 0.48;
binBaseZ = 0.02;
sensorOffset = 0.05;

% Four bins on the FLOOR (not mounted on the robot).
% Each row is [x y], where the robot will visit to inspect that bin.
binXY = [-1.35  0.72;
          0.00  0.72;
          1.35  0.72;
          1.35 -0.72];

% Example waste heights for the four separate bins (metres).
% Change these values to test different outputs.
wasteHeights = [0.12; 0.29; 0.41; 0.20];
binNames = {'Bin 1','Bin 2','Bin 3','Bin 4'};

%% Create window and 3D scene
fig = uifigure('Name','Smart Dustbin Monitoring Robot - 4 Bins', ...
    'Position',[60 45 1360 820],'Color',[0.96 0.97 0.98], ...
    'CloseRequestFcn',@closeApp);

ax = uiaxes(fig,'Position',[20 115 900 675]);
title(ax,'Robot visits four stationary dustbins');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
grid(ax,'on'); hold(ax,'on'); view(ax,38,27); axis(ax,'equal');
xlim(ax,[-2.15 2.15]); ylim(ax,[-1.45 1.45]); zlim(ax,[0 1.35]);
ax.Box='on'; ax.Color=[0.94 0.96 0.98];

% Floor plane
[Xg,Yg] = meshgrid(-2.1:0.25:2.1,-1.4:0.25:1.4);
surf(ax,Xg,Yg,zeros(size(Xg))-0.005,'FaceColor',[0.88 0.90 0.88], ...
    'EdgeColor',[0.78 0.81 0.78],'FaceAlpha',0.45);

% Draw four separate stationary bins and waste surfaces.
wastePatches = gobjects(4,1);
for k=1:4
    cx=binXY(k,1); cy=binXY(k,2);
    drawBin(ax,cx,cy,binL,binW,binH,binBaseZ,binNames{k});
    wastePatches(k)=drawWaste(ax,cx,cy,binL,binW,binBaseZ+0.025, ...
        wasteHeights(k),[0.96 0.48 0.08]);
    % Inspection point marker
    plot3(ax,cx,cy,0.025,'ko','MarkerFaceColor',[1 0.85 0.1], ...
        'MarkerSize',7);
end

% Robot assembly is grouped so its whole pose can move.
robot = hgtransform('Parent',ax);
drawCuboid(robot,[-chassisL/2,-chassisW/2,2*wheelR], ...
    [chassisL,chassisW,chassisH],[0.08 0.58 0.25]);

wheelX=[-0.29 0.29];
wheelY=[-chassisW/2-wheelW/2, chassisW/2+wheelW/2];
for ix=1:2
    for iy=1:2
        drawWheel(robot,wheelX(ix),wheelY(iy),wheelR,wheelR,wheelW,[0.12 0.13 0.15]);
    end
end

% Motor housings
for x=wheelX
    drawCuboid(robot,[x-0.09,-chassisW/2+0.01,wheelR+0.015], ...
        [0.18,0.06,0.08],[0.82 0.68 0.12]);
    drawCuboid(robot,[x-0.09,chassisW/2-0.07,wheelR+0.015], ...
        [0.18,0.06,0.08],[0.82 0.68 0.12]);
end

% ESP32 board
drawCuboid(robot,[-0.31,-0.22,2*wheelR+chassisH+0.005], ...
    [0.22,0.13,0.022],[0.08 0.22 0.48]);
text(robot,-0.20,-0.155,2*wheelR+chassisH+0.035,'ESP32', ...
    'Color','w','FontSize',7,'FontWeight','bold','HorizontalAlignment','center');

% Simulated downward-facing ultrasonic sensor on a small raised bracket
postX=0.20; postY=0;
sensorZ=2*wheelR+chassisH+0.13;
drawCuboid(robot,[postX-0.012,postY-0.012,2*wheelR+chassisH], ...
    [0.024,0.024,0.10],[0.35 0.36 0.38]);
drawCuboid(robot,[postX-0.10,postY-0.045,sensorZ-0.018], ...
    [0.20,0.09,0.025],[0.05 0.24 0.62]);
drawDiscHorizontal(robot,postX-0.04,postY,sensorZ-0.035,0.018,[0.78 0.80 0.83]);
drawDiscHorizontal(robot,postX+0.04,postY,sensorZ-0.035,0.018,[0.78 0.80 0.83]);

% Robot path and start pose.
startXY=[-1.95 -0.95];
robot.Matrix=makehgtform('translate',[startXY(1) startXY(2) 0]);
pathXY=[startXY; binXY; startXY];
plot3(ax,pathXY(:,1),pathXY(:,2),0.015*ones(size(pathXY,1),1), ...
    'k:','LineWidth',1.1);
plot3(ax,startXY(1),startXY(2),0.04,'p','MarkerSize',12, ...
    'MarkerFaceColor',[0.1 0.4 0.9],'MarkerEdgeColor','k');

%% Dashboard with four outputs
uilabel(fig,'Position',[950 755 380 30],'Text','FOUR-BIN FILL-LEVEL REPORT', ...
    'FontSize',17,'FontWeight','bold','FontColor',[0.08 0.18 0.28]);

uilabel(fig,'Position',[950 660 370 24], ...
    'Text','Example simulated waste heights (edit before running):', ...
    'FontSize',11);

fillLabels=gobjects(4,1);
distLabels=gobjects(4,1);
statusLabels=gobjects(4,1);
for k=1:4
    y=575-(k-1)*110;
    uilabel(fig,'Position',[950 y+42 100 26],'Text',binNames{k}, ...
        'FontSize',14,'FontWeight','bold');
    fillLabels(k)=uilabel(fig,'Position',[1050 y+42 250 26], ...
        'Text','Fill: waiting','FontSize',14,'FontWeight','bold');
    distLabels(k)=uilabel(fig,'Position',[950 y+16 350 22], ...
        'Text','Sensor distance: waiting','FontSize',11);
    statusLabels(k)=uilabel(fig,'Position',[950 y-20 350 30], ...
        'Text','Status: NOT CHECKED','FontSize',12,'FontWeight','bold', ...
        'BackgroundColor',[0.90 0.91 0.92], ...
        'HorizontalAlignment','center');
end

currentLabel=uilabel(fig,'Position',[950 155 370 30], ...
    'Text','Robot status: Ready','FontSize',13,'FontWeight','bold');
startBtn=uibutton(fig,'push','Text','Start inspection','Position',[950 698 170 42], ...
    'ButtonPushedFcn',@startInspection);
resetBtn=uibutton(fig,'push','Text','Reset','Position',[1135 698 150 42], ...
    'ButtonPushedFcn',@resetInspection);
uilabel(fig,'Position',[950 115 370 30], ...
    'Text','Sensor data is simulated; no physical ESP32 connection.', ...
    'FontSize',10,'FontColor',[0.35 0.35 0.35]);

%% Save state for callbacks
setappdata(fig,'robot',robot);
setappdata(fig,'binXY',binXY);
setappdata(fig,'wasteHeights',wasteHeights);
setappdata(fig,'binH',binH);
setappdata(fig,'sensorOffset',sensorOffset);
setappdata(fig,'fillLabels',fillLabels);
setappdata(fig,'distLabels',distLabels);
setappdata(fig,'statusLabels',statusLabels);
setappdata(fig,'currentLabel',currentLabel);
setappdata(fig,'startXY',startXY);
setappdata(fig,'pathXY',pathXY);
setappdata(fig,'isRunning',false);
setappdata(fig,'progress',0);
setappdata(fig,'wastePatches',wastePatches);
setappdata(fig,'binL',binL);
setappdata(fig,'binW',binW);
setappdata(fig,'binBaseZ',binBaseZ);
setappdata(fig,'appTimer',[]);
setappdata(fig,'inspectionDone',false);

%% Local functions
function drawCuboid(parent,origin,dims,color)
    x=origin(1); y=origin(2); z=origin(3);
    L=dims(1); W=dims(2); H=dims(3);
    v=[x y z; x+L y z; x+L y+W z; x y+W z; ...
       x y z+H; x+L y z+H; x+L y+W z+H; x y+W z+H];
    f=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
    patch('Parent',parent,'Vertices',v,'Faces',f,'FaceColor',color, ...
        'EdgeColor',[0.18 0.18 0.18],'LineWidth',0.55);
end

function drawBin(ax,cx,cy,L,W,H,z0,name)
    wallT=0.025; baseT=0.025;
    % Floor of bin
    drawCuboid(ax,[cx-L/2,cy-W/2,z0],[L,W,baseT],[0.45 0.48 0.52]);
    % Transparent blue walls; leave top open.
    drawCuboidAlpha(ax,[cx-L/2,cy-W/2,z0+baseT],[L,wallT,H], ...
        [0.08 0.38 0.82],0.35);
    drawCuboidAlpha(ax,[cx-L/2,cy+W/2-wallT,z0+baseT],[L,wallT,H], ...
        [0.08 0.38 0.82],0.35);
    drawCuboidAlpha(ax,[cx-L/2,cy-W/2+wallT,z0+baseT],[wallT,W-2*wallT,H], ...
        [0.08 0.38 0.82],0.35);
    drawCuboidAlpha(ax,[cx+L/2-wallT,cy-W/2+wallT,z0+baseT],[wallT,W-2*wallT,H], ...
        [0.08 0.38 0.82],0.35);
    text(ax,cx,cy-W/2-0.10,z0+H+0.07,name,'FontSize',10, ...
        'FontWeight','bold','HorizontalAlignment','center','Color',[0.08 0.12 0.20]);
end

function drawCuboidAlpha(parent,origin,dims,color,alpha)
    x=origin(1); y=origin(2); z=origin(3);
    L=dims(1); W=dims(2); H=dims(3);
    v=[x y z; x+L y z; x+L y+W z; x y+W z; ...
       x y z+H; x+L y z+H; x+L y+W z+H; x y+W z+H];
    f=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
    patch('Parent',parent,'Vertices',v,'Faces',f,'FaceColor',color, ...
        'FaceAlpha',alpha,'EdgeColor',[0.08 0.20 0.32],'LineWidth',0.8);
end

function drawWheel(parent,x,y,zCenter,r,w,color)
    [xc,yc,zc]=cylinder(r,24);
    zc=(zc-0.5)*w;
    verts=[xc(:)*r+x, zc(:)+y, yc(:)*r+zCenter];
    faces=[];
    n=size(xc,1); m=size(xc,2);
    for row=1:n-1
        for col=1:m-1
            a=(row-1)*m+col; b=a+1; c=a+m+1; d=a+m;
            faces(end+1,:)=[a b c d]; %#ok<AGROW>
        end
    end
    patch('Parent',parent,'Vertices',verts,'Faces',faces, ...
        'FaceColor',color,'EdgeColor',[0.04 0.04 0.04],'LineWidth',0.35);
end

function drawDiscHorizontal(parent,x,y,z,r,color)
    th=linspace(0,2*pi,28);
    patch('Parent',parent,'XData',x+r*cos(th),'YData',y+r*sin(th), ...
        'ZData',z+zeros(size(th)),'FaceColor',color,'EdgeColor',[0.2 0.2 0.2]);
end

function h=drawWaste(ax,cx,cy,L,W,zBase,height,color)
    inset=0.045;
    x0=cx-L/2+inset; x1=cx+L/2-inset;
    y0=cy-W/2+inset; y1=cy+W/2-inset;
    z=zBase+height;
    h=patch(ax,'Vertices',[x0 y0 z; x1 y0 z; x1 y1 z; x0 y1 z], ...
        'Faces',[1 2 3 4],'FaceColor',color,'FaceAlpha',0.95, ...
        'EdgeColor',[0.35 0.22 0.08]);
end

function startInspection(src,~)
    fig=ancestor(src,'figure');
    if isempty(fig)||~isvalid(fig), return; end
    % Stop any old timer.
    oldTimer=getappdata(fig,'appTimer');
    if ~isempty(oldTimer)
        try
            if isvalid(oldTimer) && strcmp(oldTimer.Running,'on'), stop(oldTimer); end
            if isvalid(oldTimer), delete(oldTimer); end
        catch
        end
    end
    setappdata(fig,'isRunning',true);
    setappdata(fig,'progress',0);
    setappdata(fig,'inspectionDone',false);
    currentLabel=getappdata(fig,'currentLabel');
    currentLabel.Text='Robot status: Moving to Bin 1...';
    % reset outputs to pending
    fills=getappdata(fig,'fillLabels'); dists=getappdata(fig,'distLabels');
    stats=getappdata(fig,'statusLabels');
    for k=1:4
        fills(k).Text='Fill: waiting';
        dists(k).Text='Sensor distance: waiting';
        stats(k).Text='Status: NOT CHECKED';
        stats(k).BackgroundColor=[0.90 0.91 0.92];
        stats(k).FontColor=[0.15 0.15 0.15];
    end
    t=timer('ExecutionMode','fixedSpacing','Period',0.04,'BusyMode','drop', ...
        'TimerFcn',@(~,~) inspectionStep(fig));
    setappdata(fig,'appTimer',t);
    start(t);
end

function inspectionStep(fig)
    if isempty(fig)||~isvalid(fig), return; end
    if ~getappdata(fig,'isRunning'), return; end
    progress=getappdata(fig,'progress')+0.0025;
    setappdata(fig,'progress',progress);
    pathXY=getappdata(fig,'pathXY');
    robot=getappdata(fig,'robot');
    binXY=getappdata(fig,'binXY');
    nSegments=size(pathXY,1)-1;
    scaled=progress*nSegments;
    seg=min(floor(scaled)+1,nSegments);
    alpha=min(1,max(0,scaled-(seg-1)));
    pos=(1-alpha)*pathXY(seg,:)+alpha*pathXY(seg+1,:);
    % Face the direction of travel.
    delta=pathXY(seg+1,:)-pathXY(seg,:);
    yaw=atan2(delta(2),delta(1));
    robot.Matrix=makehgtform('translate',[pos(1) pos(2) 0])*makehgtform('zrotate',yaw);

    currentLabel=getappdata(fig,'currentLabel');
    if seg>=1 && seg<=4
        binIndex=seg;
        currentLabel.Text=sprintf('Robot status: Measuring Bin %d with ultrasonic sensor...',binIndex);
        % Mark a bin checked as the robot reaches its inspection point.
        if alpha>0.88
            updateBinOutput(fig,binIndex);
        end
    else
        currentLabel.Text='Robot status: Travelling between bins...';
    end
    drawnow limitrate;

    if progress>=1
        setappdata(fig,'isRunning',false);
        setappdata(fig,'inspectionDone',true);
        currentLabel.Text='Robot status: All 4 bins inspected; returned to start.';
        t=getappdata(fig,'appTimer');
        try
            if isvalid(t), stop(t); delete(t); end
        catch
        end
    end
end

function updateBinOutput(fig,k)
    fills=getappdata(fig,'fillLabels');
    dists=getappdata(fig,'distLabels');
    stats=getappdata(fig,'statusLabels');
    wasteHeights=getappdata(fig,'wasteHeights');
    binH=getappdata(fig,'binH');
    sensorOffset=getappdata(fig,'sensorOffset');
    h=wasteHeights(k);
    % Simulated ultrasonic measurement: sensor is sensorOffset above the bin rim,
    % and the empty depth is binH. In hardware, this distance comes from HC-SR04.
    distance=sensorOffset+(binH-h);
    % Calculate fill percentage from measured distance, not directly from h.
    emptyDepth=max(0,min(binH+sensorOffset,distance));
    fillPct=max(0,min(100,100*(binH+sensorOffset-emptyDepth)/binH));
    fills(k).Text=sprintf('Fill: %.1f%%',fillPct);
    dists(k).Text=sprintf('Sensor distance: %.1f cm',distance*100);
    if fillPct>=90
        stats(k).Text='Status: BIN FULL';
        stats(k).BackgroundColor=[1.00 0.84 0.84];
        stats(k).FontColor=[0.72 0.05 0.05];
    elseif fillPct>=70
        stats(k).Text='Status: ALMOST FULL';
        stats(k).BackgroundColor=[1.00 0.93 0.78];
        stats(k).FontColor=[0.72 0.38 0.02];
    else
        stats(k).Text='Status: NORMAL';
        stats(k).BackgroundColor=[0.86 0.97 0.86];
        stats(k).FontColor=[0.05 0.45 0.15];
    end
end

function resetInspection(src,~)
    fig=ancestor(src,'figure');
    if isempty(fig)||~isvalid(fig), return; end
    t=getappdata(fig,'appTimer');
    try
        if ~isempty(t)&&isvalid(t)
            if strcmp(t.Running,'on'), stop(t); end
            delete(t);
        end
    catch
    end
    setappdata(fig,'appTimer',[]);
    setappdata(fig,'isRunning',false);
    setappdata(fig,'progress',0);
    robot=getappdata(fig,'robot');
    startXY=getappdata(fig,'startXY');
    robot.Matrix=makehgtform('translate',[startXY(1) startXY(2) 0]);
    getappdata(fig,'currentLabel').Text='Robot status: Ready';
    fills=getappdata(fig,'fillLabels'); dists=getappdata(fig,'distLabels');
    stats=getappdata(fig,'statusLabels');
    for k=1:4
        fills(k).Text='Fill: waiting';
        dists(k).Text='Sensor distance: waiting';
        stats(k).Text='Status: NOT CHECKED';
        stats(k).BackgroundColor=[0.90 0.91 0.92];
        stats(k).FontColor=[0.15 0.15 0.15];
    end
end

function closeApp(src,~)
    t=getappdata(src,'appTimer');
    try
        if ~isempty(t)&&isvalid(t)
            if strcmp(t.Running,'on'), stop(t); end
            delete(t);
        end
    catch
    end
    delete(src);
end
