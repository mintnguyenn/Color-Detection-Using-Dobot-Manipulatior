function enableTheEnvironment()
    axis equal
%% Table
%     [f,v,data] = plyread('table.ply', 'tri');
% 
%     % Scale the colours to be 0-to-1
%     vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% 
%     hold on;
%     xOffset = 0;
%     yOffset = 0;
%     zOffset = 0;
%     % Then plot the trisurf with offset verticies
%     table = trisurf(f,v(:,1) + xOffset, v(:,2) + yOffset, v(:,3) + zOffset ...
%         ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
% 
%     hold on;
%% Fence
    [f,v,data] = plyread('fence2.ply', 'tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    for xOffset = -1.6
        for yOffset = [-1.6, 1.5]
            % Then plot the trisurf with offset verticies
            trisurf(f,v(:,1)+ xOffset,v(:,2) + yOffset, v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end

    hold on;
%% Fence     
    [f,v,data] = plyread('fence.ply', 'tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    for xOffset = 0
        for yOffset = [-3, 3]
            % Then plot the trisurf with offset verticies
            trisurf(f,v(:,1)+ xOffset,v(:,2) + yOffset, v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
    end
    
    hold on;
%% Button
%     [f,v,data] = plyread('button.ply', 'tri');
% 
%     % Scale the colours to be 0-to-1
%     vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% 
%     hold on;
%     xOffset = 0;
%     yOffset = -1.2;
%     zOffset = 0.81;
%     % Then plot the trisurf with offset verticies
%     table = trisurf(f,v(:,1) + xOffset, v(:,2) + yOffset, v(:,3) + zOffset ...
%         ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
% 
%     hold on;
%% Fire extinguish
    [f,v,data] = plyread('fire.ply', 'tri');

    % Scale the colours to be 0-to-1
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

    hold on;
    xOffset = -1.4;
    yOffset = -2.8;
    zOffset = 0;
    % Then plot the trisurf with offset verticies
    table = trisurf(f,v(:,1) + xOffset, v(:,2) + yOffset, v(:,3) + zOffset ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
end