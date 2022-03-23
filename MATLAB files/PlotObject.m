classdef PlotObject < handle
    properties
        model;
        model_h;
        modelPose;
        modelVerts;
        modelVertexCount;
        updatePoints;
    end
    
    methods
%% Class constructor
        function self = PlotObject(FileName, modelLocation)            
            self.Plot(FileName, modelLocation);
            
            self.modelPose = transl(modelLocation(1), modelLocation(2), modelLocation(3));
        end
%% 
        function Plot(self, FileName, modelLocation)
            
            self.modelPose = transl(modelLocation(1), modelLocation(2), modelLocation(3));
            % Read the PLY file
            A = string(FileName);            
            [faceData, vertexData, plyData] = plyread(A,'tri');
            
            % Get vertex count
            self.modelVertexCount = size(vertexData, 1);
            
            % Move center point to origin
            midPoint    = sum(vertexData)/self.modelVertexCount;
            self.modelVerts  = vertexData - repmat(midPoint, self.modelVertexCount, 1);
            
            % Scale the colors to be [0,1]
            vertexColors = [plyData.vertex.red, plyData.vertex.green, plyData.vertex.blue] / 255;
            
            % Plot the trisurf
            self.model_h = trisurf(faceData, self.modelVerts(:,1), self.modelVerts(:,2), self.modelVerts(:,3), 'FaceVertexCData', vertexColors, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            
            % Create a transform for model pose
%             self.modelPose = eye(4) * transl(0,0,-5);
            updatePoints = [self.modelPose * [self.modelVerts, ones(self.modelVertexCount, 1)]']';
            
            % Update the vertices
            self.model_h.Vertices = updatePoints(:,1:3);
        end
        %%
        function Move(self, newPose)
            self.modelPose = newPose;
            self.updatePoints = [self.modelPose * [self.modelVerts, ones(self.modelVertexCount, 1)]']';
    
            self.model_h.Vertices = self.updatePoints(:,1:3);
            drawnow();
            
        end
    end
end