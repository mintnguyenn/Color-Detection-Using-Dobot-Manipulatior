classdef plotObject_byTon < handle 
    
    properties 
    vData; 
    pData; 
    fData;s
    v1;
    v2;
    v3;
    faceNormals; 
    VertexCount;
    CenterPoint;
    MidVerticiesPoint;
    Pose;
    VertexColours;
    mesh_h;
    translation;
    rotation;
    plyname;
    UpdatePoint;
    end
    
    methods
%% get the table 
function self = plotObject(translation,rotation,plyname)
    self.translation = translation;
    self.rotation = rotation; 
    self.plyname = plyname;
    [self.fData,self.vData,self.pData] = plyread(plyname,'tri');
    self.GetMidVerticies(); 
    self.GetTableModel();
    self.MoveTable(self.translation,self.rotation);
end
%% get the mid verticies
function GetMidVerticies(self)
    self.VertexCount = size(self.vData,1);
    self.CenterPoint = sum(self.vData)/self.VertexCount;
    self.Pose = eye(4);
    self.MidVerticiesPoint = self.vData - repmat(self.CenterPoint,self.VertexCount,1);
end 
%% get the model        
function GetTableModel(self) 
    self.VertexColours = [self.pData.vertex.red,self.pData.vertex.green,self.pData.vertex.blue] / 255; 
    self.mesh_h = trisurf(self.fData,self.MidVerticiesPoint(:,1),self.MidVerticiesPoint(:,2),self.MidVerticiesPoint(:,3)...
        ,'FaceVertexCData',self.VertexColours,'EdgeColor','interp','EdgeLighting','flat');
end
%% move the table 
function MoveTable(self,trans,rot)
    self.Pose = self.Pose*trans*rot; 
    self.UpdatePoint = [self.Pose * [self.MidVerticiesPoint,ones(self.VertexCount,1)]']' ;  
    self.mesh_h.Vertices = self.UpdatePoint(:,1:3);
%     self.faceNormals = zeros(size(self.fData,1),3);
%     for faceIndex = 1:size(self.fData,1)  
%         self.v1 = self.vData(self.fData(faceIndex,1)',:);
%         self.v2 = self.vData(self.fData(faceIndex,2)',:);
%         self.v3 = self.vData(self.fData(faceIndex,3)',:);
%         self.faceNormals(faceIndex,:) = unit(cross(self.v2-self.v1,self.v3-self.v1));
%     end
end
    end
end
