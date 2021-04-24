classdef Dobot < handle
    properties
        %> Robot model
        model;
    
        %>
        workspace = [-2 2 -2 2 -0.3 2];
    end
    
    methods
        function self = Dobot()    
            %> Define the boundaries of the workspace

            % robot = 
            self.GetDobot();
            self.PlotAndColourDobot();

        end

        %% GetDobot
        % Given a name (optional), create and return a Dobot model
        function GetDobot(self)
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = ['Dobot',datestr(now,'yyyymmddTHHMMSSFFF')];

            % Create the UR3 model mounted on a linear rail
            L(1) = Link('d',0.138,'a',0,     'alpha',-pi/2, 'offset',0,     'qlim',[deg2rad(-135),deg2rad(135)]);
            L(2) = Link('d',0,     'a',0.135, 'alpha',0,     'offset',-pi/2, 'qlim',[deg2rad(5)   ,deg2rad(80)]);
            L(3) = Link('d',0,     'a',0.147, 'alpha',0,     'offset',0,     'qlim',[deg2rad(15)  ,deg2rad(170)]);
            L(4) = Link('d',0,     'a',0.078, 'alpha',pi/2,  'offset',-pi/2, 'qlim',[deg2rad(-90) ,deg2rad(90)]);
            L(5) = Link('d',0,     'a',0,     'alpha',0,     'offset',0,     'qlim',[deg2rad(-85) ,deg2rad(85)]);
 
            self.model = SerialLink(L,'name',name);

            % Rotate robot to the correct orientation
            self.model.base = eye(4);
        end
        
        %% PlotAndColourRobot
        function PlotAndColourDobot(self)%robot,workspace
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['Dobot',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
            
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            % Display robot
            self.model.plot3d([0    0.5700    1.1800    1.3916         0],'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
    end
end