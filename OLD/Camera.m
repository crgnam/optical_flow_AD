classdef Camera < handle
    properties        
        % Input Camera parameters:
        fov_x
        fov_y
        f
        attitude
        resolution
        noise
        
        % Calculate values:
        sensor_x
        sensor_y
        K
        
        % Scene Attributes:
        path
        fig_render
        axes_render
        earth
    end

   %% Constructor
   methods
       function [self] = Camera(f,fov,resolution,attitude,noise,path)
            % FOV given as [fov_x, fov_y]
            % resolution given as [pixels_x, pixels_y]
            
            % Store values:
            self.fov_x = fov(1);
            self.fov_y = fov(2);
            self.f = f;
            self.resolution = resolution;
            self.attitude = attitude;
            self.noise = noise;
            self.path = path;
            
            % Calculate sensor size:
            self.sensor_x = 2*self.f*tand(self.fov_x/2);
            self.sensor_y = 2*self.f*tand(self.fov_y/2);
            self.K = [self.f     0     self.sensor_x/2;
                        0      self.f  self.sensor_y/2;
                        0        0           1       ];
                    
            % MATLAB scene setup:
            self.fig_render = figure();
            set(self.fig_render,'Position',[0,0,self.resolution(1),self.resolution(2)])
            set(self.fig_render,'Visible', 'off');
            self.axes_render = axes(self.fig_render);
            [x,y,z] = sphere(120); Re = 6371000; X = Re*x; Y = Re*y; Z = Re*z;
            EarthTexture = flip(imread('earthmap.jpg'), 1);
            self.earth = surface(self.axes_render, X, Y, Z,EarthTexture,...
                                 'FaceColor','texturemap',...
                                 'EdgeColor','none');
            self.earth.AmbientStrength = 1;
            self.earth.DiffuseStrength = 0;
%             self.earth.SpecularStrength = 0.9;
%             self.earth.SpecularExponent = 100;
%             self.earth.BackFaceLighting = 'unlit';
            width = 14000000; %(m) I have found this to be a good size.  Don't change.
            xlim(self.axes_render,[-width, width])
            ylim(self.axes_render,[-width, width])
            zlim(self.axes_render,[-width, width])
            set(self.axes_render,'xtick',[],'ytick',[],'ztick',[])
            set(self.axes_render,'xticklabel',[],'yticklabel',[],'zticklabel',[])
            set(self.axes_render,'Color','k')
            set(self.fig_render, 'InvertHardcopy', 'off')
            camproj('perspective')
            campos('manual')
            camva('manual')
            camup('manual')
            camtarget('manual')
            axis(self.axes_render,'square')
            axis(self.axes_render,'equal')
            set(self.axes_render, 'Position', [0 0 1 1])
       end
   end
   
   %% New Methods using MATLAB OpenGL Support:
    methods (Access = public)
        function [image] = read(self,sat)
            % Update camera pose in scene:
            campos(self.axes_render, sat.position');
            A_current = q2a(sat.attitude);
            target = .01*A_current(:,3) + sat.position;
            camtarget(self.axes_render, target)
            SensorUp = -A_current(:,2);
            camup(self.axes_render, SensorUp);
            camva(self.axes_render, self.fov_x);
            saveas(self.fig_render,fullfile(self.path,'image.jpg'));
            
            if nargout == 1
                image = imread(fullfile(self.path,'image.jpg'));
            end
        end
    end

    %% Old Camera Raster Methods:
    methods (Access = public)
        % Method to take image:
        function [pixel_points,feature_ids,image] = take_image(self,world_points,feature_ids,sc_attitude,sc_position,c,d_max)

            rotMat = sc_attitude*self.attitude;
            
            % Transform world points into body points:
            body_points_temp = (rotMat'*(world_points' - sc_position))';
            [~,dist] = normr(body_points_temp - sc_position');
            visible_inds = dist < d_max;
            
            body_points = (rotMat*(world_points' + sc_position))';
            body_points = body_points(visible_inds,:);
            feature_ids = feature_ids(visible_inds);
            
            % Project body points into image points:
            [image_points,feature_ids] = self.bp2ip(body_points,feature_ids);
            
            % Convert image points into pixel points:
            pixel_points = self.ip2pp(image_points);
            
            % Create final image:
            if nargout == 3
                image = self.pp2image(pixel_points,c);
            end
        end
        
        % Convert body points to image points:
        function [image_points,feature_ids] = bp2ip(self,body_points,feature_ids)
            % Reject points behind the camera:
            valid = body_points(:,3) > 0;
            feature_ids = feature_ids(valid);
            body_points = body_points(valid,:);

            % Project points into camera :
            body_points = (self.K*body_points')';
            image_points = body_points(:,1:2)./repmat(body_points(:,3),1,2);

            % Flip image:
            image_points = [-image_points(:,1)+self.sensor_x/2, image_points(:,2)-self.sensor_y/2];
            
            % Remove pixels outside of the image:
            invalid = image_points(:,1) >  self.sensor_x/2 | ...
                      image_points(:,1) < -self.sensor_x/2 | ...
                      image_points(:,2) >  self.sensor_y/2 | ...
                      image_points(:,2) < -self.sensor_y/2;
                
            image_points(invalid,:) = [];
            feature_ids(invalid,:) = [];
        end
        
        % Convert image points to pixels:
        function [pixel_points] = ip2pp(self,image_points)
            pixel_points = round([self.resolution(1)*(image_points(:,1)/self.sensor_x + 0.5) + 1,...
                                  self.resolution(2)*(image_points(:,2)/self.sensor_y + 0.5) + 1]);
        end
        
        % Convert pixels to image points:
        function [image_points] = pp2ip(self,pixel_points)
            
        end
        
        % Convert pixels to image:
        function [image] = pp2image(self,pixel_points,c)
            image = zeros(self.resolution(2),self.resolution(1),3);
            for ii = 1:size(pixel_points,1)
                image(pixel_points(ii,2),pixel_points(ii,1),:) = c;
            end
        end
    end
end