function [image_points] = projectPoints(body_points, K, image_size)
    %reject points behind camera
    valid = body_points(:,3) > 0;
    body_points = body_points(valid,:);

    % Project points into camera 
    body_points = (K*body_points')';
    image_points = body_points(:,1:2)./repmat(body_points(:,3),1,2);

    % Remove points outside of image:
%     if(~isempty(image_size))
%         inside = camera_points(:,1) < image_size(2);
%         inside = and(inside, camera_points(:,2) < image_size(1));
%         inside = and(inside, camera_points(:,1) >= 0);
%         inside = and(inside, camera_points(:,2) >= 0);
%         valid(valid) = inside;
%     end
%     image_points = camera_points(valid,:);
    image_points = [-image_points(:,1)+image_size(1), -image_points(:,2)+image_size(2)];
end