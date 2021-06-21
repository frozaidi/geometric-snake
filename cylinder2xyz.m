function [x_surf,y_surf,z_surf] = cylinder2xyz(rotated_cylinder_in_world)

    x_surf = zeros(numel(rotated_cylinder_in_world)-1,7);
    y_surf = zeros(numel(rotated_cylinder_in_world)-1,7);
    z_surf = zeros(numel(rotated_cylinder_in_world)-1,7);

    for i = 1:numel(rotated_cylinder_in_world)-1
        x_surf(i,1:7) = rotated_cylinder_in_world{i}{1}(1,:);
        y_surf(i,1:7) = rotated_cylinder_in_world{i}{2}(1,:);
        z_surf(i,1:7) = rotated_cylinder_in_world{i}{3}(1,:);
    end

end