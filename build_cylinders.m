function cylinder_in_world = build_cylinders(endpoints,rotations)

    rotated_cylinder = cell(size(rotations));
    endpoints_with_base = [endpoints];

    for i = 1:numel(rotations)-1
        [Z,Y,X] = cylinder(1,6);
        link_vector = endpoints_with_base(1:3,i+1)-endpoints_with_base(1:3,i);
        link_length = norm(link_vector);
        link = {X*link_length,Y*.1,Z*.1};
        B = rotations{i};
        rotated_cylinder{i} = link;
        for i2 = 1:2
            surf_rotate = B*[link{1}(i2,:);
                                link{2}(i2,:);
                                link{3}(i2,:)];
            for i3 = 1:numel(link)
                rotated_cylinder{i}{i3}(i2,:) = surf_rotate(i3,:);
            end
        end
    end
    cylinder_in_world = rotated_cylinder;
    for idx = 1:numel(rotated_cylinder)
        for idx2 = 1:numel(rotated_cylinder{idx})
            cylinder_in_world{idx}{idx2} = ...
                cylinder_in_world{idx}{idx2}+...
                endpoints_with_base(idx2,idx);
        end
    end
end