function [endpoints,rotations_in_world,x,y,z,q] = body_orient(rotations_in_frame,v,q_init)
    %%% Function to reorient a body given rotations and an orientation
    %%% matrix
    %
    % Farhan Rozaidi
    %
    % May 11, 2021

    rotation_init = q_init(1:3,1:3);
    v_init = q_init(1:3,4);
    q = cell(size(rotations_in_frame));
    q(1) = {[rotation_init, v_init;
        0,0,0,1]};
    
    for i = 2:numel(rotations_in_frame)
        q{i} = q{i-1}*[rotations_in_frame{i}, v;
                0,0,0,1];
    end
    
    endpoints = zeros(3,size(q,2));
    rotations_in_world = cell(size(q));
    for i = 1:length(q)
        endpoints(1:3,i) = q{i}(1:3,4);
        rotations_in_world{i} = q{i}(1:3,1:3)/norm(q{i}(1:3,1:3));
    end

    x = [endpoints(1,:)].';
    y = [endpoints(2,:)].';
    z = [endpoints(3,:)].';
end