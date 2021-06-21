function [x_rot,y_rot,z_rot,endpoints_rotated,rot_vec,rotations_in_world] = body_rotate(rotation_vec,rotations_in_frame,v,v_init,alpha,axis)
    %%% Function to rotate a body given an angle "alpha" and rotation axis
    %
    % Farhan Rozaidi
    %
    % May 11, 2021


    % Initialize a cell array for storing set of orientations through body
    q_movement = cell(size(rotations_in_frame));
    
    % Determine which axis to rotate around and create rotation matrix
    % based on angle "alpha" given
    % Admittedly this might be a "hacky" way to do this, but it works
    if axis == 'y'
        rot_vec = [cos(alpha) 0 -sin(alpha); 0 1 0; sin(alpha) 0 cos(alpha)]*rotation_vec;
    elseif axis == 'z'
        rot_vec = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1]*rotation_vec;
    end
    
    % Initialize a flow vector based on the rotation matrix
    v_rot = rot_vec*v_init;
    
    % Use this initialized flow vector and rotation matrix as the first
    % index of the set of orientations
    q_movement(1) = {[rot_vec, v_rot;
        0,0,0,1]};
    
    % Loop through the entire body to fill in the rest of the orientations
    for i = 2:numel(rotations_in_frame)
        q_movement{i} = q_movement{i-1}*[rotations_in_frame{i}, v;
                0,0,0,1];
    end
    
    % Create and store the rotated endpoints and rotation matrices
    endpoints_rotated = zeros(3,size(q_movement,2));
    rotations_in_world = cell(size(q_movement));
    for i = 1:length(q_movement)
        endpoints_rotated(1:3,i) = q_movement{i}(1:3,4);
        rotations_in_world{i} = q_movement{i}(1:3,1:3)/norm(q_movement{i}(1:3,1:3));
    end
    
    % Output x, y, and z arrays for debugging purposes (mostly)
    x_rot = [0,endpoints_rotated(1,:)].';
    y_rot = [0,endpoints_rotated(2,:)].';
    z_rot = [0,endpoints_rotated(3,:)].';
    
end