 function [x,y,z,endpoints,rotations_in_frame,rotations_in_world,q] = body_construct(a1,a2,s,ds,v,t)
    %%% Function to construct the body based on the alpha 1 and 2 values
    %
    % Farhan Rozaidi
    %
    % May 11, 2021
 
    % Set up equations for curvature, bicurvature, and torsion
    curvature = a1*cos(s+t)-a2*sin(s+t);
    bicurvature = a1*sin(s+t)+a2*cos(s+t);
    torsion = 0;
    
    % Set up identity matrix, orientation, and rotations matrices
    identity_mat = eye(3);
    q = cell(size(s));
    q{1} = v;
    rotations_in_frame = cell(size(s));
    rotations_in_frame{1} = identity_mat;
    rotations_in_world = rotations_in_frame;
    
    for i = 2:length(s)
        
        % Place curvature, bicurvature, and torsion into flow vector(?)
        % TODO: Ask Ross about the actual term for omega
        omega = [torsion,curvature(i), bicurvature(i)];
        
        % Set up skew symmetric matrix to exponentiate along the body
        skew_symm_mat = [0, -omega(3), omega(2);
        omega(3), 0, -omega(1);
        -omega(2), omega(1), 0];
    
        % Exponentiate the matrix along the body
        rotations_in_frame{i} = expm(skew_symm_mat*ds);
        
        % Left multiply the rotation matrix in frame by the world frame
        % TODO: Ask Ross about the proper way to say this
        rotations_in_world{i} = rotations_in_world{i-1}*rotations_in_frame{i};
        
    end
    
    % Multiply the rotation matrices by the flow vector to get the proper
    % length
    for i =2:length(s)
            q{i} = rotations_in_world{i}*v;
    end
    
    % Set up endpoints variable to store the endpoints in a single matrix
    endpoints = zeros(3,size(q,2));
    endpoints(1:3,1) = q{1}(1:3,1);
    
    % Loop through the q values of each link to get the endpoints in the
    % world frame
    for i = 2:length(q)
        endpoints(1:3,i) = q{i}(1:3,1)+endpoints(1:3,i-1);
    end
    
    % Separate out the x, y, and z values (not fully necessary, but makes
    % it easier to debug)
    x = [0,endpoints(1,:)].';
    y = [0,endpoints(2,:)].';
    z = [0,endpoints(3,:)].';
    
end