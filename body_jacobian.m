function [g_circ_tail,q_g_circ,pos_pfaffian,shape_pfaffian] = body_jacobian(ground_contact_idx,rotations_in_frame,endpoints, s, dt,da1,da2,q_rotation)
    %%% Function to calculate the jacobian values, and determine movement
    %%% of the snake body
    %
    % Farhan Rozaidi
    %
    % May 11, 2021
    
    % NOTE: all of these calculations are performed in the TAIL frame,
    % where the frame of the tail joint is the identity.
    
    % Calculate J_alpha based on problem setup, purely calculus here
    J_alpha = zeros(numel(s)*2,2);
    for i = 1:numel(s)
        J_alpha((i*2)-1:(i*2),:) = [cos(s(i)) -sin(s(i));
                                sin(s(i)) cos(s(i))];
    end
    
    % Set up empty cells to add the jacobians
    J_p_alpha = cell(size(ground_contact_idx));
    J_p_a = cell(size(ground_contact_idx));
    J_p_g = cell(size(ground_contact_idx));
    v_diff = cell(size(ground_contact_idx));
    
    % Create a loop to calculate the jacobian for the number of ground
    % contacts present
    for i = 1:numel(ground_contact_idx)
        
        % For each ground contact, we obtain the vector from the tail to
        % the contact, as well as the x, y, and z values of each contact
        % point
        v_diff{i} = zeros(size(endpoints));
        x = endpoints(1,ground_contact_idx(i));
        y = endpoints(2,ground_contact_idx(i));
        z = endpoints(3,ground_contact_idx(i));

        % Calculate the vector from the tail to the contact point
        for i2 = 1:numel(rotations_in_frame)
            v_diff{i}(:,i2) = endpoints(:,ground_contact_idx(i))-endpoints(:,i2);
        end
        
        % Determine the J_p_g element based on the location of the ground
        % contact
        % TODO: need to fully understand what this means
        J_p_g{i} = [1 0 0 0 z -y;
                    0 1 0 -z 0 x;
                    0 0 1 y -x 0];
        

        % Set up the cell array for the J_p_alpha calculation
        J_p_alpha{i} = zeros(3,numel(rotations_in_frame)*2);
        
        % Calculate J_p_alpa through the cross product of omega and r for
        % each axis (pitch and yaw)
        for i2 = 1:ground_contact_idx(i)
            J_p_alpha{i}(:,(i2*2)-1:(i2*2)) = [cross(rotations_in_frame{i2}(:,2),v_diff{i}(:,i2)),...
                                    cross(rotations_in_frame{i2}(:,3),v_diff{i}(:,i2))];
        end
        
        % Multiply J_p_alpha by J_alpha to get J_p_a
        J_p_a{i} = J_p_alpha{i}*J_alpha;
        
    end
    
    % Vertically stack J_p_a and J_p_g for g_circ calculation
    J_p_a_stacked = vertcat(J_p_a{:});
    J_p_g_stacked = vertcat(J_p_g{:});
    
    % Set up a dot vector from the da1 and da2 variables
    a_dot = [da1;da2];
    
    % Calculate g_circ
%     g_circ_tail = -lsqminnorm(J_p_g_stacked,J_p_a_stacked)*a_dot;
    g_circ_tail = [1;0;0;0;0;0];
    
    % From g_circ (Lie algebra element) convert to q_g_circ (SE3 element)
    % Sort out the rotational components for more readable code
    
    rotation = [0 g_circ_tail(6) g_circ_tail(5);
                g_circ_tail(6) 0 -g_circ_tail(4);
                -g_circ_tail(5) g_circ_tail(4) 0];
    
    %Combine the rotational component with the translational component
    
    q_g_circ = [rotation, [g_circ_tail(1);g_circ_tail(2);g_circ_tail(3)];
                0 0 0 0];
    
    % Matrix exponential converts the Lie algebra element into an SE3
    % element
    q_g_circ = expm(q_g_circ*dt);
    
    % Normalize the rotational component of the SE3 element
    % TODO: determine if this is necessary???
    q_g_circ(1:3,1:3) = q_rotation*q_g_circ(1:3,1:3)/norm(q_g_circ(1:3,1:3));
    
    % Obtain the position component of the constraint pfaffian
    pos_pfaffian = q_rotation*reshape(J_p_g_stacked*g_circ_tail,3,[]);
    
    % Obtain the shape component of the constraint pfaffian
    shape_pfaffian = q_rotation*reshape(J_p_a_stacked*a_dot,3,[]);

end