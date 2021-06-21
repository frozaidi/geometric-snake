function [x_leveled,y_leveled,z_leveled,endpoints_leveled,rotations_leveled_in_frame,rotations_leveled_in_world,ground_contact_idx,q_init] = body_level(rotations_in_frame,v,v_init)
    %%% Function to level the body to the x-axis using a form of Newton's
    %%% method
    %
    % Farhan Rozaidi
    %
    % May 11, 2021

    % Set up matrices, tolerances, flags, and limits
    identity_vec = [1 0 0; 0 1 0; 0 0 1];
    percent_error = 0.1;
    error = 0.0001;
    leveled_y = 0;
    leveled_z = 0;
    limit = 100;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% INITIAL SET OF ROTATIONS %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calculate the mean of all the endpoints, and determine the angle of
    % this vector with respect to the x-axis
    [~,~,~,endpoints,~,~] = body_rotate(identity_vec,rotations_in_frame,v,v_init,0,'y');
    mean_vector = mean(endpoints.').';
    theta_z = -atan(mean_vector(2)/mean_vector(1)); %angle of mean vector around the z axis
    theta_y = -atan(mean_vector(3)/mean_vector(1)); %angle of mean vector around the y axis

    
    % Rotate body given the angle of the mean vector around the y and z axes
    [~,~,~,~,rot_mean_y,~] = body_rotate(identity_vec,rotations_in_frame,v,v_init,(theta_y),'y');
    [x_rot,y_rot_z,z_rot_y,endpoints,rot_mean,rot_in_world] = body_rotate(rot_mean_y,rotations_in_frame,v,v_init,(theta_z),'z');
    
    % Using the rotated body, find the points on the body lowest to the
    % "ground"
    local_min_z = islocalmin(z_rot_y);
    
    % From the lowest points, find the difference bewteen the lowest and
    % highest values, and obtain its percent difference (z values)
    min_val_z = z_rot_y(local_min_z);
    diff_val_z = abs(max(min_val_z)-min(min_val_z));
    percent_diff_z = abs((max(min_val_z)-min(min_val_z))/max(min_val_z)*100);
    
    % In cases where the body is already aligned
    % TODO: Uhh actually have to look back at this, forgot my train of
    % thought here
    if diff_val_z == 0
        local_min_z = islocalmax(z_rot_y);
        min_val_z = z_rot_y(local_min_z);
        diff_val_z = abs(max(min_val_z)-min(min_val_z));
        percent_diff_z = abs((max(min_val_z)-min(min_val_z))/max(min_val_z)*100);
    end
    % In cases where there are more than 1 local minimum, then check to see
    % if they are lower than the tolerances, and set the flag to true if so
    if (numel(min_val_z)>1)
        if (diff_val_z < error) || percent_diff_z < percent_error
            leveled_z = 1;
        end
    end
    
    % From the lowest points, find the difference bewteen the lowest and
    % highest values, and obtain its percent difference (y values)
    local_min_y = islocalmin(y_rot_z);
    min_val_y = y_rot_z(local_min_y);
    diff_val_y = abs(max(min_val_y)-min(min_val_y));
    percent_diff_y = abs((max(min_val_y)-min(min_val_y))/max(min_val_y)*100);
    
    % TODO: Uhh actually have to look back at this, forgot my train of
    % thought here
    if diff_val_y == 0
            local_min_y = islocalmax(y_rot_z);
            min_val_y = y_rot_z(local_min_y);
            diff_val_y = abs(max(min_val_y)-min(min_val_y));
            percent_diff_y = abs((max(min_val_y)-min(min_val_y))/max(min_val_y)*100);
    end
        
    % In cases where there are more than 1 local minimum, then check to see
    % if they are lower than the tolerances, and set the flag to true if so
    if (numel(min_val_y)>1)
        if (diff_val_y < error) || percent_diff_y < percent_error
            leveled_y = 1;
        end
    end
    
    
    % Edge case: where the body is already aligned to the x-axis, set the
    % necessary variables, and return function
    if theta_z == 0 && theta_y == 0
        x_leveled = x_rot;
        y_leveled = y_rot_z;
        z_leveled = z_rot_y;
        fprintf('Diff z: %1.4e\n',0);
        fprintf('Diff y: %1.4e\n',0);
        endpoints_leveled = endpoints;
        rotations_leveled_in_frame = rot_mean;
        rotations_leveled_in_world = rot_in_world;
        ground_contact_idx = [];
        return
    end   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% SETUP FOR ROTATIONS %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Set up starting rotations for each axis
    x_min_z = x_rot(local_min_z);
    x_min_y = x_rot(local_min_y);
    theta_y = -atan((min_val_z(end)-min_val_z(1))/(x_min_z(end)-x_min_z(1)));
    theta_z = -atan((min_val_y(end)-min_val_y(1))/(x_min_y(end)-x_min_y(1)));
    
    % Set up number of iterations run through each axis rotation
    count_z = 0;
    count_y = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% ROTATION AROUND THE Y-AXIS %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    while leveled_z == 0 && count_z <limit
        
        % Rotate the body around the y-axis with the given "theta_y" value
        [x_rot,y_rot_z,z_rot_y,endpoints,rot_mean,rot_in_world] = body_rotate(rot_mean,rotations_in_frame,v,v_init,(theta_y),'y');
        
        % Perform ground contact checks for z values
        local_min_z = islocalmin(z_rot_y);
        min_val_z = z_rot_y(local_min_z);
        diff_val_z = abs(max(min_val_z)-min(min_val_z));
        percent_diff_z = abs((max(min_val_z)-min(min_val_z))/max(min_val_z)*100);
        
        % Checks if the body is level with the x-axis
        if diff_val_z == 0
            local_min_z = islocalmax(z_rot_y);
            min_val_z = z_rot_y(local_min_z);
            diff_val_z = abs(max(min_val_z)-min(min_val_z));
            percent_diff_z = abs((max(min_val_z)-min(min_val_z))/max(min_val_z)*100);
        end
        if (numel(min_val_z)>1)
            if (diff_val_z < error) || percent_diff_z < percent_error
                leveled_z = 1;
                break
            end
        end
        
        % If not yet leveled, then get a new angle, with the new angle
        % being based upon the contact points and its angle with respect to
        % the x-axis
        x_min_z = x_rot(local_min_z);
        theta_y = -atan((min_val_z(end)-min_val_z(1))/(x_min_z(end)-x_min_z(1)));
        count_z = count_z+1;
        
        % If the loop has gone longer than the specified limit, then break
        if count_z == limit
            break
        end
    end
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% ROTATION AROUND THE Z-AXIS %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    while leveled_y == 0 && count_y <limit
        
         % Rotate the body around the y-axis with the given "theta_z" value
        [x_rot,y_rot_z,z_rot_y,endpoints,rot_mean,rot_in_world] = body_rotate(rot_mean,rotations_in_frame,v,v_init,(theta_z),'z');
        
        % Perform ground contact checks for y values
        local_min_y = islocalmin(y_rot_z);
        min_val_y = y_rot_z(local_min_y);
        diff_val_y = abs(max(min_val_y)-min(min_val_y));
        percent_diff_y = abs((max(min_val_y)-min(min_val_y))/max(min_val_y)*100);
        
        % Checks if the body is level with the x-axis
        if diff_val_y == 0
            local_min_y = islocalmax(y_rot_z);
            min_val_y = y_rot_z(local_min_y);
            diff_val_y = abs(max(min_val_y)-min(min_val_y));
            percent_diff_y = abs((max(min_val_y)-min(min_val_y))/max(min_val_y)*100);
        end
        if (numel(min_val_y)>1)
            if (diff_val_y < error) || percent_diff_y < percent_error
                leveled_y = 1;
                break
            end
        end
        
        % If not yet leveled, then get a new angle, with the new angle
        % being based upon the contact points and its angle with respect to
        % the x-axis
        x_min_y = x_rot(local_min_y);
        theta_z = -atan((min_val_y(end)-min_val_y(1))/(x_min_y(end)-x_min_y(1)));
        count_y = count_y+1;
        
        % If the loop has gone longer than the specified limit, then break
        if count_y == limit
            break
        end
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% FINAL VARIABLE ASSIGNMENTS %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

    % Console output to see how level the body is
    fprintf('Diff z: %1.4e\n',diff_val_z);
    fprintf('Diff y: %1.4e\n',diff_val_y);
    
    % Set the leveled roation matrices to the mean values obtained above
    rotations_leveled_in_frame = rot_mean;

    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% TRANSLATION INTO (0,0,0) %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

    % An additional step to translate the entire body such that no
    % endpoints are "clipping into the ground" i.e. no values that are -y
    % or -z
    v_leveled = -[0;-endpoints(2,1)+min(min_val_y)-.1;-endpoints(3,1)+min(min_val_z)-.1];
    q_rot(1) = {[rot_mean, v_leveled;
    0,0,0,1]};
    for i = 2:numel(rotations_in_frame)
        q_rot{i} = q_rot{i-1}*[rotations_in_frame{i}, v;
        0,0,0,1];
    end
    
    % From this calculation, obtain the leveled endpoints and rotations
    endpoints_leveled = zeros(3,size(q_rot,2));
    rotations_leveled_in_world = cell(size(q_rot));
    for i = 1:length(q_rot)
        endpoints_leveled(1:3,i) = q_rot{i}(1:3,4);
        rotations_leveled_in_world{i} = q_rot{i}(1:3,1:3)/norm(q_rot{i}(1:3,1:3));
    end
    
    % Separate out x, y, and z values for debugging and ease of use for
    % plots
    x_leveled = [endpoints_leveled(1,:)].';
    y_leveled = [endpoints_leveled(2,:)].';
    z_leveled = [endpoints_leveled(3,:)].';
    
    % Obtain the orientation value for this leveled body
    q_init = q_rot{1};
    
    % Obtain the final indices for the points that contact the ground
    ground_contact_idx = find(islocalmin(z_leveled));

end