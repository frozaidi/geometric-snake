%%% Main script to run the snake modeling code
%
% Farhan Rozaidi
%
% May 11, 2021

%%% Set up input variables
% TODO: Find better phrasing for each variable comment
clear;clc;
theta = linspace(0,6*pi,400);       % Vector of times?
dt = diff(theta);                   % Time step
a1 = 1*sin(theta);                  % Actuator 1
a2 = 1*sin(theta);                  % Actuator 2
da1 = -sin(theta)*dt(1);            % Actuator 1 time derivative
da2 = cos(theta)*dt(1);             % Actuator 2 time derivative
ds = 0.1;                           % Segment length of body
s = 0:ds:3*pi;                      % Length of body
v = [ds;0;0];                       % Flow vector through body
v_init = v;                         % Set a static flow vector
t = 0.0;                            % Time (Should probably be removed)

%%% Set up empty cells for storage at each time step
% x, y, and z values at each time, leveled with the x-axis
x_leveled_time = cell(size(a2));
y_leveled_time = cell(size(a2));
z_leveled_time = cell(size(a2));

% x, y, and z surface values to plot cylindrical body
x_surf_time = cell(size(a2));
y_surf_time = cell(size(a2));
z_surf_time = cell(size(a2));

q_movement = cell(size(a2));        % Orientation of the tail at each time step
g_circ = cell(size(a2));            % Velocity of tail at each time step

% TODO: Figure out a better way to name these variables
pos_pfaffian = cell(size(a2));      % Product of J_p_g*g_circ in world frame
shape_pfaffian = cell(size(a2));    % Product of J_p_a*a_dot in world frame
shape_vec = cell(size(a2));         % Vector based on ground contact and "shape_pfaffian"
pos_vec = cell(size(a2));           % Vector based on ground contact and "pos_pfaffian"
residual_vec = cell(size(a2));      % Sum of "pos_pfaffian" and "shape_pfaffian", showing the "slip" of ground contacts

for i = 1
    
    % Construct the initial body, obtain rotation matrices and orientation
    [x,y,z,endpoints,rotations_in_frame,...
        rotations_in_world,q] = body_construct(a1(i),a2(i),s,ds,v,t);
    
    % Level the body to x-axis, obtain orientation from this transformation
    [x_leveled_time{i},y_leveled_time{i},z_leveled_time{i},...
        endpoints_leveled,rotations_leveled_in_frame,...
        rotations_leveled_in_world,...
        ground_contact_idx,q_init] = body_level(rotations_in_frame,v,v_init);
    
    % Store the initial orientation as the first cell of the orientations
    q_movement{i} = q_init;
    
    % Display iteration to keep track of progress
    disp(i);
    
end

% Run loops to the end of the time vector
for i=2:numel(a1)
    
    % Calculate how the tail moves for the next time step
    [g_circ{i},q_movement{i},pos_pfaffian{i-1},...
        shape_pfaffian{i-1}] = body_jacobian(ground_contact_idx,...
        rotations_in_world, endpoints, s,dt(i-1), da1(i-1), da2(i-1),...
        q_movement{i-1}(1:3,1:3));
    
    % Shift this orientation into the world frame
    % Note: this might not be correct, still trying to figure it out
    % At some point the transformation into the world frame might mess
    % up the actual calculation
    q_movement{i} = q_movement{i-1}*q_movement{i};
%     q_movement{i} = q_movement{1};
                        
    
    % Calculate the vectors to plot on the ground contacts
    [shape_vec{i-1},pos_vec{i-1},...
        residual_vec{i-1}] = jacobian_vectors(endpoints_leveled,...
        ground_contact_idx,pos_pfaffian{i-1},shape_pfaffian{i-1});
    
    % Construct the body for the next time step
    [~,~,~,endpoints,rotations_in_frame,rotations_in_world,...
        ~] = body_construct(a1(1),a2(1),s,ds,v,t);
    
    % Reorient the body to the x-axis using the previous orientation
    [endpoints_leveled,~,x_leveled_time{i},y_leveled_time{i},...
        z_leveled_time{i},~] = body_orient(rotations_in_frame,v,...
        q_movement{i});
    
    % Find the next set of ground contacts on the body
%     ground_contact_idx = find(islocalmin(z_leveled_time{i}));
    
    % Display the iteration to monitor progress
%     disp(i)
    
end

% Create a plot of the snake body
% NOTE: snake_plotter still a work in progress
% ax = snake_plotter(endpoints_leveled,shape_vec{1},pos_vec{1},residual_vec{1});

record_anim = input("Record animation? (y/n) ",'s');
if record_anim == "y"
    filename = input("Enter filename: ",'s');
    path = cd;
    path = fullfile(path,'animations',filename);
    myVideo = VideoWriter(path,'MPEG-4'); %open video file
    myVideo.FrameRate = 30;  %can adjust this, 5 - 10 works well for me
    myVideo.Quality = 100;
    open(myVideo)
end


ax = create_axes(1);
view(ax,3);
l = line(x_leveled_time{1},y_leveled_time{1},z_leveled_time{1});
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal
xlim([min(min(cell2mat(x_leveled_time))) max(max(cell2mat(x_leveled_time)))])
ylim([min(min(cell2mat(y_leveled_time))) max(max(cell2mat(y_leveled_time)))])
zlim([min(min(cell2mat(z_leveled_time))) max(max(cell2mat(z_leveled_time)))])
for i = 1:length(find(~cellfun(@isempty,z_leveled_time')))
    set(l,'Xdata',x_leveled_time{i},'Ydata',y_leveled_time{i},...
    'Zdata',z_leveled_time{i});
    drawnow
    if record_anim == "y"
        frame = getframe(gcf);
        writeVideo(myVideo,frame);
    end
end

if record_anim == "y"
    close(myVideo)
end