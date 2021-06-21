function ax = snake_plotter(endpoints,right_vec,left_vec,residual_vec)
    %%% Function to plot the "jacobian vectors" on the snake body
    %
    % Farhan Rozaidi
    %
    % May 11, 2021

    ax = create_axes(1);
    view(ax,3);
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    l = line(endpoints(1,:),endpoints(2,:),endpoints(3,:));
    hold on
    for i = 1:numel(right_vec)
        quiver3(right_vec{i}(1),...
                right_vec{i}(2),...
                right_vec{i}(3),...
                right_vec{i}(4),...
                right_vec{i}(5),...
                right_vec{i}(6),'r');
        quiver3(left_vec{i}(1),...
                left_vec{i}(2),...
                left_vec{i}(3),...
                left_vec{i}(4),...
                left_vec{i}(5),...
                left_vec{i}(6),'b');
        quiver3(residual_vec{i}(1),...
                residual_vec{i}(2),...
                residual_vec{i}(3),...
                residual_vec{i}(4),...
                residual_vec{i}(5),...
                residual_vec{i}(6),'color',[.7 0 .7]);
    end

end