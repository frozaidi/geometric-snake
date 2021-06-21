function [right_vec,left_vec,residual_vec] = jacobian_vectors(endpoints,ground_contact_idx,left_comp,right_comp)
    %%% Function to calculate the "jacobian vectors" on ground contacts
    %
    % Farhan Rozaidi
    %
    % May 11, 2021

    right_vec = cell(1,size(right_comp,2));
    left_vec = cell(1,size(left_comp,2));
    residual_vec = cell(1,size(right_comp,2));

    residual_comp = right_comp+left_comp;
    
    for i = 1:numel(residual_vec)
        right_vec{i} = [endpoints(:,ground_contact_idx(i));right_comp(:,i)];
        left_vec{i} = [endpoints(:,ground_contact_idx(i));left_comp(:,i)];
        residual_vec{i} = [endpoints(:,ground_contact_idx(i));residual_comp(:,i)];
    end

end