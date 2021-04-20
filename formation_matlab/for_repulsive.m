function F_n = for_repulsive(pos)
    % assume that all the electric quantity are the same.
    dim = size(pos,1);
    quantity = ones([dim,1]).*2 + (1:dim)'.*0.5
    quantity_sq = quantity*quantity';
    kr = 1;
    
    distance_sq = zeros(dim);
    for index = 1:dim
        distance_sq(index,:) = sum((repmat(pos(index,:), [dim,1]) - pos).^2,2)';
    end
    distance_sq_inv = distance_sq.^(-1);
    distance_n = distance_sq.^(0.5);
    distance_sq_inv(isinf(distance_sq_inv)) =0;
    pos_x = pos(:,1);
    pos_y = pos(:,2);
    diff_pos_x = repmat(pos_x, [1,dim]) -repmat(pos_x', [dim,1]);
    diff_pos_y = repmat(pos_y, [1,dim]) -repmat(pos_y', [dim,1]);
    
    pos_x_cos = diff_pos_x./distance_n;
    pos_y_sin = diff_pos_y./distance_n;
    pos_x_cos(isnan(pos_x_cos)) = 0;
    pos_y_sin(isnan(pos_y_sin)) = 0;
    
    f_n_x = kr.*quantity_sq.*distance_sq_inv.*pos_x_cos;
    f_n_y = kr.*quantity_sq.*distance_sq_inv.*pos_y_sin;
    F_n = [sum(f_n_x, 2), sum(f_n_y, 2)];
    
end