function f_n = for_shape(pos, tar_shape)
%     pos
    ks = 500;
    f_n = zeros(size(pos));
    for index_pos = 1:size(pos,1)
        pos_i = pos(index_pos,:);
        diff_p_a = repmat(pos_i, [size(tar_shape,1),2]) - tar_shape;
        diff_p_a1 = diff_p_a(:,1:2);
        diff_p_a2 = diff_p_a(:,3:4);
        diff_a2_a1 = tar_shape(:,3:4) - tar_shape(:,1:2);
        len_proj_a1_pp = sum(diff_p_a1.*diff_a2_a1, 2)./norm_row(diff_a2_a1);
        dir_a1_pp =  diff_a2_a1./repmat(norm_row(diff_a2_a1), [1,2]).*repmat(len_proj_a1_pp,[1,2]);
        proj_p_pp = dir_a1_pp - diff_p_a1;
%         bool_12 = norm_row(diff_p_a1)>norm_row(diff_p_a2);
%         bool_21 = norm_row(diff_p_a1)<=norm_row(diff_p_a2);
        
        bool_is_acute = repmat(sum(diff_p_a1.*diff_a2_a1, 2)>0,[1,2]);
        bool_isnot_acute = 1- bool_is_acute;
        result_p_a1 =( bool_is_acute.*proj_p_pp + bool_isnot_acute.*(-diff_p_a1));
        
        bool_is_acute = repmat(sum(diff_p_a2.*(-diff_a2_a1), 2)>0,[1,2]);
        bool_isnot_acute = 1- bool_is_acute;
        result_pp =( bool_is_acute.*result_p_a1 + bool_isnot_acute.*(-diff_p_a2));
        
        [min_v, ind] = min(norm_row(result_pp));
        vec_i = result_pp(ind, :);
        
        f_i = ks*vec_i .*min_v;
        
        f_n(index_pos,:) = f_i;
    end
end