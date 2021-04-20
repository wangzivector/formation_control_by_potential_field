function F_o = for_obstacle(cen_o, rect_o, pos, vel)
    F_o = zeros(size(pos));
    for index_o = 1:size(cen_o,1)
    x0 = cen_o(index_o,1);
    y0 = cen_o(index_o,2);
    v1 = rect_o(index_o,1);
    v2 = rect_o(index_o,2);
    add_barr = 0; % add some m to avoid the obstacle.
    barrier = 3; % or add threshold.
    x = pos(:,1);
    y = pos(:,2);
    
    A_sq = 1/(2*((v1+add_barr)^2));
    B_sq = 1/(2*((v2+add_barr)^2));
    B = B_sq^(0.5);
    A = A_sq^(0.5);
%    (((x-x0).^2).*(A_sq)+((y-y0).^2).*(B_sq)) 
    need_avoid = (((x-x0).^2).*(A_sq)+((y-y0).^2).*(B_sq)) < barrier;
    dis2ob = abs((((x-x0).^2).*(A_sq)+((y-y0).^2).*(B_sq))-1);
    dis2ob_inv = dis2ob.^(-1) - 1/barrier;
    inv_factor = dis2ob.^(-2).*5;
    
    f_cw = [- (B)*(y - y0), (A)*(x - x0);];
    f_ccw = [ (B)*(y - y0), -(A)*(x - x0)];
    
    dir_sel_cw  = sum(vel.*f_cw,2);
    dir_sel_cw = dir_sel_cw > 0;
    dir_sel_ccw = dir_sel_cw == 0;
    dir = f_cw.*repmat(dir_sel_cw, [1,2]) + f_ccw.*repmat(dir_sel_ccw, [1,2]);
    dir = dir./repmat((sum(dir.^2,2).^(0.5)),[1,2]);
    dir = dir.*repmat(need_avoid, [1,2]);
    vel_norm = sum(vel.^2,2).^(0.5);
    factors = repmat(dis2ob_inv, [1,2]).*repmat(inv_factor, [1,2]);
    
    Fs_max = 2; norm_F = sum(factors.^2,2).^(0.5); bool_bi = norm_F>Fs_max; bool_sm = norm_F<=Fs_max; 
    factors_norm_max = factors./repmat(norm_F,[1,2]).*Fs_max; factors = factors.*repmat(bool_sm,[1,2]) + factors_norm_max.*repmat(bool_bi,[1,2]); 
    
    F_o_i = dir.*repmat(vel_norm, [1,2]).*factors;
    
    F_o = F_o + F_o_i; % you can cosider sum them up or just choose the biggest one.
    end
    
end