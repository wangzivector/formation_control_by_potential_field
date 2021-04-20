function P_p = boundary (tar, pos)
    tar_cen = mean(tar);
    re_tar_cen = repmat(tar_cen,[size(pos,1),1]);
    radius = max(sum(abs(tar - repmat(tar_cen,[size(tar,1),1])).^2,2))^0.5;
    radius = radius + radius*0.2;
    diff_p_c = pos - re_tar_cen;
    diff_norm = sum(diff_p_c.^2, 2).^0.5;
    diff_p = diff_p_c./repmat(diff_norm, [1,2]);
    radius_s = repmat(diff_norm, [1,2]);
    radius_s(radius_s > radius) = radius;
    P_p = re_tar_cen + diff_p.*radius_s;
end