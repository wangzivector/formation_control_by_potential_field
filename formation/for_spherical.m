function f_n = for_spherical(cen,R, pos)
%     pos
    ks = 50;
    diff_p_c = pos - repmat(cen, [size(pos,1),1]);
    sum_p_c = sum(diff_p_c.^2,2);
    diff_sq = sum_p_c - ones(size(sum_p_c)).*(R^2);
    f_n = diff_p_c.*(repmat(diff_sq,[1,2]).*(-ks));
    
%     aplha=0:pi/40:2*pi;
%     r=R;
%     x=r*cos(aplha) + cen(1);
%     y=r*sin(aplha)+ cen(2);
%     plot(x,y,'-');
%     hold on;
%     axis equal;
%     scatter(pos(:,1), pos(:,2),50,'cs');
end