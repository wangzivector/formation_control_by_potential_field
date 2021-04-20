% implementation of Coordinated formation control design with obstacle avoidance 
% in three-dimensional space
% 1. 随意进出队形；2.简单避障；3. 避免碰撞；4. 可实现编队移动。
% problems： 1. 难以定义任意队形；2，绝对位置式编队

% 计算互斥力的曲面切线方向
% 当通过障碍，归一化其输入力

%% parameters of set
clear;
clc;
dt = 0.01;
max_F = 30;
a = 3^(0.5);
change_diff_thred = 0.005;
target = [2 0;1 a;-1 a;-2 0;-1 -a;1 0.1;1 3;1 0.5]  ;% size(6,2), no use yet
target = target(1:4,:);% num of robots

%initial position
initial_pos = (target+ randn(size(target))).*0.2 +2 ;
pos = initial_pos;
vel = zeros(size(target));

% initial value
F_last_last = 0.1;
F_last = 0.2;
pos_size = size(pos,1);
pos_save = zeros(0,pos_size,2);
vel_save = zeros(0,pos_size,2);
cen = [0 0]; % tragetary of virtual leader
shape_ind =0;
%% 
for index_t = (0:dt:30)
    %% calculate of forces
    R = 1;
    cen_o = [-3 0];
    rect_o = [1.2 0.8];
    pos_o = pos;
    vel_o = vel;
    [cen,~] = dynamics(zeros(size(cen)), [0 0], cen, dt);
    
    F_r = for_repulsive(pos)
    f_s = for_spherical(cen, R, pos);
    if shape_ind == 3
        shape = [-2 0 2 0;]; % line
    end
    if shape_ind == 2
        shape = [ -1 0 1 0; -1 -2 1 -2; -1 -2 -1 0;1 -2 1 0; ]; % square
    end
    if shape_ind == 0
        shape = [ -1 0 1 0;-1 0 0 -1.5;0 -1.5 1 0;]; % triangle
    end
    if shape_ind == 1
        shape = [ 0 0 0 1;0 0 1 -1.5;0 0 -1 -1.5]; % person
    end
    if shape_ind == 4
        break;
    end
    %     shape = [ -1 0 1 0; -1 -2 1 -2; -1 -2 -1 0;1 -2 1 0; ]; % square
    %     shape = [ -1 0 1 0;-1 0 0 -1.5;0 -1.5 1 0;]; % triangle
    f_s = for_shape(pos,shape)
    
    Fr_max = max_F; norm_F = sum(F_r.^2,2).^(0.5); bool_bi = norm_F>Fr_max; bool_sm = norm_F<=Fr_max; 
    F_r_norm_max = F_r./repmat(norm_F,[1,2]).*Fr_max; F_r = F_r.*repmat(bool_sm,[1,2]) + F_r_norm_max.*repmat(bool_bi,[1,2]);
    
    Fs_max = max_F; norm_F = sum(f_s.^2,2).^(0.5); bool_bi = norm_F>Fs_max; bool_sm = norm_F<=Fs_max; 
    f_s_norm_max = f_s./repmat(norm_F,[1,2]).*Fs_max; f_s = f_s.*repmat(bool_sm,[1,2]) + f_s_norm_max.*repmat(bool_bi,[1,2]); 
    f_s(isnan(f_s)) = 0;
%     F_n = zeros(size(F_n));
    f_a = F_r + f_s;
    f_o = for_obstacle(cen_o, rect_o, pos_o, f_a);

%     f_a = f_a + f_o;
    F = f_a
    
     norm(F)
    min_norm_thred = 0.5;
    if norm(F) < min_norm_thred
        
        shape_ind = shape_ind+1;
        for index_a = 1:(size(pos,1))
            for index_b = 1:(size(pos,1))
                plot([pos(index_a,1); pos(index_b,1)], [pos(index_a,2); pos(index_b,2)], 'k-');
            end
        end
        disp('shape done!');
        pause(3);
%         break;
    end
    
    %%  tricks to break 
    min_norm = 4;
    if norm(F) < min_norm
    F = (F./norm(F)).*min_norm;
    end
    norm(F - F_last_last);
%     if norm(F - F_last_last) < 1e-2
%         norm(F - F_last_last)
%         break;
%     end
    norm(F + F_last);
%     if norm(F + F_last) < 5
%          norm(F + F_last)
%         break;
%     end
    F_last_last = F_last;
    F_last = F;
%     
%     F(F>max_F) = max_F;
%     F(F<-max_F) = -max_F;
    
%     acc = zeros(size(f_s));
    %% dynamics
    [pos_f,vel_f] = dynamics_d(F, vel, pos, dt);
%     [pos_f,vel_f] = dynamics(zeros(size(F)) ,F, pos, dt);

    norm(vel_f) 
%     min_norm = 0.5;
%     if norm(vel_f) < min_norm
%         vel_f = (vel_f./norm(vel_f)).*min_norm; 
%     end
    
    % value save
    pos = pos_f;
    vel = vel_f
    pos_save(end+1,:,:) = pos;
    vel_save(end+1,:,:) = vel;
    
    %% plot
    aplha=0:pi/40:2*pi;
    r=0.1;
    x=r*cos(aplha) + cen(1);
    y=r*sin(aplha) + cen(2);
    hold off;
    plot(x,y,'k--');
    hold on;
    axis equal;
    for_obstacle_plot(cen_o, rect_o, pos_o, vel_o);
    % plot new points
    for i = 1:pos_size
        scatter(pos_save(:,i,1), pos_save(:,i,2),30,'.');
    end
    axis equal;
    pause(0.000001);
%     disp(pos);
%     scatter(pos(:,1), pos(:,2),20,'.');
end
%% plot formation
for index_a = 1:(size(pos,1))
    for index_b = 1:(size(pos,1))
        plot([pos(index_a,1); pos(index_b,1)], [pos(index_a,2); pos(index_b,2)], 'k-');
    end
end
%%


