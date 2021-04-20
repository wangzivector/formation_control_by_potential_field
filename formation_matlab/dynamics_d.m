function [pos_f,vel_f] = dynamics_d(F, vel, pos, dt)
    M = 1;
    D = 1;
    k = 9;
    P_v = D + k;
    
    dd = 100;
    ddt = dt / dd;
    pos_f = zeros(size(pos));
    vel_f=  zeros(size(vel));
    for index = 1:size(pos,1)% for each vehcle
        acc_i = repmat(zeros(size(vel(index,:))), [dd, 1]); 
        vel_i = repmat(vel(index,:), [dd, 1]);
        pos_i= repmat(pos(index,:), [dd, 1]);
        for i =  2:dd
            acc_i(i,:) = (F(index,:) - P_v*vel_i(i,:))./M;
            vel_i(i,:) = vel_i(i-1,:) + acc_i(i-1,:).*ddt;
            pos_i(i,:) = pos_i(i-1,:)+ vel_i(i-1,:).*ddt + acc_i(i-1,:).*(0.5*ddt^2);
        end
        pos_f(index,:) = pos_i(dd,:);
        vel_f(index,:) =vel_i(dd,:);
    end
end