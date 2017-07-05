function SolveMatOdoRot(this, measure, calib)
%SOLVEMATODOROT 此处显示有关此函数的摘要
%   此处显示详细说明

%% Read data and group constraints
disp('Init: calibrate odo_mat rotational term');

odo = measure.odo;
mk = measure.mk;

if isfield(this.setting.solver, 'init_thresh_locallp')
    threshLpLocal = this.setting.solver.init_thresh_locallp;
else
    threshLpLocal = 10;
end

cnstrRows = {};
for i = 1:numel(mk.vecMkId)
    mkId_i = mk.vecMkId(i);
    
    vecMkFind = find(mk.id == mkId_i);
    vecMkLpFind = mk.lp(vecMkFind);
    
    for j = 2:numel(vecMkLpFind)
        
        lp1 = vecMkLpFind(j-1);
        lp2 = vecMkLpFind(j);
        
        if abs(lp2-lp1) > threshLpLocal
            continue;
        end
        
        rowOdo1 = find(odo.lp == lp1);
        rowOdo2 = find(odo.lp == lp2);
        
        rowMk1 = vecMkFind(j-1);
        rowMk2 = vecMkFind(j);
        
        cnstrRow.rowOdo1 = rowOdo1;
        cnstrRow.rowOdo2 = rowOdo2;
        cnstrRow.rowMk1 = rowMk1;
        cnstrRow.rowMk2 = rowMk2;
        
%         dtheta = cnstr2period(odo.theta(rowOdo1) - odo.theta(rowOdo2), pi, -pi);
%         dx = odo.x(rowOdo1) - odo.x(rowOdo2);
%         dy = odo.y(rowOdo2) - odo.y(rowOdo2);
%         dl = sqrt(dx*dx + dy*dy);
%         
%         tvec_c1_m = mk.tvec(rowMk1,:);
%         dist_c1_m = norm(tvec_c1_m);
%         dl_m = abs(dist_c1_m*dtheta);
        
        cnstrRows{numel(cnstrRows)+1} = cnstrRow;
    end
end


%% calibrate rotational terms in mat_odo
num_cnstr = numel(cnstrRows);
A = zeros(num_cnstr, 2);
b = zeros(num_cnstr, 1);
for i = 1:num_cnstr
    rowOdo1 = cnstrRows{i}.rowOdo1;
    rowOdo2 = cnstrRows{i}.rowOdo2;
    rowMk1 = cnstrRows{i}.rowMk1;
    rowMk2 = cnstrRows{i}.rowMk2;
    
    enc_l_1 = odo.enc_l(rowOdo1);
    enc_r_1 = odo.enc_r(rowOdo1);
    ps2d_cg1_m = mk.vec_ps2d_cg_m(rowMk1,:).';
    yaw_cg1_m = ps2d_cg1_m(3);
    
    enc_l_2 = odo.enc_l(rowOdo2);
    enc_r_2 = odo.enc_r(rowOdo2);
    ps2d_cg2_m = mk.vec_ps2d_cg_m(rowMk2,:).';
    yaw_cg2_m = ps2d_cg2_m(3);
    
    d_yaw_cg_m = cnstr2period(yaw_cg2_m - yaw_cg1_m, pi, -pi);
    
    d_enc_l = enc_l_2 - enc_l_1;
    d_enc_r = enc_r_2 - enc_r_1;
    A(i,:) = [d_enc_l, d_enc_r];
    b(i) = -d_yaw_cg_m;
end

mat_odo_rot = pinv(A)*b;


%% calibrate ramaining terms with SVD
% Az = 0, z = [c; s; x; y; b]
% c = cos(theta_b_cg)
% s = sin(theta_b_cg)
% x = x_b_cg, y = y_b_cg
%^b is the length of the robot base
% mat_odo = [-b*beta_l/2, b*beta_r/2; beta_l, beta_r]

num_cnstr = numel(cnstrRows);
A = zeros(2*num_cnstr, 5);
beta_l = mat_odo_rot(1);
beta_r = mat_odo_rot(2);

for i = 1:num_cnstr
    % find data recorded
    rowOdo1 = cnstrRows{i}.rowOdo1;
    rowOdo2 = cnstrRows{i}.rowOdo2;
    rowMk1 = cnstrRows{i}.rowMk1;
    rowMk2 = cnstrRows{i}.rowMk2;
    
    % load data
    enc_l_1 = odo.enc_l(rowOdo1);
    enc_r_1 = odo.enc_r(rowOdo1);
    ps2d_cg1_m = mk.vec_ps2d_cg_m(rowMk1,:).';
    x_cg1_m = ps2d_cg1_m(1);
    y_cg1_m = ps2d_cg1_m(2);
    yaw_cg1_m = ps2d_cg1_m(3);
    
    enc_l_2 = odo.enc_l(rowOdo2);
    enc_r_2 = odo.enc_r(rowOdo2);
    ps2d_cg2_m = mk.vec_ps2d_cg_m(rowMk2,:).';
    x_cg2_m = ps2d_cg2_m(1);
    y_cg2_m = ps2d_cg2_m(2);
    yaw_cg2_m = ps2d_cg2_m(3); 
    
    d_enc_l = enc_l_2 - enc_l_1;
    d_enc_r = enc_r_2 - enc_r_1;
    
    % create block in linear equations set 
    A1 = [x_cg1_m, -y_cg1_m; y_cg1_m, x_cg1_m];
    
    d_theta_b1_b2 = mat_odo_rot.' * [d_enc_l; d_enc_r];
    A2 = (-beta_l*d_enc_l/2 + beta_r*d_enc_r/2) * ...
        [cos(d_theta_b1_b2/2); sin(d_theta_b1_b2/2)];
    
    R_b1_b2 = [cos(d_theta_b1_b2) -sin(d_theta_b1_b2);...
        sin(d_theta_b1_b2) cos(d_theta_b1_b2)];
    A3 = R_b1_b2 * [x_cg2_m, -y_cg2_m; y_cg2_m, x_cg2_m];
    
    A_block = [A1-A3, eye(2)-R_b1_b2, -A2];
    A(i*2-1:i*2,:) = A_block;
end

[~, ~, V] = svd(A);
rmse_min = inf;
v_min = V(:,1);
for i = 1:5
    v = V(:,i);
    if v(5) < 0
        v = -v;
    end
    v = v / norm(v(1:2));
    rmse = rms(A*v);
    if rmse < rmse_min
        rmse_min = rmse;
        v_min = v;
    end
end
c = v_min(1); s = v_min(2); x = v_min(3); y = v_min(4); b = v_min(5);
theta_b_cg = atan2(s, c);
ps2d_b_cg = [x; y; theta_b_cg];
mat_odo = [-b*beta_l/2, b*beta_r/2; beta_l, beta_r];


%% return
calib.mat_odo = mat_odo;
calib.SetPs2dbcg(ps2d_b_cg);

end

