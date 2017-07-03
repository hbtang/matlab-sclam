function odo_out = NoiseOdo(this, odo_true, calib, setting)
%NOISEODO create odo with noise and dt

error = setting.error;

%% add time delay
% timestep = this.timestep;
% dt_b_c = calib.dt;
% odo_true_time = odo_true;
% for i = 1:numel(odo_true.lp)
%     %% compute v
%     if i == 1 || (dt_b_c < 0 && i ~= numel(odo_true.lp))
%         i1 = i;
%         i2 = i+1;
%     else
%         i1 = i-1;
%         i2 = i;
%     end
%     vx = (odo_true.x(i2) - odo_true.x(i1))/timestep;
%     vy = (odo_true.y(i2) - odo_true.y(i1))/timestep;
%     dtheta = odo_true.theta(i2) - odo_true.theta(i1);
%     dtheta = cnstr2period(dtheta, pi, -pi);
%     vtheta = dtheta/timestep;
%     
%     %% compute odo_true_time with delay
%     odo_true_time.x(i) = -dt_b_c*vx + odo_true.x(i);
%     odo_true_time.y(i) = -dt_b_c*vy + odo_true.y(i);
%     odo_true_time.theta(i) = -dt_b_c*vtheta + odo_true.theta(i);
%     odo_true_time.theta(i) = cnstr2period(odo_true_time.theta(i), pi, -pi);
% end

%% add noise
odo_out = struct(...
    'lp',odo_true.lp(1), ...
    'x',odo_true.x(1),'y',odo_true.y(1),'theta',odo_true.theta(1), ...
    'enc_l', odo_true.enc_l(1), 'enc_r', odo_true.enc_r(1));

stdratio_lin = error.odo.stdratio_lin;
stdratio_rot = error.odo.stdratio_rot;
k_odo_lin = calib.k_odo_lin;
k_odo_rot = calib.k_odo_rot;

% mat_odo = this.calib.mat_odo;
% mat_odo_inv = inv(mat_odo);

num_odo = numel(odo_true.lp);
for i = 2:num_odo
    lp = odo_true.lp(i);
    
    ps2d_o_b1 = [odo_true.x(i-1);odo_true.y(i-1);odo_true.theta(i-1)];
    ps2d_o_b2 = [odo_true.x(i);odo_true.y(i);odo_true.theta(i)];
    ps2d_b1_b2 = FunRelPos2d(ps2d_o_b1, ps2d_o_b2);
    ps2d_b1_b2(1:2) = ps2d_b1_b2(1:2)/k_odo_lin;
    ps2d_b1_b2(3) = ps2d_b1_b2(3)/k_odo_rot;
            
    stdErrLin = stdratio_lin*norm(ps2d_b1_b2(1:2));
    stdErrRot = stdratio_rot*abs(ps2d_b1_b2(3));
    ps2d_b1_b2 = normrnd(ps2d_b1_b2, [stdErrLin;stdErrLin;stdErrRot]);
    
    ps2d_oNs_b1 = [odo_out.x(i-1);odo_out.y(i-1);odo_out.theta(i-1)];
    ps2d_oNs_b2 = FunMove2d(ps2d_oNs_b1, ps2d_b1_b2);
    
    odo_out.x(i) = ps2d_oNs_b2(1);
    odo_out.y(i) = ps2d_oNs_b2(2);
    odo_out.theta(i) = ps2d_oNs_b2(3);
    odo_out.lp(i) = lp;
    
    % add noise to encoder
    d_enc_l = odo_true.enc_l(i) - odo_true.enc_l(i-1);    
    d_enc_r = odo_true.enc_r(i) - odo_true.enc_r(i-1);
    std_enc = stdratio_lin*(abs(d_enc_l)+abs(d_enc_r))/2;
    d_enc_l = normrnd(d_enc_l, std_enc);
    d_enc_r = normrnd(d_enc_r, std_enc);
    odo_out.enc_l(i) = odo_out.enc_l(i-1) + d_enc_l;
    odo_out.enc_r(i) = odo_out.enc_r(i-1) + d_enc_r;    
end