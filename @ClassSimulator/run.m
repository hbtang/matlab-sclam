function Run( this )
%RUN main function of simulator

this.Draw;
timestep = 0.1;
lp = 0;
mat_odo = this.calib.mat_odo;
mat_odo_inv = inv(mat_odo);

while ~this.flag.bQuit
    %% set loop id
    this.lp = lp;
    
    %% move robot
    ps2d_w_b1 = this.ps2d_w_b;
    dl = this.vel_lin*timestep;
    dtheta = this.vel_rot*timestep;
    ps2d_b1_b2 = [dl; 0; dtheta];
    ps2d_w_b2 = FunMove2d(ps2d_w_b1, ps2d_b1_b2);
    this.ps2d_w_b = ps2d_w_b2;  
    
    %% compute encoders
    d_vec_enc = mat_odo_inv * [dl; dtheta];
    d_enc_l = d_vec_enc(1);
    d_enc_r = d_vec_enc(2);
    this.enc_l = this.enc_l + d_enc_l;
    this.enc_r = this.enc_r + d_enc_r;
        
    %% refresh mark observed
    this.Observe;
    
    %% refresh raw data
    this.odo_true.lp = [this.odo_true.lp; lp];
    this.odo_true.x = [this.odo_true.x; ps2d_w_b2(1)];
    this.odo_true.y = [this.odo_true.y; ps2d_w_b2(2)];
    this.odo_true.theta = [this.odo_true.theta; ps2d_w_b2(3)];
    this.odo_true.enc_l = [this.odo_true.enc_l; this.enc_l];
    this.odo_true.enc_r = [this.odo_true.enc_r; this.enc_r];

    this.mk_true.lp = [this.mk_true.lp; this.mk.lp];
    this.mk_true.id = [this.mk_true.id ; this.mk.id];
    this.mk_true.rvec = [this.mk_true.rvec; this.mk.rvec];
    this.mk_true.tvec = [this.mk_true.tvec; this.mk.tvec];
    this.mk_true.image = [this.mk_true.image; this.mk.image];
    
    %% draw results
    this.Draw;
    
    %% print status
    strLp = ['-- lp: ', num2str(lp), '; '];
    strPs = ['-- pose: ', num2str(ps2d_w_b2(1)), ' ', ...
        num2str(ps2d_w_b2(2)), ' ', num2str(ps2d_w_b2(3)), '; '];
    strVel = ['-- vel: ', num2str(this.vel_lin), ' ', ...
        num2str(this.vel_rot), '; '];
    strMk = ['-- mk: ', num2str(numel(this.mk.id)), '; '];
    disp([strLp, strPs, strVel, strMk]);
        
    %% end this loop
    lp = lp + 1;
    pause(0.1);   
    
end

end

