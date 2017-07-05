function ProjMk(this, measure, calib)
%PROJMK 此处显示有关此函数的摘要
%   此处显示详细说明

%% compute mk projection: vec_ps2d_cg_m

mk = measure.mk;
mk.vec_ps2d_cg_m = zeros(mk.num, 3);
rvec_cg_c = calib.rvec_cg_c;
tvec_cg_c = calib.tvec_cg_c;
[ T3d_cg_c, R3d_cg_c ] = FunVec2Trans3d( rvec_cg_c, tvec_cg_c );

for mkId = mk.vecMkId.'
    rows = find(mk.id == mkId);
    
    vec_yaw = [];
    vec_pitch = [];
    vec_roll = [];
    vec_tvec = [];
    for row = rows.'
       rvec_c_m = mk.rvec(row, :).';
       tvec_c_m = mk.tvec(row, :).';
       [ T3d_c_m, R3d_c_m ] = FunVec2Trans3d( rvec_c_m, tvec_c_m );
       T3d_cg_m = T3d_cg_c * T3d_c_m;        
       R3d_cg_m =  T3d_cg_m(1:3, 1:3);
       tvec_cg_m = T3d_cg_m(1:3, 4);
       [yaw, pitch, roll] = dcm2angle( R3d_cg_m.', 'ZYX' ); % dcm should be the transpose of rotational matrix
       vec_yaw = [vec_yaw; yaw];
       vec_pitch = [vec_pitch; pitch];
       vec_roll = [vec_roll; roll]; 
       vec_tvec = [vec_tvec; tvec_cg_m.'];
    end
    
    pitch_mean = mean(vec_pitch);
    roll_mean = mean(vec_roll);
    z_mean = mean(vec_tvec(:,3));
    vec_ps2d = [vec_tvec(:,1:2), vec_yaw];
    mk.vec_ps2d_cg_m(rows.', :) = vec_ps2d;
    
end
    
%% return
measure.mk = mk;


end

