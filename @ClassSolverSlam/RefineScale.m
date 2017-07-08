function RefineScale(this, calib, map)
%REFINESCALE 此处显示有关此函数的摘要
%   此处显示详细说明

%% compute scale

map_pre = this.setting.map;

mk_id_1 = map_pre(1,1);
tvec_w_m1_pre = map_pre(1,5:7).';
mk_id_2 = map_pre(2,1);
tvec_w_m2_pre = map_pre(2,5:7).';

row_1 = find(map.mks.id == mk_id_1, 1);
row_2 = find(map.mks.id == mk_id_2, 1);
tvec_w_m1 = map.mks.tvec_w_m(row_1, :).';
tvec_w_m2 = map.mks.tvec_w_m(row_2, :).';

dist_pre = norm(tvec_w_m1_pre - tvec_w_m2_pre);
dist = norm(tvec_w_m1 - tvec_w_m2);

scale = dist_pre /dist;

%% refine map
for i = 1:map.mks.numMks
    map.mks.tvec_w_m = map.mks.tvec_w_m * scale;
end

for i = 1:map.kfs.numKfs
    map.kfs.ps2d_w_b(:,1:2) = scale * map.kfs.ps2d_w_b(:,1:2);
    map.kfs.tvec_w_b = scale * map.kfs.tvec_w_b;
end

%% refine calib
calib.mat_odo(1,:) = scale * calib.mat_odo(1,:);
calib.tvec_b_c = scale * calib.tvec_b_c;
calib.RefreshByVecbc;


end

