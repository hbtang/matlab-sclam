%% record with noise
clear
name_sim_fold = 'C:\Workspace\Data\sim\sim-mkinit-enc-ceiling-2017.7.4\';
name_rec_fold = [name_sim_fold, 'record\'];
load([name_rec_fold, 'simulator.mat']);

%% record true
% simulator.odo_rec = simulator.odo_true;
% simulator.mk_rec = simulator.mk_true;
% options = struct( ...
%     'b_record_mk', true, ...
%     'b_record_odo', true);
% simulator.Record(options);

%% record odo with noise
stdratio_odo_min = 0.005;
stdratio_odo_max = 0.05;
stdratio_odo_gap = 0.005;
num_set_odo = 20;
for stdratio_odo = stdratio_odo_min:stdratio_odo_gap:stdratio_odo_max
    stdratio_lin = stdratio_odo;
    stdratio_rot = stdratio_odo;
    for set = 1:num_set_odo
        options = struct( ...
            'flag', 'odo', ...
            'stdratio_lin', stdratio_lin, ...
            'stdratio_rot', stdratio_rot, ...
            'set', set);
        simulator.AddNoiseRecord(options);
    end
end

%% record mk with noise
std_img_min = 0.1;
std_img_max = 1;
std_img_gap = 0.1;

for std_img = std_img_min:std_img_gap:std_img_max
    for set = 1:20
        options = struct( ...
            'flag', 'mk', ...
            'std_img', std_img, ...
            'set', set);
        simulator.AddNoiseRecord(options);
    end
end




