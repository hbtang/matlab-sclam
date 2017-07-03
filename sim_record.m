%% record with noise
clear
name_sim_fold = 'C:\Workspace\Data\sim\sim-mkinit-enc-sqrmap-2017.6.23\';
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
% stdratio_lin = 0.03;
% stdratio_rot = 0.03;
% for set = 1:20
%     options = struct( ...
%         'flag', 'odo', ...
%         'stdratio_lin', stdratio_lin, ...
%         'stdratio_rot', stdratio_rot, ...
%         'set', set);
%     simulator.AddNoiseRecord(options);
% end

%% record mk with noise
std_img = 1;
for set = 1:20
    options = struct( ...
        'flag', 'mk', ...
        'std_img', std_img, ...
        'set', set);
    simulator.AddNoiseRecord(options);
end




