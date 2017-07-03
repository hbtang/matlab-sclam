%% run and save true data
clear;
name_sim_fold = 'C:\Workspace\Data\sim\sim-mkinit-enc-sqrmap-2017.6.23\';
name_rec_fold = [name_sim_fold, 'record\'];
simulator = ClassSimulator(name_sim_fold);
simulator.Run;
save([name_rec_fold, '/simulator.mat']);
simulator.Stop;
