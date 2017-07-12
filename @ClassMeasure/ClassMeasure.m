classdef ClassMeasure < handle
    properties
        % io configures
        InputFolderPath = [];
        InputMkFilePath = [];
        InputOdoFilePath = [];
        
        % measurement data
        mk = struct('lp', [], 'id', [], ...
            'rvec', [], 'tvec', [], 'num', [], ...
            'numMkId', [], 'vecMkId', [], ...
            'pt1', [], 'pt2', [], 'pt3', [], 'pt4', []);
        odo = struct('lp',[], ...
            'x',[],'y',[],'theta',[], ...
            'enc_l', [], 'enc_r', [], ...
            'num', []);
        time = struct('lp',[],'t_odo',[],'t_mk',[]);
    end
    
    methods
        % constructor function, set input file path
        function this = ClassMeasure(PathFold, NameMk, NameOdo)
            if nargin == 1
                this.InputFolderPath = PathFold;
                this.InputMkFilePath = [PathFold, 'record/mk.rec'];
                this.InputOdoFilePath = [PathFold, 'record/odo.rec'];
            elseif nargin == 3
                this.InputFolderPath = PathFold;
                this.InputMkFilePath = [PathFold, 'record/', NameMk];
                this.InputOdoFilePath = [PathFold, 'record/', NameOdo];
            end
        end
        
        function CopyTo(this, copy)
            copy.mk = this.mk;
            copy.odo = this.odo;
            copy.time = this.time;
            copy.InputFolderPath = this.InputFolderPath;
            copy.InputMkFilePath = this.InputMkFilePath;
            copy.InputOdoFilePath = this.InputOdoFilePath;
        end
        
        function ClearAll(this)
            % io configures
            this.InputFolderPath = [];
            this.InputMkFilePath = [];
            this.InputOdoFilePath = [];
            % measurement data
            this.mk = struct('lp', [], 'id', [], ...
                'rvec', [], 'tvec', [], 'num', [], ...
                'numMkId', [], 'vecMkId', [], ...
                'pt1', [], 'pt2', [], 'pt3', [], 'pt4', []);
            this.odo = struct('lp',[], ...
                'x',[],'y',[],'theta',[], ...
                'enc_l', [], 'enc_r', [], ...
                'num', []);
            this.time = struct('lp',[],'t_odo',[],'t_mk',[]);
        end
        
        function ClearData(this)
            % measurement data
            this.mk = struct('lp', [], 'id', [], ...
                'rvec', [], 'tvec', [], 'num', [], ...
                'numMkId', [], 'vecMkId', [], ...
                'pt1', [], 'pt2', [], 'pt3', [], 'pt4', []);
            this.odo = struct('lp',[], ...
                'x',[],'y',[],'theta',[], ...
                'enc_l', [], 'enc_r', [], ...
                'num', []);
            this.time = struct('lp',[],'t_odo',[],'t_mk',[]);
        end
        
        
        % function read input file
        ReadRecData(this);
        PruneData( this, threshDistOdo, threshAngleOdo, bUseEnc );
        PruneDataByVecLp(this, vecLp);
        
        % function get measurement of lp
        odoLp = GetOdoLp(this, lp, bUseVel);
        mkLp = GetMkLp(this, lp);
    end
end

