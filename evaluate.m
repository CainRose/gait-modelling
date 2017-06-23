function [pop, state] = evaluate(opt, pop, state, varargin)
% Function: [pop, state] = evaluate(opt, pop, state, varargin)
% Description: Evaluate the objective functions of each individual in the
%   population.
%
%         LSSSSWC, NWPU
%    Revision: 1.0  Data: 2011-04-20
%*************************************************************************

N = length(pop);
allTime = zeros(N, 1);  % allTime : use to calculate average evaluation times

%*************************************************************************
% Evaluate objective function in parallel
%*************************************************************************
if( strcmpi(opt.useParallel, 'yes') == 1 )
    model = 'final_model_trial2.mdl';
    load_system(model);
    
    curPoolsize = matlabpool('size');

    % There isn't opened worker process
    if(curPoolsize == 0)
        if(opt.poolsize == 0)
            matlabpool open local
        else
            matlabpool(opt.poolsize)
        end
    % Close and recreate worker process
    else
        if(opt.poolsize ~= curPoolsize)
            matlabpool close
            matlabpool(opt.poolsize)
        end
    end
    spmd
        warning('off','all');
        % Setup tempdir and cd into it
        currDir = pwd;
        addpath(currDir);
        tmpDir = tempname;
        mkdir(tmpDir);
        cd(tmpDir);
        % Load the model on the worker
        load_system(model);
    end

    parfor i = 1:N
        fprintf('\nEvaluating the objective function... Generation: %d / %d , Individual: %d / %d \n', state.currentGen, opt.maxGen, i, N);
        [pop(i), allTime(i)] = evalIndividual(pop(i), opt.objfun, varargin{:});
    end

%*************************************************************************
% Evaluate objective function in serial
%*************************************************************************
else
    counter_good_ind = 0;
    for i = 1:N
        fprintf('\nEvaluating the objective function... Generation: %d / %d , Individual: %d / %d \n', state.currentGen, opt.maxGen, i, N);
        [pop(i), allTime(i)] = evalIndividual(pop(i), opt.objfun, varargin{:});
        if pop(i).nViol == 0;
            counter_good_ind = counter_good_ind + 1;
        end
        counter_good_ind
    end
end

%*************************************************************************
% Statistics
%*************************************************************************
state.avgEvalTime   = sum(allTime) / length(allTime);
state.evaluateCount = state.evaluateCount + length(pop);




function [indi, evalTime] = evalIndividual(indi, objfun, varargin)
% Function: [indi, evalTime] = evalIndividual(indi, objfun, varargin)
% Description: Evaluate one objective function.
%
%         LSSSSWC, NWPU
%    Revision: 1.1  Data: 2011-07-25
%*************************************************************************

tStart = tic;
%evalc('[y, cons] = objfun( indi.var, varargin{:} )');
[y, cons] = objfun( indi.var, varargin{:} )
evalTime = toc(tStart);
% keyboard();
% Save the objective values and constraint violations
indi.obj = y;
indi.cons = cons;
if( ~isempty(indi.cons) )
    idx = find( cons );
    if( ~isempty(idx) )
        indi.nViol = length(idx);
        indi.violSum = sum( abs(cons) );
    else
        indi.nViol = 0;
        indi.violSum = 0;
    end
end


