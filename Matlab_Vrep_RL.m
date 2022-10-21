
clc
clear all


 env=VrepEnvironment();
 validateEnvironment(env);


observationInfo = getObservationInfo(env);
numObservations = observationInfo.Dimension(1);
actionInfo = getActionInfo(env);
numActions = actionInfo.Dimension(1);

%%


L = 500; % number of neurons
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(L,'Name','fc2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu2')
    fullyConnectedLayer(100,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(50,'Name','fc4')
    reluLayer('Name','relu4')
    fullyConnectedLayer(10,'Name','fc5')
    reluLayer('Name','relu5')
    fullyConnectedLayer(1,'Name','fc9')];

actionPath = [
    featureInputLayer(numActions,'Normalization','none','Name','action')
    fullyConnectedLayer(L,'Name','fc10')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
    
criticNetwork = connectLayers(criticNetwork,'fc10','add/in2');


%%

criticOptions = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1,'L2RegularizationFactor',1e-3);

critic = rlQValueRepresentation(criticNetwork,observationInfo,actionInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

%%

actorNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(250,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(100,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(50,'Name','fc4')
    reluLayer('Name','relu4')
    fullyConnectedLayer(10,'Name','fc5')
    reluLayer('Name','relu5')
    fullyConnectedLayer(numActions,'Name','fc8')
    tanhLayer('Name','tanh1')
    scalingLayer('Name','ActorScaling1','Scale',[0.;0.5;0.5;0.5],'Bias',[0.5;0.5;0.5;0.5])];
    %scalingLayer('Name','ActorScaling1','Scale',0.00015,'Bias',0.00015)];
    %scalingLayer('Name','ActorScaling1','Scale',0.15,'Bias',0)];

%%

actorOptions = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1,'L2RegularizationFactor',1e-3);
actor = rlDeterministicActorRepresentation(actorNetwork,observationInfo,actionInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling1'},actorOptions);

%%

agentOptions = rlDDPGAgentOptions(...
    'SampleTime',0.01,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',50000,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',512);

%agentOptions.NoiseOptions.Variance = [6.25e-5;5.45e-10];
%agentOptions.NoiseOptions.Variance = 11.56e-10;
agentOptions.NoiseOptions.Variance = (0.01)*ones(4,1);
agentOptions.NoiseOptions.VarianceDecayRate = 1e-3;
agentOptions.ResetExperienceBufferBeforeTraining = false;
%%
%agentOptions.ResetExperienceBufferBeforeTraining = false;

agent = rlDDPGAgent(actor,critic,agentOptions);
%%
maxepisodes = 1500;
maxsteps = 1000;
trainingOpts = rlTrainingOptions('MaxEpisodes',maxepisodes,'MaxStepsPerEpisode',maxsteps,'Verbose',true,'StopTrainingCriteria','GlobalStepCount','StopTrainingValue',10000000,'Plots',"training-progress");
% trainOpts.UseParallel = true;
% trainOpts.ParallelizationOptions.Mode = "async";
% trainOpts.ParallelizationOptions.TransferBaseWorkspaceVariables = "on";


%%

trainingStats = train(agent,env,trainingOpts);
keyboard
% %%
% 
 %save("TrackAgent.mat",'agent')
% 
 %%
% %load('StraightLineAgent.mat')
% 
 simOpts = rlSimulationOptions('MaxSteps',1000);
 experience = sim(env,agent,simOpts);


%%
generatePolicyFunction(agent)