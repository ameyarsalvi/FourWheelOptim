classdef VrepEnvironment < rl.env.MATLABEnvironment
    

    properties
    
    error =0;
    LinVel =0;
    AngVel = 0;
    feature = zeros(1003,1);
    steps =1;
    act;
    orn;
    orn_error;
    pre_error;
    der_error;
    end
   
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false        
    end

    methods              
        function this = VrepEnvironment()
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([1003 1]);
            ObservationInfo.Name = 'Feature Vector';
            ObservationInfo.Description = 'Pixels Data; Error ;V;w';
            
            % Initialize Action settings   
            ActionInfo = rlNumericSpec([2 1],'LowerLimit',[0;0],'UpperLimit',[0.01;0.3]);
            %ActionInfo = rlNumericSpec([1 1],'LowerLimit',0,'UpperLimit',0.0003);
            %ActionInfo = rlNumericSpec([1 1],'LowerLimit',-0.3,'UpperLimit',0.3);
            ActionInfo.Name = 'Non-linear functions params';
            ActionInfo.Description = 'Beta, Alpha';
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            
        end
        
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            

            
            LoggedSignals = [];

            Action =double(Action);
            %this.act = Action;
            
            sim = remApi('remoteApi');
            %sim.simxFinish(-1);
            clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 1);

            if (clientID > -1)
                disp('Connected')
                
                % enable the synchronous mode on the client:
                sim.simxSynchronous(clientID,true);

                % start the simulation:
                sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
                sim.simxSynchronousTrigger(clientID);


                %Handle
                [returnCode, LWB] = sim.simxGetObjectHandle(clientID, 'joint_back_left_wheel', sim.simx_opmode_blocking);
                [returnCode, RWB] = sim.simxGetObjectHandle(clientID, 'joint_back_right_wheel', sim.simx_opmode_blocking);
                [returnCode, LWF] = sim.simxGetObjectHandle(clientID, 'joint_front_left_wheel', sim.simx_opmode_blocking);
                [returnCode, RWF] = sim.simxGetObjectHandle(clientID, 'joint_front_right_wheel', sim.simx_opmode_blocking);
                [returnCode, camera] = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking);
                [returnCode, SummitXL] = sim.simxGetObjectHandle(clientID, 'Summit_XL_visible', sim.simx_opmode_blocking);


                %First Call
                [returnCode, resolution, image] = sim.simxGetVisionSensorImage2(clientID, camera, 1, sim.simx_opmode_streaming);
                [returnCode,linearVel,angularVel]=sim.simxGetObjectVelocity(clientID, SummitXL, sim.simx_opmode_streaming);
                [returnCode,Position]=sim.simxGetObjectPosition(clientID,SummitXL,-1,sim.simx_opmode_streaming);
                [returnCode,eulerAngles]=sim.simxGetObjectOrientation(clientID,SummitXL,-1,sim.simx_opmode_streaming);
                
                %imshow(image);
                
                for i = 1:3
                    %Actual Call
                    [returnCode, resolution, image] = sim.simxGetVisionSensorImage2(clientID, camera, 1, sim.simx_opmode_buffer);
                    [returnCode,linearVel,angularVel]=sim.simxGetObjectVelocity(clientID, SummitXL, sim.simx_opmode_buffer);
                    [returnCode,Position]=sim.simxGetObjectPosition(clientID,SummitXL,-1,sim.simx_opmode_buffer);
                    [returnCode,eulerAngles]=sim.simxGetObjectOrientation(clientID,SummitXL,-1,sim.simx_opmode_buffer);
                    
                    load('waypoint.mat')
                    
                     search_point(:,1) =waypoint(:,1);
                     search_point(:,2) =waypoint(:,2);
                     query_point = [Position(1,1),Position(1,2)];
                     k = dsearchn(search_point,query_point);
                     this.orn = double(waypoint(k,3));
                     this.orn_error = double(eulerAngles(1,3) - this.orn); 
                    
                     
                     
                    TF =isempty(image);
                    if TF ==1
                        image = zeros(512);
                    end

                    if i>1
                        
                        %imshow(image)
                        A=image;
                        threshold = 120;
                        A_bw = A > threshold;
                        A_bw_invert=1-A_bw;
                        A_bw_invert_crop=A_bw_invert(250:512,1:512);
                        
                        thisimage=imresize(A_bw_invert_crop,[10 100]);
                        %imshow(thisimage)
                        measurements = regionprops(A_bw_invert_crop, 'Centroid');
                        
                        
                        if i==3
                        
                            T = isempty(measurements);
                            if T ==0
                                x2=measurements.Centroid(1);
                                this.pre_error = this.error;
                                this.error=256-x2;
                                this.der_error = (this.error - this.pre_error)/10;

                                this.feature = reshape(thisimage,[1000,1]);
                                this.LinVel = double(linearVel(1));
                                this.AngVel = double(angularVel(3));

                                if this.steps <25
                                    V = 0.5;
                                else
                                    V= 0.4;
                                end
                                %w = 0.025*this.error + 1*(Action(1)*this.error + (sign(this.error)*Action(2)*this.error.^2));
                                %w = 0.05*this.error + sign(this.error)*Action*this.error^.2;
                                w = Action(1)*this.error +  sign(this.error)*Action(2);
                                Action(1)
                                Action(2)
                                
%                                 load('LinearAction.mat')
%                                 load('NonLinearAction.mat')
%                                 load('Error.mat')
%                                 LinearAction = [LinearAction;Action(1)];
%                                 NonLinearAction =[NonLinearAction; Action(2)];
%                                 Error = [Error;this.error];
%                                 save('LinearAction.mat','LinearAction')
%                                 save('NonLinearAction.mat','NonLinearAction')
%                                 save('Error.mat','Error') 
                                
                                if w == 0
                                    w = double(0.0000001);
                                end

                                fprintf('Error is %d and Yaw rate is %d',this.error,w)

                                wheelrot = skidsteer(V,w);
                                w_outer = wheelrot(1);
                                w_inner = wheelrot(2);

                                    if w>0
                                        [returnCode] = sim.simxSetJointTargetVelocity(clientID, RWB, -w_outer, sim.simx_opmode_blocking);
                                        [returnCode] = sim.simxSetJointTargetVelocity(clientID, RWF, -w_outer, sim.simx_opmode_blocking);
                                        [returnCode] = sim.simxSetJointTargetVelocity(clientID, LWB,  w_inner, sim.simx_opmode_blocking);
                                        [returnCode] = sim.simxSetJointTargetVelocity(clientID, LWF,  w_inner, sim.simx_opmode_blocking);

                                    else
                                        [returnCode] = sim.simxSetJointTargetVelocity(clientID, RWB, -w_inner, sim.simx_opmode_blocking);
                                        [returnCode] = sim.simxSetJointTargetVelocity(clientID, RWF, -w_inner, sim.simx_opmode_blocking);
                                        [returnCode] = sim.simxSetJointTargetVelocity(clientID, LWB, w_outer, sim.simx_opmode_blocking);
                                        [returnCode] = sim.simxSetJointTargetVelocity(clientID, LWF, w_outer, sim.simx_opmode_blocking);
                                    end

                            end
                            
                            % Euler integration
                            %Observation =[this.feature;this.AngVel;this.orn];
                            Observation =[this.feature;this.error;this.LinVel;this.AngVel];
            
            
                            % Get reward
                            Reward = getReward(this);
            
                            % Termination
                            if abs(this.error) >200 || this.error == -0.5
                                IsDone = true ;
                            else
                                IsDone = false;
                            end
                            
                            

                            this.error
                            this.steps = this.steps+1;
                            
                        end
                        
                    end
                end 
                
                sim.simxSynchronousTrigger(clientID);
                %sim.simxPauseSimulation(clientID,sim.simx_opmode_blocking);
              
                sim.simxFinish(clientID);
                
            end
            
            

        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
         
            sim = remApi('remoteApi');
            sim.simxFinish(-1);
            clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 1);
            

                % start the simulation:
            
            
            if (clientID > -1)
                disp('Connected')
                sim.simxSynchronous(clientID,true);
                sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
                sim.simxSynchronousTrigger(clientID);
                
                

                [returnCode, SumXL] = sim.simxGetObjectHandle(clientID, 'Robotnik_Summit_XL', sim.simx_opmode_blocking);
                [returnCode]=sim.simxRemoveModel(clientID,SumXL,sim.simx_opmode_blocking);
                
                j = randi(4,1);
                if j ==1
                    [returnCode,SumXL]=sim.simxLoadModel(clientID,'SummitXLSpawn.ttm',1,sim.simx_opmode_blocking);
                elseif j==2
                    [returnCode,SumXL]=sim.simxLoadModel(clientID,'SummitXLSpawn2.ttm',1,sim.simx_opmode_blocking);
                elseif j==3
                    [returnCode,SumXL]=sim.simxLoadModel(clientID,'SummitXLSpawn3.ttm',1,sim.simx_opmode_blocking);
                elseif j==4
                    [returnCode,SumXL]=sim.simxLoadModel(clientID,'SummitXLSpawn4.ttm',1,sim.simx_opmode_blocking);
                end
                
                sim.simxPauseSimulation(clientID,sim.simx_opmode_blocking);
                sim.simxFinish(clientID);
            end

            %sim.delete();
            Observation = zeros(1003,1);
            InitialObservation = Observation;
            this.steps = 0;
        end
    end

    methods               
      
        % Reward function
        function Reward = getReward(this)
            %a = this.act - 0.005*this.error;
            %a = abs(a);
            b = abs(this.error);
            %Reward_A = 0.5/a.^0.2;
            Reward_B = 1.5/b.^0.2;
            Reward =  Reward_B;
            
        end
        
    end
    
    methods (Access = protected)
    end
end
