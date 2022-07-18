
clear

n=5000; %Set the Number of training examples

%Matlab-CoppeliaSim Remote API framework
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 10);
if (clientID > -1)
    disp('Connected')
    for i=1:n            
                % enable the synchronous mode on the client:
                %sim.simxSynchronous(clientID,true);

                % start the simulation:
                %sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);
                %sim.simxSynchronousTrigger(clientID);


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
                
                %imshow(image);
                
                for i = 1:2
                    %Actual Call
                    [returnCode, resolution, image] = sim.simxGetVisionSensorImage2(clientID, camera, 1, sim.simx_opmode_buffer);
                    [returnCode,linearVel,angularVel]=sim.simxGetObjectVelocity(clientID, SummitXL, sim.simx_opmode_buffer);
                    TF =isempty(image);
                    if TF ==1
                        image = zeros(512);
                    end
                    %imshow(image);

                    if i>1 
                        
                        
                        A=image;
                        threshold = 120;
                        A_bw = A > threshold;
                        A_bw_invert=1-A_bw;
                        A_bw_invert_crop=A_bw_invert(250:512,1:512);
                        
                        thisimage=imresize(A_bw_invert_crop,[10 100]);
                        imshow(thisimage)
                        measurements = regionprops(A_bw_invert_crop, 'Centroid');
                        T = isempty(measurements);
                        if T ==0
                            x2=measurements.Centroid(1);
                            this.error=256-x2;
%                             if abs(this.error) >150
%                                 this.error = sign(this.error)*155;
%                             end
%                             if abs(this.error) <0
%                                 this.error = 0;
%                             end
                                
                            this.feature = reshape(thisimage,[1000,1]);
                            this.LinVel = double(linearVel(1));
                            this.AngVel = double(angularVel(3));
                            
                        end
                        
                    end
                end 
                

            Observation =[this.feature;this.error;this.LinVel;this.AngVel];
            Action = evaluatePolicy(Observation);
            w = 0.0005*this.error + sign(this.error)* Action*0;
            w = double(w);
            V =0.5;
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
            
            sim.simxPauseSimulation(clientID,sim.simx_opmode_blocking);
    end
              
            sim.simxFinish(clientID);
end

sim.delete();

%%
IMG_train_cc=images4d(:,:,:,1:4000);
IMG_val_cc=images4d(:,:,:,4001:5000);
Vel_train_cc=data(1:4000,:);
Vel_val_cc=data(4001:5000,:);

%%
save('IMG_train_cc.mat','IMG_train_cc')
save('IMG_val_cc.mat','IMG_val_cc')
save('Vel_train_cc.mat','Vel_train_cc')
save('Vel_val_cc.mat','Vel_val_cc')
        