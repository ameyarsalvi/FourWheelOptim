
clear

n=5000; %Set the Number of training examples
%data=zeros(n,2);
images4d=zeros(32,32,1,n);

A_bw_invert_crop=zeros(512,512);
x2=0;
y2=0;
error=0;
e=0;
%Matlab-CoppeliaSim Remote API framework
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 10);
if (clientID > -1)
    disp('Connected')
    
    %Handle
    [returnCode, LWB] = sim.simxGetObjectHandle(clientID, 'joint_back_left_wheel', sim.simx_opmode_blocking);
    [returnCode, RWB] = sim.simxGetObjectHandle(clientID, 'joint_back_right_wheel', sim.simx_opmode_blocking);
    [returnCode, LWF] = sim.simxGetObjectHandle(clientID, 'joint_front_left_wheel', sim.simx_opmode_blocking);
    [returnCode, RWF] = sim.simxGetObjectHandle(clientID, 'joint_front_right_wheel', sim.simx_opmode_blocking);
    [returnCode, camera] = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking);
    [returnCode, SummitXL] = sim.simxGetObjectHandle(clientID, 'Summit_XL_visible', sim.simx_opmode_blocking);
    [returnCode, Floor] = sim.simxGetObjectHandle(clientID, 'ResizableFloor_5_25', sim.simx_opmode_blocking);
    
    
    %First Call
    [returnCode, resolution, image] = sim.simxGetVisionSensorImage2(clientID, camera, 1, sim.simx_opmode_streaming);
    [returnCode,linearVelocity,angularVelocity]=sim.simxGetObjectVelocity(clientID, SummitXL, sim.simx_opmode_streaming);
    [returnCode,Position]=sim.simxGetObjectPosition(clientID,SummitXL,-1,sim.simx_opmode_streaming);
    [returnCode,eulerAngles]=sim.simxGetObjectOrientation(clientID,SummitXL,-1,sim.simx_opmode_streaming);
    
    for i=1:n
        i
        
        %Get Vision Sensor Information
        [returnCode, resolution, image] = sim.simxGetVisionSensorImage2(clientID, camera, 1, sim.simx_opmode_buffer);
        [returnCode,linearVelocity,angularVelocity]=sim.simxGetObjectVelocity(clientID, SummitXL, sim.simx_opmode_buffer);
        [returnCode,Position]=sim.simxGetObjectPosition(clientID,SummitXL,-1,sim.simx_opmode_buffer);
        [returnCode,eulerAngles]=sim.simxGetObjectOrientation(clientID,SummitXL,-1,sim.simx_opmode_buffer);
        %Process the image to the format out Nueral Net underdands
        imshow(image);
        if i>1
            
            A=image;
            threshold = 120; % custom threshold value
            A_bw = A > threshold;
            A_bw_invert=1-A_bw;
            A_bw_invert_crop=A_bw_invert(250:512,1:512);
            imshow(A_bw_invert_crop)
            hold on
            measurements = regionprops(A_bw_invert_crop, 'Centroid');
            x2=measurements.Centroid(1);
            y2=measurements.Centroid(2);
            plot(x2, y2, 'b*')
            hold off
            
            % Store Images
            %FileName = sprintf('%d.png',i);
            %fullDestinationFileName = fullfile(imageFolder, FileName);
            thisimage=imresize(A_bw_invert_crop,[32 32]);
            images4d(:,:,:,i)=thisimage;
            
            
            e_d=((256-x2)- error)/(0.01);
            e_i=error+(256-x2);
            error=256-x2;
            kp=0.005;
            ki=0.0001;
            kd=0.000009;
            %k_v=0.1;
            %w=kp*error+ki*e_i+kd*e_d;
            w2=kp*error;
            w = w2 + 5*w2;
            V=2.5;
            
            LinVel(i)=V;
            Omega(i)=w;
            Posn(i,:) = Position;
            Orn(i,:) = eulerAngles;
            


            d=0.53; %Inter Wheel Distance
            r=0.115; %Wheel Radius
            wb=0.202; %wheel base

            TurnRadius = abs(V/w);

            R_in=TurnRadius-(d/2);
            R_out=TurnRadius+(d/2);

            theta_inner=atan2(wb,R_in);
            theta_outer=atan2(wb,R_out);

            V_star_in=(V/TurnRadius)*(sqrt(R_in^2 +wb^2));
            V_star_out=(V/TurnRadius)*(sqrt(R_out^2 +wb^2));

            V_in_wheel=V_star_in*cos(theta_inner);
            V_out_wheel=V_star_out*cos(theta_outer);
            

            w_outer=0.5*(V_out_wheel/r);
            w_inner=0.5*(V_in_wheel/r);


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
        
    end
    
    sim.simxFinish(-1);
end

sim.delete();

% %%
% IMG_train_cc=images4d(:,:,:,1:4000);
% IMG_val_cc=images4d(:,:,:,4001:5000);
% Vel_train_cc=data(1:4000,:);
% Vel_val_cc=data(4001:5000,:);
% 
% %%
% save('IMG_train_cc.mat','IMG_train_cc')
% save('IMG_val_cc.mat','IMG_val_cc')
% save('Vel_train_cc.mat','Vel_train_cc')
% save('Vel_val_cc.mat','Vel_val_cc')
%         
%figure
%plot(Posn(:,1),Posn(:,2))