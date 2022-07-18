x = linspace(0,150,100); %Lateral Centroid Error
y = linspace(0,0.75,100); %Like P controller

z2 = 1.5./x.^0.2; % Like P controller
z1 = 0.5./y.^0.2; %centroid error

figure
plot(x,z1)
figure
plot(y,z2)

