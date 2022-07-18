clear

load('Position.mat')
load('Orientation.mat')

%figure
%plot(Posn(9:230,1),Posn(9:230,2))

waypoint(:,1) = Posn(9:230,1);
waypoint(:,2) = Posn(9:230,2);
waypoint(:,3) = Orn(9:230,2);

save('waypoint.mat','waypoint')
