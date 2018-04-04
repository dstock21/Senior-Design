% q = 3x1 joint angle input (hip, knee, angle)
% L = 3xl link lengths (hip-knee, knee-ankle, ankle-foot)
%output x 4x2 hip, knee, angle and foot positions 
function pos=fkNew(q)
L = [1;1;1];
pos=zeros(4,2);
posPoly=zeros(5,2);
A1=[cos(q(1)+pi/2) -sin(q(1)+pi/2) 0 -L(1)*cos(q(1)+pi/2);
    sin(q(1)+pi/2) cos(q(1)+pi/2) 0 -L(1)*sin(q(1)+pi/2);
    0 0 1 0;
    0 0 0 1];
A2=[cos(q(2)) -sin(q(2)) 0 -L(2)*cos(q(2));
    sin(q(2)) cos(q(2)) 0 -L(2)*sin(q(2));
    0 0 1 0;
    0 0 0 1];
A3=[cos(q(3)-pi/2) -sin(q(3)-pi/2) 0 L(3)*cos(q(3)-pi/2);
    sin(q(3)-pi/2) cos(q(3)-pi/2) 0 L(3)*sin(q(3)-pi/2);
    0 0 1 0;
    0 0 0 1];
knee=(A1*[0;0;0;1])';
angle=(A1*A2*[0;0;0;1])';
foot=(A1*A2*A3*[0;0;0;1])';
pos(2,:)=knee(1:2);
pos(3,:)=angle(1:2);
pos(4,:)=foot(1:2);
%grid on
q(2)=-q(2);
polyD=0.03175;
A1P=[cos(q(1)+pi/2) -sin(q(1)+pi/2) 0 -(L(1)-polyD/2)*cos(q(1)+pi/2);
    sin(q(1)+pi/2) cos(q(1)+pi/2) 0 -(L(1)-polyD/2)*sin(q(1)+pi/2);
    0 0 1 0;
    0 0 0 1;];
A2P=[cos(q(2)/2) -sin(q(2)/2) 0 -polyD*cos(q(2)/2);
    sin(q(2)/2) cos(q(2)/2) 0 -polyD*sin(q(2)/2);
    0 0 1 0;
    0 0 0 1;];
A3P=[cos(q(2)/2) -sin(q(2)/2) 0 -(L(2)-polyD/2)*cos(q(2)/2);
    sin(q(2)/2) cos(q(2)/2) 0 -(L(2)-polyD/2)*sin(q(2)/2);
    0 0 1 0;
    0 0 0 1;];
A4P=[cos(q(3)-pi/2) -sin(q(3)-pi/2) 0 L(3)*cos(q(3)-pi/2);
    sin(q(3)-pi/2) cos(q(3)-pi/2) 0 L(3)*sin(q(3)-pi/2);
    0 0 1 0;
    0 0 0 1;];
PolyPos2=(A1P*[0;0;0;1])';
PolyPos3=(A1P*A2P*[0;0;0;1])';
PolyPos4=(A1P*A2P*A3P*[0;0;0;1])';
PolyPos5=(A1P*A2P*A3P*A4P*[0;0;0;1])';
posPoly(2,:)=PolyPos2(1:2);
posPoly(3,:)=PolyPos3(1:2);
posPoly(4,:)=PolyPos4(1:2);
posPoly(5,:)=PolyPos5(1:2);

%plot(posPoly(:,1),posPoly(:,2),'bo-');

%plot(pos(:,1),pos(:,2),'o-');
%axis([-1 1 -0.8 0]);
%axis equal 

