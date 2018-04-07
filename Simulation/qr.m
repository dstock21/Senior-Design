function qr = qr()

% joint angles (theta, omega, alpha: ankle, knee, hip)
dataq = xlsread('Winter_Appendix_data.xlsx','A4.RelJointAngularKinematics', 'D5:L110');
q = dataq(:,[7 4]); % degrees to radians (hip, knee, ankle)

r = 22:length(x)-15;
q = q(r,:);
q_new = q(1,:);
for i = 2:length(r)
    q_new = [q_new; (q(i-1,:)+q(i,:))/2; q(i,:)];
end
qr = q_new;