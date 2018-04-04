% computeJacobian.m

function J = computeJacobian(q)

global L

J = [L(2)*cos(q(1)-q(2))+L(1)*cos(q(1)), -L(2)*cos(q(1)-q(2));
    -L(2)*sin(q(1)-q(2))-L(1)*sin(q(1)), L(2)*sin(q(1)-q(2))];