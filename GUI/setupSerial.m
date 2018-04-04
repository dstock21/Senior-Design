function[obj] = setupSerial(comPort)
% Initialize Serial object
delete(instrfindall);
obj = serial(comPort);
set(obj,'DataBits',8);
set(obj,'StopBits',1);
set(obj,'BaudRate',9600);
set(obj,'Parity','none');
fopen(obj);
end
