function [output] = readTemp(s) 
% Read value returned via Serial communication 
output = fscanf(s,'%f');

end
