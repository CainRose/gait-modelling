% TR2DIFF	Convert a transform difference to differential representation
%
%	TR2DIFF(T) returns a 6 element differential representation of the 
%	homogeneous transform difference T.
%
% 	Last modified: August 12, 2003
%

function d = tr2diff(t)
	d = [t(1:3,4); t(3,2); t(1,3); t(2,1)];

