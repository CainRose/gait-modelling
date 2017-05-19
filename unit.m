% UNIT	Unitize a vector
%
%	UNIT(V) returns a unit vector aligned with V.
%
% 	Last modified: August 12, 2003
%

function u = unit(v)
	u = v / norm(v,'fro');
