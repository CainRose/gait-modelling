% NUMROWS	Number of rows
%
%	NUMROWS(m) returns the number of rows in the matrix m
%
% 	Last modified: August 12, 2003
%

function r = numrows(m)
	[r,~] = size(m);
