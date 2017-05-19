% NUMCOLS	Number of columns
%
%	NUMCOLS(m) returns the number of columns in the matrix m
%
% 	Last modified: August 12, 2003
%

function c = numcols(m)
	[~,c] = size(m);
