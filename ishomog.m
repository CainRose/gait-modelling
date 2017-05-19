% ISHOMOG	True for homogeneous transformation matrix
%
% 	ISHOMOG (TR) tests if argument is a homogeneous transformation
%
%	Last modified: August 12, 2003
%

function h = ishomog(tr)
	h = all(size(tr) == [4 4]);
