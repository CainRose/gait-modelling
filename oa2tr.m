% OA2TR	Homogenous transformation for the specified orientation and approach.
%
%	OA2TR(O, A) returns a homogeneous tranformation for the specified
%	orientation and approach vectors.
%
% 	Last modified: August 12, 2003
%

function r = oa2tr(o, a)
	n = cross(o, a);
	r = [unit(n(:)) unit(o(:)) unit(a(:)) zeros(3,1); 0 0 0 1];
