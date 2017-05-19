% JACOB0	Compute manipulator Jacobian in world coordinates
%
%	JACOB0(DH, Q) returns a Jacobian matrix for the current pose Q.
%
% 	The manipulator Jacobian matrix maps differential changes in joint space
%	to differential Cartesian motion (world coord frame) of the end-effector.
%			dX = J dQ
%
%	For an n-axis manipulator the Jacobian is a 6 x n matrix.
%
% 	Last modified: August 12, 2003
%

function J = jacob0(dh, q)

	%
	%   dX_tn = Jn dq
	%
	
	Jn = jacobn(dh, q);	% Jacobian from joint to world space
	tn = fkine(dh, q);

	%
	%   dX_tn = Jtn dX
	%   dX = Jtn^-1 dX_tn
	%
%	Jtn = tr2jac(tn);	% Jacobian from world to Tn frame

	%
	%  dX = Jtn^-1 Jn dq
	%
%	J = inv(Jtn) * Jn;

    % claire madification
    % J=[[tn(1:3,1:3)] zeros(3,3); zeros(3,3) [tn(1:3,1:3)]]*Jn;
	J=[tn(1:3,1:3) zeros(3,3); zeros(3,3) tn(1:3,1:3)]*Jn;