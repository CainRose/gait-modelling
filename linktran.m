% LINKTRAN	Link tranformation
% 
% Description:
% 	LINKTRAN computes the link transform from kinematic parameters.
% 
% Syntax: 
% 	tr = LINKTRAN (notation, alpha, an, theta, dn)
% 
% 	where,
% 		tr:	homogeneous transformation between link coordinate frames
% 		notation: 	Denavit and Hartenberg notation -
% 				'std' for standard Denavit and Hartenberg notation.
% 				'mod' for modified Denavit and Hartenberg notation.
% 		alpha:	the link twist angle	(rad)
% 		an:	the link length		(m)
% 		theta:	the link rotation angle	(rad)
% 		dn:	is the link offset	(m)
% 		sigma:	0 for a revolute joint, non-zero for prismatic
% 
% 	tr = LINKTRAN(notation, dh, q)
% 
% 	where,
% 		tr:	homogeneous transformation between link coordinate frames
% 		notation: 	Denavit and Hartenberg notation -
% 				'std' for standard Denavit and Hartenberg notation.
% 				'mod' for modified Denavit and Hartenberg notation.
% 		dh: 	has one row of kinematic parameters for each axis.  Each row 
% 			is of the form - [alpha A theta D sigma]	
% 		q: 	substitued for theta or dn according to sigma in DH.
% 
% Last Modified:
% 	August 23, 2003
% 
% See Also: 
% 	LOADROBOT, FORKIN.
% 


function t = linktran(a, b, c, d, e)

	if nargin == 5,
		notation = a;
		alpha = b;
		an = c;
		theta = d;
		dn = e;
		
	elseif nargin == 4,
		notation = 'std';
		alpha = a;
		an = b;
		theta = c;
		dn = d;
				
	elseif nargin == 3,
		notation = a;		
		if numcols(b) < 5,
			error('too few columns');
		end
		alpha = b(1);
		an = b(2);
		if b(5) == 0,	% revolute
			theta = c;
			dn = b(4);
		else		% prismatic
			theta = b(3);
			dn = c;
		end
	
	elseif nargin == 2,
		notation = 'std';		
		if numcols(a) < 5,
			error('too few columns');
		end
		alpha = a(1);
		an = a(2);
		if a(5) == 0,	% revolute
			theta = b;
			dn = a(4);
		else		% prismatic
			theta = a(3);
			dn = b;
		end
	end
	

	%
	% calculate tranformation matrix
	%

	sa = sin(alpha); ca = cos(alpha);
%	if ca <= 1.e-14 & ca >= -1.e-14; ca=0.; end;

	st = sin(theta); ct = cos(theta);
%	if ct <= 1.e-14 & ct >= -1.e-14; ct=0.; end;
	
	if  strcmp(notation,'std')==1
        %notation == 'std'

		% Based on the standard Denavit and Hartenberg notation.

		t =    [	ct	-st*ca	st*sa	an*ct
				st	ct*ca	-ct*sa	an*st
				0	sa	ca	dn
				0	0	0	1	];

    elseif strcmp(notation,'mod')==1
        % notation == 'mod'

		% Based on the modified Denavit and Hartenberg notation.

		t =    [	ct	-st	0	an
				st*ca	ct*ca	-sa	-sa*dn
				st*sa	ct*sa	ca	ca*dn
				0	0	0	1	];
	end