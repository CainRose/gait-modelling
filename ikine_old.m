% IKINE		Inverse manipulator kinematics
% 
% 	Q = IKINE(DH, T)
% 	Q = IKINE(DH, T, Q)
% 
% 	Returns the joint coordinates corresponding to the
% 	end-effector transform T.  Note that the inverse kinematic solution is
% 	generally not unique, and depends on the initial guess Q (which 
% 	defaults to 0).
% 
% 	Q = IKINE(DH, TG)
% 	Q = IKINE(DH, TG, Q)
% 
% 	Returns the joint coordinates corresponding to
% 	each of the transforms in flattened form which are the rows of TG.
% 	Returns one row of Q for each input transform.  The initial estimate 
% 	of Q for each time step is taken as the solution from the previous 
% 	time step.
% 
% 	Solution is computed iteratively using the pseudo-inverse of the
% 	manipulator Jacobian.
% 
% 	Such a solution is completely general, though much less efficient 
% 	than specific inverse kinematic solutions derived symbolically.
% 	
% 	This approach allows a solution to obtained at a singularity, but 
% 	the joint angles within the null space are arbitrarily assigned.
% 
% See Also:
% 	FKINE, JACOBIAN.
% 
% Last Modified:
% 	August 24, 2003
% 

function qt = ikine_old(dh, tr, q)
	
	n = numrows(dh);
    
          
	if nargin == 2,
		q = zeros(n, 1);
	else
		q = q(:);
	end

	tolerance = 1e-01;	% error tolerance
	ilimit = 1500;		% iteration limit
	count = 0;		% iteration counter
	
	if ishomog(tr), % single point is provided
		nm = 1;
		
        while nm > tolerance,
			e = tr2diff(tr - fkine(dh, q'));
			% original
			% dq = pinv( jacob0(dh, q) ) * e;
			
			% nm is added to improve the chance of convergence
			dq = pinv( jacob0(dh, q) ) * e;

%			(q * 180 / pi)'
			q = q + dq; %* norm(e); %mod(q,pi) min(norm(dq), 1) * min(norm(dq), 1)
			q = mod (q, 2*pi);
%			(dq' * 180 / pi)
			nm = norm(dq);
%			nm = norm(e);
% 			if nm <1,
% 				nm
% 			end
			% iteration limit
 			count = count+1;
            if count > ilimit,
 				q = [];
                break
            end
        end
                
		qt = q';
		
	else	% a trajectory is provided
        qt=zeros();
		for i=1:numrows(tr),
			nm = 1;
			T = reshape(tr(i,:), 4,4);
            while nm > tolerance,			
%				e = tr2diff( T - fkine(dh, q'));
% 				j0 = jacob0(dh, q);				
% 				if (ones(size(j0)) == isfinite(j0))
% 					dq = pinv(j0) * e;
% 					q = q + dq;
% 					nm = norm(dq);					
% 				else
% 					return;
% 				end								

				e = tr2diff( T - fkine(dh, q'));
				dq = pinv(jacob0(dh, q)) * e;
				q = q + dq;
				nm = norm(dq);					

				% iteration limit
	 			count = count+1;
                if count > ilimit,
	 				q = [];
                    break
                end
				
            end
            %qt=[qt; q'];
            
            %claire's modification:
            if i==1
                qt=q';
            else
			qt = cat(1,qt,q');
            end
           
		end
	end