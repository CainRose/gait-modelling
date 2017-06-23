% INVKIN	Inverse Kinematics
%
% Description:
%	INVKIN computes the inverse kinematics for one time instance if q is constant;
%	otherwise, it computes the joint angles for all time instances
%
% Syntax:
%	[q, matrices] = invkin (dh, handx, handro, handra, q0)
%
%	where,
%		q: 	is the joint angle	(rad)	(time - by - number of joint)
%		matrices: 	Jacobian matrices
%		dh: 	kinematic parameters of the robot	(see LoadRobot.m)
%		handx:  end-effector's location		(time - by - 3)
%		handro: end-effector's orientation	(time - by - 3)
%		handra: end-effector's approach		(time - by - 3)
%		q0: initial guess of q			(1 - by - number of joint)
%
% Last Modified:
%	August 23, 2003
%
% See Also: 
%	ikine_old, JACOBN, JACOB0.
%

function [q, matrices] = invkin (dh, handx, handro, handra, q0)
   
% % {
%     handx= zeros(11,3);
%     handro= zeros(11,3);
%     handra= zeros(11,3);
%     dh= zeros(6,5);
%     
%     handx(:,1)=-[0.2525 0.2620 0.2715 0.2810 0.2905 0.3000 0.3200 0.3400 0.3600 0.3800 0.4000];
%     handx(:,3)=0.3559;
%     handx(:,2)=-0.4811;
%     
%     handro(:,1)=0;
%     handro(:,2)=0;
%     handro(:,3)=1;
%     
%     handra(:,1)=0;
%     handra(:,2)=-1;
%     handra(:,3)=0;
%     
%     q0=[-1.57;2.2689;0;0;-0.6597;0];
%     
%     dh(:,1)=[-90 -90 -50 90 90 0];
%     dh(:,2)=0;
%     dh(:,3)=[90 130 -90 90 142.2 0];
%     dh(:,4)=[258.35 252.514 230.339 174.272 0 204.0875];
%     dh(:,5)=[0 0 1 0 0 0];
%     
% %    Deg -> Rad
%     dh(:,1) = dh(:,1).*(pi/180);
%     dh(:,3) = dh(:,3).*(pi/180);
% %     mm -> m
%     dh(:,2) = dh(:,2)./1000;
%     dh(:,4) = dh(:,4)./1000;
% %     }
    
    
	[numtim, ~] = size (handx);
	nn = numrows(dh);

	matrices = '';
	
	% if hand location, orientation and approach are constant
	if handx == ones(numtim,1)*handx(1,1:3) & handro == ones(numtim,1)*handro(1,1:3) & handra == ones(numtim,1)*handra(1,1:3) %#ok<AND2>
			
		% Calculate the homogeneous tranformation for the specified orientation and approach vectors.
		hhpt=oa2tr(handro(1,:),handra(1,:))+[zeros(4,3) [handx(1,:)';0]];
		
		% Calculate the inverse kinematics
		qrow=ikine_old(dh,hhpt,q0);
        if isempty(qrow)
            q = [];
            
            return
        end
        % size assignation for q - modified by claire:
        q=zeros(numtim,nn);
        
		for j=1:numtim
			q(j,1:nn)=qrow;
		end
	
		%
		% Jacobian Matrices
		%
	
		jac0=jacob0(dh,qrow);
		jacn=jacobn(dh,qrow);
	
		jstr1=sprintf('%11.5f',(jac0(1,:))');
		jstr2=sprintf('%11.5f',(jac0(2,:))');
		jstr3=sprintf('%11.5f',(jac0(3,:))');
		jstr4=sprintf('%11.5f',(jac0(4,:))');
		jstr5=sprintf('%11.5f',(jac0(5,:))');
		jstr6=sprintf('%11.5f',(jac0(6,:))');
		
		jstr7=sprintf('%11.5f',(jacn(1,:))');
		jstr8=sprintf('%11.5f',(jacn(2,:))');
		jstr9=sprintf('%11.5f',(jacn(3,:))');
		jstr10=sprintf('%11.5f',(jacn(4,:))');
		jstr11=sprintf('%11.5f',(jacn(5,:))');
		jstr12=sprintf('%11.5f',(jacn(6,:))');
	
		jstr13=sprintf('%11.5f',qrow');
	
		matrices = sprintf ('JACOBIAN MATRICES\r\nJ0 =\r\n%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n\r\nJn = \r\n%s\r\n%s\r\n%s\r\n%s\r\n%s\r\n%s \r\n\r\n Q = \r\n %s',jstr1,jstr2,jstr3,jstr4,jstr5,jstr6,jstr7,jstr8,jstr9,jstr10,jstr11,jstr12,jstr13);
	
	else
	
		% Calculate the homogeneous tranformation for the specified orientation and approach vectors.
        
        %size assignation for comhhpt - modified by claire
        comhhpt=zeros(numtim,16);
        
		for j=1:numtim
			hhpt=oa2tr(handro(j,:),handra(j,:))+[zeros(4,3) [handx(j,:)';0]];
			comhhpt(j,1:16)=reshape(hhpt,1,16);
		end
	
		% Calculate the inverse kinematics
		q=ikine_old(dh,comhhpt,q0);
        if isempty(q)
            q = [];
            
            return
        end
	
	end
