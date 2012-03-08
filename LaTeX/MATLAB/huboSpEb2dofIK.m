function [ d, di, vflag ] = huboSpEb2dofIK(x,y,di, dL)
% function [ d ] = huboSpEb2dofIK(x,y,di, dL)
%       x = desired x corordinate where di(0) is the origin
%       y = desired y corodinate where di(0) is the origin
%       di = index values for the joints
%       dL = length of members {di(1)}------(dL(1))------{di(2)}-----dL(2)----X
%       d = output joint angles for di(:)
%       vflag = 1 if the desired pos is valid, 0 if it is not a valid pos


%       xx = x;
%       yy = y;

%       x = -xx;
%       y = -yy;
        x = -x;
        y = -y;

        % joint lengths
        a1 = dL(1);
        a2 = dL(2);

        %a1 = 0.17914;
        %a2 = 0.18159;

        t2 = -2*atan(sqrt(((a1+a2)^2-(x^2+y^2))/((x^2+y^2)-(a1-a2)^2)));

        aa = atan2(y,x);
        bb = atan2(a2*sin(t2), a1+a2*cos(t2));

        t1 = aa - bb;
        %t1 = -t1;              % change of sign due to joint sign
        d = [t1, t2];

        vflag = 1;
        for(i=1:length(d))
                if( vflag == 1)
                        vflag = (imag(d(i)) == 0);
                end
        end

end
