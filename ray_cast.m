function ztrue_out=ray_cast(walls, laser_temp, robot_pose)

sense_angles=laser_temp{1}.laser_angles;
zmax=laser_temp{1}.z_max;

[rw,cw]=size(walls);

x0=robot_pose(1);
y0=robot_pose(2);
th0=robot_pose(3);

min_sense=min(sense_angles);
max_sense=max(sense_angles);
sense_fov=max_sense-min_sense;

for i=1:rw
    z_true(i,:)=zmax*ones(size(sense_angles));
    
    x1=walls(i,1);
    y1=walls(i,2);
    x2=walls(i,3);
    y2=walls(i,4);
    
    %closest point to the line
    d=abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/sqrt((x2-x1)^2+(y2-y1)^2);
    
    if d<zmax %check if wall is within range
        q(i,1)=atan2(y1-y0,x1-x0);
        q(i,2)=atan2(y2-y0,x2-x0);
        
        p(i,1)=sqrt((x1-x0)^2+(y1-y0)^2);
        p(i,2)=sqrt((x2-x0)^2+(y2-y0)^2);
        
        
        line_a=[cos(th0) sin(th0)];
        line_b=[p(i,1)*cos(q(i,1)) p(i,1)*sin(q(i,1))];
        line_c=[p(i,2)*cos(q(i,2)) p(i,2)*sin(q(i,2))];
        
        alpha1_t=atan2(line_b(2),line_b(1))-atan2(line_a(2),line_a(1));
        alpha2_t=atan2(line_c(2),line_c(1))-atan2(line_a(2),line_a(1));
        
        
        alpha1(i)=atan2(sin(alpha1_t),cos(alpha1_t));
        alpha2(i)=atan2(sin(alpha2_t),cos(alpha2_t));
        
        %check if alphas intersect with sense_angles
        alpha1_test=0;
        alpha2_test=0;
        
        %Assumes sensor coverage
        if alpha1(i)<=max(sense_angles) && alpha1(i)>=min(sense_angles)
            alpha1_test=1;
        end
        
        if alpha2(i)<=max(sense_angles) && alpha2(i)>=min(sense_angles)
            alpha2_test=1;
        end
        
        alpha3_test=(sense_angles<max(alpha1(i),alpha2(i)));
        alpha4_test=(sense_angles>min(alpha1(i),alpha2(i)));
        
        if (alpha1_test||alpha2_test||any(alpha3_test)||any(alpha4_test))
            alpha_min=min(alpha1(i),alpha2(i));
            alpha_max=max(alpha1(i),alpha2(i));
            vert=0;
            m=0;
            b=0;
            %equation of the side of the wall
            if abs(x2-x1)>1e-8  %y=mx+b
                m=(y2-y1)/(x2-x1);
                b=y2-m*x2;
                bp=y0-m*x0;
                
                %                 %check
                %                 y1-m*x1-b
                %                 y2-m*x2-b
            else
                %vertical line x=x2=x1
                vert=1;
            end
            
            
            
            for j =1:length(sense_angles)
                yr=y0+zmax*sin(sense_angles(j)+th0);
                xr=x0+zmax*cos(sense_angles(j)+th0);
                if vert==0
                    if abs(xr-x0)>1e-8  %y=mx+b
                        m2=(yr-y0)/(xr-x0);
                        b2=yr-m2*xr;
                        %                         %check
                        %                         y0-m2*x0-b2
                        %                         yr-m2*xr-b2
                        if abs(m-m2)>1e-8
                            x3=(b2-b)/(m-m2);
                            y3=m2*x3+b2;
                        else %wall and ray are parallel
                            x3=xr;
                            y3=yr;
                        end
                    else %ray is vertical and wall is not
                        %vertical line x=x0=xr
                        x3=xr;
                        y3=m*x3+b;
                    end
                else  %wall was vertical
                    if abs(xr-x0)>1e-8  %y=mx+b
                        m2=(yr-y0)/(xr-x0);
                        b2=yr-m2*xr;
                        x3=x1;
                        y3=m2*x3+b2;
                    else %ray and wall are both vertical - no intersection
                        x3=xr;
                        y3=yr;
                    end
                end
                z_true_temp=sqrt((x0-x3)^2+(y0-y3)^2);
                xt=x0+z_true_temp*cos(sense_angles(j)+th0);
                yt=y0+z_true_temp*sin(sense_angles(j)+th0);
                
%                 qt(i)=atan2(yt-y0,xt-x0);
%                 line_t=[z_true_temp*cos(qt(i)) z_true_temp*sin(qt(i))];
%                 
%                 alphat_t=atan2(line_t(2),line_t(1))-atan2(line_a(2),line_a(1));
%                 
%                 
%                 alphat(i)=atan2(sin(alphat_t),cos(alphat_t));
        
                if z_true_temp<=zmax
                    if xt>=(min(x1,x2)-1e-1) && xt <=(max(x1,x2)+1e-1) && ...
                        yt>=(min(y1,y2)-1e-1) && yt<=(max(y1,y2)+1e-1) && ...
                        ((abs(yt-m*xt-b)<1e-1) || vert)
                        z_true(i,j)=z_true_temp;
                    end
                end
            end %end for
            
        end % if in FOV
        %               plot([robot_pose(1) robot_pose(1)+p(i,1)*cos(q(i,1))],[robot_pose(2) robot_pose(2)+p(i,1)*sin(q(i,1))])
        %               plot([robot_pose(1) robot_pose(1)+p(i,2)*cos(q(i,2))],[robot_pose(2) robot_pose(2)+p(i,2)*sin(q(i,2))])
    end  %end if d<zmax
end  %end for number of walls

for k=1:length(sense_angles)
    ztrue_out(k)=min([z_true(:,k);zmax]);
end