function d_obs=nearest_obs(walls, test_point,thresh)

[rw,cw]=size(walls);

x0=test_point(1);
y0=test_point(2);
d_min=thresh;
d_obs=[];

for i=1:rw
    
    x1=walls(i,1);
    y1=walls(i,2);
    x2=walls(i,3);
    y2=walls(i,4);
    
    xmax=max(x1, x2);
    xmin=min(x1, x2);
    ymax=max(y1, y2);
    ymin=min(y1, y2);
    
%     if ((x0~=xmax || x0~=xmin) && (y0~=ymax || y0~=ymin))
        
        %closest point to the infinite line
        d_test=abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/sqrt((x2-x1)^2+(y2-y1)^2);
        check_ep=0;
        if d_test<=thresh
            vert=0;
            m=0;
            b=0;
            %equation of the side of the wall
            if abs(x2-x1)>1e-8  %y=mx+b
                m=(y2-y1)/(x2-x1);
                b=y2-m*x2;
                
                if abs(m)>1e-8
                    mp=-1/m; %slope of perpendicular line
                    
                    bp=y0-mp*x0;
                    
                    %intersection point of two lines
                    x_int=(bp-b)/(m-mp);
                    y_int=m*x_int+b;
                    
                    %Check if intersection is on line
                    if (x_int>=xmin && x_int<=xmax) && (y_int>=ymin && y_int<=ymax)
                        d=d_test;
                    else
                        check_ep=1;
                    end
                else
                    %Line is horizontal, perpendicular line is x=x0
                    x_int=x0;
                    if (x_int>=xmin && x_int<=xmax)
                        d=d_test;
                    else
                        check_ep=1;
                    end
                end
            else
                %vertical line x=x2=x1, equation of perpendicular line is
                %y=b
                vert=1;
                y_int=y0;
                if (y_int>=ymin && y_int<=ymax)
                    d=d_test;
                else
                    check_ep=1;
                end
            end
            
            %Intersection Point NOT on the line
            if check_ep
                d_test=min(sqrt((x1-x0)^2+(y1-y0)^2),sqrt((x2-x0)^2+(y2-y0)^2));
                if d_test<=thresh
                    d=d_test;
                else
                    d=inf;
                end
            end
            
            %     if (x0>=xmin && x0<=xmax) || (y0>=ymin && y0<=ymax)
            %         d=abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/sqrt((x2-x1)^2+(y2-y1)^2);
            %     else
            %         d=min(sqrt((x1-x0)^2+(y1-y0)^2),sqrt((x2-x0)^2+(y2-y0)^2));
            %     end
            
            if d<=d_min
                d_obs=d;
                d_min=d;
            end
        end
%     end
end