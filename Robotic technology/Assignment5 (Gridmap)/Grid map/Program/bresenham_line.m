function B_line = bresenham_line(xs,ys,xe,ye)
%BRESENHAM_LINE Summary of this function goes here


xs = round(xs); ys = round(ys); xe = round(xe); ye = round(ye);
dx = abs(xe-xs); dy = abs(ye-ys); sleep = abs(dy) > abs(dx);

if sleep
    t=dx;dx=dy;dy=t; 
end

if dy == 0 
    q = zeros(dx+1,1);
else
    q =[0;diff(mod([floor(dx/2):-dy:-dy*dx+floor(dx/2)]',dx))>=0];
end

if sleep
        
    if ys <= ye 
        y = [ys:ye]'; 
    else
        y = [ys:-1:ye]';
    end
    
    if xs <= xe 
        x = xs+cumsum(q);
    else
        x = xs-cumsum(q);
    end
    
else
    if xs <= xe 
        x = [xs:xe]'; 
    else
        x = [xs:-1:xe]';
    end
    
    if ys <= ye 
        y = ys+cumsum(q);
    else
        y = ys-cumsum(q); 
    end
end    
B_line = [x,y];
end

