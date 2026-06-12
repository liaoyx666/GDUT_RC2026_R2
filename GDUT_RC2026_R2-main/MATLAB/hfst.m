function out = hfst(u1,u2,r,h)
d=r*h;
d0=h*d;
y=u1+h*u2;
a0=sqrt(d*d+8*r*abs(y));
a=0;
out1=0;
if abs(y)>d0
    a=u2+(a0-d)/2*sign(y);
end
 
if abs(y)<=d0
    a=u2+y/h;
end
 
if abs(a)>d
    out1=-r*sign(a);
end
 
if abs(a)<=d
    out1=-r*a/d;
end
out=out1;
end