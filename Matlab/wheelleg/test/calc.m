function result = calc(L,theta)
     result = sqrt(L^2 / (1 + (1./(tan(theta)).^2) + (tan(theta)).^2));
end