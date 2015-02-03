function [ pi ] = UpdatePi(pi, s, d, n)
%Updates pi

for i = 1:n
    temp = pi(i)-s*d(i);
    pi(i) = max([0  temp]);

end

