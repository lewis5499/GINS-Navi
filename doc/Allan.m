% 
% function [av,tau] = Allan(x0,tau0,radix)
% 
% Calculate the Allan deviation
%
% inputs: 
%       x0 --- the time series to be analyzed 
%       tau0 - sampling period (s)
%       radix -- the radix for the cluster length's increment, 2 for
%       default
% outputs: 
%       av ------ Allan deviations versus cluster time series
%       tau ----- series of cluster time
%
% Written by: Qijin Chen, December 9, 2012
% Email: chenqijin@whu.edu.cn

function [av, tau] = Allan(x0, tau0,radix)

if nargin < 2
    warning('Incorrect number of input arguments');
    return;
elseif nargin == 2
    radix = 2;
end

y = cumsum(x0) * tau0;
N = length(y); 
n_a = floor(log(N/3)/log(radix));

for i_index = 0 : n_a
    m = floor(radix^i_index);
    tau(i_index + 1) = tau0 * m; % cluster time
    temp2 = y(2*m + 1 : N); 
    temp1 = y(m + 1 : (N- m));
    temp0 = y(1 : (N - 2*m));
    av(i_index+1) = sqrt(mean((temp2-2*temp1+...
        temp0).^2/(2*tau(i_index+1)^2))); 
end

