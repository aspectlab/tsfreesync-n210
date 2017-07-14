function [ res ] = difquo( vec, ts )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if nargin <= 1;
    ts = 1;
end

vec0 = [0, vec];
temp = zeros(size(vec));
for n = 1:length(vec0)-1;
    temp(n) = (vec(n)-vec0(n))/ts;
end

res = temp(2:end);