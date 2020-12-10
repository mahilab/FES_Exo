function [peakTime] = computePeakTime(time, hmForce, pulseTime)
% returns the peak time given the time, corresponding force profile, and
% the time that the pulse starts
%   Detailed explanation goes here
    pulseIndex = find(time>=pulseTime,1);
    prePulseIndex  = find(time>=(pulseTime-0.5),1);
    postPulseIndex = find(time>=(pulseTime+1.0),1);
    % average force in the 0.5 seconds before the impulse
    preForce = mean(hmForce(prePulseIndex:pulseIndex,:));
    % compute the force magnitude immediately following the pulse
    postForce = zeros(length(hmForce(pulseIndex:postPulseIndex,:)),size(hmForce,2));
    postForceMag = zeros(length(hmForce(pulseIndex:postPulseIndex,:)),1);
    for i = 1:length(hmForce(pulseIndex:postPulseIndex,:))
        postForce(i,:) = hmForce(pulseIndex+i-1,:) - preForce;
        % compute the magnitude of the force
        postForceMag(i) = norm(postForce(i,:));
    end
    % find the peak time
    % maximum output
    [maxPostForce,maxIndex] = max(postForceMag);
    % the onset time is the first time at which the output is greater than 10% or the
    % max output
    onsetIndex = find(postForceMag>0.1*maxPostForce,1);
    % time to peak
    peakTime = time(maxIndex) - time(onsetIndex);
end

