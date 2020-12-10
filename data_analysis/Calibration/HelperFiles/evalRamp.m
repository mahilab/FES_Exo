function [curve, rampInput, rampOutput] = evalRamp(time, hmForce, rampTime, halfRamp, stimInput, fs, f, nfft, cutoff, filterOrder, HofFreq)
%evalRamp filters, and performs ramp deconvolution to 

    rampIndex     = find(time>=rampTime,1);
    preRampIndex  = find(time>=(rampTime-0.5),1);
    postRampIndex = find(time>=(rampTime+(2*halfRamp+0.5)),1);
    % average force in the 0.5 seconds before the ramp
    preForceRamp  = mean(hmForce(preRampIndex:rampIndex,:));
    % compute the force magnitude during and 0.5 seconds after the ramp
    postForceRamp = zeros(length(hmForce(rampIndex:postRampIndex,:)),size(hmForce,2));
    postForceRampMag = zeros(length(hmForce(rampIndex:postRampIndex,:)),1);
    for i = 1:length(hmForce(rampIndex:postRampIndex,:))
        postForceRamp(i,:) = hmForce(rampIndex+i-1,:) - preForceRamp;
        % compute the magnitude of the force
        postForceRampMag(i) = norm(postForceRamp(i,:));
    end
    % make a vector of ramp times
    rampTime = time(rampIndex:postRampIndex) - time(rampIndex);
    rampInput = stimInput(rampIndex:postRampIndex);
    rampOutput = [postForceRampMag' zeros(1,postRampIndex-rampIndex+1)];
    % compute the FFT
    
    rampFFT = fft(rampOutput,nfft);
    % take the magnitude
    magRampFFT = abs(rampFFT);
    % filter divide each member of the FFT by the transfer function of the impulse
    % response
    [b,a] = butter(filterOrder,cutoff/(fs));
    rampOutputFilt = filtfilt(b,a,rampOutput);
    rampFFTfiltNathan = fft(rampOutputFilt,nfft);
    
    newSignal = zeros(length(f),1);
    rampFFTfilt = zeros(length(f),1);
    magRampFFTfilt = zeros(length(f),1);
    for i = 1:length(f)
        rampFFTfilt(i) = rampFFT(i)/(1+(f(i)/cutoff)^(2*filterOrder));
        newSignal(i) = rampFFTfilt(i)/HofFreq(i);
        newSignalNathan(i) = rampFFTfiltNathan(i)/HofFreq(i);
        magRampFFTfilt = abs(rampFFTfilt);
    end
    % transform the signal back to the time domain to get the recruitment curve
    curve = ifft(newSignal);
    curve = real(curve);

%     curveNathan = ifft(newSignalNathan);
%     curveNathan = real(curveNathan);
    
    rampOutput = rampOutput(1:length(rampTime));
    curve      = curve(1:length(rampTime));
    
    figure()
    hold on
    plot(f,magRampFFT)
    plot(f,magRampFFTfilt,'r')
    plot(f,abs(HofFreq),'c')
    plot(f,abs(newSignal),'g')

    figure()
    hold on
    plot(rampTime,rampInput)
    plot(rampTime,rampOutput*100,'r')
    plot(rampTime,curve*100,'g')