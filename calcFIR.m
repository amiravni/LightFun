clc
close all
N=30;
Fs=16000;
d1 = fdesign.lowpass('N,Fc',N-1,300,Fs);
d2 = fdesign.bandpass('N,Fc1,Fc2',N-1,2500,4500,Fs);
d3 = fdesign.highpass('N,Fc',N-1,4000,Fs);

for i=1:3
    if i==1
        Hd =window(d1,'window',@kaiser);
    elseif i==2
        Hd =window(d2,'window',@kaiser);
       
    elseif i==3
        Hd =window(d3,'window',@kaiser);
    end
     fvtool(Hd);
    fprintf('{');
    for i=1:N
        fprintf(num2str(Hd.Numerator(i)));
        if i<N
            fprintf(',');
        end
    end
    fprintf('};');
    fprintf('\n');
    sum(abs(Hd.Numerator))
    
end

