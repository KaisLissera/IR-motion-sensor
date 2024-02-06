%FIR Design
close all;
por = 20;
b = fir1(por,[0.95*1e3/1e4,1.05*1e3/1e4]);
%b = fir1(por,2*1e3/1e4,'low');
freqz(b);
b_norm = round(b*256);
disp(b);
disp(b_norm);
%---------------------------
%FIR test
%Signal setup
fd = 1e4;
dt = 1/fd;
fs = 1000;
Ts = 1/fs;
N = 500;
time = linspace(0,dt*N,N);
noise_pwr = 35;
noise = wgn(1,N,noise_pwr);
razmach = 255;
signal = uint8(razmach/2*(sin(2*pi*time*fs) + 1) + noise);
%signal = uint8(razmach/2*(square(2*pi*time*fs) + 1) + noise);
%--------------------
FIR_signal = filter(b,1,signal);
%
FIR2 = FIR_signal;
for i = 1:N
    FIR2(i) = 0;
    for j = 1:(por + 1)
        if i > j
            FIR2(i) = FIR2(i) + b(j)*signal(i - j);
        end
    end
end
%
%Y = [signal.',FIR_signal.'];
Y = [signal.',FIR2.'];
figure;
h = stackedplot(time,Y);
h.AxesProperties(1).YLimits = [0 255];
h.AxesProperties(2).YLimits = [0 255];
%xlim([0,5/fs]);
legend('show');
grid on;