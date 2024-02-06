%IIR Design
close all;
%---------------------------
%FIR test
%Signal setup
fd = 1e4;
dt = 1/fd;
fs = 1000;
Ts = 1/fs;
N = 5000;
time = linspace(0,dt*N,N);
noise_pwr = 20;
noise = wgn(1,N,noise_pwr);
razmach = 255;
DutyCycle = 50;
Carrier = razmach*0.5*(square(2*pi*time*fs, DutyCycle) + 1);
Modulation = 0.5*(square(2*pi*time*fs/20) + 1);
%Modulation = 1;
%signal = uint8(razmach/2*(sin(2*pi*time*fs) + 1) + 0);
%signal = uint8(razmach/2*(square(2*pi*time*fs, DutyCycle) + 1) + noise);
%signal = razmach/2*(sin(2*pi*time*fs) + 1);
%signal = uint8(.*(square(2*pi*time*fs/10) + 1) + noise);
%signal = razmach/2*(square(2*pi*time*fs, DutyCycle) + 1).*(square(2*pi*time*fs/10) + 1) + noise;
Signal = Carrier.*Modulation + noise;
Signal = ConstrainSignal(Signal);
%--------------------
b2 = fir1(20,(fd/5)/(fd/2),'low'); a2 = 1;
[b,a] = iirpeak(2*fs/fd,0.1)
%[b,a] = cheby1(4,1,[2*fs/fd,2*]);
figure;
freqz(b,a);
FILT1 = filter(b,a,Signal);
for i = 1:N
    if FILT1(i) < 0
        FILT1(i) = -FILT1(i);
    end
end
FILT1 = ConstrainSignal(FILT1);
FILT2 = filter(b2,a2,FILT1);
Y = [Signal.',FILT1.'];
figure;
h = stackedplot(time,Y);
h.AxesProperties(1).YLimits = [0 300];
h.AxesProperties(2).YLimits = [-255 255];
%xlim([0,5/fs]);
legend('show');
grid on;
%
dec = 20;
Decimated = zeros(1,floor(N/dec));
for i = dec:dec:N
    Sum = 0;
    for j = i - dec + 1:i
        Sum = Sum + FILT1(j);
    end
    Decimated(floor(i/dec)) = Sum/dec;
end
figure;
plot(Decimated);
%FUNCTIONS===================================
function Output = ConstrainSignal(Data)
N = length(Data);
Output = Data;
for i = 1:N
    if Output(i) > 255
        Output(i) = 255;
    end
    if Output(i) < 0
        Output(i) = 0;
    end
end
end