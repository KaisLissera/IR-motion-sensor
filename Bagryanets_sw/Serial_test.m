close all;
%Сигнал задан согласно спецификации протокола RC5
CarrierFreq = 36000;        %Несущая частота
Ts = 1/CarrierFreq;         %Период несущей
DutyCycle = 50;             %Скважность несущей в процентах
ModFreq = CarrierFreq/64;   %Модулирующая частота
Tbit = 1/ModFreq;           %Период модулирующей
N = 10000;                  %Количество генерируемых отсчётов
fd = 10*CarrierFreq;        %Частота дискретизации
dt = 1/fd;                  %Период дискретизации
time = linspace(0,dt*N,N);  %Отсчёты оси времени сигнала
razmach = 255;              %Размах генерируемого сигнала

%Параметры шума
noise_pwr = 0^2;            %Мощность шума в Вт
noise = wgn(1,N,noise_pwr,'linear');

%Генерация сигнала
Carrier = razmach*0.5*(square(2*pi*time*CarrierFreq, DutyCycle) + 1);
Modulation = 0.5*(square(2*pi*time*ModFreq + pi) + 1);
Signal = Carrier.*Modulation + noise;
Signal = ConstrainSignal(Signal); %Ограничение сигнала от 0 до 255
%-----------------------------------
%COM port setup
s = serialport("COM4",115200);
configureTerminator(s,"CR","CR");
write(s,Signal,"uint8");
pause(2);
STM_signal = read(s,s.NumBytesAvailable,"uint8");
clear s;
%------------------------------------
N_STM = 156;
STM_data = STM_signal(1:N_STM);
TickCount = STM_signal(N_STM+1) + 256*STM_signal(N_STM+2) + 256*256*STM_signal(N_STM+3) + 256*256*256*STM_signal(N_STM+4);
disp(TickCount);
%Y = [Signal.',STM_data.'];
%h = stackedplot(time,Y);
%h.AxesProperties(1).YLimits = [0 255];
%h.AxesProperties(2).YLimits = [0 255];
figure;
%plot(STM_data);
dec = 64;   %Коэффициент прореживания отсчётов
DecN = floor(N/dec)    %Длительность последовательности после децимации
timedec = linspace(0,DecN*64*dt,DecN);  %Ось времени
plot(time,Signal,timedec,STM_data);
xlim([0,2*Tbit]);
xlabel('Время, с');
ylim([0,300]);
legend('Исходный сигнал','Обработанный сигнал');
legend('show');
grid on;
legend('show');
grid on;

%Функция ограничения сигнала
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