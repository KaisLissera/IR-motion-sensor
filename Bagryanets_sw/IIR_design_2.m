%IIR Design
close all;

% Сигнал
CarrierFreq = 100000;        %Несущая частота
Ts = 1/CarrierFreq;         %Период несущей
DutyCycle = 50;             %Скважность несущей в процентах
ModFreq = CarrierFreq/128;   %Модулирующая частота
Tbit = 1/ModFreq;           %Период модулирующей
Ni = 10;                   %Количество генерируемых импульсов
            
fd = 3*CarrierFreq;         %Частота дискретизации
dt = 1/fd; %Период дискретизации
N = ceil(Ni*Tbit/dt)  % Количество отсчётов
time = linspace(0,dt*N,N);  %Отсчёты оси времени сигнала
razmach = 255;              %Размах генерируемого сигнала

% Параметры шума
noise_pwr = 0;            %Мощность шума в Вт
flatNoise = 0;

% Расчёт коэффициентов цифрового резонатора
figure;
a = [0 0 0];
b = [0 0 0];

a(1) = (1-exp(-2*pi*4*ModFreq/fd));

b(1) = 1;
b(2) = -(2*exp(-pi*4*ModFreq/fd)*cos(2*pi*CarrierFreq/fd));
b(3) = exp(-2*pi*4*ModFreq/fd);

% Вывод рассчитанных коэффициентов фильтра
disp(['a = ',num2str(a)]);
disp(['b = ',num2str(b)]);

% Построение АЧХ и ФЧХ фильтра
freqz(a,b);

% Масштабирование коэффициентов фильтра
a(1) = round(256*a(1));

b(1) = round(256*b(1));
b(2) = round(256*b(2));
b(3) = round(256*b(3));

disp(['a_norm = ',num2str(a)]);
disp(['b_norm = ',num2str(b)]);

freqz(a,b);

dec = 32;   %Коэффициент прореживания отсчётов
decCoeffs = fir1(dec, 0.1);
figure;
freqz(decCoeffs);

perr = zeros(1,100);
%noise_pwr = linspace(0,200,100);
%flatNoise = linspace(0,200,100);
%for k = 1:100
%-------------------------------------------------------------------------

% Cлучайный информационный сигнал
info = [zeros(1,floor(Ni/2)) ones(1,floor(Ni/2))];
info = info(randperm(Ni));
InfoModulation = zeros(1,N);
for i = 1:Ni-1
    for j = 1:(floor(Tbit/dt/2))
        InfoModulation(i*floor(Tbit/dt)+j) = info(i);
    end
end

% Генерация сигнала
Carrier = razmach*0.5*(square(2*pi*time*100000, DutyCycle) + 1);
Modulation = 0.5*(square(2*pi*time*ModFreq + pi) + 1);

%Modulation = InfoModulation;

noise = wgn(1,N,noise_pwr^2,'linear');
Signal = Carrier.*Modulation + noise + flatNoise;
%Signal = Carrier.*InfoModulation + noise + flatNoise;
Signal = ConstrainSignal(Signal); %Ограничение сигнала от 0 до 255

%Реализация цифрового резонатора
FILT = zeros(1,N);
for i = 3:N
    Akkum = 0;
    Akkum = Akkum + Signal(i)*a(1);
    %
    Akkum = Akkum - FILT(i - 1)*b(2);
    Akkum = Akkum - FILT(i - 2)*b(3);
    FILT(i) = Akkum/256;
end

AD = zeros(1,N);

%Амплитудный детектор
for i = 1:N
    if FILT(i) < 0
        AD(i) = -FILT(i);
    else
        AD(i) = FILT(i);
    end
end

%Децимация сигнала
max = 0;
DecN = floor(N/dec);    %Длительность последовательности после децимации
Decimated = zeros(1,DecN);
timedec = linspace(0,DecN*dec*dt,DecN);  %Ось времени
for i = dec:dec:N
    Sum = 0;
    for j = 1:dec
        Sum = Sum + FILT(i - dec + j)*decCoeffs(j);   
    end
    if Sum > max
        max = Sum;
    end
    Decimated(floor(i/dec)) = Sum;  %Усреднение прореженных отсчётов
end

% Согласованная фильтрация
SFiltered = (1:DecN);
SF = zeros(1,6);
Koeffs = [1 1 1 -1 -1 -1];
for i = 1:DecN
    for j = 1:6
        if (i-j > 0)
            SF(j) = Decimated(i-j);
        end
    end
    SFiltered(i) = SF*Koeffs.';
end

% Пороговая обработка
Porog = (1:DecN);
max = 50;
for i = 6:DecN
    if SFiltered(i) > max
        %max = SFiltered(i);
    end

    if SFiltered(i) > max*0.5
        Porog(i) = 255;
    else
        Porog(i) = 0;
    end
end

NumErrors = 0;
flag = 0;
for i = 2:DecN
    if Porog(i) > Porog(i - 1)
        flag = 1;
    end

    if Modulation(i*dec) ~= Modulation((i-1)*dec)
        if Modulation((i-1)*dec) ~= flag
            NumErrors = NumErrors + 1;
        end
        flag = 0;
    end
end
disp("Errors")
disp(NumErrors);
%perr(k) = NumErrors/Ni;
%end
%-------------------------------------------------------------------------

%figure;
%SNR = zeros(0,N);
%for t = 1:100
    %SNR(t) = 255/noise_pwr(t);
    %SNR(t) = 255/flatNoise(t);
%end
%semilogy(SNR,perr);
%disp(perr)
%xlim([1,5])
%grid on;

% Сигнал после фильтра и детектора
figure;
Y = [Signal.',FILT.',AD.'];
h = stackedplot(time,Y);
h.AxesProperties(1).YLimits = [0 275];
h.AxesProperties(2).YLimits = [-275 275];
h.AxesProperties(3).YLimits = [-275 275];
xlim([0,2*Tbit]);
legend('show');
grid on;

figure;
plot(time,Signal.','k','LineWidth',1.5);
ylim([0 300]);
xlim([0,2*Tbit]);
grid on;

figure;
plot(time,FILT.','k','LineWidth',1.5);
ylim([-300 300]);
xlim([0,2*Tbit]);
grid on;

figure;
plot(time,AD.','k','LineWidth',1.5);
ylim([0 300]);
xlim([0,2*Tbit]);
grid on;

% Сигнал после децимации
figure;
plot(timedec,Decimated,'k','LineWidth',1.5);
xlim([0,2*Tbit]);
grid on;

% Сигнал после СФ
figure;
plotpor = max*0.5*ones(1,N);
plot(timedec,SFiltered,'k',time,255*Modulation,'k:',time, plotpor,'k--','LineWidth',1.5);
xlim([0,4*Tbit]);
grid on;


figure;
%plot(timedec,Porog,time,255*InfoModulation);
plot(timedec,Porog,time,255*Modulation,time, plotpor);
xlim([0,10*Tbit]);
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