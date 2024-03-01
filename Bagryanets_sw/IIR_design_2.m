%IIR Design
close all;

%Сигнал задан согласно спецификации протокола RC5
CarrierFreq = 36000;        %Несущая частота
Ts = 1/CarrierFreq;         %Период несущей
DutyCycle = 10;             %Скважность несущей в процентах
ModFreq = CarrierFreq/64;   %Модулирующая частота
Tbit = 1/ModFreq;           %Период модулирующей
N = 10000;                  %Количество генерируемых отсчётов
fd = 10*CarrierFreq;        %Частота дискретизации
dt = 1/fd;                  %Период дискретизации
time = linspace(0,dt*N,N);  %Отсчёты оси времени сигнала
razmach = 255;              %Размах генерируемого сигнала

%Параметры шума
noise_pwr = 100^2;            %Мощность шума в Вт
noise = wgn(1,N,noise_pwr,'linear');

%Генерация сигнала
Carrier = razmach*0.5*(square(2*pi*time*CarrierFreq, DutyCycle) + 1);
Modulation = 0.5*(square(2*pi*time*ModFreq + pi) + 1);
Signal = Carrier.*Modulation + noise;
Signal = ConstrainSignal(Signal); %Ограничение сигнала от 0 до 255

%Параметры цифрового резонатора
f0 = CarrierFreq/(fd/2);    %Центральная частота
bw = 10*ModFreq/(fd/2);     %Полоса
[b,a] = iirpeak(f0,bw);     %Расчёт коэффициентов цифрового резонатора
%Построение АЧХ и ФЧХ фильтра
figure;
a(1) = 1;
disp(2*exp(-pi*2*ModFreq/fd)*cos(2*pi*CarrierFreq/fd));
disp(-exp(-2*pi*2*ModFreq/fd));
disp(1-exp(-2*pi*2*ModFreq/fd));

disp(2*exp(-pi*2*0.005)*cos(2*pi*0.18)*256);
disp(-exp(-2*pi*2*0.005)*256);
disp((1-exp(-2*pi*2*0.005))*256);

a(2) = -(2*exp(-pi*2*ModFreq/fd)*cos(2*pi*CarrierFreq/fd));
a(3) = exp(-2*pi*2*ModFreq/fd);
b(1) = (1-exp(-2*pi*2*ModFreq/fd))*0.6;
a(2) = -215/256;
a(3) = 248/256;
b(1) = 5/256;
b(2) = 0;
b(3) = 0;
freqz(b,a);     
%Вывод рассчитанных коэффициентов фильтра
disp(['a = ',num2str(a)]);
disp(['b = ',num2str(b)]);
%Вывод нормированных коэффициентов фильтра
b(1) = round(256*b(1));
b(2) = 0*round(256*b(2));
b(3) = round(256*b(3));
a(2) = round(256*a(2));
a(3) = round(256*a(3));
disp(['a_norm = ',num2str(a)]);
disp(['b_norm = ',num2str(b)]);

%Реализация цифрового резонатора
FILT1 = zeros(1,N);
for i = 3:N
    Akkum = 0;
    Akkum = Akkum + Signal(i)*b(1);
    Akkum = Akkum + Signal(i - 1)*b(2);
    Akkum = Akkum + Signal(i - 2)*b(3);
    %
    Akkum = Akkum - FILT1(i - 1)*a(2);
    Akkum = Akkum - FILT1(i - 2)*a(3);
    FILT1(i) = Akkum/256;
end

%Амплитудный детектор
for i = 1:N
    if FILT1(i) < 0
        FILT1(i) = -FILT1(i);
    end
end

%Построение графиков исходного сигнала и после прохождения детектора
figure;
Y = [Signal.',FILT1.'];
h = stackedplot(time,Y);
h.AxesProperties(1).YLimits = [0 275];
h.AxesProperties(2).YLimits = [0 275];
xlim([0,4*Tbit]);
legend('show');
grid on;

%Децимация сигнала с усреднением
dec = 128;   %Коэффициент прореживания отсчётов
DecN = floor(N/dec)    %Длительность последовательности после децимации
Decimated = zeros(1,DecN);
timedec = linspace(0,DecN*128*dt,DecN);  %Ось времени
for i = dec:dec:N
    Sum = 0;
    for j = i - dec + 1:i
        Sum = Sum + FILT1(j);   
    end
    Decimated(floor(i/dec)) = Sum/dec;  %Усреднение прореженных отсчётов
end
%figure;
%plot(timedec,Decimated); %Построение сигнала после децимации

%Пороговая обработка сигнала после прореживания
PorUsr = 32;    %Количество отсчётов для вычисления порога
Akkum = 0;      %Аккумулятор для вычисления порога
TresholdSignal = zeros(1,DecN); %Сигнал после пороговой обработки
OldSignalPor = 0; %Предыдущее значение сигнала после пороговой обработки
for i = 1:DecN
    Akkum = Akkum + Decimated(i);
    if i > PorUsr   %Вычисление порога при достаточном количестве отсчётов
         Akkum = Akkum - Decimated(i - PorUsr);
         Threshold = Akkum/PorUsr;
    end
    if i < PorUsr   %Вычисление порога в начальный момент времени
        Threshold = Akkum/i;
    end
    %Пороговая обработка
    if OldSignalPor == 0    %Реализация гистерезиса
        Gisteresis = 1.2;
    else
        Gisteresis = 0.8;
    end
    if Decimated(i) > Threshold*Gisteresis
        TresholdSignal(i) = 255;
        OldSignalPor = 255;
    else
        TresholdSignal(i) = 0;
        OldSignalPor = 0;
    end
end
%Построение графика сигнала после пороговой обработки
figure;
plot(time,Signal,timedec,TresholdSignal);
xlim([0,4*Tbit]);
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