close all
clear all
clc

% Leer los datos del archivo .log
data = readmatrix('prueba2_paciente_256Hz.log');

% data será una matriz Nx3 con las columnas X, Y, Z

N = size(data,1);

% PCLK2 = 72e6;       %frec del reloj (72MHz)
% prescaler = 8;      %prescaler para el ADC
% samp_cycles = 480;  %sampling cycles para cada canal del adc
% conv_cycles = 15;   %(12 bits de resol. del adc)
% adc_buff_size = 6;
% ch_send_size = adc_buff_size/3/2;
% fs = (PCLK2/prescaler)/(samp_cycles+conv_cycles); %frec muestreo por cada muestra (no por cada canal)
% fs_ch = fs/3;       %frec muestreo de un canal
% Ts = 1 / fs;        % Tiempo entre conversiones individuales (entre cada muestra) (~55us)
% Ts_ch = 1/fs_ch;    % Tiempo entre muestras del mismo canal (~165us)
% Tsend = Ts_ch*ch_send_size;
% fsend = 1/Tsend;
% t = (0:N-1)*Tsend;  % vector de tiempos para un canal 

%t_x = (0:N-1)*Ts_ch; % vector de tiempos reales para canal 'x'
%t_y = t + Ts;        % vector de tiempos reales para canal 'y' (no empieza en t = 0)
%t_z = t + 2*Ts;      % vector de tiempos reales para canal 'z' (no empieza en t = 0)

fs = 256; % 250 Hz (para los 3 canales) (se adquieren casi simultáneamente)(diferencia de 55 us entre una adquisición y otra)
Ts = 1/fs; % 4 ms
t = (0:N-1)*Ts;

X = data(:,1);
Y = data(:,2);
Z = data(:,3);
X = X -mean(X);
Y = Y -mean(Y);
Z = Z -mean(Z);

X = X((10*fs):(40*fs));
Y = Y((10*fs):(40*fs));
Z = Z((10*fs):(40*fs));
t = t((10*fs):(40*fs));

% 
% % Calcular la magnitud de la aceleración en cada instante de tiempo (norma
% % euclidia)
% magnitud = sqrt(X.^2 + Y.^2 + Z.^2);
% %mag_offset = magnitud - 1;
% mag_offset = magnitud - mean(magnitud);
% 
% rmsX = rms(X);
% rmsY = rms(Y);
% rmsZ = rms(Z);
% rmsMag = rms(mag_offset);

% Graficar datos en crudo
figure(1);
subplot(3,1,1);
h1 = plot(t, X, 'r');
%ylim([-2 2]);
hold on;
xlabel('Time [s]')
ylabel('Accel [g]')
title('X axe')
hold off
subplot(3,1,2);
h2 = plot(t, Y, 'g');
%ylim([-2 2]);
hold on;
xlabel('Time [s]')
ylabel('Accel [g]')
title('Y axe')
hold off
subplot(3,1,3);
h3 = plot(t, Z, 'b');
%ylim([-2 2]);
hold on;
xlabel('Time [s]')
ylabel('Accel [g]')
title('Z axe')
hold off
linkaxes([ancestor(h1, 'axes'), ancestor(h2, 'axes'), ancestor(h3, 'axes')], 'xy');



% z_offset = mean(Z);
% z_centered = Z - z_offset;

L = length(Z);
NFFT = 2^nextpow2(L);
Y = fft(Z, NFFT);

P2 = abs(Y/NFFT).^2;
P1 = P2(1:NFFT/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = fs*(0:(NFFT/2))/NFFT;

figure;
plot(f, 10*log10(P1));
xlabel('Frecuencia [Hz]');
ylabel('Potencia [dB]');
title('PSD del eje Z');
xlim([0 fs/2]);
grid on;

idx_4_12 = (f >= 4 & f <= 12);
energia_temblor = sum(P1(idx_4_12));
fprintf('Energía en 4–12 Hz: %.6f\n', energia_temblor);

