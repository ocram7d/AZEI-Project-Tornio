%% NUOVO MOTORE
 % 311M 

%% Calcolo coppia motrice per vite a ricircolo di sfere (sgrossatura acciaio)
clc; clear;

%% Parametri di ingresso 
Flavoro = 670;     % Forza di lavoro (N) per sgrossatura acciaio
mu = 0.005;        % Coefficiente attrito vite-chiocciola
Fw = Flavoro * (1 + mu);  % Forza totale (Flavoro + Fattrito)

P = 5;             % Passo vite (mm)
eta1 = 0.9;        % Efficienza meccanica diretta
eta2 = 0.85;       % Efficienza meccanica inversa
Tb = 0.02;         % Coppia attrito supporto (N·m)

N1 = 60;           % Denti pignone (rapporto 1:1)
N2 = 60;           % Denti corona

Cp_dyn = 1560;     % 1560Carico dinamico (N)
fpr = 0.05;        % Precarico percentuale [%]

d_n = 20;          % Diametro nominale vite (mm)

J_G1 = 0.0021;     % Inerzia pignone (kg·m²)
J_G2 = 0.0021;     % Inerzia corona (kg·m²)

m_r = 2.0;        % Massa vite (kg)
m_l = 2.5;         % Massa chiocciola + porta utensile + utensile (kg)

n1 = 0;            % Velocità iniziale (rpm)
n2 = 300;          % Velocità finale della vite (rpm)
ta = 1.5;          % Tempo accelerazione (s)

% Velocità lineare della chiocciola
v_lineare = (P * n2) / 60;  % mm/s
acc_max = (n2-n1)/ta;  % ;  16.67 mm/s^2

%  Coppia motore diretta
Ta = (Fw * P) / (2000 * pi * eta1);

% Forza di precarico
Fpr = (fpr / 100) * Cp_dyn;

% Coefficiente di attrito precarico
Kp = (1 / eta1) - eta2;

% Coppia precarico
Td = (Kp * Fpr * P) / (2000 * pi);

% Coppia totale per moto uniforme
TM = (Ta + Tb + Td) * (N1 / N2);

% Accelerazione angolare a valle del riduttore
delta_n = n2 - n1;
alpha = (2 * pi * delta_n) / (60 * ta); % rad/s^2

% Inerzia totale del sistema

J = J_G1 + J_G2 * (N1/N2)^2 + ...
    0.5 * m_r * (d_n/2000)^2 * (N1/N2)^2 + ...
    m_l * (P / (2000*pi))^2 * (N1/N2)^2;    
    
% Coppia accelerazione
Ta_acc = J * alpha;

% Coppia totale richiesta
TMa = TM + Ta_acc;

% Output risultati
fprintf('--- Risultati ---\n');
fprintf('Forza totale Fw: %.2f N\n', Fw);
fprintf('Coppia totale TM (moto uniforme): %.3f Nm\n', TM);
fprintf('Accelerazione angolare alpha: %.2f rad/s^2\n', alpha);
fprintf('Inerzia totale J: %.6e kg·m²\n', J);
fprintf('Coppia accelerazione Ta_acc: %.3f Nm\n', Ta_acc);
fprintf('Coppia totale richiesta TMa: %.3f Nm\n', TMa);
fprintf('Velocità lineare chiocciola: %.2f mm/s\n', v_lineare);

%% PARAMETRI MOTORE 

Ra = 0.85; % Resistenza Armatura (Ohm)
La = 1.34e-3; % Induttanza armatura (mH )

% kt = C_nominale / Ia_nom %  Si rompe
kvC = 7.3; % V/Krpm
kv = kvC*60/(1000*2*pi); % Da catalogo V/(rad/s) , 7.3 V/krpm
kt = kv; % suppongo uguali

C_nominale = 0.22 ;% Newton metri (Nm)
Va_nom = 24; % Volt (V)
Ia_nom = 3.65; % Ampere (A)

Jmotor = 4.7e-5; % KG M^2 
% ATTRI CACCOLI DI FINO 

omega_nom = 3000; %rpm


wmaxRad = ((n2*2*pi)/60); % rad/s a valle 
omega_max = 3000; % rpm monte riduttore
omega_max_rad = ((omega_max*2*pi)/60); % 314.159265 




%%  Calcolo potenze 

potElettNom = Va_nom*Ia_nom;
potMecc = C_nominale*omega_max_rad; % Coppia nominale * w nominale
potDissElettr = Ra*Ia_nom^2;
potDissMecc = potElettNom-potDissElettr-potMecc;
B_mecc = potDissMecc/omega_max_rad^2;

%% Coppia e Inerzie ...

Ctot = TMa/10 + 0.072 + Jmotor * omega_max_rad/ta;  % Nm Coppia a monte riduttore necessaria  
% i / 10 sono per via del riduttore 

Jtot = (Ctot*ta) / omega_max_rad; % [ kg * m^2 ]

Iamax = Ctot/kt; % Ampere

Eg = kv*omega_max_rad; % Volt

Va = Eg+Ra*Iamax; % Volt

%% Funzione di trasferimento del motore

P_el = tf(1,[La Ra]);
P_mec = tf(1,[Jtot B_mecc]);

Ls = kt*series(P_el,P_mec);

M = feedback(Ls,kv);

figure
bode(M);
pole(M)
figure
rlocus(M)
%%  Grafici motore vuoto 
out = sim("DCmotormodelVuoto.slx");
figure
plot(out.omega_vuoto.Time, out.omega_vuoto.Data,LineWidth=1.5) %plot velocità
grid on
legend("Velocità ")
xlabel("Time [s]")
ylabel("Ω [rad/s]")

figure
plot(out.ia_vuoto.Time, out.ia_vuoto.Data,LineWidth=1.5) %plot corrente
grid on
legend("Corrente di Armatura")
xlabel("Time [s]")
ylabel("Ia [A]")

figure
plot(out.coppia_vuoto.Time, out.coppia_vuoto.Data,LineWidth=1.5) %plot corrente
grid on
legend("Coppia")
xlabel("Time [s]")
ylabel("Qm [Nm]")

 %% Amplificatore Tensione
dcvolt =5 ;
Va_lim = 1.2*Va_nom; % 120$ Va nominale
kamp = Va_lim/dcvolt;  % costante di amplificazione

Vctr = Va/kamp; % da mettere in ingresso all'amplificatore di potenza

%% Limitatore di corrente
ilim = 1*Ia_nom; % 20% in più della nominale
klim_corr = dcvolt/(0.2*Ia_nom); % soglie per deadzone 

%% Grafici con limitatore ed amplificatore
close all

out = sim("DCmotormodelLIMAMP.slx");
out2 = sim("DCmotormodelVuoto.slx");
figure
plot(out2.omega_vuoto.Time, out2.omega_vuoto.Data,LineWidth=1.5) %plot velocità
hold on
plot(out.omega_vuoto2.Time, out.omega_vuoto2.Data,LineWidth=1.5) %plot velocità
grid on
legend("Senza limitatore di corrente","Con limitatore di corrente")
xlabel("Time [s]")
ylabel("Ω [rad/s]")

hold off

figure
plot(out2.ia_vuoto.Time, out2.ia_vuoto.Data,LineWidth=1.5)  %plot corrente di armatura
hold on
plot(out.ia_vuoto2.Time, out.ia_vuoto2.Data,LineWidth=1.5) 
grid on
legend("Senza limitatore di corrente","Con limitatore di corrente")
xlabel("Time [s]")
ylabel("Ia [A]")

hold off

figure
plot(out2.coppia_vuoto.Time, out2.coppia_vuoto.Data,LineWidth=1.5) %plot coppia motrice
hold on
plot(out.coppia_vuoto2.Time, out.coppia_vuoto2.Data,LineWidth=1.5)
grid on
legend("Senza limitatore di corrente","Con limitatore di corrente")
xlabel("Time [s]")
ylabel("Qm [Nm]")

figure
plot(out.va_lim.Time, out.va_lim.Data,LineWidth=1.5) %plot coppia Va dopo amplificazione
grid on
legend("Va Amplificato ")
xlabel("Time [s]")
ylabel("Va [V]")


%% DINAMIO Resp. Davide Sagliocco   Firma: ___________ , lì Aversa, Data: _______

kdt = Vctr/omega_max_rad; % Vs/rad


%% PI 
% kp tar, 5% ripple di Iamax

% irip = .05*Iamax;  %= 0.1093
% kp=20
% 
% %%soglia di corrente Ia_ripple
% N = 20;
% 
% V_kp = 0.8*Vctr;
% figure
% legendCell = cell(1,4);
% start = 1;
% step = 1;
% stop = N;
% i = 1;
% for kp = start:step:stop
%     out = sim('DCmotormodel.slx');
%     % simulation step: 1e-5
%     time = out.ia_vuoto2.Time;
%     Iaa = out.ia_vuoto2.Data;
%     plot(time,Iaa);
%     hold on
% 
%     legendCell{i} = ['kp =' num2str(kp)];
%     %Seleziona la seconda metà dei dati di corrente, che si presume rappresenti il regime stazionario.
%     Ia_steady_state = Iaa(round(size(Iaa)/2):end);
%     %calcolo il ripple picco (max) valor medio (mean)
%     Ia_ripple(i) = max(Ia_steady_state)-mean(Ia_steady_state);
%     %Controllo se il ripple calcolato è maggiore del 10% del valore nominale di corrente
%     if Ia_ripple(i) > irip
%         k_p = kp-step;
%         break
%     end
%     i=i+1;
% end
% kp = k_p;
% title('Iterazioni Kp');
% xlabel('Time [s]');
% ylabel('Current [A]');
% legend(legendCell);
% grid on


%%
VctrKP = Vctr*.9; % Tensione per taratura Kp, 90% Vctr

irip = .05*Iamax;  %= 0.1093
 
%Taratura kp    
%rif = 3.465
% Kp_vec = [3 3.2 3.5 4 4.5 5 5.5 6 6.5];
Kp_vec = [3.5 3.6 3.7 3.8 3.9 4 5 6 7];
Ia_ripple = 0;
legendcell = cell(1,5);
hold off
for ii=1:length(Kp_vec)
    kp = Kp_vec(ii);
    out = sim("DCmotormodelKP.slx");
    Iaa = out.ia_vuoto2.Data;
    subplot(2,1,1)
    plot(out.ia_vuoto2.Time, out.ia_vuoto2.Data,LineWidth=1.5)
    hold on
    subplot(2,1,2)
    hold on
    plot(out.omega_vuoto2.Time, out.omega_vuoto2.Data,LineWidth=0.8)
    legendcell{1,ii} = ['kp = ' num2str(kp)];
    Ia_steady_state = Iaa(round(size(Iaa)/2):end);
    Ia_ripple(ii) = max(Ia_steady_state)-mean(Ia_steady_state);
    fprintf('ripple corrente: %.4f \n', Ia_ripple(ii));
    fprintf('kp corrispondente: %.4f \n', kp);

    if(Ia_ripple(ii)<irip)
        kpf=kp;     % kpf è quello finale ammissibile per ripple max 5%
    end

end

subplot(2,1,1)
grid on
xlabel("Time [s]")
ylabel("Ia [A]")
legend(legendcell)

subplot(2,1,2)
grid on
xlabel("Time [s]")
ylabel("Ω [rad/s]")
legend(legendcell)

% kp = 3.5; %kp scelto per avere un ripple poco più del 5% della corrente di armatura.
kp = kpf; 
out=sim("DCmotormodelKP.slx")    
plot(out.ia_vuoto2.Time, out.ia_vuoto2.Data,LineWidth=1.5)
grid on
xlabel("Time [s]")
ylabel("Ia [A]")

figure
hold on
plot(Kp_vec, Ia_ripple)
plot(Kp_vec, Ia_ripple, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b')
yline(irip, '--r', 'Ripple Massimo Desiderato')
grid on
xlabel("Kp ")
ylabel("|Ripple Corrente|")

figure
plot(out.ia_vuoto2.Time, out.ia_vuoto2.Data,LineWidth=1.5)
hold on
grid on
xlabel("Time [s]")
ylabel("Ia [A]")


%% KI Grafici

VctrKI = Vctr*.8; % Vctr a 80%  per tarare Ki
VStep = VctrKI*0.01; % step 

clc, close all
Kilin = linspace(50,150,10);

legendcell = cell(1,5);
hold off
for ii=1:length(Kilin)
    ki = Kilin(ii);
    out = sim("DCmotormodelKI.slx");
    subplot(2,1,1)
    plot(out.ia_vuoto2.Time, out.ia_vuoto2.Data,LineWidth=0.8)
    hold on
    subplot(2,1,2)
    hold on
    plot(out.omega_vuoto2.Time, out.omega_vuoto2.Data,LineWidth=0.8)
    legendcell{1,ii} = ['ki = ' num2str(ki)];

end
subplot(2,1,1)
grid on
xlabel("Time [s]")
ylabel("Ia [A]")
legend(legendcell)

subplot(2,1,2)
grid on
xlabel("Time [s]")
ylabel("Ω [rad/s]")
legend(legendcell)



%% Taratura automatica di Ki con step a 4 s e baseline a 3.5 s
% Parametri di tuning
Ki_init   = 56;      % Ki iniziale
ki_step   = 1.05;    % fattore di incremento ad ogni iterazione
max_iter  = 20;     % numero massimo di tentativi
SovrMax   = 0.20;   % overshoot massimo ammesso  (20%)

% Variabili di appoggio
Ki_current   = Ki_init;
iter         = 1;

while iter <= max_iter
    % Assegna Ki al workspace del modello
    ki = Ki_current;

    % Simula (assicurati che DCmotormodel.slx legga la variabile 'Ki')
    out = sim('DCmotormodelKI.slx','StopTime','10');  
    t     = out.omega_vuoto2.Time;     % vettore tempi
    omega = out.omega_vuoto2.Data;     % risposta in omega

    % 1) trova indice a 3.5 s e 4.0 s
    idx_base = find(t >= 6.8, 1, 'first');
    idx_step = find(t >= 7.0, 1, 'first');

    % 2) calcola baseline e risposta al gradino
    baseline    = omega(idx_base);
    omega_step  = omega(idx_step:end);

    % 3) calcolo picco e regime sul segmento post‑4 s
    picco       = max(omega_step);
    regime      = omega_step(end);

    % 4) sovraelongazione relativa al baseline
    sovr = (picco - regime) / (regime - baseline);

    fprintf('Iter %2d: Ki = %.4f → overshoot = %.2f%% (baseline=%.3f)\n', ...
            iter, Ki_current, sovr*100, baseline);

    if sovr > SovrMax
        % superato il limite → prende l’ultimo Ki valido
        Ki_opt = Ki_current / ki_step;
        fprintf('  Superato %2.0f%% → Ki ottimale = %.4f\n', SovrMax*100, Ki_opt);
        break;
    else
        % incremento Ki e proseguo
        Ki_current = Ki_current * ki_step;
    end

    iter = iter + 1;
end

% Se termina il ciclo senza break
if iter > max_iter
    Ki_opt = Ki_current / ki_step;
    fprintf('Raggiunto max_iter → Ki ottimale = %.4f\n', Ki_opt);
end

% Restituisce Ki ottimale in workspace
assignin('base','Ki_opt',Ki_opt);
ki = Ki_opt;

close all

figure
plot(out.omega_vuoto2.Time, out.omega_vuoto2.Data,LineWidth=1.5)
grid on
xlabel("Time [s]")
ylabel("Ω [rad/s]")


 %% Bode PI+Motore
PI = tf([kp ki], [1 0]);
L = kamp*series(PI, M); 
T_s = L/(1+kdt*L);

figure, margin(T_s)    %luogo delle radici
grid on

%% Nichols
figure
nichols(T_s)
ngrid
%% WIND up senza FF
close all; clc
% Inserisco saturazione a +-5 V per l'integratore
S_wu = 5;

Vctr = Va/kamp; 
stoptimesim = '2';
outWU = sim("DCmotormodelWindUpp5.slx", "StopTime", stoptimesim); % simulazioni con solo anti windup a +-5V
VctrKI = Vctr; VStep = 0;
out2 = sim("DCmotormodelKI.slx", "StopTime", stoptimesim);

subplot(2,1,1)
hold on
plot(outWU.ia_vuoto2.Time, outWU.ia_vuoto2.Data,LineWidth=0.8)
plot(out2.ia_vuoto2.Time, out2.ia_vuoto2.Data,LineWidth=0.8)
xlabel("Time [s]")
ylabel("Ia [A]")
legend('Correzione WindUp', 'Senza Correzione')
title('Corrente con Anti-WindUp')
hold on
subplot(2,1,2)
hold on
plot(outWU.omega_vuoto2.Time, outWU.omega_vuoto2.Data,LineWidth=0.8)
plot(out2.omega_vuoto2.Time, out2.omega_vuoto2.Data,LineWidth=0.8)
legend('Correzione WindUp', 'Senza Correzione')
xlabel("Time [s]")
ylabel("Ω [rad/s]")
title('Velocità con Anti-WindUp')


%% Progetto FF
% clear out out2  % per pietà della ram

S_wu = 1.2*(Ra*Iamax/kamp);
kff = kv/(kamp*kdt);

VctrKI = Vctr; VStep = 0;
stoptimesim = '2';
out = sim("DCmotormodelKI.slx","StopTime", stoptimesim);
outWUFF = sim("DCmotormodelWindUpFF.slx", "StopTime", stoptimesim); % simulazioni con solo anti windup a +-5V

figure
hold on
subplot(2,1,1)
plot(outWUFF.ia_vuoto2.Time, outWUFF.ia_vuoto2.Data, ...
    outWU.ia_vuoto2.Time, outWU.ia_vuoto2.Data, ...
    out.ia_vuoto2.Time, out.ia_vuoto2.Data,LineWidth=0.8)

legend('Anti WindUp + FF', 'Anti WindUp', 'WindUp')
xlabel("Time [s]")
ylabel("Ia [A]")
title('Corrente Ia')
hold on
subplot(2,1,2)
hold on
plot(outWUFF.omega_vuoto2.Time, outWUFF.omega_vuoto2.Data, ...
    outWU.omega_vuoto2.Time, outWU.omega_vuoto2.Data, ...
    out.omega_vuoto2.Time, out.omega_vuoto2.Data, LineWidth=0.8)
legend('Anti WindUp + FF', 'Anti WindUp', 'WindUp')
xlabel("Time [s]")
title('Velocità Ω')
ylabel("Ω [rad/s]")

%%  Calibrazione kpos

PI_vel = tf ([ kp ki ], [1 0]) ;
G_vel = kamp*series(PI_vel, M); %catena diretta
fdt_vel = feedback (G_vel, kdt); %sistema a ciclo chiuso
Integrator = tf(1, [1 0]); %Integratore
figure , hold on, close all
Kpos_vec = logspace (-4, 0, 20);
legendcell = cell(1,5);
rlocus(1 + (series(fdt_vel, Integrator))) % Da guardare per impostare Im(p) per il Kpos
 
close all
figure(1)
figure(2)
for ii = 1:length(Kpos_vec)
    Kpos = Kpos_vec(ii);
    G_pos = Kpos*series(fdt_vel, Integrator);

    denF = 1 + G_pos;
    [Nn, Dn]= pzmap(denF);
    index_pole_im = find(imag(Dn)>=31); % Im(p) >= 44 per avere circa 20% Overshooting
    % out = sim("DCmotormodelREF.slx");
    figure(1),     
    hold on, step(G_pos/denF)

    % plot(out.omega_vuoto2.Time, out.omega_vuoto2.Data,LineWidth=0.8)
    legendcell{1,ii} = ['Kpos = ' num2str(Kpos)];
    Kpos

    if(~isempty(index_pole_im))
        fprintf(' Kpos_lim = %d \n', Kpos);
        break;
    end
    figure(2)
    hold on
    rlocus(denF);
end
% legend(legendcell)