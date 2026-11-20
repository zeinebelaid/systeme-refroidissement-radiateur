%% COSIM_WEBOTS  Simulink -> Webots (UDP et/ou ROS 2)
%  Lance ton modele refroidissement_intelligent_final_original sur 4 scenarios (4 x 300 s),
%  puis rejoue les resultats en direct dans Webots (voiture 3D) a la vitesse ACCEL.
%  A executer comme SCRIPT (pas comme fonction) pour que les variables restent dans le workspace de base.

%% ---------- reglages ----------
MODELE   = "refroidissement_intelligent_final_original";
USE_UDP  = true;      % envoi direct vers Webots (127.0.0.1:5005) : marche sous Windows sans ROS
USE_ROS2 = false;     % true : publie aussi /cooling/state (ROS Toolbox) -> voir ros2/ros2_webots_bridge.py
ACCEL    = 5;         % rejeu : 5 s simulees par seconde reelle (1200 s -> 240 s)
DUREE_SC = 300;       % duree d'un scenario (s simulees) ; 4 x 300 = StopTime 1200 du modele
PANNE    = true;      % panne du ventilateur pendant l'ete (t = 1000 .. 1150 s)
MES_SCENARIOS = false;% true : utilise tes variables speed_ts, load_ts, ambient_ts, engine_ts, pump_ts, fault_ts deja creees
HOTE = "127.0.0.1"; PORT = 5005; DT_WALL = 0.04;   % 25 messages par seconde
USE_WSL  = false;     % true : envoie aussi en UDP vers le pont ROS 2 qui tourne dans WSL (ros2/lancer_pont_wsl.sh)
HOTE_WSL = "localhost"; PORT_WSL = 5006;          % sinon mets l'IP donnee par la commande Windows : wsl hostname -I

%% ---------- 1. parametres P ----------
if ~exist("P","var") || ~isstruct(P)
    warning("P absent du workspace : parametres ILLUSTRATIFS utilises (ceux du mode demo de Webots). " + ...
            "Charge tes vrais parametres (ton script d'initialisation) avant de relancer.");
    P = struct("Cm",30000,"Cl",25000,"Cr",10000,"Kmc",2000,"Kcr",2500,"UA0",40,"Kv",2.0,"Kfan",150, ...
               "Pfan",350,"Qbase",4000,"Qload",20000,"Tref",90,"Ttherm0",75,"Ttherm1",90, ...
               "Kp",0.2,"Ki",0.005,"Kaw",0.1,"Tm0",80,"Tl0",75,"Tr0",65);
end

%% ---------- 2. scenarios -> 6 entrees du modele (timeseries) ----------
% Embouteillage / Autoroute / Pluie / Ete.   Vitesse en km/h (unite supposee : adapte VIT_UNITE si besoin).
t  = (0:1:DUREE_SC*4)';
sc = min(4, floor(t/DUREE_SC) + 1);
tl = t - (sc-1)*DUREE_SC;
if ~MES_SCENARIOS
    AMB = [32; 28; 16; 43];   CHG = [0.35; 0.60; 0.45; 0.72];   VIT = [0; 120; 80; 90];
    vit = VIT(sc);  vit(sc==1) = max(0, 18*sin(2*pi*tl(sc==1)/110));
    fault = zeros(size(t));  if PANNE, fault(t >= 1000 & t < 1150) = 1; end
    speed_ts   = timeseries(vit,        t);
    load_ts    = timeseries(CHG(sc),    t);
    ambient_ts = timeseries(AMB(sc),    t);
    engine_ts  = timeseries(ones(size(t)), t);
    pump_ts    = timeseries(ones(size(t)), t);
    fault_ts   = timeseries(fault,      t);
end

%% ---------- 3. simulation Simulink ----------
load_system(MODELE);
set_param(MODELE, "StopTime", num2str(DUREE_SC*4));
out = sim(MODELE);

lire = @(nom) lire_signal(out, nom);
tq = (0:DT_WALL*ACCEL:DUREE_SC*4)';
itp = @(nom, meth) interp_ts(lire(nom), tq, meth);
Tm = itp("Tm","linear");   Tl = itp("Tl","linear");   Tr = itp("Tr","linear");
PWM = itp("PWM","linear"); Tamb = itp("Tamb","linear"); vitesse = itp("vitesse","linear");
charge = itp("charge","linear"); Pfan = itp("Pfan_W","linear"); Efan = itp("Efan_kWh","linear");
alarme = itp("alarme","previous"); thermo = itp("thermostat","linear"); Qrad = itp("Qrad_W","linear");
scen   = min(4, floor(tq/DUREE_SC) + 1);
pump   = interp1(speed_ts.Time, double(pump_ts.Data),   tq, "previous", "extrap");
panne  = interp1(speed_ts.Time, double(fault_ts.Data),  tq, "previous", "extrap");
moteur = interp1(speed_ts.Time, double(engine_ts.Data), tq, "previous", "extrap");
% ordre = celui attendu par cooling_supervisor.py
S = [tq Tm Tl Tr PWM Tamb vitesse charge Pfan Efan alarme thermo Qrad scen pump panne moteur];

%% ---------- 4. courbes (utiles pour le rapport) ----------
figure("Name","Resultats Simulink"); 
subplot(3,1,1); plot(tq,Tm,tq,Tl,tq,Tr); legend("Tm","Tl","Tr"); ylabel("Temperature (C)"); grid on
subplot(3,1,2); plot(tq,PWM,tq,thermo); legend("PWM","thermostat"); grid on
subplot(3,1,3); plot(tq,vitesse); ylabel("Vitesse (km/h)"); xlabel("t (s)"); grid on

%% ---------- 5. rejeu en direct vers Webots ----------
if USE_UDP || USE_WSL, u = udpport("datagram","IPV4"); end
if USE_ROS2
    noeud = ros2node("/cooling_matlab");
    pub   = ros2publisher(noeud, "/cooling/state", "std_msgs/Float64MultiArray");
    msg   = ros2message(pub);
end
input("Lance la simulation dans Webots, puis appuie sur Entree pour demarrer le rejeu... ", "s");
t0 = tic;
for k = 1:size(S,1)
    d = (k-1)*DT_WALL - toc(t0);
    if d > 0, pause(d); end
    if USE_UDP
        ligne = "M," + strjoin(compose("%.4f", S(k,:)), ",");
        write(u, uint8(char(ligne)), "uint8", HOTE, PORT);
    end
    if USE_WSL
        ligne = "M," + strjoin(compose("%.4f", S(k,:)), ",");
        write(u, uint8(char(ligne)), "uint8", HOTE_WSL, PORT_WSL);
    end
    if USE_ROS2
        msg.data = double(S(k,:))';
        send(pub, msg);
    end
end
disp("Rejeu termine.");

%% ---------- fonctions locales ----------
function ts = lire_signal(out, nom)
    try, ts = out.get(nom); catch, ts = evalin("base", nom); end
end
function y = interp_ts(ts, tq, meth)
    [tt, iu] = unique(ts.Time, "last");
    d = double(squeeze(ts.Data));  d = d(iu);
    y = interp1(tt, d(:), tq, meth, "extrap");
end
