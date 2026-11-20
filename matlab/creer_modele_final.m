function creer_modele_final
% Modele pedagogique de refroidissement automobile, sans Simscape.
% Genere un SLX organise et une interface de scenarios externe.
mdl='refroidissement_intelligent_final';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);
set_param(mdl,'StopTime','1200','Solver','ode45','MaxStep','1');
P.Cm=90000; P.Cl=35000; P.Cr=18000;
P.Kmc=550; P.Kcr=380; P.UA0=35; P.Kv=3; P.Kfan=210;
P.Qbase=1700; P.Qload=170; P.Tref=90; P.Ttherm0=78; P.Ttherm1=88;
P.Kp=.09; P.Ki=.0018; P.Kaw=.1; P.Pfan=280;
P.Tm0=98; P.Tl0=90; P.Tr0=75;
assignin('base','P',P);
% Profil par defaut : autoroute, embouteillage, autoroute
assignin('base','speed_ts',timeseries([120;0;120;120],[0;300;600;1200]));
assignin('base','load_ts',timeseries([55;30;55;55],[0;300;600;1200]));
assignin('base','ambient_ts',timeseries([25;35;25;25],[0;300;600;1200]));
assignin('base','pump_ts',timeseries(ones(4,1),[0;300;600;1200]));
assignin('base','fault_ts',timeseries(zeros(4,1),[0;300;600;1200]));
assignin('base','engine_ts',timeseries(ones(4,1),[0;300;600;1200]));
% 6 entrees From Workspace avec maintien derniere valeur
names={'Vitesse','Charge','Air_ambiant','Pompe_active','Panne_ventilateur','Moteur_actif'};
vars={'speed_ts','load_ts','ambient_ts','pump_ts','fault_ts','engine_ts'};
for i=1:6
    block(mdl,'simulink/Sources/From Workspace',names{i},[40 55+70*(i-1) 165 90+70*(i-1)], ...
        'VariableName',vars{i},'Interpolate','off','OutputAfterFinalValue','Holding final value');
end
% Sous-systemes: fonctions via blocs standards et Fcn u(index) sans variables P dans Fcn
% Production thermique
block(mdl,'simulink/Math Operations/Gain','Gain_charge',[225 125 305 160],'Gain','P.Qload');
block(mdl,'simulink/Sources/Constant','Chaleur_base',[225 185 305 215],'Value','P.Qbase');
block(mdl,'simulink/Math Operations/Sum','Chaleur_totale',[345 145 375 195],'Inputs','++');
block(mdl,'simulink/Math Operations/Product','Chaleur_moteur',[415 145 450 195]);
wire(mdl,'Charge/1','Gain_charge/1'); wire(mdl,'Gain_charge/1','Chaleur_totale/1');
wire(mdl,'Chaleur_base/1','Chaleur_totale/2'); wire(mdl,'Chaleur_totale/1','Chaleur_moteur/1');
wire(mdl,'Moteur_actif/1','Chaleur_moteur/2');
% Temperature states
for i=1:3
    n={'Temperature_moteur','Temperature_liquide','Temperature_radiateur'};
    ic={'P.Tm0','P.Tl0','P.Tr0'};
    block(mdl,'simulink/Continuous/Integrator',n{i},[1180 110+180*(i-1) 1215 145+180*(i-1)],'InitialCondition',ic{i});
end
% Echange moteur -> liquide (Tm-Tl)*Kmc*pump
block(mdl,'simulink/Math Operations/Sum','Delta_moteur_liquide',[515 130 545 180],'Inputs','+-');
block(mdl,'simulink/Math Operations/Gain','K_moteur_liquide',[590 130 680 165],'Gain','P.Kmc');
block(mdl,'simulink/Math Operations/Product','Q_moteur_liquide',[730 130 770 170]);
wire(mdl,'Temperature_moteur/1','Delta_moteur_liquide/1'); wire(mdl,'Temperature_liquide/1','Delta_moteur_liquide/2');
wire(mdl,'Delta_moteur_liquide/1','K_moteur_liquide/1'); wire(mdl,'K_moteur_liquide/1','Q_moteur_liquide/1');
wire(mdl,'Pompe_active/1','Q_moteur_liquide/2');
% Thermostat
block(mdl,'simulink/Math Operations/Bias','Ecart_thermostat',[230 520 315 555],'Bias','-P.Ttherm0');
block(mdl,'simulink/Math Operations/Gain','Normaliser_thermostat',[355 520 450 555],'Gain','1/(P.Ttherm1-P.Ttherm0)');
block(mdl,'simulink/Discontinuities/Saturation','Thermostat_0_1',[490 520 580 555],'LowerLimit','0','UpperLimit','1');
wire(mdl,'Temperature_liquide/1','Ecart_thermostat/1');wire(mdl,'Ecart_thermostat/1','Normaliser_thermostat/1');wire(mdl,'Normaliser_thermostat/1','Thermostat_0_1/1');
% Echange liquide radiateur
block(mdl,'simulink/Math Operations/Sum','Delta_liquide_radiateur',[515 300 545 350],'Inputs','+-');
block(mdl,'simulink/Math Operations/Gain','K_liquide_radiateur',[590 305 680 340],'Gain','P.Kcr');
block(mdl,'simulink/Math Operations/Product','Q_liquide_radiateur',[730 300 775 355],'Inputs','***');
wire(mdl,'Temperature_liquide/1','Delta_liquide_radiateur/1');wire(mdl,'Temperature_radiateur/1','Delta_liquide_radiateur/2');
wire(mdl,'Delta_liquide_radiateur/1','K_liquide_radiateur/1');wire(mdl,'K_liquide_radiateur/1','Q_liquide_radiateur/1');
wire(mdl,'Thermostat_0_1/1','Q_liquide_radiateur/2');wire(mdl,'Pompe_active/1','Q_liquide_radiateur/3');
% PI antiwindup
block(mdl,'simulink/Sources/Constant','Consigne_liquide',[230 680 310 710],'Value','P.Tref');
block(mdl,'simulink/Math Operations/Sum','Erreur_PI',[350 655 380 710],'Inputs','+-');
wire(mdl,'Temperature_liquide/1','Erreur_PI/1');wire(mdl,'Consigne_liquide/1','Erreur_PI/2');
block(mdl,'simulink/Math Operations/Gain','Kp',[430 625 500 655],'Gain','P.Kp');
block(mdl,'simulink/Math Operations/Gain','Ki',[430 745 500 775],'Gain','P.Ki');
block(mdl,'simulink/Math Operations/Sum','Somme_integrale',[585 725 615 775],'Inputs','++');
block(mdl,'simulink/Continuous/Integrator','Etat_integral',[650 730 685 765],'InitialCondition','0');
block(mdl,'simulink/Math Operations/Sum','PI_brut',[730 645 760 700],'Inputs','++');
block(mdl,'simulink/Discontinuities/Saturation','PWM_limite',[805 650 895 690],'LowerLimit','0','UpperLimit','1');
block(mdl,'simulink/Math Operations/Sum','Ecart_saturation',[925 745 955 795],'Inputs','+-');
block(mdl,'simulink/Math Operations/Gain','Anti_windup',[995 755 1080 785],'Gain','P.Kaw');
wire(mdl,'Erreur_PI/1','Kp/1');wire(mdl,'Erreur_PI/1','Ki/1');wire(mdl,'Ki/1','Somme_integrale/1');
wire(mdl,'Anti_windup/1','Somme_integrale/2');wire(mdl,'Somme_integrale/1','Etat_integral/1');
wire(mdl,'Kp/1','PI_brut/1');wire(mdl,'Etat_integral/1','PI_brut/2');wire(mdl,'PI_brut/1','PWM_limite/1');
wire(mdl,'PWM_limite/1','Ecart_saturation/1');wire(mdl,'PI_brut/1','Ecart_saturation/2');wire(mdl,'Ecart_saturation/1','Anti_windup/1');
% Fan fault: 1-fault
block(mdl,'simulink/Math Operations/Bias','Panne_moins_1',[230 430 320 465],'Bias','-1');
block(mdl,'simulink/Math Operations/Gain','Disponibilite_ventilateur',[360 430 455 465],'Gain','-1');
block(mdl,'simulink/Math Operations/Product','PWM_effectif',[1130 650 1170 695]);
wire(mdl,'Panne_ventilateur/1','Panne_moins_1/1');wire(mdl,'Panne_moins_1/1','Disponibilite_ventilateur/1');
wire(mdl,'PWM_limite/1','PWM_effectif/1');wire(mdl,'Disponibilite_ventilateur/1','PWM_effectif/2');
% Air dynamique et ventilateur
block(mdl,'simulink/Math Operations/Gain','UA_vitesse',[230 55 315 90],'Gain','P.Kv');
block(mdl,'simulink/Math Operations/Gain','UA_ventilateur',[1240 650 1330 685],'Gain','P.Kfan');
block(mdl,'simulink/Sources/Constant','UA_passif',[920 430 995 460],'Value','P.UA0');
block(mdl,'simulink/Math Operations/Sum','UA_total',[1370 390 1400 460],'Inputs','+++');
wire(mdl,'Vitesse/1','UA_vitesse/1');wire(mdl,'PWM_effectif/1','UA_ventilateur/1');
wire(mdl,'UA_vitesse/1','UA_total/1');wire(mdl,'UA_passif/1','UA_total/2');wire(mdl,'UA_ventilateur/1','UA_total/3');
block(mdl,'simulink/Math Operations/Sum','Delta_radiateur_air',[1450 400 1480 450],'Inputs','+-');
block(mdl,'simulink/Math Operations/Product','Q_radiateur_air',[1520 400 1560 450]);
wire(mdl,'Temperature_radiateur/1','Delta_radiateur_air/1');wire(mdl,'Air_ambiant/1','Delta_radiateur_air/2');
wire(mdl,'Delta_radiateur_air/1','Q_radiateur_air/1');wire(mdl,'UA_total/1','Q_radiateur_air/2');
% Bilans
balance(mdl,'moteur',{'Chaleur_moteur/1','Q_moteur_liquide/1'},'P.Cm','Temperature_moteur/1',[930 125]);
balance(mdl,'liquide',{'Q_moteur_liquide/1','Q_liquide_radiateur/1'},'P.Cl','Temperature_liquide/1',[930 305]);
balance(mdl,'radiateur',{'Q_liquide_radiateur/1','Q_radiateur_air/1'},'P.Cr','Temperature_radiateur/1',[1640 500]);
% Energie
block(mdl,'simulink/Math Operations/Math Function','PWM_carre',[1230 800 1305 835],'Operator','square');
block(mdl,'simulink/Math Operations/Product','PWM_cube',[1350 800 1390 840]);
block(mdl,'simulink/Math Operations/Gain','P_ventilateur_W',[1430 800 1520 835],'Gain','P.Pfan');
block(mdl,'simulink/Math Operations/Gain','Conversion_kWh',[1560 800 1650 835],'Gain','1/3600000');
block(mdl,'simulink/Continuous/Integrator','Energie_kWh',[1690 800 1730 835],'InitialCondition','0');
wire(mdl,'PWM_effectif/1','PWM_carre/1');wire(mdl,'PWM_carre/1','PWM_cube/1');wire(mdl,'PWM_effectif/1','PWM_cube/2');
wire(mdl,'PWM_cube/1','P_ventilateur_W/1');wire(mdl,'P_ventilateur_W/1','Conversion_kWh/1');wire(mdl,'Conversion_kWh/1','Energie_kWh/1');
% Alarme
block(mdl,'simulink/Sources/Constant','Seuil_alarme_105',[1350 580 1420 610],'Value','105');
block(mdl,'simulink/Logic and Bit Operations/Relational Operator','Alarme_surchauffe',[1480 565 1550 610],'Operator','>');
wire(mdl,'Temperature_liquide/1','Alarme_surchauffe/1');wire(mdl,'Seuil_alarme_105/1','Alarme_surchauffe/2');
% Logs et Scope
outs={'Temperature_moteur','Temperature_liquide','Temperature_radiateur','Vitesse','Charge','Air_ambiant','PWM_effectif','Thermostat_0_1','P_ventilateur_W','Energie_kWh','Alarme_surchauffe','Q_radiateur_air'};
varnames={'Tm','Tl','Tr','vitesse','charge','Tamb','PWM','thermostat','Pfan_W','Efan_kWh','alarme','Qrad_W'};
for k=1:numel(outs)
    y=60+(k-1)*62;
    block(mdl,'simulink/Sinks/To Workspace',['LOG_' varnames{k}],[1880 y 2005 y+35], 'VariableName',varnames{k},'SaveFormat','Timeseries');
    wire(mdl,[outs{k} '/1'],['LOG_' varnames{k} '/1']);
end
for j=1:3
    labels={'Temperatures','Vitesse_PWM','Puissance_Energie'};
    inputs={{'Temperature_moteur/1','Temperature_liquide/1','Temperature_radiateur/1'}, ...
            {'Vitesse/1','PWM_effectif/1'}, {'P_ventilateur_W/1','Energie_kWh/1'}};
    ss=inputs{j};y=880+(j-1)*115;
    block(mdl,'simulink/Signal Routing/Mux',['Mux_' labels{j}],[1780 y 1785 y+55],'Inputs',num2str(numel(ss)));
    block(mdl,'simulink/Sinks/Scope',['Scope_' labels{j}],[1840 y 1905 y+55]);
    for k=1:numel(ss),wire(mdl,ss{k},sprintf('Mux_%s/%d',labels{j},k));end
    wire(mdl,['Mux_' labels{j} '/1'],['Scope_' labels{j} '/1']);
end
% Presentation
Simulink.Annotation(mdl,'MODELE PEDAGOGIQUE - Valeurs thermiques illustratives. Choisir les scenarios via interface_scenarios_final.');
save_system(mdl,[mdl '.slx']);
open_system(mdl);
fprintf('Modele cree : %s.slx\nLancer ensuite : interface_scenarios_final\n',mdl);
end
function block(mdl,lib,name,pos,varargin)
add_block(lib,[mdl '/' name],'Position',pos,varargin{:});
end
function wire(mdl,src,dst)
add_line(mdl,src,dst,'autorouting','on');
end
function balance(mdl,name,signals,C,integrator,pos)
x=pos(1);y=pos(2);
block(mdl,'simulink/Math Operations/Sum',['Bilan_' name],[x y x+30 y+45],'Inputs','+-');
block(mdl,'simulink/Math Operations/Gain',['Inv_C_' name],[x+65 y+5 x+150 y+40],'Gain',['1/' C]);
wire(mdl,signals{1},['Bilan_' name '/1']);wire(mdl,signals{2},['Bilan_' name '/2']);
wire(mdl,['Bilan_' name '/1'],['Inv_C_' name '/1']);wire(mdl,['Inv_C_' name '/1'],integrator);
end
