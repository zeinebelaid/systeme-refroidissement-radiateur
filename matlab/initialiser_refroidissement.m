function P = initialiser_refroidissement(mdl)
% Parametres pedagogiques, non calibres sur un vehicule reel.
if nargin < 1, mdl = 'refroidissement_intelligent_final'; end
load_system(mdl);
P = struct('Cm',90000,'Cl',35000,'Cr',18000, ...
 'Kmc',550,'Kcr',380,'UA0',35,'Kv',3,'Kfan',210, ...
 'Qbase',1700,'Qload',170,'Tref',90, ...
 'Ttherm0',78,'Ttherm1',88, ...
 'Kp',0.09,'Ki',0.0018,'Kaw',0.1,'Pfan',280, ...
 'Tm0',98,'Tl0',90,'Tr0',75);
% Reutiliser les valeurs deja presentes si elles sont disponibles.
mw = get_param(mdl,'ModelWorkspace');
if hasVariable(mw,'P')
    oldP = getVariable(mw,'P');
elseif evalin('base','exist(''P'',''var'')')
    oldP = evalin('base','P');
else
    oldP = struct();
end
if isstruct(oldP)
    fn = fieldnames(oldP);
    for k=1:numel(fn), P.(fn{k}) = oldP.(fn{k}); end
end
assignin(mw,'P',P);
assignin('base','P',P);
% Conserver les valeurs finales des profils discrets.
names = {'Vitesse','Charge','Air_ambiant','Pompe_active','Panne_ventilateur','Moteur_actif'};
for k=1:numel(names)
    path = [mdl '/' names{k}];
    if getSimulinkBlockHandle(path) ~= -1
        set_param(path,'Interpolate','off','OutputAfterFinalValue','Holding final value');
    end
end
save_system(mdl);
end
