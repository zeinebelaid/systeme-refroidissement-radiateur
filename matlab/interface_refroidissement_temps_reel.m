function interface_refroidissement_temps_reel
% Tableau de bord anime du modele Simulink existant.
% Simulation Simulink d'abord, puis lecture animee des resultats.
% MATLAB + Simulink. Aucune connexion ROS 2 dans cette version.
mdl = 'refroidissement_intelligent_final';
if ~bdIsLoaded(mdl) && ~exist([mdl '.slx'],'file')
    error('Modele %s.slx introuvable. Lancez creer_modele_final.m.',mdl);
end
load_system(mdl);
names = {'Autoroute 120 km/h','Circulation urbaine','Embouteillage', ...
 'Feu rouge','Montee forte charge','Descente','Canicule','Hiver', ...
 'Acceleration','Ralentissement','Arret moteur','Panne ventilateur', ...
 'Cycle complet','Autoroute - bouchon - autoroute','Manuel'};
S = struct('time',[],'Tm',[],'Tl',[],'Tr',[],'v',[],'pwm',[], ...
    'th',[],'alarm',[],'power',[],'energy',[]);
idx=1; playing=false; tick=[]; timerObj=[]; playbackStep=5;
C.bg=[.055 .09 .15]; C.panel=[.10 .15 .23]; C.blue=[.25 .63 .95];
C.orange=[1 .58 .27]; C.green=[.24 .78 .56]; C.white=[.94 .96 1];
f=figure('Name','Refroidissement automobile | Tableau de bord Simulink', ...
 'NumberTitle','off','MenuBar','none','ToolBar','none','Color',C.bg, ...
 'Position',[60 50 1500 840],'CloseRequestFcn',@closeUI);
% Layout in normalized coordinates for resizing.
annotation(f,'textbox',[.02 .935 .96 .055],'String', ...
 'REFROIDISSEMENT AUTOMOBILE  |  SIMULINK DIGITAL DASHBOARD', ...
 'EdgeColor','none','Color',C.white,'FontWeight','bold','FontSize',18);
annotation(f,'textbox',[.02 .903 .96 .03],'String', ...
 'Lecture animee des resultats Simulink  •  valeurs de simulation, non mesures vehicule', ...
 'EdgeColor','none','Color',[.63 .72 .84],'FontSize',10);
% Sidebar
panel=uipanel(f,'Title',' CONDUITE ET SCENARIOS ','Units','normalized', ...
 'Position',[.02 .12 .27 .77],'BackgroundColor',C.panel, ...
 'ForegroundColor',C.white,'FontWeight','bold','FontSize',11);
label(panel,[.05 .92 .9 .045],'Scenario');
popup=uicontrol(panel,'Style','popupmenu','Units','normalized', ...
 'Position',[.05 .865 .9 .06],'String',names,'FontSize',10, ...
 'BackgroundColor',[1 1 1]);
label(panel,[.05 .785 .9 .04],'Temperature exterieure (C)');
ambient=uicontrol(panel,'Style','slider','Units','normalized', ...
 'Position',[.05 .735 .65 .05],'Min',-10,'Max',50,'Value',25, ...
 'Callback',@updateLabels);
ambientText=label(panel,[.73 .735 .23 .05],'25 C');
label(panel,[.05 .675 .9 .04],'Charge moteur (%), mode manuel');
loadSlider=uicontrol(panel,'Style','slider','Units','normalized', ...
 'Position',[.05 .625 .65 .05],'Min',0,'Max',100,'Value',40, ...
 'Callback',@updateLabels);
loadText=label(panel,[.73 .625 .23 .05],'40 %');
label(panel,[.05 .565 .9 .04],'Vitesse (km/h), mode manuel');
speedSlider=uicontrol(panel,'Style','slider','Units','normalized', ...
 'Position',[.05 .515 .65 .05],'Min',0,'Max',140,'Value',60, ...
 'Callback',@updateLabels);
speedText=label(panel,[.73 .515 .23 .05],'60');
label(panel,[.05 .455 .9 .04],'Commande ventilateur');
mode=uicontrol(panel,'Style','popupmenu','Units','normalized', ...
 'Position',[.05 .405 .9 .05],'String',{'PI automatique','Ventilateur OFF (test)'}, ...
 'BackgroundColor',[1 1 1]);
runBtn=uicontrol(panel,'Style','pushbutton','Units','normalized', ...
 'Position',[.05 .305 .9 .075],'String','SIMULER ET AFFICHER', ...
 'FontSize',12,'FontWeight','bold','BackgroundColor',C.green, ...
 'Callback',@simulateNow);
playBtn=uicontrol(panel,'Style','pushbutton','Units','normalized', ...
 'Position',[.05 .22 .43 .065],'String','PAUSE','Callback',@togglePlay);
uicontrol(panel,'Style','pushbutton','Units','normalized', ...
 'Position',[.52 .22 .43 .065],'String','RECOMMENCER','Callback',@restart);
uicontrol(panel,'Style','pushbutton','Units','normalized', ...
 'Position',[.05 .13 .9 .06],'String','OUVRIR MODELE SIMULINK', ...
 'Callback',@(~,~)open_system(mdl));
status=label(panel,[.05 .015 .9 .09],'Pret : choisissez un scenario.');
% Center: simplified cooling loop diagram
axDiagram=axes('Parent',f,'Units','normalized','Position',[.315 .49 .40 .39], ...
 'Color',C.panel,'XColor',C.panel,'YColor',C.panel,'XLim',[0 10], ...
 'YLim',[0 7],'XTick',[],'YTick',[]); hold(axDiagram,'on');
rectangle(axDiagram,'Position',[.5 2.0 2.6 3.2],'Curvature',.1, ...
 'FaceColor',[.18 .43 .73],'EdgeColor',C.blue,'LineWidth',2);
text(axDiagram,1.8,3.65,'MOTEUR','Color','w','HorizontalAlignment','center','FontWeight','bold');
rectangle(axDiagram,'Position',[7.2 2.15 1.25 3.0], ...
 'FaceColor',[.15 .46 .75],'EdgeColor',C.blue,'LineWidth',2);
for x=7.35:.19:8.35
 line(axDiagram,[x x],[2.3 5],'Color',[.68 .83 .97]);
end
text(axDiagram,7.8,5.5,'RADIATEUR','Color',C.white,'HorizontalAlignment','center');
plot(axDiagram,[3.1 7.2],[4.65 4.65],'LineWidth',9,'Color',C.blue);
plot(axDiagram,[3.1 7.2],[2.35 2.35],'LineWidth',9,'Color',C.blue);
text(axDiagram,5.1,5.2,'Thermostat','Color',C.white,'HorizontalAlignment','center');
text(axDiagram,5.1,1.7,'Pompe + liquide','Color',C.white,'HorizontalAlignment','center');
text(axDiagram,6.0,3.6,'VENTILATEUR','Color',C.white,'HorizontalAlignment','center');
fanGraphic=plot(axDiagram,6,3.0,'o','MarkerSize',25,'MarkerEdgeColor',C.orange,'LineWidth',3);
flowGraphic=plot(axDiagram,4.7,4.65,'o','MarkerFaceColor',C.green,'MarkerEdgeColor',C.green,'MarkerSize',8);
% Right: numerical gauges
right=uipanel(f,'Title',' TABLEAU DE BORD ','Units','normalized', ...
 'Position',[.73 .49 .25 .40],'BackgroundColor',C.panel, ...
 'ForegroundColor',C.white,'FontWeight','bold','FontSize',11);
readings=cell(6,1);
titles={'VITESSE','MOTEUR','LIQUIDE','RADIATEUR','VENTILATEUR','ENERGIE'};
for k=1:6
 row=floor((k-1)/2); col=mod(k-1,2);
 x=.04+col*.49; y=.68-row*.31;
 label(right,[x y+.17 .45 .08],titles{k});
 readings{k}=uicontrol(right,'Style','text','Units','normalized', ...
 'Position',[x y .45 .17],'String','--','BackgroundColor',C.panel, ...
 'ForegroundColor',C.green,'FontSize',18,'FontWeight','bold');
end
% Bottom plots
axT=axes('Parent',f,'Units','normalized','Position',[.34 .275 .62 .18], ...
 'Color',C.panel,'XColor',C.white,'YColor',C.white);
axP=axes('Parent',f,'Units','normalized','Position',[.34 .075 .62 .15], ...
 'Color',C.panel,'XColor',C.white,'YColor',C.white);
set([axT axP],'FontSize',9);grid(axT,'on');grid(axP,'on');
ylabel(axT,'Temperature (C)','Color',C.white);
ylabel(axP,'PWM (%)','Color',C.white);xlabel(axP,'Temps simule (s)','Color',C.white);
% footer
annotation(f,'textbox',[.02 .015 .96 .035], ...
 'String','Architecture ROS 2 a venir : /cooling/coolant_temperature, /cooling/fan_pwm, /vehicle/speed', ...
 'EdgeColor','none','Color',[.63 .72 .84],'FontSize',9);

    function h=label(parent,pos,str)
        h=uicontrol(parent,'Style','text','Units','normalized','Position',pos, ...
            'String',str,'HorizontalAlignment','left', ...
            'BackgroundColor',C.panel,'ForegroundColor',C.white,'FontSize',10);
    end
    function updateLabels(~,~)
        set(ambientText,'String',sprintf('%.0f C',ambient.Value));
        set(loadText,'String',sprintf('%.0f %%',loadSlider.Value));
        set(speedText,'String',sprintf('%.0f',speedSlider.Value));
    end
    function simulateNow(~,~)
        stopTimer(); playing=false;
        n=popup.Value; [sc,tend]=scenarioData(n);
        if n==15 % manual
            sc=[0 speedSlider.Value loadSlider.Value ambient.Value 1 0 1; ...
                900 speedSlider.Value loadSlider.Value ambient.Value 1 0 1];tend=900;
        end
        varNames={'speed_ts','load_ts','ambient_ts','pump_ts','fault_ts','engine_ts'};
        for j=1:6
            assignin('base',varNames{j},timeseries(sc(:,j+1),sc(:,1)));
        end
        set(status,'String','Simulation Simulink en cours...');drawnow;
        try
            if mode.Value==2
                % Essai ventilateur OFF : simuler une panne volontaire.
                assignin('base','fault_ts',timeseries(ones(size(sc,1),1),sc(:,1)));
            end
            out=sim(mdl,'StopTime',num2str(tend),'ReturnWorkspaceOutputs','on');
            nm={'Tm','Tl','Tr','vitesse','PWM','thermostat','alarme','Pfan_W','Efan_kWh'};
            signals=cell(size(nm));
            for j=1:numel(nm), signals{j}=out.get(nm{j}); end
            S.time=signals{2}.Time(:);
            for j=1:numel(nm)
                vals=signals{j}.Data(:);
                if numel(signals{j}.Time)==numel(S.time) && all(signals{j}.Time(:)==S.time)
                    values=vals;
                else
                    values=interp1(signals{j}.Time(:),vals,S.time,'previous','extrap');
                end
                fields={'Tm','Tl','Tr','v','pwm','th','alarm','power','energy'};
                S.(fields{j})=values;
            end
            idx=1; playing=true;
            set(status,'String',sprintf('%s | %.0f s | Lecture animee',names{n},tend));
            refreshView();
            timerObj=timer('ExecutionMode','fixedSpacing','Period',.08, ...
                'BusyMode','drop','TimerFcn',@advance);
            start(timerObj);set(playBtn,'String','PAUSE');
        catch ME
            set(status,'String',['Erreur : ' ME.message]);
            fprintf(2,'%s\n',getReport(ME,'basic'));
        end
    end
    function advance(~,~)
        if ~ishandle(f),stopTimer();return;end
        if ~playing || isempty(S.time),return;end
        target=S.time(idx)+playbackStep;
        next=find(S.time>=target,1,'first');
        if isempty(next)
            idx=numel(S.time);playing=false;stopTimer();
            set(playBtn,'String','REPRENDRE');
        else
            idx=next;
        end
        refreshView();
    end
    function refreshView()
        if isempty(S.time),return;end
        j=idx;t=S.time(1:j);
        set(readings{1},'String',sprintf('%.0f km/h',S.v(j)));
        set(readings{2},'String',sprintf('%.1f C',S.Tm(j)));
        set(readings{3},'String',sprintf('%.1f C',S.Tl(j)));
        set(readings{4},'String',sprintf('%.1f C',S.Tr(j)));
        set(readings{5},'String',sprintf('%.0f %%',100*S.pwm(j)));
        set(readings{6},'String',sprintf('%.4f kWh',S.energy(j)));
        cla(axT);plot(axT,t,S.Tm(1:j),'Color',C.orange,'LineWidth',1.5);hold(axT,'on');
        plot(axT,t,S.Tl(1:j),'Color',C.blue,'LineWidth',1.5);
        plot(axT,t,S.Tr(1:j),'Color',C.green,'LineWidth',1.5);
        yline(axT,90,'--','Color',[.8 .8 .8]);hold(axT,'off');
        legend(axT,{'Moteur','Liquide','Radiateur','Consigne'},'TextColor',C.white, ...
            'Color',C.panel,'Location','northwest','FontSize',8);
        xlim(axT,[0 max(1,S.time(end))]);grid(axT,'on');
        cla(axP);plot(axP,t,100*S.pwm(1:j),'Color',C.orange,'LineWidth',1.6);
        ylim(axP,[0 105]);xlim(axP,[0 max(1,S.time(end))]);grid(axP,'on');
        set(fanGraphic,'MarkerSize',20+15*S.pwm(j));
        set(flowGraphic,'XData',3.3+3.5*mod(S.time(j)/30,1));
        if S.alarm(j)>.5
            set(status,'String',sprintf('ALARME SURCHAUFFE | t = %.0f s',S.time(j)));
        else
            set(status,'String',sprintf('Temps simule : %.0f / %.0f s | thermostat %.0f %%', ...
                S.time(j),S.time(end),100*S.th(j)));
        end
        drawnow limitrate;
    end
    function togglePlay(~,~)
        if isempty(S.time),return;end
        playing=~playing;
        if playing
            if idx==numel(S.time),idx=1;end
            if isempty(timerObj) || ~isvalid(timerObj)
                timerObj=timer('ExecutionMode','fixedSpacing','Period',.08, ...
                    'BusyMode','drop','TimerFcn',@advance);start(timerObj);
            end
            set(playBtn,'String','PAUSE');
        else
            set(playBtn,'String','REPRENDRE');
        end
    end
    function restart(~,~)
        if isempty(S.time),return;end
        idx=1;playing=false;set(playBtn,'String','REPRENDRE');refreshView();
    end
    function stopTimer()
        if ~isempty(timerObj) && isvalid(timerObj)
            stop(timerObj);delete(timerObj);
        end
        timerObj=[];
    end
    function closeUI(~,~)
        stopTimer();delete(f);
    end
end

function [sc,tend]=scenarioData(n)
% temps, vitesse, charge, ambiant, pompe, panne, moteur actif
switch n
 case 1, x=[120 55 25 1 0 1];
 case 2, x=[35 35 25 1 0 1];
 case 3, x=[0 30 35 1 0 1];
 case 4, x=[0 15 25 1 0 1];
 case 5, x=[45 95 30 1 0 1];
 case 6, x=[85 12 24 1 0 1];
 case 7, x=[20 60 43 1 0 1];
 case 8, x=[70 40 0 1 0 1];
 case 9, x=[20 30 25 1 0 1];
 case 10,x=[120 55 25 1 0 1];
 case 11,x=[0 0 25 1 0 0];
 case 12,x=[0 30 35 1 1 1];
 case 13
     sc=[0 0 15 22 1 0 1;120 35 35 22 1 0 1;300 0 30 30 1 0 1; ...
         480 120 55 25 1 0 1;690 45 95 30 1 0 1;850 85 12 24 1 0 1; ...
         980 20 45 42 1 0 1;1120 0 35 38 1 0 1;1260 100 60 27 1 0 1; ...
         1420 15 55 30 1 0 1;1570 0 25 28 1 1 1;1690 0 20 26 1 0 1; ...
         1800 0 20 26 1 0 1];tend=1800;return
 case 14
     sc=[0 120 55 25 1 0 1;300 0 30 35 1 0 1;600 120 55 25 1 0 1; ...
         900 120 55 25 1 0 1];tend=900;return
 otherwise,x=[60 40 25 1 0 1];
end
if n==9
    sc=[0 x;300 70 60 25 1 0 1;600 120 85 25 1 0 1;900 120 85 25 1 0 1];
elseif n==10
    sc=[0 x;300 70 40 25 1 0 1;600 0 15 25 1 0 1;900 0 15 25 1 0 1];
else
    sc=[0 x;900 x];
end
tend=900;
end
