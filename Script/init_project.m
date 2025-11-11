function P = init_project()
% =========================================================================
%  Projet : Robot Aspirateur Autonome
%  Fichier : init_project.m
%  Auteur  : [Ton nom]
%  Date    : [date]
%
%  Rôle :
%  Initialise tous les paramètres physiques, numériques et environnementaux
%  nécessaires à la simulation. Centralise la configuration pour garantir
%  reproductibilité et clarté. A appeler en début de session ou depuis
%  run_simulation.m.
% =========================================================================

%% 1) Initialisation de l'environnement MATLAB
close all;            % Fermer les figures ouvertes
clc;                  % Nettoyer la console
clearvars -except P;  % Nettoyer l'espace de travail (sauf P si déjà existant)
rng(42);              % Graine aléatoire pour la reproductibilité

% Ajout des chemins de façon robuste (selon l'arborescence locale)
root = fileparts(mfilename('fullpath'));  % dossier où se trouve ce fichier
folders = {'Models','Scripts','Scripts/Utils','Data'};
for i = 1:numel(folders)
    p = fullfile(root, '..', folders{i}); % si init_project.m est dans Scripts/, remonter d'un cran
    if exist(p, 'dir')
        addpath(genpath(p));
    end
end

% Créer les dossiers Data/ et Output/ s'ils n'existent pas (au même niveau que Scripts/)
dataDir = fullfile(root, '..', 'Data');
if ~exist(dataDir,'dir'), mkdir(dataDir); end
outDir = fullfile(root, '..', 'Output');
if ~exist(outDir,'dir'), mkdir(outDir); end

disp('--- Initialisation du projet de robot mobile ---');

%% 2) Monde de simulation (salle + obstacle)
% Salle carrée centrée en (0,0) de 20 m de côté (soit [-10,10] x [-10,10])
P.room.half = 10;                  % Demi-longueur de la salle (m)
P.room.size = 2 * P.room.half;     % Longueur totale (m)

% Obstacle cylindrique : centre (3,3), rayon 0.25 m
P.obs.center = [3, 3];             % Coordonnées (m)
P.obs.R = 0.25;                    % Rayon (m)

%% 3) Robot (modèle unicycle, disque de rayon 0.2 m)
P.robot.radius   = 0.2;            % Rayon du robot (m) pour les collisions
P.robot.v_max    = 0.3;            % Vitesse linéaire maximale (m/s)
P.robot.omega_max= 1.5;            % Vitesse angulaire maximale (rad/s)

% Discrétisation temporelle
P.sim.dt = 0.1;                    % Pas de temps de simulation (s)
P.sim.T  = 3000;                   % Durée totale (s)

% Pose initiale aléatoire (dans 80% de la salle pour éviter d'être collé aux murs)
P.init.x0    = (rand*2 - 1) * P.room.half * 0.8;
P.init.y0    = (rand*2 - 1) * P.room.half * 0.8;
P.init.theta0= rand*2*pi - pi;     % Orientation initiale (rad)

%% 4) Capteur ultrason (mesure sur l'axe X du robot, seulement en marche avant)
% Le capteur renvoie la distance au premier obstacle dans la direction
% de l'axe X du robot. La consigne "forward-only" implique que la mesure
% est considérée valide uniquement lorsque la commande de vitesse v > 0.
P.sensor.d_max      = 3.0;         % Portée maximale du capteur (m)
P.sensor.d_th       = 0.5;         % Seuil d'évitement (m)
P.sensor.noise_std  = 0.02;        % Bruit (écart-type) simulé (m)
P.sensor.forward_only = true;      % Actif uniquement quand v_cmd > 0
P.sensor.invalid_value = inf;      % Valeur renvoyée si capteur inactif (inf ou NaN)

%% 5) Grille de couverture (mémoire de passage)
% Grille N x N couvrant la salle. Chaque cellule accumule le nombre de
% visites. Sert à calculer la couverture et à biaiser l'exploration.
P.grid.N = 200;                             % Résolution de la grille
P.grid.x = linspace(-P.room.half, P.room.half, P.grid.N);
P.grid.y = linspace(-P.room.half, P.room.half, P.grid.N);

%% 6) Paramètres de la stratégie de contrôle (hybride réactive + stochastique + mémoire)
% w_nov  : poids de la "nouveauté" (préférence pour les zones peu visitées)
% w_rot  : pénalisation de la rotation (favorise le mouvement en avant)
% sector : ouverture angulaire autour de l'axe avant pour échantillonner des directions
% temp   : température du softmax (plus petit => choix plus déterministe)
% stag_T : durée de stagnation avant reorientation plus forte
P.ctrl.w_nov        = 1.0;
P.ctrl.w_rot        = 0.3;
P.ctrl.sector_deg   = 120;
P.ctrl.temp         = 1.0;
P.ctrl.stagnation_T = 7.0;

%% 7) Dossiers de sortie et sauvegarde de la configuration
save(fullfile(outDir,'config_last.mat'),'P');
disp(['Configuration sauvegardée : ' fullfile(outDir,'config_last.mat')]);

end
