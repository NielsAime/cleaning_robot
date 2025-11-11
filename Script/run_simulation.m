function run_simulation()
% ========================================================================
%  Projet  : Robot Aspirateur Autonome
%  Fichier : run_simulation.m
%  Auteur  : [Ton nom]
%  Date    : [date]
%
%  Rôle :
%  Lance une simulation en MATLAB (sans Simulink) de la stratégie hybride :
%    - évitement réactif (capteur ultrason frontal)
%    - exploration stochastique biaisée par la "nouveauté"
%    - mémoire de passage via une grille de couverture
% ========================================================================

% 1) Charger la configuration
if exist('init_project','file') ~= 2
    error('init_project.m introuvable. Place ce fichier dans Scripts/ et ajoute le chemin.');
end
P = init_project();

% 2) Préparer les structures de simulation
dt   = P.sim.dt;
T    = P.sim.T;
Nstp = floor(T/dt);

% Pose (x,y,theta)
x     = P.init.x0;
y     = P.init.y0;
theta = P.init.theta0;

% Grille de couverture (matrice d'entiers)
cover = zeros(P.grid.N, P.grid.N, 'uint16');

% Historique pour affichage
xh = zeros(Nstp,1);
yh = zeros(Nstp,1);
cov_hist = zeros(Nstp,1);
time = (0:Nstp-1)' * dt;

% 3) Constantes de contrôle
v_max     = P.robot.v_max;
om_max    = P.robot.omega_max;
d_th      = P.sensor.d_th;
d_max     = P.sensor.d_max;
sigma_meas= P.sensor.noise_std;
sector    = deg2rad(P.ctrl.sector_deg);
w_nov     = P.ctrl.w_nov;
w_rot     = P.ctrl.w_rot;
temp      = P.ctrl.temp;
stagT     = P.ctrl.stagnation_T;

% Paramètres divers
lookahead = 1.5;       % Distance de prospection (m) pour l'évaluation de nouveauté
K_heading = 13;        % Nombre de directions candidates dans le secteur avant
K_heading = max(3, K_heading);
decay_vis = 0.0;       % Décroissance des visites (0 = pas d'oubli)
min_step  = 1e-3;      % Seuil pour détecter stagnation
stagn_acc = 0.0;       % Accumulateur stagnation

% 4) Boucle de simulation
for k = 1:Nstp
    % 4.1) Mise à jour de la grille de couverture
    [ii, jj, in] = xy_to_grid(x, y, P.grid.x, P.grid.y);
    if in
        cover(ii,jj) = cover(ii,jj) + 1;
    end

    % 4.2) Mesure ultrason (forward-only)
    d_meas = raycast_distance_forward(x, y, theta, P, d_max);
    if sigma_meas > 0
        d_meas = max(0, d_meas + sigma_meas*randn());
    end

    % 4.3) Sélection de la direction cible (theta_star)
    if d_meas < d_th
        % Mode évitement : v faible, on tourne vers le côté le plus dégagé
        v_cmd = max(0.0, 0.1 * v_max);
        d_left  = raycast_distance_forward(x, y, theta + pi/12, P, d_max);
        d_right = raycast_distance_forward(x, y, theta - pi/12, P, d_max);
        if d_left >= d_right
            om_cmd =  0.7 * om_max;
        else
            om_cmd = -0.7 * om_max;
        end
        theta_star = wrapToPi(theta + sign(om_cmd)*pi/4);
    else
        % Mode libre : directions candidates dans le secteur avant
        alphas = linspace(theta - sector/2, theta + sector/2, K_heading);
        scores = zeros(size(alphas));
        for i = 1:numel(alphas)
            nov = 1 - avg_visits_ahead(x, y, alphas(i), lookahead, cover, P.grid.x, P.grid.y);
            rot = abs(angdiff(alphas(i), theta));
            scores(i) = w_nov*nov - w_rot*rot;
        end
        smax = softmax(scores, temp);
        % Tirage aléatoire pondéré sans toolbox
        cdf = cumsum(smax) / sum(smax);
        r = rand();
        idx = find(r <= cdf, 1, 'first');

        theta_star = wrapToPi(alphas(idx));

        ang_err = angdiff(theta_star, theta);
        om_cmd  = sat(1.0 * ang_err, -om_max, om_max);
        v_cmd   = v_max * max(0.0, 1.0 - (abs(ang_err)/(sector/2)));
        v_cmd   = sat(v_cmd, 0, v_max);
    end

    % Capteur "forward-only" : si v <= 0, capteur inactif (pour info)
    if P.sensor.forward_only && v_cmd <= 0
        d_meas = P.sensor.invalid_value;
    end

    % 4.4) Intégration du modèle unicycle (Euler explicite)
    x_prev = x; y_prev = y;
    x     = x     + dt * v_cmd * cos(theta);
    y     = y     + dt * v_cmd * sin(theta);
    theta = wrapToPi(theta + dt * om_cmd);

    % 4.5) Contrainte géométrique et collisions
    [x, y] = keep_inside_room(x, y, P.room.half, P.robot.radius);
    [x, y] = push_out_of_circle(x, y, P.obs.center(1), P.obs.center(2), P.obs.R + P.robot.radius);

    % 4.6) Stagnation
    step_move = hypot(x - x_prev, y - y_prev);
    if step_move < min_step
        stagn_acc = stagn_acc + dt;
    else
        stagn_acc = max(0, stagn_acc - dt);
    end
    if stagn_acc > stagT
        theta = wrapToPi(theta + (rand*2-1)*pi/2);
        stagn_acc = 0.0;
    end

    % 4.7) Historique et indicateurs
    xh(k) = x; yh(k) = y;
    cov_hist(k) = coverage_ratio(cover);

    % 4.8) Décroissance éventuelle
    if decay_vis > 0
        cover = max(0, cover - uint16(decay_vis));
    end
end

% 5) Figures et sauvegardes
thisFile = mfilename('fullpath');
root = fileparts(thisFile);
outDir = fullfile(root, '..', 'Output');
if ~exist(outDir,'dir'), mkdir(outDir); end

figure; plot(xh,yh,'-'); axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');
title('Trajectoire du robot');
saveas(gcf, fullfile(outDir,'trajet.png'));

figure; plot(time, 100*cov_hist,'-'); grid on;
xlabel('Temps (s)'); ylabel('Couverture (%)');
title('Couverture en fonction du temps');
saveas(gcf, fullfile(outDir,'coverage_curve.png'));

figure; imagesc(P.grid.x, P.grid.y, cover'); axis xy equal tight;
xlabel('x (m)'); ylabel('y (m)'); colorbar;
title('Heatmap des visites (grille de couverture)');
saveas(gcf, fullfile(outDir,'heatmap.png'));

save(fullfile(outDir,'sim_data.mat'), 'P', 'xh', 'yh', 'time', 'cov_hist', 'cover');

disp(['Simulation terminée. Résultats enregistrés dans ' outDir]);
end

% ========================= FONCTIONS LOCALES =============================

function y = softmax(x, temp)
    x = x - max(x);
    y = exp(x./max(1e-9,temp));
    s = sum(y);
    if s > 0, y = y/s; else, y = ones(size(x))/numel(x); end
end

function [ii, jj, inside] = xy_to_grid(x, y, gx, gy)
    [~, ii] = min(abs(gx - x));
    [~, jj] = min(abs(gy - y));
    inside = ~(x < gx(1) || x > gx(end) || y < gy(1) || y > gy(end));
end

function r = coverage_ratio(cover)
    v = cover > 0;
    r = sum(v(:)) / numel(v);
end

function v = sat(u, a, b)
    v = min(max(u, a), b);
end

function a = angdiff(a1, a2)
    a = wrapToPi(a1 - a2);
end

function [x, y] = keep_inside_room(x, y, half, rad)
    % Replie la position à l'intérieur du carré en tenant compte du rayon du robot
    if x > (half - rad), x = half - rad; end
    if x < -(half - rad), x = -(half - rad); end
    if y > (half - rad), y = half - rad; end
    if y < -(half - rad), y = -(half - rad); end
end

function [x, y] = push_out_of_circle(x, y, cx, cy, R)
    % Si le robot chevauche le disque (centre cx,cy rayon R + radius_robot), repousser à la frontière
    dx = x - cx; dy = y - cy;
    d  = hypot(dx,dy);
    if d < R && d > 0
        x = cx + dx * (R/d);
        y = cy + dy * (R/d);
    end
end

function d = raycast_distance_forward(x, y, theta, P, d_max)
    % Distance au premier obstacle sur l'axe X du robot (direction theta).
    half = P.room.half;
    cx   = P.obs.center(1); cy = P.obs.center(2);
    R    = P.obs.R;

    % Ray paramétrique : p(t) = [x; y] + t * [cos(theta); sin(theta)], t >= 0
    dx = cos(theta); dy = sin(theta);

    % Intersections murs
    t_candidates = [];

    if abs(dx) > 1e-12
        t_right = ( half - x)/dx; if t_right >= 0, t_candidates(end+1) = t_right; end %#ok<AGROW>
        t_left  = (-half - x)/dx; if t_left  >= 0, t_candidates(end+1) = t_left;  end %#ok<AGROW>
    end
    if abs(dy) > 1e-12
        t_top   = ( half - y)/dy; if t_top   >= 0, t_candidates(end+1) = t_top;   end %#ok<AGROW>
        t_bot   = (-half - y)/dy; if t_bot   >= 0, t_candidates(end+1) = t_bot;   end %#ok<AGROW>
    end

    % Conserver seulement les intersections réellement sur les bords
    t_wall = inf;
    for t = t_candidates
        X = x + t*dx; Y = y + t*dy;
        if abs(X) <= half + 1e-9 && abs(Y) <= half + 1e-9
            t_wall = min(t_wall, t);
        end
    end

    % Intersection avec l'obstacle circulaire
    ox = x - cx; oy = y - cy;
    A = dx*dx + dy*dy;
    B = 2*(ox*dx + oy*dy);
    C = ox*ox + oy*oy - R*R;
    disc = B*B - 4*A*C;
    t_circ = inf;
    if disc >= 0
        r1 = (-B - sqrt(disc)) / (2*A);
        r2 = (-B + sqrt(disc)) / (2*A);
        if r1 >= 0, t_circ = min(t_circ, r1); end
        if r2 >= 0, t_circ = min(t_circ, r2); end
    end

    t_hit = min(t_wall, t_circ);
    if isinf(t_hit) || t_hit < 0
        d = d_max;
    else
        d = min(t_hit, d_max);
    end
end

function m = avg_visits_ahead(x, y, alpha, lookahead, cover, gx, gy)
    % Estime la "moyenne de visites" dans la direction alpha sur une distance lookahead.
    % On échantillonne des points le long du rayon et on moyenne (cover>0) ∈ {0,1}.
    % nb d'échantillons basé sur la résolution de grille
    dx = abs(gx(2) - gx(1)); dy = abs(gy(2) - gy(1));
    step = max( min(dx,dy), 0.1 );
    L = max(5, ceil(lookahead/step));
    s_vals = linspace(0, lookahead, L);
    acc = 0; n = 0;
    for s = s_vals
        xs = x + s*cos(alpha);
        ys = y + s*sin(alpha);
        [ii, jj, inside] = xy_to_grid(xs, ys, gx, gy);
        if inside
            acc = acc + (cover(ii,jj) > 0);
            n = n + 1;
        end
    end
    if n == 0
        m = 1.0; % si rien d'échantillonné (en dehors de la salle), considérer "couvert"
    else
        m = acc / n; % entre 0 et 1
    end
end
