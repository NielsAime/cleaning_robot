# 🤖 PROJECT_CONTEXT_FOR_AI.md — Brief pour Assistant IA (ChatGPT / Copilot)

## 🎯 Rôle attendu de l’assistant
- Lire et comprendre le code du dépôt, **expliquer** clairement les choix et les algorithmes.
- **Améliorer** et **optimiser** la stratégie hybride (réactif + aléatoire biaisé + mémoire).
- Aider à **porter** la logique MATLAB vers **Simulink** (schéma, sous-systèmes, masques).
- Proposer des **campagnes d’expériences** et générer des **figures** propres pour le rapport.
- S’assurer que le code reste **sans dépendances** toolbox non standard.

## 📘 Résumé du sujet (rappels)
- Robot unicycle, disque R=0,2 m.  
- Capteur ultrason frontal, **forward-only** (mesure sur l’axe X, valide si `v_cmd>0`).  
- Pose `(x,y,θ)` disponible à chaque pas (proprioception).  
- Salle 20×20 m, obstacle disque (3,3), R=0,25 m.  
- Commande en `(v, ω)` ; préférer **mouvement avant** quand c’est libre.

## 🧠 Stratégie à respecter
1. **Réactif** : si `d < d_th` → `v` faible, `ω` saturé côté le plus dégagé.  
2. **Exploration biaisée** : en libre, orientations candidates dans un secteur avant ; score
   \( S = w_{nov}(1-\text{avg\_visits\_ahead}) - w_{rot}|\Delta\theta| \) ; choix **softmax**.  
3. **Mémoire** : grille de couverture pour KPI et biais de nouveauté.  
4. **Anti-stagnation** : réorientation aléatoire si progrès faible > `stagnation_T`.

## 🧰 Fichiers clés
- `Scripts/init_project.m` : configuration, chemins, constantes.  
- `Scripts/run_simulation.m` : simulation MATLAB “tout-en-un” (cinématique + capteur + contrôleur + grille + figures).  
- `Docs/…` : documents de stratégie et requirements.

## 🧪 Bonnes pratiques attendues
- Commenter le code en **français technique clair**.  
- Éviter les fonctions toolbox non standard (`randsample` → softmax + CDF).  
- Découper en fonctions réutilisables (ray-cast, grille, scores…).  
- Proposer des paramètres **dans `init_project.m`** pour centraliser le tuning.  
- Générer des **graphs** lisibles (labels, unités, titres).

## ✅ Do / ❌ Don’t
- ✅ Faire des **PR**/diffs compréhensibles ; justifier chaque changement.  
- ✅ Isoler l’implémentation Simulink par sous-systèmes.  
- ❌ Introduire des dépendances lourdes (toolbox, packages externes) sans discussion.  
- ❌ Casser la compatibilité MATLAB de base.

## 🧭 Tâches suggérées pour commencer
1. Centraliser `lookahead`, `K_heading`, `decay_vis` dans `init_project.m`.  
2. Ajouter une option de **visualisation live** pendant la simulation.  
3. Créer un **script d’expériences** (sweep de paramètres).  
4. Portage **Simulink** : Unicycle, Sonar (ray-cast), Controller, Coverage, KPIs.  
5. Générer des **figures standard** pour le rapport (templates).

## 🔗 Repo
- URL : **à jour sur GitHub** (public pendant le travail avec l’IA).  
- L’assistant doit référencer les fichiers par chemin relatif au dépôt.
