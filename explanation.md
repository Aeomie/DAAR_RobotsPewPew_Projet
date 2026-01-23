# Documentation – Robot (Main) & RobotSecondary (Scout)

Ce document décrit le comportement des deux IA :
- **`Robot`** : “main bot” (WARIO / MARIO / LUIGI) orienté **combat + coordination**.
- **`RobotSecondary`** : “secondary bot” (Explorer Alpha / Beta) orienté **exploration + repérage + support**.

---

## 1) Robot (Main bot) – `algorithms.Robot`

### Objectif global
Le **main bot** alterne entre :
- **Exploration / déplacement** (éviter les obstacles, ne pas se bloquer avec les alliés),
- **Combat** (tir prioritaire sur les ennemis),
- **Coordination d’équipe** via **broadcast** des positions ennemies et **convergence** vers une zone.

---

### Rôles (WARIO / MARIO / LUIGI)
Le robot choisit son rôle dans `activate()` selon la position des coéquipiers détectés au radar :
- Il compte combien de **TeamMainBot** sont “au-dessus” vs “en-dessous” (angle relatif à son heading).
- Déduit s’il est :
    - **WARIO** (plutôt “top”),
    - **MARIO** (milieu),
    - **LUIGI** (bas).

Le rôle sert ensuite à :
- donner une **priorité** lors des blocages (MARIO attend, WARIO/LUIGI contournent),
- calculer un **offset de formation** lors de la convergence.

---

### États (machine à états)
`state` pilote toute la logique dans `step()` :

- **STOPPED**
    - Phase “défense passive” au début : le bot reste en attente (`STOPPED_TIME`).
    - Sortie immédiate si :
        - un ennemi est détecté → **ATTACKING**
        - une position ennemie est reçue → **CONVERGING**
        - cooldown terminé → **MOVE**

- **MOVE**
    - Comportement normal :
        - Si info ennemie reçue (sharedEnemyX/Y) et pas d’ennemi visible → **CONVERGING**
        - Si ennemi visible → **ATTACKING**
        - Sinon avance (`myMove()`), sauf obstacle.

- **CONVERGING**
    - Va vers une position “partagée” (`sharedEnemyX/Y`) via `meetAtPoint(x,y,precision)`.
    - Dès qu’un ennemi est visible → **ATTACKING**
    - Si info ennemie expirée → retour **MOVE**

- **ATTACKING**
    - Tire tant qu’un ennemi est détecté.
    - Sinon → **MOVE**

- **BACKING_UP**
    - Recule quelques steps (`MAX_BACKUP_STEPS`), puis passe à **TURNING** avec un angle d’évitement.

- **TURNING**
    - Tourne progressivement vers `targetAngle` (avec `stepTurn`) puis revient à **MOVE**.

- **WAITING**
    - Utilisé surtout quand MARIO est bloqué par un allié : attend un petit temps, sinon finit par reculer.

- **IDLE**
    - État tampon : bascule en MOVE.

---

### Détection / obstacles
Le robot considère comme obstacle :
- Mur (`detectFront() == WALL`)
- Épave (`Wreck` proche devant)
- Coéquipier (TeamMainBot, TeamSecondaryBot)
- Adversaire

Fonctions clés :
- `isInFront()` / `isBehind()` : classification des objets par angle relatif.
- `obstacleCheck()` : mur ou épave ou bot.
- `calculateAvoidanceAngle()` : choisit une rotation (gauche/droite/demi-tour) selon obstacles détectés.

**Gestion des blocages :**
- Si bloqué **uniquement par un TeamMainBot** :
    - **MARIO** attend (`WAITING`)
    - WARIO/LUIGI reculent et contournent
- Si bloqué par un **secondary** : recule direct
- Sinon (mur/épave/ennemi etc.) : recule puis tourne

---

### Combat : sélection de cible + tir
Méthode `shootEnemy()` :
1. Scanne le radar et garde uniquement les ennemis dans `ENEMY_DETECTION_DISTANCE`.
2. Ignore les ennemis **derrière une épave** via `isBlockedByWreck(direction, distance)`.
3. Priorité :
    - d’abord **OpponentMainBot** (shooters)
    - sinon **OpponentSecondaryBot** (scouts)
4. Tire sur **le plus proche** dans la catégorie prioritaire.
5. Après chaque tir, diffuse la position ennemie : `broadcastEnemyPosition(target)`.

---

### Communication d’équipe : partage de positions ennemies
Message envoyé par `broadcastEnemyPosition` :
- **`ENEMY_LOCATION|spotterName|myX|myY|enemyX|enemyY`**

Réception dans `readTeammateMessages()` :
- Ignore ses propres messages.
- Met à jour `stepsSinceEnemyUpdate`.
- Calcule une **position de convergence** avec offset via `applyFormationOffset()` :
    - convertit rôle en index : WARIO=-1, MARIO=0, LUIGI=1
    - offset en X : `(myPos - spotterPos) * FLANK_OFFSET_X`
    - convergence sur `(enemyX + offset, enemyY)`

Expiration :
- si pas de mise à jour depuis `ENEMY_INFO_EXPIRY`, reset `sharedEnemyX/Y`.

---

### Partage des bordures (map bounds)
Le `Robot` sait aussi lire les messages :
- **`BORDER|NORTH|pos`**, **`BORDER|SOUTH|pos`**, **`BORDER|EAST|pos`**, **`BORDER|WEST|pos`**
  et stocke les limites (northBound/southBound/eastBound/westBound).

---

## 2) RobotSecondary (Scout) – `algorithms.RobotSecondary`

### Objectif global
Le **secondary bot** sert de **scout** :
- Explore et détecte des **bords de map** (N/S/E/W), puis les broadcast.
- Roam (patrouille) ensuite en évitant les collisions.
- Repère un ennemi et broadcast sa position régulièrement.
- S’éloigne si un ennemi est trop proche.
- Si le bot prend des dégâts, il tente un **repli vers le main bot**.

---

### Rôles (Explorer Alpha / Explorer Beta)
Dans `activate()` :
- Il détecte l’autre TeamSecondaryBot au radar pour deviner s’il est en haut/bas.
- Assigne :
    - **Explorer Alpha** (bottom) : explore vers le **NORTH** (puis WEST)
    - **Explorer Beta** (top) : explore vers le **SOUTH** (puis EAST)

---

### États (exploration puis roaming)
États principaux :

- **TURNING_NORTH / GOING_NORTH**
    - S’aligne vers le nord puis avance.
    - Quand un mur est détecté : enregistre `northBound`, broadcast `BORDER|NORTH|...`, puis tourne.

- **TURNING_SOUTH / CHECKING_SOUTH**
    - Symétrique pour le sud : enregistre `southBound`, broadcast `BORDER|SOUTH|...`, puis tourne.

- **TURNING_WEST / CHECKING_WEST**
    - Avance vers l’ouest jusqu’au mur, enregistre `westBound`, broadcast `BORDER|WEST|...`,
    - puis passe en **EXPLORATION_COMPLETE**.

- **TURNING_EAST / CHECKING_EAST**
    - Pareil vers l’est, puis **EXPLORATION_COMPLETE**.

- **EXPLORATION_COMPLETE**
    - Transition vers le roaming : devient **MOVE**.

- **MOVE**
    - Mode patrouille/évite-collisions + repérage ennemis.

---

### “Wall latch” (approche contrôlée du mur)
La fonction `detectWall()` a une logique spéciale :
- Quand le latch est actif, au lieu de s’arrêter au premier contact,
  le scout “considère” le mur atteint après avoir parcouru `WALLOFFSET` (ex: 400).
- Cela évite de se coller au mur trop tôt / rend l’approche plus stable.
- `latchJustCompleted` empêche un re-latch immédiat le tick suivant.

---

### Roaming & anti-deadlock
En **MOVE**, le scout gère plusieurs priorités :

1) **Évasion ennemie (panic-back)**
- Si `evadeEnemySteps > 0` : recule plusieurs steps.

2) **Détection ennemi**
- `getFirstEnemy()` récupère un ennemi s’il existe.
- S’il est visible et cooldown ok → broadcast la position (toutes les `ENEMY_BROADCAST_PERIOD` steps).
- Si l’ennemi est **trop proche** (< `ENEMY_MAINBOT_KEEP_DISTANCE`) :
    - prépare une rotation “dos à l’ennemi” (`targetAngle = enemyDir + PI`)
    - déclenche un recul sur plusieurs ticks (`EVADE_BACK_STEPS`)
    - passe en **TURNING_BACK** puis revient à MOVE.

3) **Yield face aux alliés**
- Si un **main bot** est devant (`isBlockedByTeamMainBotOnlyFront()`), le scout recule quelques steps (`YIELD_BACK_STEPS_MAIN`).
- Si un **secondary** bloque devant :
    - Alpha préfère reculer
    - Beta préfère casser le blocage en tournant (30°)

4) **Obstacle normal**
- Si libre → avance
- Si bloqué :
    - tourne par incréments de 30° (`ROAM_TURN_INCREMENT`)
    - alterne parfois l’autre sens
    - après trop de blocages → demi-tour

---

### Scan du main bot + repli si dégâts
Le scout scanne périodiquement les TeamMainBot (`scanTeamMates()`), pour obtenir `(mateX, mateY)`.

`damageTakenCheck()` :
- Compare `getHealth()` à `lastHealth`.
- Si la santé baisse (dégât) et cooldown ok :
    - active `retreatToMate`
    - pendant `RETREAT_COOLDOWN_STEPS`, il tente `meetAtPoint(mateX, mateY, precision)`
    - s’il est déjà proche du mate, il retourne vers une position “par défaut” (moyenne des positions init).

---

### Communication envoyée par le scout
- Bordures :
    - `BORDER|NORTH|pos`, `BORDER|SOUTH|pos`, `BORDER|EAST|pos`, `BORDER|WEST|pos`
- Ennemis :
    - `SCOUT_ENEMY_LOCATION|robotName|myX|myY|enemyX|enemyY`

Le scout lit aussi les messages `BORDER|...` pour synchroniser ses limites.

---

## Résumé rapide des deux bots

- **Robot (main)** :
    - combat prioritaire + tir intelligent (ligne de vue vs épave),
    - broadcast positions ennemies,
    - convergence en formation (offset selon WARIO/MARIO/LUIGI),
    - évitement obstacles + gestion de priorité entre coéquipiers.

- **RobotSecondary (scout)** :
    - découvre les bords et les broadcast,
    - roam pour repérer / déranger,
    - broadcast ennemi régulièrement,
    - évite ennemis trop proches,
    - se replie vers un main bot quand il prend des dégâts.

---
