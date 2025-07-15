
# Simulation Physique & Commande (2024/2025)

Projet de simulation et commande de systèmes physiques réalisé en Python, dans le cadre de l'UE "Simulation Physique". Ce dépôt regroupe plusieurs modules : moteur CC, centrifugeuse, PID, pendule double, barre rigide 2D, et TurtleBots.

## Fonctionnalités principales

- **Moteur à courant continu (CC)**
  - Modélisation électromécanique
  - Simulation numérique (boucle ouverte)
  - Contrôle en vitesse avec correcteur P / PI
  - Contrôle en position avec correcteur P+D

- **Centrifugeuse**
  - Simulation d'une particule attachée à un ressort en rotation
  - Modèle dynamique vs formule théorique
  - Animation de l’élongation radiale

- **Barre rigide en 2D**
  - Modèle de pendule physique
  - Couplage torsionnel entre barres
  - Animation Pygame et analyse des modes propres

- **Robot TurtleBot**
  - Simulation cinématique vs dynamique (moteurs CC)
  - Comparaison de trajectoires
  - Préparation à l’ajout de régulation PID

## Exemples de visualisation

- Animation dynamique de pendules dans `barre2D.py`
- Courbes de réponse moteur (PID, vitesse, position)
- Comparaison de trajectoires TurtleBot

## Résultats & Validations

- Bonne concordance entre simulations et solutions analytiques
- Réponse correcte aux consignes avec régulateurs PI / P+D
- Visualisation claire grâce à `pygame`

## Limitations actuelles

- Pas encore de commande clavier pour les TurtleBots
- Manque d’intégration des contraintes mécaniques (liaisons Revolute, Prism)
- Simulation non temps réel (pré-calculée)

## Perspectives

- Ajout d’un retour utilisateur (clavier / souris)
- Contrôle en boucle fermée pour les TurtleBots
- Extension à des systèmes multi-corps avec contacts

