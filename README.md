# Mesure de Déformation par Jauges – Dispositif de Commande Haptique

## 📌 Description du projet
Ce projet a pour objectif de concevoir un **système de commande haptique** basé sur la mesure de déformation d’une lame métallique.  
Quatre jauges de déformation, montées en **pont de Wheatstone**, sont fixées sur la lame afin de mesurer les contraintes appliquées.  
Le signal issu du pont est amplifié par un **AD624** (carte de puissance) puis lu par une carte **Nucleo STM32** via l’entrée analogique PA0.  
Le traitement embarqué permet d’estimer la force appliquée et de piloter un **moteur pas à pas** en asservissement de position.

---

## 🎯 Objectifs
- Concevoir un système de mesure haptique fiable et précis.  
- Assurer la cohérence entre la force appliquée et le mouvement du moteur.  
- Calibrer le système pour obtenir une relation linéaire entre la tension mesurée et la déformation.  
- Valider le bon fonctionnement par comparaison entre résultats théoriques et expérimentaux.

---

## 🛠 Réalisations techniques
- **Capteur** : 4 jauges de déformation en pont de Wheatstone.  
- **Conditionnement du signal** : Amplificateur AD624.  
- **Acquisition** : Carte Nucleo STM32 (entrée analogique PA0).  
- **Actionneur** : Moteur pas à pas commandé en position.  
- **Interface Homme-Machine (IHM)** pour le suivi en temps réel.  
- **Carte de puissance** dédiée.  

---

## 📐 Méthodologie
1. **Montage mécanique** : fixation des jauges sur la lame métallique.  
2. **Montage électronique** : pont de Wheatstone → amplification analogique → acquisition STM32.  
3. **Calibration** : série de tests pour établir la relation tension–force–déformation.  
4. **Traitement logiciel** : calcul de la force appliquée et pilotage du moteur pas à pas.  
5. **Validation** : comparaison avec le modèle théorique.

---

## 📊 Résultats
- Relation linéaire observée entre tension mesurée et déformation/poids appliqué.  
- Écart faible entre théorie et expérience (dus aux pertes et imprécisions expérimentales).  
- Bonne réactivité et positionnement précis du moteur pas à pas.  
- Système fiable et stable lors des essais.

---

## 🔭 Perspectives
- Miniaturisation du dispositif.  
- Intégration d’un **retour de force** pour applications médicales ou aéronautiques.  
- Optimisation de l’IHM et du traitement embarqué.  

---

## 👥 Équipe
- Alexandre P.  
- Mathieu L.  
- Yvan M.  
- Madaï M.  
