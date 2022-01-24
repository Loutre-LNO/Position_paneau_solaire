# Asservissement position panneau solaire

## Orienter un panneau solaire de façon optimale par rapport à la position du soleil

### Principe
On cherche à orienter le panneau solaire par rapport au soleil de telle sorte à optimiser son rendement.
Au fil de la journée, on compare la position du panneau avec un tableau de consigne, et s'il y a écart, on actionne le(s) moteur(s) pour que le panneau se trouve dans la position souhaitée.

- Un moteur sur chaque axe avec des fins de course début et fin directement sur le moteur (non gérés dans le programme)
- Un encodeur sur chaque axe pour mesurer la position relative en degrés par rapport à la position initiale
- Les mesures sont comparées à un tableau de consigne variable selon le mois et l'heure
- En élévation, si la mesure est différente de la consigne, le moteur est activé dans un sens ou dans l'autre
- En azimut, si la mesure est inférieure de la consigne, le moteur est activé dans le sens horaire
- Initialisation de la position quotidiennement à heure fixe


### Composants
- Le système est géré par un Arduino.
- Les moteurs sont actionnés via des relais.
- La position est lue avec des capteurs incrémentaux installés sur les axes de rotation du panneau.
- La gestion de la date et l'heure est confiée à un module DS3231.
- Un écran LCD affiche la date et l'heure, la consigne, la position, et l'état des moteurs.
- Une liaison série permet quelques interactions avec le système.


![Photo1](Photos/photo_panneau.jpg)

![Photo2](Photos/photo_montage.jpg)


### Licence
Distribué sous [licence MIT](license.txt)
