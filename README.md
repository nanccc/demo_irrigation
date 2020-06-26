# irrigation
26/6/2020: Mettre à jour la periode d'échantillonnage après chaque tour d'eau.
           3 échantillons pour stade 1, 4 pour stade 2, 5 pour stade 3. Stade 4 va continuer jusqu'à l'humidité du sol atteint le seuil référence.  
           à la fin de stade 1, enregistrer la valeur de l'humidité mesurée dans "Humitide_fin_stade1" et faire la comparaison entre "Humitide_fin_stade1" et "DH* 0.4+seuil_h".   
           DH est la différence entre l'humidité initial et le seuil référence, on suppose que l'humidité du sol doive descendre 60% DH à la fin de stade 1,  
           Sinon, on diminue la fréquence d'échantillonnage de stade 1, chaque fois prolonger la période d'échantillonnage de 2 secondes.

           
12/6/2020: changer les seuils avec le bouton en bleu
