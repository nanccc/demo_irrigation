# irrigation
26/6/2020: Mettre à jour la periode d'échantillonage après chaque tour d'eau.
           3 échantillons pour stade 1, 4 pour stade 2, 5 pour stade 3. Stade 4 va continuer jusqu'à l'humidité du sol atteint le seuil référence.  
           à la fin de stade 1, enregistrer la valeur de l'humidité mesurée dans "Humitide_fin_stade1" et faire le comparaison entre "Humitide_fin_stade1" et "DH* 0.4+seuil_h".   
           DH est la différence entre l'humidité initial et le seuil référence, on suppose que l'humidité du sol doive descendere 60% DH à la fin de stade 1,  
           sinon, on diminue la fréquence d'échantillonage de stade 1, chaque fois prolonger la periode d'échantillonage de 2 seconde.  
           Le temps total est fixé, si le temps pour stade 1 est plus large, le temps pour stade 4 est plus petit, nombre d'échantillons diminue dans ce cas.
           
12/6/2020: changer les seuils avec le bouton en bleu
