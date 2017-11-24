# Résumé des commandes


## Initialiser un répertoire git

* Si le répertoire est vide :
		git init
* Si on veut travailler sur un projet déjà existant :
		git clone adresse_du_projet

## Commandes de base

* Connaître le statut actuel du répertoire :
		git status
* Ajout d'un fichier :
		git add nom_fichier
		git add -A # pour ajouter tous les fichiers d'un coup
* Commit (faire une photographie de l'état du projet en expliquant ce qu'on a modifié depuis le dernier commit) :
		git commit -m "message that explains all the last modifications (in english please)"

## Travail avec un dépot distant

### Manipulation des remotes

* Création d'une remote :
		git remote add nom_remote url
Si le répertoire git a été initialiser avec un `git clone`, alors il existe déjà un remote du nom de `origin` lié au dépot d'orgine du clone.
* Liste des remote :
		git remote show
* Supprimer une remote :
		git remote remove nom_remote
* Push (mettre à jour un dépot distant sur lequel on a les droits) :
		git push
* Pull (mettre à jour son dossier local depuis un dépot distant) :
		git pull nom_remote
		git pull --set_upstream nom_remote #Pour ne pas avoir à redire à chaque fois le nom de la remote sur laquelle on veut push

### Travail sur des branches

Ne _jamais_ travailler directement sur la branche `master` (qui est la branche de base).

* Créer une nouvelle branche (et aller dessus) :
		git checkout -b nom_branche
* Changer de branche :
		git checkout nom_branche
* Lister les branches :
		git branch -l
* Supprimer une branche :
		git branch -d nom_branche
		git branch -D nom_branche # Si on veut la supprimer même si elle n'a pas été fusionné
* Fusionner une branche dans la branche actuelle (créer un commit) :
		git merge nom_branche

### Mettre à jour son fork

* Télécharger les mises à jour :
		git fetch nom_remote # en général cette remote s'appelle upstream
* Fusion sur la branche master :
		git rebase nom_remote/master # ou nom_remote/nom_branche_intéressante

