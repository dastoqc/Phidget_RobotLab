# Phidget_RobotLab
Phidget C++ wrapper.

Programme démo pour Phidget motor DC plus détection de tags Aruco.

Avant de rouler, s'assurer de décompresser le dossier de dll (opencvdll_debugx64.zip ou opencvdll_releasex64) dans le répertoire de l'exécutable.

Les fonctions liées au Phidget sont contenues dans PhidgetWrapper.cpp et .h pour se connecter à un controlleur moteur DC ou à un controlleur de moteur pas-à-pas. Des callbacks en C font le lien avec les 'événements' des Phidgets et s'assurent de maintenir une fréquence d'acquisition stable à 8ms. Un PID permet le contrôle en boucle fermé avec des encodeurs en position ou en vitesse. Cette dernière est filtrée avec un filtre gaussien. L'export des données importantes est fait dans un fichier .csv et un script Matlab 'plotcsv.m' est inclu pour afficher les courbes produtes.

Les fonctions nécessaires à l'utilisation des marqueurs sont dans le répertoire aruco125 qui lui-même utilise OpenCV. Un exemple pour plusieurs marqueurs, affichant seulement leur vecteur position par rapport à la caméra et leur matrice de rotation sur la console est joint dans le fichier 'arucofct.cpp'. Un filtre de Kalman discret à 1DDL est aussi montré basé sur la classe OpenCV. Si le mode enregistrement est sélectionné (REC à 1 dans le fichier principal) en plus de produire un .csv, le programme enregitre les images de la caméra avec les marqueurs détectés dans un .avi.

Codé par David St-Onge, 02-12-2015.
