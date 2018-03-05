# -*- coding: utf-8 -*-
"""
Created on Fri Nov 24 17:26:46 2017

@author: tutonso
"""
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from roblib import *
import csv

"""
Calcul le vecteur x_dot tel que : 
x = x + dt*f(x,u,i).T

x : vecteur d'état
u : commande
i : numéro du robot
"""
def f(x,u,i):
    x  = x.flatten() #x,y,z,ψ 
    ψ = x[3]  
    v = 10
    if Stop[i] :
        v = 0
    return array([[v*cos(ψ)],[v*sin(ψ)],[0],[3*u] ])


"""
Génère la carte bathymétrique
"""
def Generate_Map_csv():
    mapFile = open("MNT_ATL100m_HOMONIM_GEO_refNM_ZNEG254487_kriging_Zhat.csv",'r')
    texte = mapFile.readlines()
    mapFile.close()
    
    for i,ligne in enumerate(texte) :
        texte[i] = ligne.split(',')
        texte[i][-1] = texte[i][-1][:len(texte[i])-1]
    Map = np.array(texte).astype(float)
    return(Map)


"""
Estime la mesure de bathymétrie au point(x,y) et le gradient en ce point en supposant que :
    si A,B et C sont trois points assez proche de P dans le plan (x,y) alors P appartien au plan (ABC)

"""
def altitude(x,y):        
    #Cas particulier : sur les médiatrices de ses côté
    if (np.ceil(x) == round(x) or np.floor(x) == round(x)) and x !=round(x):
        x = x + 0.01 
    if (np.ceil(y) == round(y) or np.floor(y) == round(y)) and y !=round(y):
        y = y + 0.01   
    
    #Cas particulier : sur les coin du carré (points connus)
    if x == round(x) and y == round(y):
        gradient = gradh(x,y)
        alt = h(x,y)
        return(alt, gradient)
   
   #Cas particulier : sur le carre 
    elif x == round(x) :
        x = x + 0.01
 
    elif y == round(y):
        y = y +0.01  
       
    x2 = round(x)
    y2 = round(y)

    #Cas 1 (coin bas droit d'un carré)
    if x2 > x and y2 < y:
#        print("Cas1")
        x1 = x2
        y1 = np.ceil(y)
        x3 = np.floor(x)
        y3 = y2
    
    #Cas 2 (coin bas gauche d'un carré) 
    elif x2 < x and y2 < y:
#        print("Cas2")
        x1 = np.ceil(x)
        y1 = y2
        x3 = x2
        y3 = np.ceil(y)

    #Cas 3 (coin haut gauche d'un carré)
    elif x2 < x and y2 > y:
#        print("Cas3")
        x1 = x2
        y1 = np.floor(y)
        x3 = np.ceil(x)
        y3 = y2

    #Cas 4 (coin haut droit d'un carré)
    elif x2 > x and y2 > y:
        x1 = np.floor(x)
        y1 = y2
        x3 = x2
        y3 = np.floor(y)
        
    hp1 = h(x1,y1) #bathymétrie du point A
    hp2 = h(x2,y2) #bathymétrie du point B
    hp3 = h(x3,y3) #bathymétrie du point C
    
    v1 = [x1-x2, y1-y2, hp1-hp2] #vecteur BA
    v2 = [x1-x3, y1-y3, hp1-hp3] #vecteur CA

    n = np.cross(v1,v2) #vecteur perpendiculaire au plan (ABC)
    a,b,c = n[0],n[1],n[2]
    d = a*x3 + b*y3 + c*hp3
    
    hp = -(a*x + b*y - d)/c #bathymétrie
    gradXY =  np.array([-a/c,-b/c]) #gradient
        
    return(hp,gradXY)



"""
renvoie la bathymétrie du point (X1,X2) dans la carte des mesures.
A n'utiliser que pour les points de mesures connues.
"""
def h(X1,X2):
    Abs,Ord = int(X2), int(X1)
    if Abs > Map.shape[0] or Abs < 0 or Ord > Map.shape[1] or Ord < 0:
        print("Attention, terrain inconnu")
        return(0)
    else:
        return(Map[Abs][Ord])

"""
renvoie le gradient du fond-marin à la position (x,y)
"""
def gradh(x,y):
    eps=6.1 #Ne pas mettre d'entier
    return np.array([(altitude(x+eps,y)[0]-h(x,y))/eps,(altitude(x,y+eps)[0]-h(x,y))/eps])

"""
renvoie la commande u pour que les robots suivent les phases suivante :
    - phase I : avance dans une direction donné jusqu'à trouver l'isobathe voulue (isoBathWanted)
    - phase II : suivre l'isobathe voulue jusqu'à être dans une orientation donné (x[3]-np.pi)%(2*np.pi)  ici)
"""
def control(x,i):
    global FindIsoBath, Stop
    u = 0
    
    #phase II validée ?
    if FindIsoBath[i] and (x[3]-np.pi)%(2*np.pi) <= 0.35:
            Stop[i] = True
    
    #phase I validée ?
    alt = altitude(x[0],x[1])
    if abs(alt[0] + isoBathWanted)  <= 0.1 and not FindIsoBath[i] :
        FindIsoBath[i] =  True    
    x = x.flatten() 

    #phase II si phase I validée
    if FindIsoBath[i] :
        gardXY = alt[1]
        
        ψ = x[3]
        φ = tanh(alt[0] + isoBathWanted)+pi/2
        u = sawtooth( (φ+angle(gardXY)-ψ) )
        ax.plot([x[0], x[0] + 4*np.cos(φ+angle(gardXY))],
                [x[1], x[1] + 4*np.sin(φ+angle(gardXY))],
                [x[2], x[2]])
        ax.plot([x[0], x[0] + 4*np.cos(0)],
                [x[1], x[1] + 4*np.sin(0)],
                [x[2], x[2]])
        
    return(u)


"""
affiche la cartographie du fond-marin
""" 
def draw_field():
    x,y = Map.shape
    Mx = np.arange(0,x,1)
    My  = np.arange(0,y,1)
    X1,X2 = np.meshgrid(My,Mx)
    R = Map
   
    aff = ax.plot_surface(X1,X2,Map,cmap=cm.spectral_r)
    
    return(aff)

"""
affiche l'AUV d'état x
""" 
def draw(x):
    x = x.flatten()    
    ψ=x[3]
    M=0.1*eulermat(0,0,ψ) @ motif_auv3D()
    M=translate_motif(M,x[0:3].reshape(3,1)) 
    ax.plot(M[0],M[1],color='blue')
    return()


if __name__ == "__main__" :
    ax=figure().gca(projection='3d')
    ax.view_init(90,0 )
    Map = Generate_Map_csv()
    
    #Initialisation des robots
    X = array([[100,350,-2,-np.pi/3]]).T #x,y,z,ψ
    dt = 0.05
    isoBathWanted = 36
    Save = []
    
    NbRob = 20;
    for i in range(NbRob):
        FindIsoBath = [False]*NbRob
        Stop = [False]*NbRob
        rand = np.random.randn(4)
        x = array([[100 + 10*rand[0],350 + 10*rand[1],-2, -np.pi/3  + 0.1*rand[2]]]).T
        X = np.concatenate( (X,x),axis = 1)
        Save.append(np.concatenate((x,np.array([[0]]))))
    
    
    # Simulation
    for t in arange(0,100,dt):
        aff=draw_field()
        if t ==0:
            plt.colorbar(aff,shrink=0.5,aspect=5)
        gca().set_ylim(100,300)
        gca().set_xlim(100,300)
        for i in range(NbRob):
                x = X[:,i]
                u = control(x,i)
                x = x + dt*f(x,u,i).T
                draw(x)
                
                X[:,i] = x
                SaveRobot = np.concatenate((x.T,np.array([[t]])))
                Save[i] = np.concatenate( (Save[i],SaveRobot), axis = 1 )
        pause(0.001)    
        cla()
     
    # Save  
    for i in range(NbRob): 
        fname = "Robot"+str(i) +".csv"
        file = open(fname, "w")
        try:
            writer = csv.writer(file)
            for indRow in range(Save[i].shape[1]):
                print(Save[i][:,indRow])
                writer.writerow(list(Save[i][:,indRow]))
        finally:
            file.close()
        #np.savetxt("Robot.csv" + str(i) + ".txt", Save[i],delimiter=',') #décommenter pour enregistrer la trajectoire des robots
