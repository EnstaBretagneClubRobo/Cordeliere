from roblib import *

def f(x,u):
    x  = x.flatten()
    ψ = x[3]    
    return array([[cos(ψ)],[sin(ψ)],[0],[u] ])

def h(X1,X2):
    return 2*exp(-((X1+2)**2+(X2+2)**2)/10) + 2*exp(-((X1-2)**2+(X2-2)**2)/10) - 10

def gradh(x,y):
    eps=0.1
    return array([(h(x+eps,y)-h(x,y))/eps,(h(x,y+eps)-h(x,y))/eps])

def control(x):
    global FindIsoBath
    u = 0
    if abs(h(x[0],x[1])[0] + isoBathWanted) <= 0.5 :
        FindIsoBath =  True
    else:    
        if FindIsoBath and (x[3]-pi/4) <= 0.02:
            while 1:
                pass
    x = x.flatten()  
    
        
    if FindIsoBath :
        ψ = x[3]
        φ = tanh(h(x[0],x[1])+ isoBathWanted)+pi/2
        u = sawtooth(φ+angle(gradh(x[0],x[1]))-ψ)
    return u
    
    
def draw_field():
    Mx    = arange(-L,L,1.5)
    My    = arange(-L,L,1.5)
    X1,X2 = meshgrid(Mx,My)
    R = h(X1,X2)
    ax.plot_surface(X1,X2,R)
    return()
    
    
def draw(x):
    x = x.flatten()    
    ψ=x[3]
    draw_auv3D(ax,x[0:3],0,0,ψ,'blue',0.1)
    draw_auv3D(ax,array([[x[0],x[1],h(x[0],x[1])]]),0,0,ψ,'black',0.1)
    draw_field()
    return()

fig = figure(1)
ax = Axes3D(fig)
ax.set_zlim(-10,-5)
x    = array([[-10,-10,-2,np.pi/4]]).T #x,y,z,ψ
dt   = 0.1
L=10 #size of the world
isoBathWanted = 9
FindIsoBath = False
NbRob = 10;



for t in arange(0,30,dt):
    pause(0.001)
    cla()      
    
    #print(inIsoBath,x[3],h(x[0],x[1])+ isBathWanted)
    
    ax.set_zlim(-10,-5)
    u = control(x)
    x = x + dt*f(x,u)
    draw(x)

    


