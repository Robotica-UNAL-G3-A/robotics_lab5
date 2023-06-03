# Robotic's lab 5: inverse kinematics

## Inverse kinematics 

We use the strategy to define a wirst and use the geometric method. In order to do that, we use the next isometric view: 

![capture robotStudio signal creation](/media/Proyeccion_q1.png) 

As we can see, the first joint can be calculated as: 

$q_1=atan2(y,x)$

Then, we proceed with the thrid joint. As the method say we proyect the vector to the wirst on the x2-y2 plane. We get:

![capture robotStudio signal creation](/media/Proyeccion_q3.png) 

First, we note that $L=\sqrt((z_c-L_1)^2+x_c^2+y_c^2)$

We construct a triangule with the sides $L,L_2$ and $L_3$. From this, we can use the cosine theorem and get:

$L^2 = L_2^2 + L_3^2 - 2L_2L_3cos(180-q3)$

We solve the equation for $cos(q_3)$ and finally get the next equation system:

$c_{q3}=\frac{L^2-(L_2^2+L_3^2)}{2L_2L_3}$

$s_{q_3}=\sqrt(1-c_{q3})$

$q_3=atan2(s_{q3},c_{q3})$

![capture robotStudio signal creation](/media/Proyeccion_q2_q4.png) 


## Video: 

[![Alt text](https://img.youtube.com/vi/Wdw1Ll6Tfwo/0.jpg)](https://youtu.be/Wdw1Ll6Tfwo)

## Conclusions 


## Contributors
- [Juan Sebastian Duenas](https://github.com/jsduenass) (jsduenass@unal.edu.co)
- [German Andres Urbina Gutierrez](https://github.com/gurbinaUn)  (gurbina@unal.edu.co)
- [Brayan Daniel Barrera Galvis](https://github.com/brayandan) (bdbarrerag@unal.edu.co)

## Reference



<!---

Design doc
Position movement control of the Phantom X Pincher robot 

TODO
- Define architecture
- Define message for target position
- 

--->
