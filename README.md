# grp-olive Challenge 2

## Sommaire

- [Présentation du projet](#Présentation-du-projet)
- [Présentation du code](#Présentation-du-code)
- [Résultats](#Résultats)
- [Perspectives d'amélioration](#Perspectives-d'amélioration)

## Présentation du projet

L'objectif de ce challenge est de démontrer la capacité d'un robot à cartographier un environnement et à retrouver des objets spécifiques dans ce dernier.

## Présentation du code

```python
#!/usr/bin/env python3
from threading import local
import rospy
import tf
from geometry_msgs.msg import *
```
Les premières lignes servent à importer les librairies et classes nécessaires au projet.
```python
myNode()
rospy.spin()
```
Ces lignes sont les dernières du code : on instancie un objet myNode que nous avons défini plus tôt, puis on recommence grâce à la méthode spin() de la librairie ros.
```python
class myNode:
    def __init__(self):
        rospy.init_node('move_to', anonymous=True)
        self.tfListener = tf.TransformListener()
        self.sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)
        self.pub = rospy.Publisher(
            '/cmd_vel_mux/input/navi',
            Twist, queue_size=10
        )
        rospy.Timer( rospy.Duration(0.1), self.callback2, oneshot=False )
```
L'instanciation de la classe myNode commence par une commande init_node qui permet au noeud de se présenter sur le roscore. Ensuite vient la commande Subscriber qui permet d'appeler la méthode callback de notre classe myNode à chaque changement de valeur sur le topic "/move_base_simple/goal". La commande Publisher indique au roscore que les données vont être publiées sur le topic '/cmd_vel_mux/input/navi'. Ce topic est écouté par le robot, il interprètera des données de type Twist, qui est une classe contenant un vecteur de 3 dimensions donnant les vitesses de translation et un second vecteur de 3 dimensions donnant les vitesses angulaires.


```python
    def callback(self, goal):
        rospy.loginfo("I Got a goal : ")
        print(goal)
        self.local_goal= self.tfListener.transformPose("/base_footprint", goal)
        print(self.local_goal)
```
Les dernières lignes utilisent la librairie ros :

- init_node : permet à la node de se présenter sur le roscore
- Subscriber : à chaque changement de valeur sur le topic "/scan", on appelle la méthode scanner de notre classe Brain. Cela servira à détecter les obstacles et à juger de leur proximité
- Timer : on appelle finallement la méthode run toutes les 0.1s
```python
    def callback2(self,data):
        #Do some work
        cmd= Twist()
        cmd.linear.x= 0.1
        cmd.angular.z = self.local_goal.pose.orientation.z
        self.pub.publish(cmd)
```
On publie sur le topic /cmd_vel un ordre d'avancer, sauf, si on détecte un obstacle, la vitesse de translation est mise à 0.

On va maintenant présenter comment on décide de tourner, donc de changer le boolean isTurning de False vers True
```python
    def getROI(self, data):
        return data.ranges[355:365]

    def shouldTurn(self, roi):
        return min(roi) < 2

    def setAmountRotation(self):
        self.maxTurning = random.randint(30, 60)
        if(self.verbose):
            print(f"amount of turn : {self.maxTurning}")

    def scanner(self, data):
        roi = self.getROI(data)

        if(self.shouldTurn(roi) and not self.isTurning):
            self.isTurning = True
            self.setAmountRotation()
            if(self.verbose):
                print("start turning")

        self.handleTurning()
```
getROI retourne les données des scanners qui sont devant le robot.

scanner appelle la méthode shouldTurn qui renvoie un résultat si un distance < 2 est détectée, de plus, si isTurning est à False, on passe alors isTurning à True, puis appelle la méthode setAmountRotation qui renvoie un nombre aléatoire entre 30 et 60. Cette valeur permet au robot de tourner plus ou moins, ce qui lui permettra de voyager un peu plus "naturellement". Finalement, scanner appelle handleTurning.

```python
    def handleTurning(self):
        if(self.isTurning):
            self.turningCount += 1
            self.cmd.angular.z = 1

        if(self.turningCount > self.maxTurning):
            self.isTurning = False
            self.cmd.angular.z = 0
            self.turningCount = 0

            if(self.verbose):
                print("stop turning")
```
Tant que turningCount est inférieur à maxTurning (définit entre 30:60) cmd.angular.z est passé à 1 : le robot tourne quand l'ordre est publié. S'il devient supérieur, isTurning passe à False, cmd.angular.z passe à 0 et turningCount est réinitialisé à 0.

La commande est publié de cete manière :
rospy.Timer appelle run() toutes les 0.1s, méthode qui publie les valeurs cmd.
Dans notre implémentation :

- isTurning = False → cmd.linear.x = vitesse, cmd.angular.z = 0 → le robot avance
- isTurning = True → cmd.linear.x = 0, cmd.angular.z = 1 → le robot tourne

## Résultats

On observe dans la simulation que le robot ne se bloque pas, évite les obstacles et découvre son environnement.

## Perspectives d'amélioration

La vitesse linéaire est suffisante étant donné la surface à parcourir.
Néanmoins, son comportement est simpliste, surtout la gestion des obstacles. On pourrait imaginer que le robot suive une courbe précise en fonction de sa vitesse et de sa distance à l'obstacle, ou bien qu'il tourne à droite ou a gauche en fonction de la position de l'obstacle...
