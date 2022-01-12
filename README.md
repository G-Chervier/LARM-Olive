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
L'instanciation de la classe myNode commence par une commande init_node qui permet au noeud de se présenter sur le roscore. Ensuite vient la commande Subscriber qui permet d'appeler la méthode callback de notre classe myNode à chaque changement de valeur sur le topic "/move_base_simple/goal". La commande Publisher indique au roscore que les données vont être publiées sur le topic '/cmd_vel_mux/input/navi'. Ce topic est écouté par le robot, il interprètera des données de type Twist, qui est une classe contenant un vecteur de 3 dimensions donnant les vitesses de translation et un second vecteur de 3 dimensions donnant les vitesses angulaires. On appelle finalement la méthode callback2 toutes les 0.1s à l'aide de la commande Timer.
```python
    def callback(self, goal):
        rospy.loginfo("I Got a goal : ")
        print(goal)
        self.local_goal= self.tfListener.transformPose("/base_footprint", goal)
        print(self.local_goal)
```
```python
    def callback2(self,data):
        #Do some work
        cmd= Twist()
        cmd.linear.x= 0.1
        cmd.angular.z = self.local_goal.pose.orientation.z
        self.pub.publish(cmd)
```
## Résultats
## Perspectives d'amélioration
