# grp-olive Challenge 3
## Sommaire
- [Présentation du projet](#Présentation-du-projet)
- [Elements requis](#Elements-requis)
- [Présentation du code](#Présentation-du-code)
- [Résultats](#Résultats)
- [Perspectives d'amélioration](#Perspectives-d'amélioration)
## Présentation du projet
L'objectif de ce challenge est de démontrer la capacité d'un robot à cartographier un environnement et à retrouver des objets spécifiques dans ce dernier.

## Elements requis
Il faut au préalable avoir installé Gmapping ainsi que le package mb6-tbot
## Présentation du code

### Présentation du fichier de lancement (launchfile)
On a tout d'abord spécifié l'utilisation d'une horloge simulée afin de jouer un rosbag avec l'option "clock".
```bash
<param name="/use_sim_time" value="true" />
```
On lance aussi un "gmapping" afin de cartopgraphier l'environnement.
```bash
<node pkg="gmapping" type="slam_gmapping" name="gmapping_node" output="screen" >
```
De plus, on démarre automatiquement le logiciel rViz dans la configuration spécifiée afin d'observer la construction de la carte et le placement des marqueurs.
```bash
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find grp-olive)/rviz/challenge2.rviz"/>
```

```python
#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA
import cv2
from cv_bridge import CvBridge
import os
```
Les premières lignes servent à importer les librairies et classes nécessaires au projet.
```python
rospy.init_node('bottle_detector',anonymous=True)
myNode = MyNode()
rospy.spin()
```
Ces lignes sont les dernières du code : on commence par initialiser notre node sous le nom "bottle_detector". Ensuite on instancie un objet myNode que nous avons défini plus tôt, puis on lance la boucle infinie de gestion du robot grâce à la méthode spin() de la librairie rospy.
```python
def __init__(self):
        self.odom = rospy.Subscriber("/odom",Odometry,self.getPose)
        self.sub = rospy.Subscriber("camera/color/image_raw",Image,self.detection)
        self.pub = rospy.Publisher(
            '/bottle',
            Marker, queue_size=5
        )
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, 'cascade.xml')
        self.classifier = cv2.CascadeClassifier(filename)
        self.marker= Marker()

```
L'instanciation de la classe myNode commence par créer deux Subscriber qui vont permettre de récupérer un objet "Odometry" du robot via le topic 
/odom puis on envoie les données dans la fonction getPose. Le second Subscriber se connecte au topic /camera/color/image_raw afin de récupérer l'image de la caméra et d'envoyer les données dans la foncition detection de notre classe.  La commande Publisher indique au roscore que les données vont être publiées sur le topic '/bottle'. Ce topic est écouté par le robot, il interprètera des données de type Marker, qui est une classe contenant la 'pose' du robot, c'est à dire position+orientation. Ensuite, on récupère les données du fichier cascade.xml situé dans le même dossier que ce script. on crée ensuite un objet classifier qui sera utilisé dans une autre fonction. On procède de même avec la création d'un objet Marker, qui servira à faire le lien entre deux fonctions. 
```python
    def detection(self,img):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        objs = self.classifier.detectMultiScale(frame, 1.1, 0)

        for (x, y, w, h) in objs:
            cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,0), 2)
            self.publish()
        #cv2.imshow('bot view', gray)
        #cv2.imshow('frame', frame)
        if cv2.waitKey(1)&0xFF==ord('q'):
            cv2.destroyAllWindows()
```
La fonction Detection récupère l'image de la caméra et utilise la cascade calculée afin de déterminer les bouteilles sur les images. Si un objet a été détecté, alors on appelle la fonction 'publish' de notre classe
```python
    def getPose(self,data):
        self.marker.pose = data.pose.pose

    def publish(self):
        cmd = Marker(
                type=Marker.CUBE,
                id=0,
                lifetime=rospy.Duration(0.5),
                pose=self.marker.pose,
                scale=Vector3(0.1, 0.1, 0.1),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        cmd.header.frame_id='map'
        self.pub.publish(cmd)
```
La fonction getPose permet de récupérer et stocker temporairement la pose du robot afin de permettre à 'publish' de récupérer cette pose et d'envoyer un objet Marker d'une forme cubique de 0.1 de côté et ainsi l'envoyer sur le topic /bottle.
## Résultats
## Perspectives d'amélioration
Pour le moment le robot ne détecte pas que des bouteilles, il y a aussi quelques erreurs. Une bonne perspective serait de mieux l'entraîner à détecter ces bouteilles.
