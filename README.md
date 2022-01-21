# grp-olive Challenge 3
## Sommaire
- [Présentation du projet](#Présentation-du-projet)
- [Elements requis](#Elements-requis)
- [Présentation du code](#Présentation-du-code)
- [Lien Vidéo](#Lien-vidéo)
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
On lance aussi un "gmapping" afin de cartopgraphier l'environnement, avec une limitation du laser à 4m pour la fiabilité.
```bash
	<node pkg="gmapping" type="slam_gmapping" name="gmapping_node" output="screen" >
		<param name="maxUrange" value="4.0"/> <!-- limit laser range to 4 meters -->
		<param name="odom_frame" value="/odom"/>
	</node>
```
De plus, on démarre automatiquement le logiciel rViz dans la configuration spécifiée afin d'observer la construction de la carte et le placement des marqueurs.
```bash
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find grp-olive)/rviz/challenge3.rviz"/>
```
Il faut aussi lancer nos scripts, ici nous en utilisons deux "main" et "move"
```bash
	<node pkg="grp-olive" type="main.py" name="main" output="screen"></node>
	<node pkg="grp-olive" type="move.py" name="main" output="screen"></node>
```

```python
#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA
import cv2
from cv_bridge import CvBridge
import math
import tf

```
Les premières lignes servent à importer les librairies et classes nécessaires au projet.

```python
rospy.init_node('bottle_detector',anonymous=True)
myBottle = Bottle()
rospy.spin()
```
Ces lignes sont les dernières du code : on commence par initialiser notre node sous le nom "bottle_detector". Ensuite on instancie un objet Bottle que nous avons défini plus tôt, puis on lance la boucle infinie de gestion du robot grâce à la méthode spin() de la librairie rospy.
```python
    def __init__(self):
        self.tflistener = tf.TransformListener()
        self.sub = rospy.Subscriber("camera/color/image_raw",Image,self.detection)
        self.sub2 = rospy.Subscriber("camera/depth/image_rect_raw",Image,self.coords)
        self.pub = rospy.Publisher('bottle',MarkerArray, queue_size=1)
        self.pub2 = rospy.Publisher("bottle_in_base_footprint", PoseStamped, queue_size=1)
        self.sub3 = rospy.Subscriber("bottle_in_base_footprint",PoseStamped,self.convert)
        self.markerArray = MarkerArray() #List of the markers detected
        self.objx = 0 #declaration of X detected object in the frame
        self.objy = 0 #declaration of Y detected object in the frame
        self.detected = False #declaration of variable used if object detected
        self.countframes = 0 #count the frames an object has been detected
        self.allowdetection = True #allows detection


```
La méthode __init__ est un constructeur de notre classe. Cela signifie que c'est ici qu'il nous faut créer tous les paramètres qui seront utiles à plusieurs méthodes. Cela permet d'éviter au maximum les variables globales et permet aussi de créer autant de fois l'objet que l'on souhaite. Ainsi, s'il nous faut deux robots, sans variables globales ils pourront être totalement autonomes, sans se perturber l'un-l'autre. 
```python
    def pixtoangle(self, f,pix):
        ang = pix * CAMERA_ANGLE / f.shape[1] ##Calc angle from 0 to f.shape
        ang -= CAMERA_ANGLE/2 #Apply offset to set 0 in the middle
        #print("DBG : Angle = " + str(ang))
        #print("DBG : where x = "+ str(pix))
        return math.radians(-ang) #-ang is for the Y values

    def coords(self,img):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(img,desired_encoding="passthrough") #convert the image from topic sent to list of distances by pixel
        if self.detected and self.countframes>10:
            self.detected=False
            self.allowdetection=False
            
            if(math.isnan(frame[self.objy,self.objx])):
                self.allowdetection=True
            else:
                self.publish(self.pixtoangle(frame, self.objx),frame[self.objy,self.objx])
```
pixtoangle est une méthode interne à la classe qui permet de traduire un pixel d'abscisse sur l'image en un angle en prenant pour repère le milieu de l'image. cela permet ainsi de se placer dans le repère 'base_footprint'
coords permet de récupérer la distance qui sépare le robot de l'objet détecté grâce au topic /camera/depth/image_raw

```python

    def detection(self,img):
        bridge = CvBridge()     
        frame = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8') #convert the image from topic sent to readable image for opencv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)        #Conert from BRG to HSV
        hsv = cv2.GaussianBlur(hsv,(7,3),1/9)               #Reduce noise
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)       #Get a gray image
        mask = cv2.inRange(hsv,(0,97*255/100,60*255/100),(40,255,255))  #sets the color (HSV) range to detect
        mask = cv2.dilate(mask,(3,3),iterations=1)
        mask = cv2.erode(mask,(3,3),iterations=1)
        #mask2 = cv2.inRange(hsv,(20,20,20),(self.i,self.i,self.i))     #To detect the black bottles (not working, too much black)
        detect = cv2.bitwise_and(gray,gray,mask=mask)   #get only the detected pixels
        color_infos = 255
        elements=cv2.findContours(detect, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  #Find the contours of the objects detected
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)  #Get the position of object in the frame
            if rayon>15:
                if self.allowdetection:
                    self.objx = int(x)  # X of the object on the image
                    self.objy = int(y)  # Y of the object on the image
                    self.countframes +=1  #Add 1 to the number of successive frames the object has been detected
                    self.detected=True  # sets a bool to True => An object has been detected (useful for avoiding errors of detection)
                cv2.circle(frame, (int(x), int(y)), int(rayon), color_infos, 2)
                cv2.circle(frame, (int(x), int(y)), 5, color_infos, 10)
                cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), color_infos, 2)
                cv2.putText(frame, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_infos, 1, cv2.LINE_AA)
            else:
                # if nothing detected, reset all the values. 
                self.detected = False
                self.allowdetection=True
                self.countframes = 0
        #cv2.imshow('frame', frame)
        cv2.imshow('detected',detect)
        if cv2.waitKey(1)&0xFF==ord('q'):
            cv2.destroyAllWindows()

```
La fonction Detection récupère l'image de la caméra et effectue différent traitements afin de déterminer les bouteilles sur les images. Si un objet a été détecté, on récupère ses coordonnées sur l'image qui seront utilisées par la fonction coords vue précédemment pour obtenir la distance avec l'objet. 
```python
        def publish(self, angle,d):
        # Create our PoseStamped object of the detected bottle with the coordinates in 'base_footprint' in order to be converted for 'map'

        obj = PoseStamped()
        obj.header.frame_id="base_footprint"
        #print("DBG : d = " + str(d))
        obj.pose.position.y= d * math.sin(angle)
        obj.pose.position.x= d * math.cos(angle)
        obj.pose.position.z = 0
        obj.pose.orientation = Quaternion()
        #print("DBG : Obj")
        #print(obj)
        self.pub2.publish(obj)

    def convert(self, obj):
        # Convert the PoseStamped object and get the coordinates in 'map'
        # then Publish a /bottle topic (Marker) with the coordinates. 

        #print("Got an object")
        globalpos = self.tflistener.transformPose('map',obj)
        marker= Marker( #declaration of a marker
                id =len(self.markerArray.markers)+1,
                type=Marker.CUBE,
                lifetime=rospy.Duration(0),
                scale=Vector3(0.1, 0.1, 0.1),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                pose = globalpos.pose)
        marker.header.frame_id='map'
        #print("DBG : Marker")
        #print(marker)
        self.markerArray.markers.append(marker)
        self.pub.publish(self.markerArray)
```
La fonction publish permet de publier un topic détaillant une PoseStamped dans le repère 'base_footprint' qui correspond à la position de la bouteille dans le repère mobile du robot. Ce topic est donc récupérable. 
La fonction convert est active dès que le topic défini précédemment est communiqué. Elle permet donc de changer de repère et d'envoyer un topic /bottle comme Marker avec les coordonnées approximatives de l'objet détecté dans le repère fixe 'map' cette fois. 

## Lien vidéo

La vidéo de présentation du projet est disponible à cette adresse: 

## Perspectives d'amélioration
 - La détection des bouteilles a été optimisée mais reste toujours améliorable, en mixant des techniques par exemple (detection par hsv puis kmeans)
 - Il est toujours possible d'améliorer la précision de la détection des coordonnées d'une bouteille après détection. 
 - Si le robot passe deux fois près d'une bouteille, il enregistrera deux objets différents sur Rviz. Cela est dû à la méthode de stockage des objets détectés (ajout dans une liste de 'Markers'). Une amélioration possible serait de prendre en compte les coordonnées de l'objet à ajouter et vérifier si l'on a déjà pas un objet à cet emplacement. 
 - Le déplacement du robot peut être optimisé largement. Pour l'instant le robot se déplace simplement dans son environnement de façon un peu aléatoire tout en évitant les obstacles. Une amélioration possible serait de récupérer les zones d'inconnues du robot et de le diriger par là via des algorithmes de path_finding.
 - Comme le robot se déplace en évitant les obstacles seulement sans prendre de décision de point à atteindre, il se peut qu'il aille se coincer à tourner indéfiniment sur un même chemin. Cela peut être crée par des zones recluses de l'environnement. 
