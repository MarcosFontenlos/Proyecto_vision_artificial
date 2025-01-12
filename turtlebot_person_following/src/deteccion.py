#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class PersonDetector:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node("person_detector", anonymous=True)
        
        # Suscriptor a la cámara RGB
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        # Publicador de comandos de movimiento
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # Herramienta para convertir entre ROS y OpenCV
        self.bridge = CvBridge()
	# Configuración del detector HOG para detección de personas
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
	
	print('inicializado nodo y parametros')
    def image_callback(self, msg):
        try:
            # Convierte la imagen de ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error al convertir la imagen: ", e)
            return

     # Detecta personas en la imagen
        boxes, _ = self.hog.detectMultiScale(cv_image, winStride=(8, 8), padding=(8, 8), scale=1.05)
        
        # Dibuja las cajas alrededor de las personas detectadas
        for (x, y, w, h) in boxes:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Publica comandos de movimiento si detecta personas
        twist = Twist()
        if len(boxes) > 0:
	    twist.linear.x = 0.2  # Avanzar hacia la persona
        else:
            twist.angular.z = 0.1  # Girar buscando personas
        self.cmd_pub.publish(twist)

        # Muestra la imagen en una ventana
        cv2.imshow("Person Detector", cv_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        detector = PersonDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
