#!/usr/bin/env python

import rospy
import roslaunch
from sensor_msgs.msg import Joy, CompressedImage
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import cv2
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion #Vektor uzayinda donusturmeler icin...
import actionlib #Server-Client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #Hedef koordinat gonderme...
import math

uuid = roslaunch.rlutil.get_or_generate_uuid( None, False )
roslaunch.configure_logging( uuid )
joy_launch_file = "/home/zeobora/Desktop/TemizlikRobotu/launch/joy.launch"
roslaunch_parent = roslaunch.parent.ROSLaunchParent( uuid, [joy_launch_file] )
pub = rospy.Publisher( 'compressed_image_topic_2', CompressedImage, queue_size = 1 )

hiz=Twist()
hiz.linear.x = 0.0
hiz.angular.z = 0.0

maxHizSiniri = 0.5 
minHizSiniri = -0.5
hizDegistirmeMiktari = 0.01
basamakSayisi = maxHizSiniri / hizDegistirmeMiktari 

def HizAlma( hizVerisi ):
    global hiz, kontrol
    if kontrol == 1 or kontrol == -2:
        hiz.linear.x = hizVerisi.linear.x
        hiz.angular.z = hizVerisi.angular.z
        rospy.Publisher( "/cmd_vel", Twist, queue_size = 1 ).publish( hiz ) 
        print("Lineer Hiz: ", hiz.linear.x)
        print("Acisal Hiz: ", hiz.angular.z)

def HizYayinlama():
    global hiz, hizDegistirmeMiktari
    rospy.Publisher( "/cmd_vel", Twist, queue_size = 1 ).publish( hiz ) 
    print("Lineer Hiz: ", hiz.linear.x)
    print("Acisal Hiz: ", hiz.angular.z)

def Dur():
    global hiz, hizDegistirmeMiktari
    print("DUR!")
    for i in range( int( abs( ( hiz.linear.x ) * 10 * ( 0.1 / hizDegistirmeMiktari ) ) ) ):
        if hiz.linear.x > 0:
            hiz.linear.x -= hizDegistirmeMiktari
        else:
            hiz.linear.x += hizDegistirmeMiktari
        HizYayinlama()
    for i in range( int( abs( (hiz.angular.z) * 10 * ( 0.1 / hizDegistirmeMiktari ) ) ) ):
        if hiz.angular.z > 0:
            hiz.angular.z -= hizDegistirmeMiktari
        else:
            hiz.angular.z += hizDegistirmeMiktari
            HizYayinlama()

def LineerHizArttirma():
    global hiz, hizDegistirmeMiktari
    if hiz.linear.x < maxHizSiniri:
        hiz.linear.x += hizDegistirmeMiktari
        HizYayinlama()

def LineerHizAzaltma():
    global hiz, hizDegistirmeMiktari
    if hiz.linear.x > minHizSiniri:
        hiz.linear.x -= hizDegistirmeMiktari
        HizYayinlama()

def AcisalHizArttirma():
    global hiz, hizDegistirmeMiktari
    if hiz.angular.z < maxHizSiniri:
        hiz.angular.z += hizDegistirmeMiktari
        HizYayinlama()

def AcisalHizAzaltma():
    global hiz, hizDegistirmeMiktari
    if hiz.angular.z > minHizSiniri:
        hiz.angular.z -= hizDegistirmeMiktari
        HizYayinlama()

def JoystickAcisalHizAyarlama( veri ):
    global hiz
    hiz.angular.z = veri
    HizYayinlama()

def JoystickLineerHizAyarlama( veri ):
    global hiz
    hiz.linear.x = veri
    HizYayinlama()

def OtonomIslemler( kameraVerisi ):
    global kontrol
    np_arr = np.frombuffer(kameraVerisi.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    frame = cv2.resize(frame, (1280, 720))
    encode_param = [ int( cv2.IMWRITE_JPEG_QUALITY ), 100 ]
    ( _, img_encoded ) = cv2.imencode( '.jpeg', frame, encode_param )
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = img_encoded.tobytes()
    pub.publish(msg)
    if kontrol == 2:
        print("Otonom Hareket")
    
kontrol = -1
def KontrolIslemleri( kontrolVerisi ):
    global roslaunch_parent, kontrol
    kontrol = int( kontrolVerisi.data )
    print("Kontrol: ", kontrol)
    if kontrol == -1:
        Dur()
        roslaunch_parent.shutdown()
    elif kontrol == 0:
        roslaunch_parent = roslaunch.parent.ROSLaunchParent( uuid, [joy_launch_file] )
        roslaunch_parent.start()
        rospy.loginfo("Joystick Moda Geciliyor!")
    elif kontrol == 1:
        roslaunch_parent.shutdown()
        rospy.loginfo("Arayuz Moda Geciliyor")
    elif kontrol == 2:
        rospy.loginfo("Otonom Moda Geciliyor")

def JoystickIslemleri( joystickVerisi ):
    global basamakSayisi
    if joystickVerisi.buttons[0] == 1: #Yesil tus
        Dur()
    if joystickVerisi.axes[7] == 1: #Siyah ust tus
        LineerHizArttirma()
    elif joystickVerisi.axes[7] == -1: #Siyah alt tus
        LineerHizAzaltma()
    if joystickVerisi.axes[6] == 1: #Siyah sol tus
        AcisalHizArttirma()
    elif joystickVerisi.axes[6] == -1: #Siyah sag tus
        AcisalHizAzaltma() 
    if joystickVerisi.axes[3] != 0: #Sag Joysitck Sola-Saga
        JoystickAcisalHizAyarlama( hizDegistirmeMiktari * int( joystickVerisi.axes[3] * ( basamakSayisi ) ) ) #Hizi olceklendirip gonderiyoruz.
    if joystickVerisi.axes[1] != 0: #Sol Joystick Ileri-Geri
        JoystickLineerHizAyarlama( hizDegistirmeMiktari * int( joystickVerisi.axes[1] * ( basamakSayisi ) ) )

x = 0.0
y = 0.0
def HaritaIslemleri( haritaVerisi ):
    global kontrol, x, y
    print("( ", haritaVerisi.x, " , ", haritaVerisi.y, " )")
    x = ( haritaVerisi.x - 185) * 0.048
    y = ( 138 - haritaVerisi.y ) * 0.048
    print("( ", x, " , ", y, " )")
    HedefNoktayaGitme()

def HedefNoktayaGitme():
    global x, y
    istemci = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    istemci.wait_for_server()
    hedef = MoveBaseGoal()
    hedef.target_pose.header.frame_id = "map"
    hedef.target_pose.pose.position.x = x #Hedef Koordinatlar
    hedef.target_pose.pose.position.y = y
    hedef.target_pose.pose.orientation = Quaternion( *quaternion_from_euler ( 0.0, 0.0, 0.0 ) ) #Hedef korrdinata gelince istenilen roll, pitch ve yaw degerleri.
    istemci.send_goal(hedef)
    sonuc = istemci.wait_for_result( timeout = rospy.Duration( 30 ) ) #Hedefe gitmesi icin 30 saniye veriliyor.
    if sonuc == True: #30 saniyede hedefe varirsa...
        print("Hedefe Varildi!") 
    else: #30 saniyede hedefe varamazsa hedefi iptal ediyor.
        print("Hedefe Zamaninda Varilamadi")
        istemci.cancel_goal()
        print("Durduruluyor!")
        Dur()

def OdomIslemleri( odomVerisi ):
    ok = Point()
    ok.x = ( odomVerisi.pose.pose.position.x / 0.048 ) + 185
    ok.y = ( ( -1 ) * ( odomVerisi.pose.pose.position.y / 0.048 ) ) + 138
    rot_q = odomVerisi.pose.pose.orientation
    anlikYaw = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])[2]
    ok.z = math.degrees( anlikYaw ) * (-1)
    rospy.Publisher( "/ok", Point, queue_size=1 ).publish( ok ) 

def kapatma():
    rospy.loginfo("Kapatiliyor!")
    Dur()
    
roslaunch.pmon._init_signal_handlers()
rospy.init_node('joy_topic_launcher', anonymous=True)   
rospy.Subscriber('/cmd_vel1', Twist, HizAlma) 
rospy.Subscriber("/joy", Joy, JoystickIslemleri) 
rospy.Subscriber("/modKontrolu", String, KontrolIslemleri) 
rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, OtonomIslemler) 
rospy.Subscriber("/clicked_point", Point, HaritaIslemleri)
rospy.Subscriber("/odom", Odometry, OdomIslemleri)
rospy.on_shutdown(kapatma)
rospy.spin()
