#!/usr/bin/env python

import signal
import rospy
import roslaunch
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion #Vektor uzayinda donusturmeler icin...
import actionlib #Server-Client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #Hedef koordinat gonderme...
from math import pi, degrees
import threading
import subprocess
import os

uuid = roslaunch.rlutil.get_or_generate_uuid( None, False )
roslaunch.configure_logging( uuid )
joy_launch_file = "../launch/joy.launch"
roslaunch_parent = roslaunch.parent.ROSLaunchParent( uuid, [joy_launch_file] )

hiz=Twist()
hiz.linear.x = 0.0
hiz.angular.z = 0.0

maxHizSiniri = 0.5 
minHizSiniri = -0.5
hizDegistirmeMiktari = 0.01
basamakSayisi = maxHizSiniri / hizDegistirmeMiktari 

cmd = ['pgrep', "python3"]
output = subprocess.check_output(cmd)
pids = output.decode().splitlines()

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

def DurHizYayinlama():
    global hiz, hizDegistirmeMiktari
    rospy.Publisher( "/cmd_vel", Twist, queue_size = 1 ).publish( hiz ) 

def Dur():
    global hiz, hizDegistirmeMiktari
    print("DUR!")
    for i in range( int( abs( ( hiz.linear.x ) * 10 * ( 0.1 / hizDegistirmeMiktari ) ) ) ):
        if hiz.linear.x > 0:
            hiz.linear.x -= hizDegistirmeMiktari
        else:
            hiz.linear.x += hizDegistirmeMiktari
        DurHizYayinlama()
    for i in range( int( abs( (hiz.angular.z) * 10 * ( 0.1 / hizDegistirmeMiktari ) ) ) ):
        if hiz.angular.z > 0:
            hiz.angular.z -= hizDegistirmeMiktari
        else:
            hiz.angular.z += hizDegistirmeMiktari
        DurHizYayinlama()

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
        
kontrol = -1
istemci=actionlib.SimpleActionClient("/move_base", MoveBaseAction)
a = 0

yukariMiGidiliyor=1 #Yukari Gidiyorsa -> 1 ; Asagi Gidiyorsa -> 0
def YilanAlgoritmasi():
    global engel, yukariMiGidiliyor, sonSutun
    while ( kontrol == 2 ):
        if yukariMiGidiliyor == 1:
            print("Yukariya Gidilmeye Baslaniyor...")
            YukaridakiEngelinTespitiVeEngeleGidis() #Engel/Duvar Tespiti
            while engel==1 and kontrol == 2:
                YukaridakiEngelinDigerTarafininTespitiVeGidis() #Engelin diger tarafina her gidisinde sutun tamamlanmamis olacak.
                engel=0
                YukaridakiEngelinTespitiVeEngeleGidis() #Bundan dolayi bir daha engel/duvar tespiti yapilir.
            print("Bir Sutun Tamamlandi!")
            if sonSutun==1 and kontrol == 2: #Son Sutun Tamamlandiysa...
                print("Oda Tamamlandi!")
                kapatma()
            SagaGidis()
            yukariMiGidiliyor=0
        elif yukariMiGidiliyor==0:
            print("Asagiya Gidilmeye Baslaniyor...")
            AsagidakiEngelinTespitiVeEngeleGidis() #Engel/Duvar Tespiti
            while engel==1 and kontrol == 2:
                AsagidakiEngelinDigerTarafininTespitiVeGidis() #Engelin diger tarafina her gidisinde sutun tamamlanmamis olacak.
                engel=0
                AsagidakiEngelinTespitiVeEngeleGidis() #Bundan dolayi bir daha engel/duvar tespiti yapilir.
            print("Bir Sutun Tamamlandi!")
            if sonSutun==1: #Son Sutun Tamamlandiysa...
                print("Oda Tamamlandi!")
                kapatma()
            SolaGidis()
            yukariMiGidiliyor=1

thread = threading.Thread( target = YilanAlgoritmasi )
def KontrolIslemleri( kontrolVerisi ):
    global roslaunch_parent, kontrol, istemci, a, thread
    print("Buraya geldi")
    kontrol = int( kontrolVerisi.data )
    print("Kontrol: ", kontrol)
    if kontrol == -1:
        Dur()
        roslaunch_parent.shutdown()
    elif kontrol == 0:
        roslaunch_parent = roslaunch.parent.ROSLaunchParent( uuid, [joy_launch_file] )
        roslaunch_parent.start()
        print("Joystick Moda Geciliyor!")
    elif kontrol == 1:
        roslaunch_parent.shutdown()
        print("Arayuz Moda Geciliyor")
    elif kontrol == 2:
        print("Otonom Moda Geciliyor")
        if a == 0:
            a = 1
            thread.start()
            
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
    global kontrol, x, y, istemci
    if kontrol == 1:
        istemci.cancel_goal()
        print("Arayuz Modunda Haritaya Tiklandi, Tiklanan Yere Gidiliyor!")
        x = ( haritaVerisi.x - 176) * 0.05
        y = ( 116 - haritaVerisi.y ) * 0.05
        HedefNoktayaGitme1()

def HedefNoktayaGitme1():
    global x, y, istemci
    istemci.wait_for_server()
    hedef = MoveBaseGoal()
    hedef.target_pose.header.frame_id = "map"
    hedef.target_pose.pose.position.x = x #Hedef Koordinatlar
    hedef.target_pose.pose.position.y = y
    hedef.target_pose.pose.orientation = Quaternion( *quaternion_from_euler ( 0.0, 0.0, 0.0 ) ) #Hedef korrdinata gelince istenilen roll, pitch ve yaw degerleri.
    istemci.send_goal(hedef)
    sonuc = istemci.wait_for_result( timeout = rospy.Duration( 60 ) ) #Hedefe gitmesi icin 30 saniye veriliyor.
    if sonuc == True: #30 saniyede hedefe varirsa...
        print("Hedefe Varildi!") 
    else: #30 saniyede hedefe varamazsa hedefi iptal ediyor.
        print("Hedefe Zamaninda Varilamadi")
        istemci.cancel_goal()
        print("Durduruluyor!")
        Dur()

anlikYaw = 0.0
anlikX = 0.0
anlikY = 0.0
def OdomIslemleri( odomVerisi ):
    global anlikYaw, anlikX, anlikY, kontrol
    ok = Point()
    anlikX = odomVerisi.pose.pose.position.x
    anlikY = odomVerisi.pose.pose.position.y
    ok.x = ( odomVerisi.pose.pose.position.x / 0.05 ) + 176
    ok.y = ( ( -1 ) * ( odomVerisi.pose.pose.position.y / 0.05 ) ) + 116
    rot_q = odomVerisi.pose.pose.orientation
    anlikYaw = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])[2]
    ok.z = degrees( anlikYaw ) * (-1)
    rospy.Publisher( "/ok", Point, queue_size=1 ).publish( ok ) 

liste=[] #En basta gelecek verilerin tutulacagi liste...
resimdekiSatirPikselSayisi=resimdekiSutunPikselSayisi=0
sonMatris=[] #Yeni listemiz (En sonda matris olacak.)

def HaritaVerisi(msj):
    global liste, resimdekiSatirPikselSayisi, resimdekiSutunPikselSayisi, sonMatris
    print("Resim Okunuyor...")
    resimdekiSatirPikselSayisi=msj.info.height #Resimin piksel sayilari aliniyor. (Default: 384x384)
    resimdekiSutunPikselSayisi=msj.info.width
    print("Resmin boyutu elde edildi.")
    print("Pikseller inceleniyor...")
    liste=list(msj.data) #Resimin renklerinden piksellerin durumu (-1, 0, 100) aliniyor.
    geciciMatris=np.array(liste) #Listeyi, matrise ceviriyoruz. (Gecici Matris)
    geciciMatris=geciciMatris.reshape(resimdekiSatirPikselSayisi, resimdekiSutunPikselSayisi) #Matrisimize sekil veriyoruz.
    minIndeks=resimdekiSatirPikselSayisi
    maxIndeks=0
    for i in range (0, resimdekiSatirPikselSayisi): #Haritanin bir satirindaki -1 harici sayilarin hangi iki index arasinda oldugunu buluyoruz.
        for j in range (0, resimdekiSutunPikselSayisi): #Yani en soldaki -1 harici sayi ile en sagdaki -1 harici sayinin indeksi bulunuyor.
            if geciciMatris[i][j]!=-1 and j<minIndeks:
                minIndeks=j 
            if geciciMatris[i][j]!=-1 and j>maxIndeks:
                maxIndeks=j
    sonMatris.append([])
    eksiBirSayisi=0 #Bir satirdaki -1 adetini tutacak.
    satirSayisi=0 #Yeni listemizdeki satir sayisi
    for i in range (0, resimdekiSatirPikselSayisi): #Bulunan indexler arasindaki verileri aliyoruz, boylece haritanin tamami gelmis oluyor.
        for j in range (minIndeks-1, maxIndeks+1):
            sonMatris[satirSayisi].append(geciciMatris[i][j]) #Matrise eklemeleri yapiyoruz.
        if sonMatris[satirSayisi]!=[]:
            eksiBirSayisi=0
            for k in range(0, len(sonMatris[satirSayisi])): #Bir satirdaki -1 sayisi bulunuyor.
                if sonMatris[satirSayisi][k]==-1:
                    eksiBirSayisi+=1
            if eksiBirSayisi==len(sonMatris[satirSayisi]): #Bir satirdaki tum elemanlar -1'se, o satiri siliyoruz.
                del sonMatris[satirSayisi]
                satirSayisi-=1
            sonMatris.append([]) #Yeni satir ekleniyor.
            satirSayisi+=1

    if sonMatris[satirSayisi]==[]: #Son satiri bos gelebilecegi icin bossa siliyoruz.
        del sonMatris[satirSayisi]
        satirSayisi-=1
    print(satirSayisi)
    sonMatris=np.array(sonMatris) #Listemizi matrise ceviriyoruz.
    print("Evin Matrisi Basariyla Olusturuldu!")

#Sutun -> X Ekseni ; Satir -> Y Ekseni
#Her piksel 5 cm

baslangicSutunPikseli=8 #Robotun baslangica bulundugu koordinatlarinin (-8.5, -5) sol alt koseye (-8, -5,5) gore pikselleri
baslangicSatirPikseli=8 

# baslangicSutunPikseli=150 #Robotun baslangica bulundugu koordinatlarinin (-8.5, -5) sol alt koseye (-8, -5,5) gore pikselleri
# baslangicSatirPikseli=10 

# baslangicSutunPikseli=270 #Robotun baslangica bulundugu koordinatlarinin (-8.5, -5) sol alt koseye (-8, -5,5) gore pikselleri
# baslangicSatirPikseli=10 

# baslangicSutunPikseli=150 #Robotun baslangica bulundugu koordinatlarinin (-8.5, -5) sol alt koseye (-8, -5,5) gore pikselleri
# baslangicSatirPikseli=180 

orijinSutunPikseli=178 #Orijinin korrdinatlarinin (0,0) sol alt koseye (-8, -5,5) gore pikselleri
orijinSatirPikseli=100 

anlikSatirPikseli=baslangicSatirPikseli #Robotun bulundugu anlik koordinatlarin pikselleri
anlikSutunPikseli=baslangicSutunPikseli

engelSutunPiksel=0 #Engelin Pikselleri
engelSatirPiksel=0

#Oda1: 0-180 Satir - 0-140 Sutun
odaSatirUstPiksel=180 #Odalarin duvarlarinin pikselleri
odaSatirAltPiksel=0
OdaSutunSagPiksel=130
OdaSutunSolPiksel=0

# odaSatirUstPiksel=170 #Odalarin duvarlarinin pikselleri
# odaSatirAltPiksel=0
# OdaSutunSagPiksel=250
# OdaSutunSolPiksel=140

# odaSatirUstPiksel=220 #Odalarin duvarlarinin pikselleri
# odaSatirAltPiksel=0
# OdaSutunSagPiksel=370
# OdaSutunSolPiksel=270

# odaSatirUstPiksel=220 #Odalarin duvarlarinin pikselleri
# odaSatirAltPiksel=170
# OdaSutunSagPiksel=270
# OdaSutunSolPiksel=140

#Yaw Degerleri:
#1.57  -> Kuzey
#-1.57 -> Guney
#0.0   -> Dogu
#3.14  -> Bati

#Sutun -> X Ekseni ; Satir -> Y Ekseni
#Her piksel 5 cm.

def HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, hedefYaw):
    global anlikSatirPikseli, anlikSutunPikseli, orijinSatirPikseli, orijinSutunPikseli, anlikX, anlikY, kontrol
    while ( kontrol != 2):
        continue
    hedefX=((hedefSutunPikseli-orijinSutunPikseli)*0.05) #Hedef Koordinatlar hesaplaniyor.
    hedefY=((hedefSatirPikseli-orijinSatirPikseli)*0.05)
    sure=((((anlikX-hedefX)**2)+((anlikY-hedefY)**2))**0.5)*5
    print ("Hedef X: ", '%.2f' % hedefX, ", Hedef Y: ", '%.2f' % hedefY, ", Hedef Yaw: ", '%.2f' % hedefYaw, ", Sure: ", '%.2f' % sure)
    HedefNoktayaGitme(hedefX, hedefY, hedefYaw, sure)
    anlikSatirPikseli=hedefSatirPikseli
    anlikSutunPikseli=hedefSutunPikseli

def SolaGidis():
    global orijinSatirPikseli, orijinSutunPikseli, anlikSatirPikseli, anlikSutunPikseli, OdaSutunSagPiksel, sonSutun, kontrol
    print("Sola Gidiliyor...")
    if anlikSutunPikseli+20>OdaSutunSagPiksel-9:
        sonSutun=1
        print("Odanin Son Sutunu...")
        gidilecekPiksel=OdaSutunSagPiksel-9-anlikSutunPikseli
    else:
        gidilecekPiksel=20
    hedefSatirPikseli=anlikSatirPikseli
    hedefSutunPikseli=anlikSutunPikseli+gidilecekPiksel
    HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 1.57)
    
def SagaGidis():
    global orijinSatirPikseli, orijinSutunPikseli, anlikSatirPikseli, anlikSutunPikseli, sonMatris, odaSatirAltPiksel, odaSatirUstPiksel, OdaSutunSagPiksel, sonSutun, kontrol
    print("Saga Gidiliyor...")
    if anlikSutunPikseli+20>OdaSutunSagPiksel-9:
        sonSutun=1
        print("Odanin Son Sutunu...")
        gidilecekPiksel=OdaSutunSagPiksel-9-anlikSutunPikseli
    else:
        gidilecekPiksel=20
    hedefSatirPikseli=anlikSatirPikseli
    hedefSutunPikseli=anlikSutunPikseli+gidilecekPiksel
    gecicikontrol1=0
    if(sonMatris[hedefSatirPikseli][hedefSutunPikseli]!=0):
        print("Sag Tarafinda Engel Tespit Edildi! Yeni Hedef Hesaplaniyor...")
        for i in range(hedefSatirPikseli, odaSatirAltPiksel, -1):
            if(sonMatris[i][hedefSutunPikseli]==0):
                HedefKoordinatHesaplama(anlikSatirPikseli, anlikSutunPikseli, -1.57)
                HedefKoordinatHesaplama(i-9, anlikSutunPikseli, 0.0)
                HedefKoordinatHesaplama(i-9, hedefSutunPikseli, -1.57)
                gecicikontrol1=1
                break
    elif(sonMatris[hedefSatirPikseli+10][hedefSutunPikseli]==0):
        for i in range(hedefSatirPikseli, odaSatirUstPiksel):
            if(sonMatris[i][hedefSutunPikseli]==0 and sonMatris[i+1][hedefSutunPikseli]!=0):
                HedefKoordinatHesaplama(anlikSatirPikseli, hedefSutunPikseli, 1.57)
                HedefKoordinatHesaplama(i-9, anlikSutunPikseli, -1.57)
                gecicikontrol1=1
                break
    if gecicikontrol1==0:
        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, -1.57)

engel=0 #Gittigi Yonde Engel Varsa -> 1 ; Duvar Varsa -> 0
def AsagidakiEngelinTespitiVeEngeleGidis():
    print("AsagidakiEngelinTespitiVeEngeleGidis")
    global sonMatris, orijinSatirPikseli, orijinSutunPikseli, anlikSatirPikseli, anlikSutunPikseli, engelSutunPiksel, engelSatirPiksel, engel, odaSatirAltPiksel, kontrol
    kontrol1=0
    for i in range(anlikSatirPikseli, odaSatirAltPiksel, -1): #Bulundugu satirdan, en asagidaki satira kadar...
        kontrol1=0
        if sonMatris[i][anlikSutunPikseli]==0 and sonMatris[i-1][anlikSutunPikseli]==100: #Engelin oldugu satiri (i-1) bulur.
            for j in range(i-1, odaSatirAltPiksel, -1): #Engele mi duvara mi gidiliyor...
                if sonMatris[j][anlikSutunPikseli]==100 and sonMatris[j-1][anlikSutunPikseli]==0 and sonMatris[j-2][anlikSutunPikseli]==0 and sonMatris[j-3][anlikSutunPikseli]==0:
                        engel=1 #Onunde engel var.
                        break
                if engel==1:
                    print("Engel Bulundu!")
                    engelSutunPiksel=anlikSutunPikseli
                    engelSatirPiksel=i
                    print("Engelin Koordinatlari: ", ((engelSutunPiksel-orijinSutunPikseli)*0.05), ((engelSatirPiksel-orijinSatirPikseli)*0.05))
                else:
                    print("Duvar Bulundu!")
                    print("Duvarin Koordinatlari: ", ((anlikSutunPikseli-orijinSutunPikseli)*0.05), ((i-orijinSatirPikseli)*0.05))
                    engel=0 #Her duvarin arkasi bosluk oldugu icin engel olamaz.
                if (anlikSatirPikseli-i)>=29: #Gidilecek piksel sayisi hesaplaniyor.
                    gidilecekPiksel=20 #Onunde 28 pikselden fazla yer varsa, 20 piksel git.
                else: #Onunde 28 pikselden az yer varsa...
                    if (anlikSatirPikseli-i)>9: #Onundeki piksel sayisi 9<piksel<28 ise...
                        gidilecekPiksel=anlikSatirPikseli-i-9 #9 piksel kalana kadar git.
                        kontrol1=1 #Duvar/Engelden onceki son gidis oldugunu belirtir.
                    else: #Onunde 8 pikselden az bosluk kaldiysa...
                        kontrol1=1
                        break
                while(1):
                    hedefSatirPikseli=anlikSatirPikseli-gidilecekPiksel
                    hedefSutunPikseli=anlikSutunPikseli
                    if kontrol1==0:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, -1.57) #Asagi giderken yonu hep asagi olsun.
                    elif kontrol1==1:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 0.0) #Son asagi gidisinde yonu sola baksin.
                    if (anlikSatirPikseli-i)>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                        gidilecekPiksel=20
                    else:
                        if (anlikSatirPikseli-i)>9:
                            gidilecekPiksel=anlikSatirPikseli-i-9
                            kontrol1=1
                        else:
                            break
                break

def AsagidakiEngelinDigerTarafininTespitiVeGidis():
    print("AsagidakiEngelinDigerTarafininTespitiVeGidis")
    global sonMatris, anlikSatirPikseli, anlikSutunPikseli, engelSutunPiksel, engelSatirPiksel, odaSatirUstPiksel, OdaSutunSagPiksel, OdaSutunSolPiksel, sonSutun, kontrol
    hatirlanacakSutunPikseli=anlikSutunPikseli #Engelin etrafindan dolandiginda ayni sutun pikseline gelmesi icin kullanilacak.
    print("Engelin Diger Tarafina Gidiliyor")
    if sonSutun==0:
        for i in range (engelSutunPiksel, OdaSutunSagPiksel):
            if sonMatris[engelSatirPiksel][i]==100 and sonMatris[engelSatirPiksel][i-1]==0: #Engele geldiginde sagdaki ilk bos pikseli bulur.
                hedefSatirPikseli=anlikSatirPikseli
                hedefSutunPikseli=i-9
                HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, -1.57)
                break
        kontrol1=0
        for i in range (anlikSatirPikseli, odaSatirAltPiksel, -1):
            if sonMatris[i-2][anlikSutunPikseli+9]==0 and sonMatris[i-1][anlikSutunPikseli+9]==0 and sonMatris[i][anlikSutunPikseli+9]==0 and sonMatris[i+1][anlikSutunPikseli+9]==100: #Engelin bittigi satiri (i) bulur.
                if (anlikSatirPikseli-i)>=29: #Gidilecek piksel sayisi hesaplaniyor.
                    gidilecekPiksel=20 #Onunde 28 pikselden fazla yer varsa, 20 piksel git.
                else: #Onunde 28 pikselden az yer varsa...
                    if (anlikSatirPikseli-i)>9: #Onundeki piksel sayisi 9<piksel<28 ise...
                        gidilecekPiksel=anlikSatirPikseli-i-9 #9 piksel kalana kadar git.
                        kontrol1=1 #Duvar/Engelden onceki son gidis oldugunu belirtir.
                    else: #Onunde 8 pikselden az bosluk kaldiysa...
                        break
                while(1):
                    hedefSatirPikseli=anlikSatirPikseli-gidilecekPiksel
                    hedefSutunPikseli=anlikSutunPikseli
                    if kontrol1==0:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, -1.57)
                    elif kontrol1==1:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 0.0)
                    if (anlikSatirPikseli-i)>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                        gidilecekPiksel=20
                    else:
                        if (anlikSatirPikseli-i)>9:
                            gidilecekPiksel=anlikSatirPikseli-i-9
                            kontrol1=1
                        else:
                            break
                break
    else:
        for i in range (engelSutunPiksel, OdaSutunSolPiksel, -1):
            if sonMatris[engelSatirPiksel][i]==100 and sonMatris[engelSatirPiksel][i-1]==0: #Engele geldiginde soldaki ilk bos pikseli bulur.
                hedefSatirPikseli=anlikSatirPikseli
                hedefSutunPikseli=i-9
                HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 1.57)
                sonSutun=0
                break
        kontrol1=0
        for i in range (anlikSatirPikseli, odaSatirUstPiksel):
            if sonMatris[i+2][anlikSutunPikseli+9]==0 and sonMatris[i+1][anlikSutunPikseli+9]==0 and sonMatris[i][anlikSutunPikseli+9]==0 and sonMatris[i-1][anlikSutunPikseli+9]==100: #Engelin bittigi satiri (i) bulur.
                if (i-anlikSatirPikseli)>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                    gidilecekPiksel=20
                else:
                    if (i-anlikSatirPikseli)>9:
                        gidilecekPiksel=i-anlikSatirPikseli+9
                        kontrol1=1
                    else:
                        break
                while(1):
                    hedefSatirPikseli=anlikSatirPikseli+gidilecekPiksel
                    hedefSutunPikseli=anlikSutunPikseli
                    if kontrol1==0:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 1.57)
                    elif kontrol1==1:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 0.0)
                    if i-anlikSatirPikseli>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                        gidilecekPiksel=20
                    else:
                        if (i-anlikSatirPikseli)>9:
                            gidilecekPiksel=i-anlikSatirPikseli+9
                            kontrol1=1
                        else:
                            break
                break
    hedefSatirPikseli=anlikSatirPikseli
    hedefSutunPikseli=hatirlanacakSutunPikseli
    HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 0.0)

sonSutun=0 #Gezilen Sutun Son Sutunsa -> 1 ; Degilse -> 0
def YukaridakiEngelinTespitiVeEngeleGidis():
    print("YukaridakiEngelinTespitiVeEngeleGidis")
    global sonMatris, orijinSatirPikseli, orijinSutunPikseli, anlikSatirPikseli, anlikSutunPikseli, engelSutunPiksel, engelSatirPiksel, engel, odaSatirUstPiksel, OdaSutunSagPiksel, sonSutun, kontrol
    kontrol1=0
    for i in range(anlikSatirPikseli, odaSatirUstPiksel+10): #Bulundugu satirdan, en yukaridaki satira kadar...
        if sonMatris[i][anlikSutunPikseli]==0 and sonMatris[i+1][anlikSutunPikseli]==100: #Engelin oldugu satiri (i+1) bulur.
            for j in range(i+1, odaSatirUstPiksel): #Engele mi duvara mi gidiliyor...
                if sonMatris[j][anlikSutunPikseli]==100 and sonMatris[j+1][anlikSutunPikseli]==0 and sonMatris[j+2][anlikSutunPikseli]==0 and sonMatris[j+3][anlikSutunPikseli]==0:
                    engel=1
                    break
            if engel==1:
                print("Engel Bulundu!")
                engelSutunPiksel=anlikSutunPikseli
                engelSatirPiksel=i+1
                print("Engelin Koordinatlari: ", ((anlikSutunPikseli-orijinSutunPikseli)*0.05), ((i+1-orijinSatirPikseli)*0.05))
            else:
                print("Duvar Bulundu!")
                print("Duvarin Koordinatlari: ", ((anlikSutunPikseli-orijinSutunPikseli)*0.05), ((i+1-orijinSatirPikseli)*0.05))
                engel=0
            if (i-anlikSatirPikseli)>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                gidilecekPiksel=20
            else:
                if (i-anlikSatirPikseli)>9:
                    gidilecekPiksel=i-anlikSatirPikseli-9
                    kontrol1=1
                else:
                    kontrol1=1
                    break
            while(1):
                hedefSatirPikseli=anlikSatirPikseli+gidilecekPiksel
                hedefSutunPikseli=anlikSutunPikseli
                if kontrol1==0:
                    HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 1.57)
                elif kontrol1==1:
                    if anlikSutunPikseli+10<OdaSutunSagPiksel:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 0.0)
                    else:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 3.14)
                        sonSutun=1
                if i-anlikSatirPikseli>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                    gidilecekPiksel=20
                else:
                    if (i-anlikSatirPikseli)>9:
                        gidilecekPiksel=i-anlikSatirPikseli-9
                        kontrol1=1
                    else:
                        break
            break

def YukaridakiEngelinDigerTarafininTespitiVeGidis():
    print("YukaridakiEngelinDigerTarafininTespitiVeGidis")
    global sonMatris, anlikSatirPikseli, anlikSutunPikseli, engelSutunPiksel, engelSatirPiksel, odaSatirUstPiksel, OdaSutunSagPiksel, OdaSutunSolPiksel, sonSutun, kontrol
    hatirlanacakSutunPikseli=anlikSutunPikseli #Engelin etrafindan dolandiginda ayni sutun pikseline gelmesi icin kullanilacak.
    print("Engelin Diger Tarafina Gidiliyor")
    if sonSutun==0:
        for i in range (engelSutunPiksel, OdaSutunSagPiksel):
            if sonMatris[engelSatirPiksel][i]==100 and sonMatris[engelSatirPiksel][i+1]==0: #Engele geldiginde sagdaki ilk bos pikseli bulur.
                hedefSatirPikseli=anlikSatirPikseli
                hedefSutunPikseli=i+9
                HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 1.57)
                break
        kontrol1=0
        for i in range (anlikSatirPikseli, odaSatirUstPiksel):
            if sonMatris[i+2][anlikSutunPikseli-9]==0 and sonMatris[i+1][anlikSutunPikseli-9]==0 and sonMatris[i][anlikSutunPikseli-9]==0 and sonMatris[i-1][anlikSutunPikseli-9]==100: #Engelin bittigi satiri (i) bulur.
                if (i-anlikSatirPikseli)>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                    gidilecekPiksel=20
                else:
                    if (i-anlikSatirPikseli)>9:
                        gidilecekPiksel=i-anlikSatirPikseli+5
                        kontrol1=1
                    else:
                        break
                while(1):
                    hedefSatirPikseli=anlikSatirPikseli+gidilecekPiksel
                    hedefSutunPikseli=anlikSutunPikseli
                    if kontrol1==0:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 1.57)
                    elif kontrol1==1:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 3.14)
                    if i-anlikSatirPikseli>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                        gidilecekPiksel=20
                    else:
                        if (i-anlikSatirPikseli)>9:
                            gidilecekPiksel=i-anlikSatirPikseli+9
                            kontrol1=1
                        else:
                            break
                break
    else:
        for i in range (engelSutunPiksel, OdaSutunSolPiksel, -1):
            if sonMatris[engelSatirPiksel][i]==100 and sonMatris[engelSatirPiksel][i-1]==0: #Engele geldiginde soldaki ilk bos pikseli bulur.
                hedefSatirPikseli=anlikSatirPikseli
                hedefSutunPikseli=i-9
                HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 1.57)
                sonSutun=0
                break
        kontrol1=0
        for i in range (anlikSatirPikseli, odaSatirUstPiksel):
            if sonMatris[i+2][anlikSutunPikseli+9]==0 and sonMatris[i+1][anlikSutunPikseli+9]==0 and sonMatris[i][anlikSutunPikseli+9]==0 and sonMatris[i-1][anlikSutunPikseli+9]==100: #Engelin bittigi satiri (i) bulur.
                if (i-anlikSatirPikseli)>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                    gidilecekPiksel=20
                else:
                    if (i-anlikSatirPikseli)>9:
                        gidilecekPiksel=i-anlikSatirPikseli+9
                        kontrol1=1
                    else:
                        break
                while(1):
                    hedefSatirPikseli=anlikSatirPikseli+gidilecekPiksel
                    hedefSutunPikseli=anlikSutunPikseli
                    if kontrol1==0:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 1.57)
                    elif kontrol1==1:
                        HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 0.0)
                    if i-anlikSatirPikseli>=29: #Gidilecek piksel sayisi hesaplaniyor. (194. satirda anlatildi.)
                        gidilecekPiksel=20
                    else:
                        if (i-anlikSatirPikseli)>9:
                            gidilecekPiksel=i-anlikSatirPikseli+9
                            kontrol1=1
                        else:
                            break
                break
    hedefSatirPikseli=anlikSatirPikseli
    hedefSutunPikseli=hatirlanacakSutunPikseli
    HedefKoordinatHesaplama(hedefSatirPikseli, hedefSutunPikseli, 0.0)

hiz=Twist()
def Dur(): #Hedefe belirtilen surede varamazsa durur.
    global hiz
    hiz.linear.x=0.0
    hiz.angular.z=0.0
    rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(hiz) #Hiz Yayinlama

def KendiEtrafindaSagaDonus(): #1 sn durduktan sonra kendi etrafinda hedef yone dogru saga/sola doner.
    global hiz
    hiz.linear.x=0.0
    hiz.angular.z=-0.5
    rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(hiz)

def KendiEtrafindaSolaDonus():
    global hiz
    hiz.linear.x=0.0
    hiz.angular.z=0.5
    rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(hiz)

istemci=actionlib.SimpleActionClient("/move_base", MoveBaseAction)
def HedefNoktayaGitme(hedefX, hedefY, hedefYaw, sure): #Parametre olarak; hedef koordinatlari, hedef yonu ve oraya gitmesi gereken sureyi aliyor.
    global anlikYaw, kontrol, istemci
    istemci.wait_for_server()
    hedef=MoveBaseGoal()
    hedef.target_pose.header.frame_id="map"
    hedef.target_pose.pose.position.x=hedefX #Hedef Koordinatlar
    hedef.target_pose.pose.position.y=hedefY
    hedef.target_pose.pose.orientation=Quaternion(*quaternion_from_euler(0.0, 0.0, hedefYaw)) #Hedef korrdinata gelince istenilen roll, pitch ve anlikYaw degerleri.
    istemci.send_goal(hedef)
    sonuc=istemci.wait_for_result(timeout=rospy.Duration(sure)) 
    if sonuc==True: #10 saniyede hedefe varirsa...
        print("Hedefe Varildi!") 
    else: #10 saniyede hedefe varamazsa hedefi iptal ediyor.
        print("Hedefe Zamaninda Varilamadi")
        istemci.cancel_goal()
        Dur()
        while( abs(hedefYaw-anlikYaw) > 0.05 ): #Hedef yone dogru manuel sekilde donus...
            if ( hedefYaw >= ( ( -1 ) * pi ) and hedefYaw <= ( ( -1 ) * ( pi / 2 ) ) and anlikYaw > ( pi / 2 ) and anlikYaw < ( pi ) ) or ( hedefYaw >= ( pi / 2 ) and hedefYaw <= ( pi ) and anlikYaw < ( ( -1 ) * ( pi / 2 ) ) and anlikYaw > ( ( -1 ) * pi ) ):
                if hedefYaw-anlikYaw > 0.05:
                    hiz.angular.z = -0.5
                    HizYayinlama()
                elif hedefYaw-anlikYaw < -0.05:
                    hiz.angular.z = 0.5
                    HizYayinlama()
                else:
                    hiz.angular.z = 0.0
                    HizYayinlama()
            else:
                if hedefYaw-anlikYaw > 0.05:
                    hiz.angular.z = 0.5
                    HizYayinlama()
                elif hedefYaw-anlikYaw < -0.05:
                    hiz.angular.z = -0.5
                    HizYayinlama()
                else:
                    hiz.angular.z = 0.0
                    HizYayinlama()

def kapatma():
    global pids
    rospy.loginfo("Durduruluyor!")
    Dur()
    os.kill(int(pids[-1]), signal.SIGKILL)

roslaunch.pmon._init_signal_handlers()
rospy.init_node('BitirmeProjem', anonymous=True)   
rospy.Subscriber("/map", OccupancyGrid, HaritaVerisi) 
rospy.Subscriber('/cmd_vel1', Twist, HizAlma) 
rospy.Subscriber("/joy", Joy, JoystickIslemleri) 
rospy.Subscriber("/modKontrolu", String, KontrolIslemleri) 
rospy.Subscriber("/clicked_point", Point, HaritaIslemleri)
rospy.Subscriber("/odom", Odometry, OdomIslemleri)
rospy.on_shutdown(kapatma)
rospy.spin()