import socket, cv2, numpy, time, py_qmc5883l, datetime
import RPi.GPIO as zan

kompas = int(datetime.datetime.now().strftime("%H%M%S"))

def nothing(x):
    pass

zan.setmode(zan.BCM)
zan.setwarnings(False)                       #do not show any warnings
zan.setmode (zan.BCM)

#motor penggerak
zan.setup(18, zan.OUT)        #pin 7
zan.setup(17, zan.OUT)        #pin 11
zan.setup(27, zan.OUT)        #pin 13
zan.setup(22, zan.OUT)        #pin 15
zan.setup(6 , zan.OUT)        #pin 31
zan.setup(13, zan.OUT)        #pin 33
zan.setup(19, zan.OUT)        #pin 35
zan.setup(26, zan.OUT)        #pin 37
#motor handling
zan.setup(25, zan.OUT)        #pin 22
zan.setup(8 , zan.OUT)        #pin 24
zan.setup(7 , zan.OUT)        #pin 26
zan.setup(12, zan.OUT)        #pin 32

hda = zan.PWM(25, 40000)    #HANDLING          #GPIO as PWM output, with 100Hz frequency
hdb = zan.PWM(8 , 40000)    #HANDLING
hdc = zan.PWM(7 , 40000)    #HANDLING
hdd = zan.PWM(12, 40000)    #HANDLING
mpy = zan.PWM(18, 1000)   #PENENDANGy
mpx = zan.PWM(17, 1000)   #PENENDANGx
m3x = zan.PWM(27, 1000)   #MOTOR3x
m3y = zan.PWM(22, 1000)   #MOTOR3y
m2x = zan.PWM(6 , 1000)   #MOTOR2x
m2y = zan.PWM(13, 1000)   #MOTOR2y
m1x = zan.PWM(19, 1000)   #MOTOR1x
m1y = zan.PWM(26, 1000)   #MOTOR1y

hda.start(0)                               #generate PWM signal with 0% duty cycle
hdb.start(0)
hdc.start(0)
hdd.start(0)
mpy.start(0)
mpx.start(0)
m3x.start(0)
m3y.start(0)
m2x.start(0)
m2y.start(0)
m1x.start(0)
m1y.start(0)

#trackbar gawang
cv2.namedWindow("Kalibrasi Bar", flags=cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar("ghl", "Kalibrasi Bar", 22, 179, nothing)
cv2.createTrackbar("ghu", "Kalibrasi Bar", 36, 179, nothing)
cv2.createTrackbar("gsl", "Kalibrasi Bar", 35, 255, nothing)
cv2.createTrackbar("gsu", "Kalibrasi Bar", 80, 255, nothing)
cv2.createTrackbar("gvl", "Kalibrasi Bar", 165, 255, nothing)
cv2.createTrackbar("gvu", "Kalibrasi Bar", 225, 255, nothing)

#trackbar bola
cv2.namedWindow("Kalibrasi Bar", flags=cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar("bhl", "Kalibrasi Bar", 0, 179, nothing)
cv2.createTrackbar("bhu", "Kalibrasi Bar", 40, 179, nothing)
cv2.createTrackbar("bsl", "Kalibrasi Bar", 160, 255, nothing)
cv2.createTrackbar("bsu", "Kalibrasi Bar", 225, 255, nothing)
cv2.createTrackbar("bvl", "Kalibrasi Bar", 177, 255, nothing)
cv2.createTrackbar("bvu", "Kalibrasi Bar", 255, 255, nothing)

#trackbar kawan
cv2.namedWindow("Kalibrasi Kawan", flags=cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar("khl", "Kalibrasi Kawan", 0, 179, nothing)
cv2.createTrackbar("khu", "Kalibrasi Kawan", 40, 179, nothing)
cv2.createTrackbar("ksl", "Kalibrasi Kawan", 160, 255, nothing)
cv2.createTrackbar("ksu", "Kalibrasi Kawan", 225, 255, nothing)
cv2.createTrackbar("kvl", "Kalibrasi Kawan", 177, 255, nothing)
cv2.createTrackbar("kvu", "Kalibrasi Kawan", 255, 255, nothing)

#Starting PID
#kp=0.15 kd=0.03 speed=70 frekuensi=1000
error = 0
errors= 0
speed = 70
Kp = 0.102
Kd = 0.01367
Ki = 0.019025
PID= 0
PIDkn = 0
PIDkr = 0
m1 = 0
m2 = 0
m3 = 0
m4 = 0
hand = 0
z = 0
f = 0
nah = ''
siapa = ''
ket = ''

#set sudut gawang musuh
delu = 295
dell = 235

while True:
    try:
        cam = cv2.VideoCapture(0)
        cam.set(3,320)
        cam.set(4,240)
        img=cam.read()[1]
        cv2.imshow("testing",img)
        cv2.destroyWindow("testing")
        break
    except:
        print ("\nError Camera!!! (＞﹏＜)\n")
        cam.release()

font = cv2.FONT_HERSHEY_SIMPLEX

serper = socket.socket()
ipBaseStation = "10.10.12.29" #wifi URO
portBaseStation = 7645
serper.settimeout(0.1)
while(1):
    try:
        serper.connect((ipBaseStation,portBaseStation))
        break
    except:
        continue

def caribola():
    global m1,m2,m3,m4,error,PIDkn,PIDkr,hand,z
    konturBola, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    bola = []
    for c in konturBola :
        bola.append(cv2.contourArea(c))
        max_index = numpy.argmax(bola)
        cnt=konturBola[max_index]        

    #kotak objek
    if(bola !=[]):  
        x,y,w,h = cv2.boundingRect(cnt)
        xp = int((x+x+w)/2)
        yp = int((y+y+h)/2)
        vx = xp*0.3125
        vy = yp*0.24
        img = cv2.rectangle(frame, (x,y),(x+w,y+h),(0,255,255),1)
        img = cv2.rectangle(frame, (xp,yp),(xp,yp),(0,0,255),3)

        #deteksi lokasi
        error=160-xp
        PID=Kp*error+Kd*(error-errors)+Ki*(error+errors)
        PID=abs(PID)
        errors=error
        if(yp>205 and xp>150 and xp<190):
            hand = 100
            if siapa == 'bot1':
                if ket == 'bot1ketemu':
                    z = 44 #cari gawang
                else:
                    z = 33 #oper
                
            elif siapa == "bot2":
                z = 22 #cari kawan
            else:
                pass

        elif(error>0):
            hand = 0
            PIDkr=0
            PIDkn=PID
            if (PIDkn > 100):
                PIDkn = 100
            valkn=PIDkn
            m4=speed-valkn
            m3=0
            if(m4<0):
                m3=m4*(-1)
                m4=0
            m2=0
            m1=speed
        elif(error<0):
            hand = 0
            PIDkr=PID
            PIDkn=0
            if (PIDkr > 100):
                PIDkr = 100
            valkr=PIDkr
            m1=speed-valkr
            m2=0
            if(m1<0):
                m2=m1*(-1)
                m1=0
            m3=0
            m4=speed

        else:
            hand = 0
            PIDkn=0
            PIDkr=0

    else:
        z = 0

def carikawan():
    global m1,m2,m3,m4,error,PIDkn,PIDkr,hand,nah,z,ket
    konturKawan, _ = cv2.findContours(kawan,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    areaKawan = []
    for c in konturKawan:
        areaKawan.append(cv2.contourArea(c))
        max_index = numpy.argmax(areaKawan)
        cnt=konturKawan[max_index]
    if areaKawan != []:
        xx,yy,ww,hh = cv2.boundingRect(cnt)
        xk = int((xx+xx+ww)/2)
        yk = int((yy+yy+hh)/2)
        img = cv2.rectangle(frame, (xx,yy),(xx+ww,yy+hh),(0,255,255),1)
        img = cv2.rectangle(frame, (xk,yk),(xk,yk),(0,0,255),3)
        error=160-xk
        PID=Kp*error+Kd*(error-errors)+Ki*(error+errors)
        PID=abs(PID)
        errors=error
        if(xk>150 and xk<190):
            if siapa == 'bot1':
                nah = 'bot1ketemu'
                serper.sendall(str.encode(nah))
                ket = nah
                z = 0
            if siapa == 'bot2':
                pass
        elif(error>0):
            PIDkr=0
            PIDkn=PID/2
            if (PIDkn > 100/2):
                PIDkn = 100/2
            valkn=PIDkn
            m4=0
            m3=valkn
            if(m3<0):
                m4=m3*(-1)
                m3=0
            m2=0
            m1=valkn
        elif(error<0):
            PIDkr=PID/2
            PIDkn=0
            if (PIDkr > 100/2):
                PIDkr = 100/2
            valkr=PIDkr
            m1=0
            m2=valkr
            if(m1<0):
                m2=m1*(-1)
                m1=0
            m3=0
            m4=valkr
        else:
            PIDkn=0
            PIDkr=0

    else:
        z = 2

def oper():
    global m1,m2,m3,m4,error,PIDkn,PIDkr,hand,nah,z
    konturKawan, _ = cv2.findContours(kawan,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    areaKawan = []
    for c in konturKawan:
        areaKawan.append(cv2.contourArea(c))
        max_index = numpy.argmax(areaKawan)
        cnt=konturKawan[max_index]
    if areaKawan != []:
        xx,yy,ww,hh = cv2.boundingRect(cnt)
        xk = int((xx+xx+ww)/2)
        yk = int((yy+yy+hh)/2)
        img = cv2.rectangle(frame, (xx,yy),(xx+ww,yy+hh),(0,255,255),1)
        img = cv2.rectangle(frame, (xk,yk),(xk,yk),(0,0,255),3)
        error=160-xk
        PID=Kp*error+Kd*(error-errors)+Ki*(error+errors)
        PID=abs(PID)
        errors=error
        if(xk>150 and xk<190):
            m3x.ChangeDutyCycle(0)
            m3y.ChangeDutyCycle(0)
            m2x.ChangeDutyCycle(0)
            m2y.ChangeDutyCycle(0)
            m1x.ChangeDutyCycle(0)
            m1y.ChangeDutyCycle(0)
            PIDkn=0
            PIDkr=0
            mpy.ChangeDutyCycle(0)
            mpx.ChangeDutyCycle(20)
            time.sleep(0.39)
            mpy.ChangeDutyCycle(0)
            mpx.ChangeDutyCycle(0)
            time.sleep(0.4)
            mpy.ChangeDutyCycle(30)
            mpx.ChangeDutyCycle(0)
            time.sleep(0.36)
            if siapa == 'bot1':
                z = 12 #ke gawang
                nah = 'lepas1'
                serper.sendall(str.encode(nah))
                hand = 0
            elif siapa == 'bot2':
                z = 00 #diem
            else:
                pass

        elif(error>0):
            PIDkr=0
            PIDkn=PID/2
            if (PIDkn > 100/2):
                PIDkn = 100/2
            valkn=PIDkn
            m4=0
            m3=valkn
            if(m3<0):
                m4=m3*(-1)
                m3=0
            m2=0
            m1=valkn

        elif(error<0):
            PIDkr=PID/2
            PIDkn=0
            if (PIDkr > 100/2):
                PIDkr = 100/2
            valkr=PIDkr
            m1=0
            m2=valkr
            if(m1<0):
                m2=m1*(-1)
                m1=0
            m3=0
            m4=valkr

        else:
            PIDkn=0
            PIDkr=0
    else:
        pass

def kegawang():
    global m1,m2,m3,m4,error,PIDkn,PIDkr,hand,z
    luarG=cv2.morphologyEx(gawang, cv2.MORPH_OPEN, numpy.ones((5,5),"uint8"))
    dalamG = cv2.morphologyEx(luarG, cv2.MORPH_CLOSE, numpy.ones((5,5),"uint8"))
    ret,threshG = cv2.threshold(dalamG,127,255,0)
    contoursGawang, _ = cv2.findContours(threshG,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    areaGaw = []
    for c in contoursGawang :
        areaGaw.append(cv2.contourArea(c))
        max_index = numpy.argmax(areaGaw)
        cnt=contoursGawang[max_index]
    if(areaGaw !=[]):
        hand = 100
        xg,yg,wg,hg = cv2.boundingRect(cnt)
        xpg = int((xg+xg+wg)/2)
        ypg = int((yg+yg+hg)/2)
        img = cv2.rectangle(frame, (xg,yg),(xg+wg,yg+hg),(0,255,255),1)
        img = cv2.rectangle(frame, (xpg,ypg),(xpg,ypg),(0,0,255),3)
        img = cv2.putText(frame, "ypg:"+str(ypg)+"  xpg:"+str(xpg), (10,20),1,1.5,(0,255,255),2)

        #deteksi lokasi
        error=160-xpg
        PID=Kp*error+Kd*(error-errors)
        PID=abs(PID)
        errors=error
        while True:
                try:
                    sensor = py_qmc5883l.QMC5883L()
                    m = sensor.mode_continuous()
                    m = sensor.get_bearing()
                    break
                except:
                    print("Compass Error (ᗒᗣᗕ)՞")
                    continue

        if(m>dell and m<delu):    #tidak jalan
            pass
        elif(m>delu):    #tidak jalan
            m1 = 10
            m2 = 0
            m3 = 10
            m4 = 0
            PIDkn=10
            PIDkr=0
        elif(m<dell):    #
            m1 = 0
            m2 = 10
            m3 = 0
            m4 = 10
            PIDkn=0
            PIDkr=10
        else:      #kearah musuh
            m1 = 0
            m2 = 0
            m3 = 0
            m4 = 0
            PIDkn=0
            PIDkr=0

        if(ypg>85 and xpg>130 and xpg<210):
            z = 22 #cari kawan

        elif(error>0):
            PIDkr=0
            PIDkn=PID
            if (PIDkn > 100):
                PIDkn = 100
            valkn=PIDkn
            m4=speed-valkn
            m3=0
            if(m4<0):
                m3=m4*(-1)
                m4=0
            m2=0
            m1=speed

        elif(error<0):
            PIDkr=PID
            PIDkn=0

            if (PIDkr > 100):
                PIDkr = 100
            valkr=PIDkr
            m1=speed-valkr
            m2=0
            if(m1<0):
                m2=m1*(-1)
                m1=0
            m3=0
            m4=speed

        else:
            PIDkn=0
            PIDkr=0

    else:
        z = 12

def carigawang():
    global m1,m2,m3,m4,error,PIDkn,PIDkr,hand,nah,z
    luarG=cv2.morphologyEx(gawang, cv2.MORPH_OPEN, numpy.ones((5,5),"uint8"))
    dalamG = cv2.morphologyEx(luarG, cv2.MORPH_CLOSE, numpy.ones((5,5),"uint8"))
    ret,threshG = cv2.threshold(dalamG,127,255,0)
    contoursGawang, _ = cv2.findContours(threshG,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    areaGaw = []
    for c in contoursGawang :
        areaGaw.append(cv2.contourArea(c))
        max_index = numpy.argmax(areaGaw)
        cnt=contoursGawang[max_index]
    if(areaGaw !=[]):
        hand = 100
        xg,yg,wg,hg = cv2.boundingRect(cnt)
        xpg = int((xg+xg+wg)/2)
        ypg = int((yg+yg+hg)/2)
        img = cv2.rectangle(frame, (xg,yg),(xg+wg,yg+hg),(0,255,255),1)
        img = cv2.rectangle(frame, (xpg,ypg),(xpg,ypg),(0,0,255),3)
        img = cv2.putText(frame, "ypg:"+str(ypg)+"  xpg:"+str(xpg), (10,20),1,1.5,(0,255,255),2)

        #deteksi lokasi
        error=160-xpg
        PID=Kp*error+Kd*(error-errors)
        PID=abs(PID)
        errors=error
        while True:
                try:
                    sensor = py_qmc5883l.QMC5883L()
                    m = sensor.mode_continuous()
                    m = sensor.get_bearing()
                    break
                except:
                    print("Compass Error (ᗒᗣᗕ)՞")
                    continue

        if(m>dell and m<delu):    #tidak jalan
            pass
        elif(m>delu):    #tidak jalan
            m1 = 10
            m2 = 0
            m3 = 10
            m4 = 0
            PIDkn=10
            PIDkr=0
        elif(m<dell):    #
            m1 = 0
            m2 = 10
            m3 = 0
            m4 = 10
            PIDkn=0
            PIDkr=10
        else:      #kearah musuh
            m1 = 0
            m2 = 0
            m3 = 0
            m4 = 0
            PIDkn=0
            PIDkr=0

        if(ypg>85 and xpg>130 and xpg<210):
            if (int(datetime.datetime.now().strftime("%H%M%S")) > (kompas + 5)):
                m3x.ChangeDutyCycle(0)
                m3y.ChangeDutyCycle(0)
                m2x.ChangeDutyCycle(0)
                m2y.ChangeDutyCycle(0)
                m1x.ChangeDutyCycle(0)
                m1y.ChangeDutyCycle(0)
                PIDkn=0
                PIDkr=0
                mpy.ChangeDutyCycle(0)
                mpx.ChangeDutyCycle(20)
                time.sleep(0.39)
                mpy.ChangeDutyCycle(0)
                mpx.ChangeDutyCycle(0)
                time.sleep(0.4)
                mpy.ChangeDutyCycle(100)
                mpx.ChangeDutyCycle(0)
                time.sleep(0.36)
                kompas = int(datetime.datetime.now().strftime("%H%M%S"))
            z = 00

        elif(error>0):
            PIDkr=0
            PIDkn=PID
            if (PIDkn > 100):
                PIDkn = 100
            valkn=PIDkn
            m4=speed-valkn
            m3=0
            if(m4<0):
                m3=m4*(-1)
                m4=0
            m2=0
            m1=speed

        elif(error<0):
            PIDkr=PID
            PIDkn=0

            if (PIDkr > 100):
                PIDkr = 100
            valkr=PIDkr
            m1=speed-valkr
            m2=0
            if(m1<0):
                m2=m1*(-1)
                m1=0
            m3=0
            m4=speed

        else:
            PIDkn=0
            PIDkr=0
            z=0

    else:
        z = 44

def diem():
    global m1,m2,m3,m4,PIDkn,PIDkr,hand,z
    hand = 0
    PIDkr = 0
    PIDkn = 0
    m1 = 0
    m2 = 0
    m3 = 0
    m4 = 0
    z = 00

while (1):
	frame = cam.read()[1]
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    bhl = cv2.getTrackbarPos("bhl", "Kalibrasi Bar")
    bsl = cv2.getTrackbarPos("bsl", "Kalibrasi Bar")
    bvl = cv2.getTrackbarPos("bvl", "Kalibrasi Bar")
    bhu = cv2.getTrackbarPos("bhu", "Kalibrasi Bar")
    bsu = cv2.getTrackbarPos("bsu", "Kalibrasi Bar")
    bvu = cv2.getTrackbarPos("bvu", "Kalibrasi Bar")
    ghl = cv2.getTrackbarPos("ghl", "Kalibrasi Bar")
    gsl = cv2.getTrackbarPos("gsl", "Kalibrasi Bar")
    gvl = cv2.getTrackbarPos("gvl", "Kalibrasi Bar")
    ghu = cv2.getTrackbarPos("ghu", "Kalibrasi Bar")
    gsu = cv2.getTrackbarPos("gsu", "Kalibrasi Bar")
    gvu = cv2.getTrackbarPos("gvu", "Kalibrasi Bar")
    khl = cv2.getTrackbarPos("khl", "Kalibrasi Kawan")
    ksl = cv2.getTrackbarPos("ksl", "Kalibrasi Kawan")
    kvl = cv2.getTrackbarPos("kvl", "Kalibrasi Kawan")
    khu = cv2.getTrackbarPos("khu", "Kalibrasi Kawan")
    ksu = cv2.getTrackbarPos("ksu", "Kalibrasi Kawan")
    kvu = cv2.getTrackbarPos("kvu", "Kalibrasi Kawan")
    batasbawahG= numpy.array([ghl,gsl,gvl])
    batasatasG= numpy.array([ghu,gsu,gvu])
    gawang = cv2.inRange(hsv1,batasbawahG,batasatasG)
    batasbawah= numpy.array([bhl,bsl,bvl])
    batasatas= numpy.array([bhu,bsu,bvu])
    mask= cv2.inRange(hsv, batasbawah, batasatas)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    batasbawahK = numpy.array([khl,ksl,kvl])
    batasatasK = numpy.array([khu,ksu,kvu])
    kawan = cv2.inRange(hsv1,batasbawahK,batasatasK)
    try:
        while(1):
            nah = (serper.recv(32)).decode('utf-8')
            if not nah:
                break
            elif nah:
                nah = nah
	except Exception as e:
		pass

    try:
    	if nah == '`' :
    		serper.close()
    		break
        elif (nah == 'bot1start') or (nah== 'lepas2'):
            if nah == 'bot1start':
                siapa = 'bot1'
                z = int(nah.replace('bot1start', '0'))
            elif nah == 'lepas2':
                z = int(nah.replace('lepas2', '0'))
            else:
                pass
        elif (nah == 'bot2ketemu'):
            z = int(nah.replace('bot2ketemu', '33'))
        elif (nah == 'bot2start'):
            siapa = 'bot2'
            z = int(nah.replace('bot2start', '22'))
        elif nah == 'nembak':
            z = int(nah.replace('nembak', '00'))
        else:
            pass
    except:
        pass

    try:
        if z == 0:
            caribola()
        elif z == 22:
            carikawan()
        elif z == 44:
            carigawang()
        elif z == 33:
            oper()
        elif z == 12:
            kegawang()
        elif z == 00:
            diem()
        else:
            pass
    except:
        pass

    hda.ChangeDutyCycle(0)
    hdb.ChangeDutyCycle(hand)
    hdc.ChangeDutyCycle(0)
    hdd.ChangeDutyCycle(hand)
    mpy.ChangeDutyCycle(0)
    mpx.ChangeDutyCycle(0)
    m3x.ChangeDutyCycle(PIDkn)
    m3y.ChangeDutyCycle(PIDkr)
    m2x.ChangeDutyCycle(m1)
    m2y.ChangeDutyCycle(m2)
    m1x.ChangeDutyCycle(m3)
    m1y.ChangeDutyCycle(m4)
    cv2.imshow('frame',frame)
    cv2.imshow('mask bola',mask)
    cv2.imshow('mask gawang', gawang)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
cam.release()
cv2.destroyAllWindows()
hda.ChangeDutyCycle(0)
hdb.ChangeDutyCycle(0)
hdc.ChangeDutyCycle(0)
hdd.ChangeDutyCycle(0)
mpy.ChangeDutyCycle(0)
mpx.ChangeDutyCycle(0)
m3x.ChangeDutyCycle(0)
m3y.ChangeDutyCycle(0)
m2x.ChangeDutyCycle(0)
m2y.ChangeDutyCycle(0)
m1x.ChangeDutyCycle(0)
m1y.ChangeDutyCycle(0)
hda.stop()
hdb.stop()
hdc.stop()
hdd.stop()
mpy.stop()
mpx.stop()
m3x.stop()
m3y.stop()
m2x.stop()
m2y.stop()
m1x.stop()
m1y.stop()
zan.cleanup()