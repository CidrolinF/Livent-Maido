import RPi.GPIO as GPIO
import glob
import time
import threading
import pickle
import os



PWM_PIN      = 12
ONEWIRE_PIN  = 7
VENTIL_PIN   = 13
LED_PIN      = 11
base_dir = '/sys/bus/w1/devices/'
try: 
  device_folder = glob.glob(base_dir + '28*')[0]
  device_file = device_folder + '/w1_slave'
except:
  device_file = 'pas de sonde'
  
  
class Regul(object):
  """Raspberry Pi regulation via page web"""
  
  def __init__(self):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PWM_PIN, GPIO.OUT)
    self.pinPWM = GPIO.PWM(PWM_PIN, 40)
    self.pinPWM.stop()

    # GPIO.setup(ONEWIRE_PIN, GPIO.IN)
    GPIO.setup(VENTIL_PIN,GPIO.OUT)
    GPIO.setup(LED_PIN,GPIO.OUT)
    self._lock = threading.Lock()
    
    #Regulation sur off par defaut
    self.etat_regul = 'off'
    # self._temperature = None
    
    #Creation du thread de lecture temp continue
    self._temp_thread=threading.Thread(target=self._temp_update)
    self._temp_thread.daemon = True
    self._temp_thread.start()

    self._regul_thread=threading.Thread(target=self.update_regul)
    self._regul_thread.daemon = True
    self._regul_thread.start()

    self._temp_callback = None
    self._pid_callback = None
    self._Tconsigne_callback = None
    self._regul_callback = None


    self._temperature = None
    self.pid=self.read_PID()
    self.Tconsigne=self.read_Tconsigne()



#-------------------------
# Gestion etat regul :
  def read_etat_regul(self):
    with self._lock:
      return self.etat_regul
  
  def start_regul(self):
    self.pinPWM.start(0)
    self.pinPWM.ChangeFrequency(40)
    self.etat_regul = 1
    GPIO.output(LED_PIN,GPIO.HIGH) #LED allumee
    #ventilateur ON :
    GPIO.output(VENTIL_PIN,GPIO.HIGH)
    


    
  def stop_regul(self):
    self.etat_regul = 0
    self.clear()
    self.pinPWM.stop()
    GPIO.output(LED_PIN,GPIO.LOW) #LED eteinte
    #ventilateur OFF :
    GPIO.output(VENTIL_PIN,GPIO.LOW)
  

  def update_regul(self):

    #Init
    print("Init regul")
    
    self.sample_time = 1
    self.current_time = time.time()
    self.last_time = self.current_time
    boucle = 0
    self.clear()
    time.sleep(2)

    while True :
        if self.etat_regul == 1:
            with self._lock:
                error = float(self.Tconsigne) - float(self._temperature)
                print(self._temperature)

                self.current_time = time.time()
                delta_time = self.current_time - self.last_time
                delta_error = error - self.last_error

                if (delta_time >= self.sample_time):
                    self.PTerm = float(self.pid['p']) * error
                    self.ITerm += error * delta_time

                    if (self.ITerm < -self.windup_guard):
                        self.ITerm = -self.windup_guard
                    elif (self.ITerm > self.windup_guard):
                        self.ITerm = self.windup_guard

                    self.DTerm = 0.0
                    if delta_time > 0:
                        self.DTerm = delta_error / delta_time

                    # Remember last time and last error for next calculation
                    self.last_time = self.current_time
                    self.last_error = error
                    boucle += 1
                    self.output = self.PTerm + (float(self.pid['i']) * self.ITerm) + (float(self.pid['d']) * self.DTerm)
                    self.corr=self.corr_PWM(self.output)
                    #Changement 
                    print("duty cycle : " , self.corr)
                    self.pinPWM.ChangeDutyCycle(self.corr)
                    if self._regul_callback is not None:
                        self._regul_callback(self.output)
                    print(boucle, ' output :' , self.output , 'error: ', error)
        else:
            self.clear()
            boucle=0
        time.sleep(2.0)

  def on_regul_change(self, callback):
      """register a callback function when the temp change state"""
      print("regul.py : callback on_regul_change")
      self._regul_callback = callback


  def clear(self):
        with self._lock:
            """coefficients"""
            self.last_error = 0.0

            self.PTerm = 0.0
            self.ITerm = 0.0
            self.DTerm = 0.0
            # Windup Guard
            self.int_error = 0.0
            self.windup_guard = 60.0

            self.output = 0.0
            self.corr = 0

  def corr_PWM(self,output):
        if output < 0:
            corr=0
        else :
            corr=output
            #saturation :
            if corr > 100:
                corr =100

        return corr


#--------------------------------
  def read_temp_raw(self):
    with self._lock:
      f = open(device_file, 'r')
      lines = f.readlines()
      f.close()
      return lines
   
  def get_temp(self):
      with self._lock:
        return self._temperature

  def _temp_update(self):
      """ Thread lisant la temperature en permanence"""
      while True :
        lines = self.read_temp_raw()
        while lines[0].strip()[-3:] != 'YES':
            # time.sleep(0.2)
            lines = self.read_temp_raw()
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            temp_c = float(temp_string) / 1000.0
            self._temperature=temp_c
            # self._temp_callback(temp_c)
        # if self._temp_callback is not None:
        #     self._temp_callback(temp_c)
        time.sleep(2.0)
        print("temp update : ", self._temperature)

  def on_temp_change(self, callback):
      """register a callback function when the temp change state"""
      #DESACTIVE CAR PROBLEME AVEC MONKEY PATCH DE GEVENT
      print("regul.py : callback on_temp_change")
      # self._temp_callback = callback

#-----------------------
#  gestion des PID :
#-----------------------
  def on_pid_change(self,callback):
        print("regul.py callback pid")
        self._pid_callback = callback
      
  def get_PID(self):
    with self._lock:
        return self.pid

  def write_PID(self,pid):  
    with open('pid.txt', 'wb') as fichier:
        pickler = pickle.Pickler(fichier)
        pickler.dump(pid)
        print("regul.py : PIDwrite : ", pid)
    self.pid=self.read_PID()
    if self._pid_callback is not None:
        self._pid_callback(self.pid)


  def read_PID(self):
      if not os.path.isfile('pid.txt') :
        print("Pas de PID enregistre")
        pid = {
        "p" : 100,
        "i" : 2,
        "d" : 1,
        }
        self.write_PID(pid)
      with open('pid.txt', 'rb') as fichier:
        depickler = pickle.Unpickler(fichier)
        pid = depickler.load()
        print('regul.py : PIDread : ', pid)
        return pid
        if self._pid_callback is not None:
              print("regul.py : appel callback read PID")
              self._pid_callback(pid)

#-------------------------------
#Gestion temperature de consigne :
#------------------------------
  def on_Tconsigne_change(self,callback):
        print("regul.py callback Tconsigne")
        self._Tconsigne_callback = callback
      
  def get_Tconsigne(self):
    with self._lock:
        return self.Tconsigne

  def write_Tconsigne(self,Tconsigne):  
    with open('Tconsigne.txt', 'w') as fichier:
        fichier.write(str(Tconsigne))
        print("regul.py : Tconsigne write : ", Tconsigne)
    self.Tconsigne=self.read_Tconsigne()
    if self._Tconsigne_callback is not None:
        self._Tconsigne_callback(self.Tconsigne)


  def read_Tconsigne(self):
      if not os.path.isfile('Tconsigne.txt') :
        print("Pas de Tconsigne enregistre")
        self.write_Tconsigne('28')
      with open('Tconsigne.txt', 'r') as fichier:
        Tconsigne=int(fichier.read())
        return Tconsigne
        if self._Tconsigne_callback is not None:
              self._Tconsigne_callback(Tconsigne)



class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time