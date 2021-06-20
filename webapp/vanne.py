import RPi.GPIO as GPIO
import time
import threading

POS1_PIN = 31
POS2_PIN = 29
MOVE_SWITCH_PIN = 16
POS1_SWITCH_PIN = 22
POS2_SWITCH_PIN = 18





class Vanne(object):

	def __init__(self):
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BOARD)
		#Setup sorties :
		GPIO.setup(POS1_PIN, GPIO.OUT)
		GPIO.setup(POS2_PIN,GPIO.OUT)
		#Setup entrées :
		GPIO.setup(MOVE_SWITCH_PIN,GPIO.IN)
		GPIO.setup(POS1_SWITCH_PIN,GPIO.IN)
		GPIO.setup(POS2_SWITCH_PIN,GPIO.IN)


		#mise à zéro par défaut :
		GPIO.output(POS1_PIN,GPIO.LOW)
		GPIO.output(POS2_PIN,GPIO.LOW)


		


		self._lock = threading.Lock()
		
		self._position_callback = None

		self.position = None

		

		#Creation du thread de lecture "pos vanne" continue
		self._get_position_thread=threading.Thread(target=self._get_position)
		self._get_position_thread.daemon = True
		self._get_position_thread.start()


		try:
			print("CHARGE LAST POSITION")
			last_position = self.charge_last_position()

			if last_position is not None:
				print("GO TO LAST POSITION : ", last_position)
				self.go_to_position(last_position)
				self.position=last_position

		except:
			self.position = None


	def on_position_change(self,callback):
		print("vanne.py callback position")
		self._position_callback = callback
		


	def read_position(self):
		# print("read_position")
		# print(GPIO.input(POS1_SWITCH_PIN),GPIO.input(POS2_SWITCH_PIN),GPIO.input(MOVE_SWITCH_PIN))
		with self._lock:
			if GPIO.input(MOVE_SWITCH_PIN):
				return "moving"
			elif GPIO.input(POS1_SWITCH_PIN):
				return "pos1"
			elif GPIO.input(POS2_SWITCH_PIN):
				return "pos2"
			else :
				return "None"


	def _get_position(self):
		""" Thread lisant la position des switchs en permanence"""
		while True :
			
			position = self.read_position()
			if position != self.position :
				#on update la variable si la position a change
				self.position = position
				
				
				if self._position_callback is not None :
					self._position_callback(self.position)
					print("position callback : ", self.position )

			time.sleep(0.5)


	def go_to_position(self,position):
		if position !=self.position :
			#mise à zéro par défaut :
			GPIO.output(POS1_PIN,GPIO.LOW)
			GPIO.output(POS2_PIN,GPIO.LOW)
			time.sleep(0.1)
			if position == "pos1":
				print("********* GOTO POS1 *********")
				GPIO.output(POS1_PIN,GPIO.HIGH)
			elif position == "pos2":
				print("***** GOTO POS2 *************")
				GPIO.output(POS2_PIN,GPIO.HIGH)
			self.sauvegarde_position(str(position))

	def sauvegarde_position(self,position):
		#sauvegarde la dernière position dans un fichier, pour la connaitre au prochain demarrage
		print("VANNE : sauvegarde position : ", position)
		with open('pos_vanne.txt', 'w') as fichier:
			fichier.write(position)

	def charge_last_position(self):
		#charge la dernière position connue de la vanne
		with open('pos_vanne.txt', 'r') as fichier:
			last_position = fichier.read()
			print("VANNE : last_position : ", last_position)
			return last_position
			

	def stop(self):
		GPIO.output(POS1_PIN,GPIO.LOW)
		GPIO.output(POS2_PIN,GPIO.LOW)