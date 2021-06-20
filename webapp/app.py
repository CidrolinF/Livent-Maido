from flask import *
from flask_socketio import SocketIO
import RPi.GPIO as GPIO
from regul import Regul
from vanne import Vanne
from thorpy.comm.discovery import discover_stages
from thorpy.comm.port import Port
import random
import time
import json
import threading
# from multiprocessing import Process
from gevent import monkey
from eventlet import *



# https://www.youtube.com/watch?v=s1omSb9iwKE a 39:06 

# global drv_thorlabs
# stages = list(discover_stages())
# drv_thorlabs=stages[0]
# time.sleep(1)

# monkey.patch_all() #gevent
monkey_patch() #eventelet
#Creation de l'app et de l'objet regul :
app = Flask(__name__)
regul = Regul()
vanne = Vanne()
# socketio = SocketIO(app)
# socketio = SocketIO(app,async_mode='threading')
socketio = SocketIO(app,async_mode='eventlet')
# socketio = SocketIO(app,async_mode='gevent')




###########################
#Page principale
@app.route('/')
def index():
    return render_template('index.html')


@socketio.on('get_variables')
def get_variables(state):
    socketio.emit('regul_changed',{'regul_state' : regul.etat_regul})
    print("APP.py : get variables")
    socketio.emit('pid_changed',{'pid' : regul.pid})
    socketio.emit('Tconsigne_changed',{'Tconsigne' : regul.Tconsigne})
    print("socket emit vanne position",  vanne.position)
    socketio.emit('vanne_switch_changed',{'pos' : vanne.position})



@socketio.on('regul')
def change_regul(state):
    if state == 'off':
        regul.stop_regul()
        socketio.emit('regul_changed',{'regul_state' : regul.etat_regul})
    elif state == 'on':
        regul.start_regul()
        socketio.emit('regul_changed',{'regul_state' : regul.etat_regul})


# def temp_changed(temperature):
#         socketio.emit('temp_changed',{'temp' : temperature})
#         time.sleep(2)

def temp_changed():
    while True:
        temperature = regul._temperature
        # print("app : temp changed ", temperature)
        socketio.emit('temp_changed',{'temp' : temperature})
        time.sleep(2)


@socketio.on('pid_new')
def change_pid(pid_new):
    print('pid_new : ', pid_new)
    regul.write_PID(pid_new)
    regul.clear()
    

def pid_changed(pid):
    print("APP.py : pid_changed")
    socketio.emit('pid_changed',{'pid' : regul.pid})

 

@socketio.on('Tconsigne_new')
def change_Tconsigne(Tconsigne_new):
    print('Tconsigne_new : ', Tconsigne_new)
    regul.write_Tconsigne(Tconsigne_new)

def Tconsigne_changed(Tconsigne):
    print("APP.py : Tconsigne_changed")
    socketio.emit('Tconsigne_changed',{'Tconsigne' : regul.Tconsigne})
    regul.clear()


def regul_update(regulation):
    socketio.emit('regul_update',{
        'correction' : round(regul.corr,2),
        'last_error' : round(regul.last_error,2),
        'output' : round(regul.output,1),
        'PTerm' : round(regul.PTerm,1),
        'ITerm' : round(regul.ITerm,1),
        'DTerm' : round(regul.DTerm,1),
        })

###########################
#Page POMPE
@app.route('/pompe')
def page_pompe():
    return render_template('pompe.html')



@socketio.on('drv_connect')
def on_connect(state):
    print("DRV Connect")
    global drv_thorlabs
    stages = list(discover_stages())
    drv_thorlabs=stages[0]
    drv_thorlabs.on_position_change(position_changed)
    # drv_thorlabs.home()

def position_changed(position):
    # print('APP.py : position_hanged')
    socketio.emit('position_changed',{'position' : -1*drv_thorlabs._state_position / 409600})

@socketio.on('go_home')
def go_home(state):
    print('Action go_home :')
    drv_thorlabs.home()


@socketio.on('jog')
def move_jog(direction):
    print('jog',direction)
    drv_thorlabs.move_jog(direction)

@socketio.on('stop')
def move_stop(state):
    print('mov_stop')
    drv_thorlabs.move_stop()

@socketio.on('position_go')
def position_go(pos):
    print('position go', pos)
    drv_thorlabs.position = -1*float(pos)

# ------------- Vanne -------------------
def vanne_switch_changed(pos_switch):
    print("app : vanne_switch_changed ", pos_switch)
    socketio.emit('vanne_switch_changed',{'pos' : pos_switch})

@socketio.on('vanne_position_go')
def vanne_go_to_pos(pos):
    print('vanne_position_go ',pos)
    vanne.go_to_position(pos)

# @socketio.on('get_vanne_pos')
# def get_vanne_pos():
#     socketio.emit('vanne_switch_changed',{'pos' : vanne.position})



#*********************************************
if __name__ == '__main__':
    global drv_thorlabs

    #Page REGULATION
    # regul.on_temp_change(temp_changed) # MARCHE PAS AVEC MONKEY PATCH
    try:
	    regul.on_pid_change(pid_changed)
	    regul.on_Tconsigne_change(Tconsigne_changed)
	    regul.on_regul_change(regul_update)
	    
	    temp_Thread=threading.Thread(target=temp_changed)
	    temp_Thread.daemon=True
	    temp_Thread.start()   
	except :
		print("problem regul main")


    vanne.on_position_change(vanne_switch_changed)

    # vanne_Thread=threading.Thread(target=vanne_switch_changed)
    # vanne_Thread.daemon=True
    # vanne_Thread.start()

    #Page CONTROLE POMPE



    



    socketio.run(app, debug=False, host='0.0.0.0', port=5000)


    
