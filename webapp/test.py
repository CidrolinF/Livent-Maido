from regul import Regul
import time

regul = Regul()


# while True:
# 	print(regul.read_temp())
# 	time.sleep(0.2)

# regul.write_PID()
# regul.read_PID()
print(regul.pid)
print(regul.pid["p"])
print(regul.pid["i"])
print(int(regul.pid["d"])+1)
print(regul.get_temp())

# regul.write_Tconsigne('28')
print(regul.read_Tconsigne())
regul.start_regul()
regul.update_regul()