from vanne import Vanne
import time


ma_vanne=Vanne()
print("pos1")
ma_vanne.go_to_position("pos1")
#print("read")
## ma_vanne.read_position()
#print("pos2")
#ma_vanne.go_to_position("pos2")
#time.sleep(12)
#print("pos1")
#
#ma_vanne.go_to_position("pos1")
time.sleep(10)
print("pos2")
ma_vanne.go_to_position("pos2")
time.sleep(10)

print("stop")

ma_vanne.stop()