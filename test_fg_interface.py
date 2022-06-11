from  flightgear_interface import FG_com
import time
fgcom=FG_com(True)
fgcom.start()
fgcom.connect_and_wait_until_ready()
time.sleep(30)
while(1):
    print(fgcom.get_param("roll"))
fgcom.quit()

