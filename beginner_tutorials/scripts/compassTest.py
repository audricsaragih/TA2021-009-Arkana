import sys
sys.path.insert(1,'/home/ubuntu/mybot_ws/src/beginner_tutorials/scripts/i2clibraries/')
from i2c_hmc5883l import *

hmc5883l = i2c_hmc5883l(1)
i = 1

hmc5883l.setContinuousMode ()
hmc5883l.setDeclination (2,4)


print(hmc5883l) 
i = i+1
print(i)
sleep(1)
