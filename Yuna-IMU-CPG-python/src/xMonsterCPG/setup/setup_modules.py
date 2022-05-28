## Establishes connection with hebi modules and creates hexapod and imu groups


import numpy as np
import hebi
from time import sleep
import rospkg


def setup_modules():

    #Get Names of All the Modules
    rospack = rospkg.RosPack()
    pkgPath = rospack.get_path('xMonsterCPG')
    names = np.load(pkgPath+'/src/xMonsterCPG/setup/setupFiles/names.npy')
    bases = names[::3]

    HebiLookup = hebi.Lookup()
    imu = HebiLookup.get_group_from_names('*',bases)
    hexapod = HebiLookup.get_group_from_names('*',names)

    while imu == None or hexapod == None or imu.size != 6 or hexapod.size != 18:
        print('Waiting for modules')
        imu = HebiLookup.get_group_from_names('*',bases)
        hexapod = HebiLookup.get_group_from_names('*',names)

    print('Found {} modules in shoulder group, {} in robot.'.format(imu.size, hexapod.size))
    

    #Set the Gains (Multiple Times)
    
    gains_command = hebi.GroupCommand(hexapod.size)
    gains_command.read_gains(pkgPath+'/src/xMonsterCPG/setup/setupFiles/gains18.xml')
    for i in range(3):
        hexapod.send_command(gains_command)
        sleep(0.1)
    hexapod.command_lifetime = 5;
    hexapod.feedback_frequency = 100;
    
    return imu , hexapod


if __name__ == "__main__":
    imu , hexapod = setup_modules()