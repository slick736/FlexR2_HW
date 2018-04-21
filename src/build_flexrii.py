import os
import shutil

snfile = "serial_nums_flexrii.txt"
sourcefile = "app/main.h"
tmpfile = "main.h"
outputfile = "../tirtos/iar/app/FlashROM_StackLibrary/Exe/cc2640r2lp_app.bin"

sn = open(snfile, 'r')
i = 1

for serialNumber in sn:
    serialNumber = serialNumber.replace('\n', '')
    serialNumberHex = ''
    serialNumberHex += hex(ord('F')) + ', '
    serialNumberHex += hex(ord('R')) + ', '
    
    for c in serialNumber:
        serialNumberHex += hex(ord(c)) + ', '
    serialNumberHex = serialNumberHex[0:len(serialNumberHex) - 2]
    
    #for c in serialNumber:
        #serialNumberHex += c
    
    source = open(sourcefile, 'r')
    tmp = open(tmpfile, 'w')
    
    for line in source:
        if line.startswith('#define HEALEREMG_SERIAL_NUMBER'):
            line = '#define HEALEREMG_SERIAL_NUMBER    ' + serialNumberHex + '\n'
        tmp.write(line)
    
    source.close()
    tmp.close()
    
    renamefile = "../tirtos/iar/app/FlashROM_StackLibrary/Exe/FlexRII_" + serialNumber + ".bin"
    
    os.system("move /y " + tmpfile + " " + sourcefile)
    os.system("D:/IAR811/IAR811/common/bin/IarBuild.exe ../tirtos/iar/app/cc2640r2lp_app.ewp -build * -varfile ../tirtos/iar/simple_peripheral.custom_argvars")
    shutil.copy(outputfile, renamefile)
    i += 1
#os.system("copy serial_nums_flexrii.txt outputs\serial_nums_flexrii.txt")
