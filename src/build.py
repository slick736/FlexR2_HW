import os

snfile = "serial_nums.txt"
sourcefile = "app/main.h"
tmpfile = "main.h"

os.system("rm -rf ../outputs/*")
sn = open(snfile, 'r')
i = 1

for serialNumber in sn:
    serialNumber = serialNumber.replace('\n', '')
    serialNumberHex = ''

    for c in serialNumber:
        serialNumberHex += hex(ord(c)) + ', '
    serialNumberHex = serialNumberHex[0:len(serialNumberHex) - 2]

    source = open(sourcefile, 'r')
    tmp = open(tmpfile, 'w')

    for line in source:
        if line.startswith('#define HEALEREMG_SERIAL_NUMBER '):
            line = '#define HEALEREMG_SERIAL_NUMBER    ' + serialNumberHex + '\n'
        tmp.write(line)
    source.close()
    tmp.close()

    os.system("mv " + tmpfile + " " + sourcefile)
    #os.system("C:/Program Files (x86)/IAR Systems/Embedded Workbench 8.0/common/bin/IarBuild.exe ../tirtos/iar/app/cc2640r2lp_app.ewp")
    #os.system("D:/Software/IAR_EW/common/bin/IarBuild.exe ../tirtos/iar/app/cc2640r2lp_app.ewp -build CC2640R2F")
    #os.system("cp ../CC2540/Exe/HealerBLEPeripheral.hex ../outputs/" + str(i) + "." + serialNumber + ".hex")
    i += 1
os.system("cp serial_nums.txt ../outputs/")
