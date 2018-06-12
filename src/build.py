import os
import sys
import shutil

#Sample: python build.py "2018" "0620" "01"

if (len(sys.argv) != 4):
    print("Invalid number of arguments: must be 3")
    sys.exit(-1)

productYear = sys.argv[1]
productDate = sys.argv[2]
productBatch = sys.argv[3]

snfile = "SerialNumbers.txt"
xlsfile = "serials.xlsx"
sourcefile = "app/main.h"
tmpfile = "main.h"
sequencefile = "Sequence.txt"
outputfile = "../tirtos/iar/app/FlashROM_StackLibrary/Exe/cc2640r2lp_app.bin"

sn = open(snfile, 'r')
toGo = 0

for serialNumber in sn:
    toGo += 1

sn = open(snfile, 'r')

i = 1
overwriteBatchFlag = 0
source = open(sequencefile, 'r')
tmpsequence = int(source.readline())
if tmpsequence <= 0:
    i = 1
    overwriteBatchFlag = 1
else:
    i = tmpsequence
    overwriteBatchFlag = 0
    # At This Case Should Read The Date And Batch Rather Than Write
    tmpsequence = int(source.readline())
    productYear = source.readline()
    productYear = productYear.strip('\n')
    productDate = source.readline()
    productDate = productDate.strip('\n')
    productBatch = source.readline()
    productBatch = productBatch.strip('\n')

source.close()

# First Build Target Bins FileCase
outputfilecase = "../tirtos/iar/app/FlashROM_StackLibrary/Exe/" + productYear + "_" + productDate + "_" + productBatch
isExists = os.path.exists(outputfilecase)
if not isExists:
    os.makedirs(outputfilecase)

source = open(sequencefile, 'w')
source.write(str(i))
source.write("\n")
source.write(str(toGo))
source.write("\n")
source.write(productYear)
source.write("\n")
source.write(productDate)
source.write("\n")
source.write(productBatch)
source.close()

jumpSequence = 1
for serialNumber in sn:
    if jumpSequence < i:
        jumpSequence += 1
        toGo -= 1
        continue
    
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
    
    renamefile = outputfilecase + "/FlexRII_" + '{0:05}'.format(i) + "_" + serialNumber + ".bin"
    
    os.system("move /y " + tmpfile + " " + sourcefile)
    os.system("D:/IAR811/common/bin/IarBuild.exe ../tirtos/iar/app/cc2640r2lp_app.ewp -build * -varfile ../tirtos/iar/simple_peripheral.custom_argvars")
    shutil.copy(outputfile, renamefile)
    toGo -= 1
    
    print "\nAlready Compiled:"
    print i
    print "To Go:"
    print toGo
    print "\n"
    
    i += 1
    jumpSequence = i
    source = open(sequencefile, 'w')
    source.write(str(i))
    source.write("\n")
    source.write(str(i + toGo - 1))
    source.write("\n")
    source.write(productYear)
    source.write("\n")
    source.write(productDate)
    source.write("\n")
    source.write(productBatch)
    source.close()

source = open(sequencefile, 'w')
source.write("0")
source.write("\n")
source.write(str(i + toGo - 1))
source.write("\n")
source.write(productYear)
source.write("\n")
source.write(productDate)
source.write("\n")
source.write(productBatch)
source.close()

# Last Copy Serial And XLS To Target FileCase
renamefile = outputfilecase + "/" + snfile
shutil.copy(snfile, renamefile)
renamefile = outputfilecase + "/" + xlsfile
shutil.copy(xlsfile, renamefile)
renamefile = outputfilecase + "/" + sequencefile
shutil.copy(sequencefile, renamefile)
