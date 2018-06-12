import sys

#Sample: python varidate_serial.py "180522011000001"

if (len(sys.argv) != 2):
    print("Invalid number of arguments: must be 1")
    sys.exit(-1)

serialNum = sys.argv[1]

if (len(serialNum) != 15):
    print("Invalid number of digits: must be 15")
    sys.exit(-1)

# Generate Validation Map
validationMap = []
validationMap.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
validationMap.append([0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
validationMap.append([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
validationMap.append([0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
validationMap.append([0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
validationMap.append([0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
validationMap.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0])
validationMap.append([0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0])
validationMap.append([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
validationMap.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0])
validationMap.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
validationMap.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0])
validationMap.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0])
validationMap.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1])
validationMap.append([0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])

# Generate prime factor
primeFactor = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43];

# Define standard serial number array
serialNumbersStandard = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
# serialNumbersStr = ''
for j in range(0, 15):
    # serialNumbersStr += serialNum[j]
    serialNumbersStandard[j] = int(serialNum[j])

# Plus validation matrix-translation
a = 0
serialNumOrd = 0
serialNumbersStr = ''
serialNumbersOriginal = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
for j in range(0, 15):
    a = 0
    for k in range(0, 15):
        a += (serialNumbersStandard[k] * validationMap[j][k])
    serialNumbersOriginal[j] = a
    # serialNumbersStr += str(a)
# print '\nThe original serial number is:'
# print serialNumbersStr
# print '\n'

# Validate the original serial number
a = 0
for j in range(0, 14):
    a += (serialNumbersOriginal[j] * primeFactor[j])
a %= 10
if (serialNumbersOriginal[14] == a):
    print 'The serial number is valid.'
    print '\n'
else:
    print 'The serial number is INVALID!'
    print '\n'
