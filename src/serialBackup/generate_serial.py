import sys

#Sample: python generate_serial.py "18" "0620" "01" 20

if (len(sys.argv) != 5):
    print("Invalid number of arguments: must be 4")
    sys.exit(-1)

productYear = sys.argv[1]
productDate = sys.argv[2]
productBatch = sys.argv[3]
numProducts = int(sys.argv[4])

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

with open('serial_nums_flexrii.txt', 'w') as f:
    for i in range(1, numProducts + 1):
        serialNum = productYear + productDate + productBatch

        # Serial Number Must Be 8 Digits
        if (len(serialNum) != 8):
            print("Invalid number of digits: must be 8")
            sys.exit(-1)

        serialNum += '{0:06}'.format(i)

        # Plus prime factor
        a = 0
        serialNumNeo = ''
        serialNumOrd = 0

        for j in range(0, 15):
            serialNumbersStandard[j] = 0

        for j in range(len(serialNum)):
            # Get the whole serial number without validation digit
            serialNumOrd = int(serialNum[j])
            serialNumbersStandard[j] = serialNumOrd
            a += (serialNumOrd * primeFactor[j])
            serialNumNeo += str(serialNumbersStandard[j])
        
        a %= 10
        serialNumbersStandard[14] = a
        # serialNumNeo += str(a)
        # print serialNumNeo

        # Plus validation matrix
        a = 0
        serialNumNeo = ''
        serialNumOrd = 0
        for j in range(0, 15):
            a = 0
            for k in range(0, 15):
                a += (serialNumbersStandard[k] * validationMap[k][j])
            serialNumNeo += str(a)

        f.write(serialNumNeo + '\n')

