import sys

#Sample: python generate_serial_flexrii.py "01" "0001" "01" "001" 5

if (len(sys.argv) != 6):
    print("Invalid number of arguments")
    sys.exit(-1)

productDate = sys.argv[1]
productModel = sys.argv[2]
productColor = sys.argv[3]
productSerial = sys.argv[4]
numProducts = int(sys.argv[5])

b = [1, 2, 8, 7, 10, 4, 2, 5, 9, 2, 3, 6, 4]
mod = 16
#validationMap = ['5', '4', '3', '2', '1', '0', 'X', '9', '8', '7', '6']
validationMap = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F']

with open('serial_nums_flexrii.txt', 'w') as f:
    for i in range(1, numProducts + 1):
        serialNum = productDate + productModel + productColor + productSerial + '{0:05}'.format(i)
        #print(serialNum)
        #a = []
        #for j in range(len(serialNum)):
            #if j < 2:
                #a.append(ord(serialNum[j]) - ord('A') + 1)
            #else:
                #a.append(int(serialNum[j]))
        
        #validationNum = 0
        #for j in range(len(a)):
            #validationNum += a[j] * b[j]
        #validationNum %= mod
        #validationNum = validationMap[validationNum]
        #serialNum += str(validationNum)
        
        a = 0
        serialNumNeo = ''
        serialNumOrd = ''
        serialNumTmp = ''
        for j in range(len(serialNum)):
            serialNumOrd = int(serialNum[j])
            a += serialNumOrd
            #serialNumOrd += 3
            #serialNumOrd %= 10
            serialNumTmp = str(serialNumOrd)
            serialNumNeo += serialNumTmp
        
        a %= mod
        serialNumEnd = validationMap[a]
        serialNumNeo += serialNumEnd
        
        f.write(serialNumNeo + '\n')

