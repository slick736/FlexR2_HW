import sys

if (len(sys.argv) != 6):
    print("Invalid number of arguments")
    sys.exit(-1)

productType = sys.argv[1]
productNumber = sys.argv[2]
year = sys.argv[3]
batch = sys.argv[4]
numProducts = int(sys.argv[5])

b = [1, 2, 8, 7, 10, 4, 2, 5, 9, 2, 3, 6, 4]
mod = 11
validationMap = ['5', '4', '3', '2', '1', '0', 'X', '9', '8', '7', '6']

with open('serial_nums.txt', 'w') as f:
    for i in range(1, numProducts + 1):
        serialNum = productType + productNumber + year + batch + '{0:05}'.format(i)
        a = []
        for j in range(len(serialNum)):
            if j < 2:
                a.append(ord(serialNum[j]) - ord('A') + 1)
            else:
                a.append(int(serialNum[j]))
        
        validationNum = 0
        for j in range(len(a)):
            validationNum += a[j] * b[j]
        validationNum %= mod
        validationNum = validationMap[validationNum]
        serialNum += str(validationNum)
        
        f.write(serialNum + '\n')

