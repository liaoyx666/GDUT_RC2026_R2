import numpy
def polyfit(x, y, degree):
    results = {}
    coeffs = numpy.polyfit(x, y, degree)
    results['polynomial'] = coeffs.tolist()
    p = numpy.poly1d(coeffs)
    yhat = p(x)
    ybar = numpy.sum(y)/len(y)
    ssreg = numpy.sum((yhat-ybar)**2)
    sstot = numpy.sum((y - ybar)**2)
    results['determination'] = ssreg / sstot #准确率
    return results






x=[3,2,1,6,2]
y=[1,2,4,1,2]
Power = 2
z1 = polyfit(x, y, Power)
count = 0
while count <= Power:
    if count != Power:
        print(z1['polynomial'][count],end='')
    if count != Power:
        print("*x^",end='')
        print(Power-count,end='')
        if z1['polynomial'][count]>0:
            print("+")
    else :
        if z1['polynomial'][count]>0:
            print("+",z1['polynomial'][count])   
    count +=1

