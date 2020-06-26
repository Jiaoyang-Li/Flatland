def return_many():
    return [1], [2,3,4], [(1,2),(3,4)]

print(return_many())

a,b,c = return_many()

print(a)
print(b)
print(c)
