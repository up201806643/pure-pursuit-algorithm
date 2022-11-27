file = open('target_speed.txt', 'r+')
while int(file.readline().split()[0]) < 2100:
    if int(file.readline().split()[0]) == 12:
        print(file.write())
