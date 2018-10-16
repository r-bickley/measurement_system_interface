#Demonstrates a possible implementation of the input / output functionality

#takes the csv as input
#Read values from the .csv into lists
#Designators:
Dlist = [1,2,3,4,5]
#X-coordinates:
Xlist = [1,2,3,4,5]
#Y-Coordinates:
Ylist = [1,2,3,4,5]
#Tuner bottom thickness
tb_t = 0.001 #to be replaced with the correct standard value
#PCB Thickness
pcb_t = float(input('input the thickness of the PCB for this model: '))

output = [['Designator', 'X', 'Y', 'Tuner Length', 'Depth']]

for i in range(len(Dlist)):
    a = float(input('Position the beam over PCB next to tuner ' + str(Dlist[i]) + ' and report: '))
    b = float(input('Aim the beam down the center of tuner ' + str(Dlist[i]) + ' and report: '))
    c = float(input('Position the beam at the top of tuner' + str(Dlist[i]) + ' and report: '))

    length = b - a + tb_t
    depth = length - (a - c) - pcb_t

    output.append([Dlist[i], Xlist[i], Ylist[i], length, depth])

print(output)
#write to csv
#return the csv
