# Functions for reading and writing data from input excel file
# Author: Matthew Kramer, Team ME46

import xlrd
import xlwt
from xlutils.copy import copy

def read(loc):
    wb = xlrd.open_workbook(loc, on_demand = True)
    ws = wb.sheet_by_index(0)

    ddts = []
    fids = []
    
    for i in range(ws.nrows):
        refdes = ws.cell_value(i,0)
        symx = ws.cell_value(i,4)
        symy = ws.cell_value(i,5)
        values = [refdes, symx, symy, 0, 0]
        if refdes[:2] == 'TE':
            ddts.append(values)
        elif refdes[:3] == 'FID':
            fids.append(values)

    wb.release_resources()
    del wb
    return ddts, fids

def writeFids(loc, fids):
    tempWb = xlrd.open_workbook(loc, on_demand = True)
    wb = copy(tempWb)
    ws = wb.sheet_by_index(0)

    for i in range(ws.nrows):
        refdes = ws.cell_value(i,0)
        if refdes[:3] == 'FID':
            ws.write(i,8,'height')

    wb.save(loc)
    wb.release_resources()
    del wb

def writeDDTs(loc, fids):
    pass

loc = ("155100254006.xls")
ddts, fids = read(loc)
writeFids(loc,[])
#print("DDTs: ", ddts)
#print("Fiducials: ", fids)
