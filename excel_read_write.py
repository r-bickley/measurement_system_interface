# Functions for reading and writing data from input excel file

import xlrd
import xlwt

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

def write(loc):
    #wb = xlwt.
    pass

loc = ("155100254006.xls")
ddts, fids = read(loc)
print("DDTs: ", ddts)
print("Fiducials: ", fids)
