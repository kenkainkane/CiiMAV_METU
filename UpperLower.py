import yaml
import io

## To open file name test.yaml as stream ##
with open("test.yaml" , 'r') as stream :
    data_loaded = yaml.load(stream)


## To repair itself because sometime it has 'IOERROR' ##
while True:
    try:        
        ## To pull a values from data_loaded as 'dict' type ##
        MyMem=data_loaded.values()[0]
        break
    except Exception:
        continue

## Input color ##
color=raw_input('Color : ').lower()

## Check Upper Color ##
for col in MyMem.keys():
    if col[0]=='u':
        if col[6:]==color:
            print 'Upper :',MyMem[col]
        

## Check Lower Color ##
for col in MyMem.keys():
    if col[0]=='l':
        if col[6:]==color:
            print 'Lower :',MyMem[col]
        
    

