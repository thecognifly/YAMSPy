filename = "msp_codes_from_inav.txt"

with open(filename, 'r') as f:
    data = f.readlines()


with open('msp_codes.py','w') as f:
    f.write("MSPCodes = {\n")
    for l in data:
        if '#define MSP' in l:
            if l[0] != '/':
                code_str, code_value = l.split()[1:3]
                f.write(f"\'{code_str}\': {code_value},\n")
    f.write('}\n')
    f.write("# The idea is to automate, so only the dictionary above needs to be updated\n")
    f.write("MSPCodes2Str = {v: i for i,v in MSPCodes.items()}\n")