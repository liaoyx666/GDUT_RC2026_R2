import csv

def Data_Loader_CSV(x,y,File_Path):
    with open(File_Path,mode= 'r',encoding='utf-8') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
            print(row)