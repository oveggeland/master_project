import os

if __name__ == "__main__":
    print("hei")
    list10 = ["main10_mono_initdepth", "main10_nocross_notriangulation_initdepth", "main10_mono_noinitdepth", "main10_nocross_notriangulation_noinitdepth"]
    list20 = ["main20_mono_initdepth", "main20_nocross_notriangulation_initdepth", "main20_mono_noinitdepth", "main20_nocross_notriangulation_noinitdepth"]
    list25 = ["main25_mono_initdepth", "main25_nocross_notriangulation_initdepth", "main25_mono_noinitdepth", "main25_nocross_notriangulation_noinitdepth"]
    
    for name in list25:
        os.system(f"python3 ../evo/scripts/run_evo.py {name}")
    #os.system("python3 data/run_rovio.py main25")