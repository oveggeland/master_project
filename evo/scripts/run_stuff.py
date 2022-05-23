import os

if __name__ == "__main__":
    list10 = ["main10_mono_initdepth", "main10_nocross_notriangulation_initdepth", "main10_mono_noinitdepth", "main10_nocross_notriangulation_noinitdepth", "main10_stereo_initdepth", "main10_stereo_noinitdepth"]
    list20 = ["main20_mono_initdepth", "main20_nocross_notriangulation_initdepth", "main20_mono_noinitdepth", "main20_nocross_notriangulation_noinitdepth", "main20_stereo_initdepth", "main20_stereo_noinitdepth"]
    list25 = ["main25_mono_initdepth", "main25_nocross_notriangulation_initdepth", "main25_mono_noinitdepth", "main25_nocross_notriangulation_noinitdepth", "main25_stereo_initdepth", "main25_stereo_noinitdepth"]

    all = list10
    all.append(list20)
    all.append(list25)

    os.system("python3 ../../rovio/data/run_rovio.py main25 main20 main10")

    """

    for name in list10:
        os.system(f"python3 full_pipeline.py {name} 10")
    
    for name in list20:
        os.system(f"python3 full_pipeline.py {name} 20")

    for name in list25:
        os.system(f"python3 full_pipeline.py {name} 25")
    
    """