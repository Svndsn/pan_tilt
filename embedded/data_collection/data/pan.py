# Read a file from input
import sys
import pandas as pd

with open(sys.argv[1], "r") as file:
    df = pd.read_csv(file)
    # Subtracting 32768 from Count column
    df["Count"] = (df["Count"] - 32768) 
    df["Time"] = df["Time"] / 1000
    outputPath = sys.argv[1].split(".")[0] + "_o.csv"
    df.to_csv(outputPath, index=False)
