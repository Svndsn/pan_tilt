# Read a file from input
import sys
import pandas as pd
import os


def processFile(file):
    df = pd.read_csv(file)
    outputPath = "processed/" + os.path.basename(file.name)
    if not os.path.exists("processed"):
        os.makedirs("processed")
    # Subtracting 32768 (center/start) from "Count" column
    if df["Axis"].iloc[0] == "T":
        # If Axis is T, multiply by 0.8 450/360
        df["Count"] = round((df["Count"] - 32768) * 0.8, 3)
    elif df["Axis"].iloc[0] == "P":
        # If Axis is P, multiply by
        df["Count"] = round(df["Count"] - 32768, 3)
    # Divide by 1000 to convert ms to s
    df["Time"] = df["Time"] / 1000

    # Rename count since it's now an angle
    df.rename(columns={"Count": "Angle"}, inplace=True, errors="raise")
    print("Processed file: " + outputPath)
    df.to_csv(outputPath, index=False)


if __name__ == "__main__":
    # Check if is a folder
    if os.path.isdir(sys.argv[1]):
        for file in os.listdir(sys.argv[1]):
            if file.endswith(".csv"):
                with open(sys.argv[1] + "/" + file, "r") as f:
                    processFile(f)
            else:
                print(f"Invalid file type: {file}. Must be a .csv file.")
    elif os.path.isfile(sys.argv[1]):
        # Check if is a csv file
        if not sys.argv[1].endswith(".csv"):
            print("Invalid file type. Must be a .csv file.")
            sys.exit()
        with open(sys.argv[1], "r") as file:
            processFile(file)
    else:
        print("Invalid path or file.")
        print("Usage: python tilt.py <file/folder>")
